"""The actuator database: what a motor can actually hold, and who says so.

The curated part of mechlint. Geometry comes from the CAD and the chain comes
from the URDF, but *how much torque this servo delivers at this voltage* exists
only on a datasheet, and hobby-servo datasheets are marketing. So every entry
carries the URL it was read from, the date it was read, and a confidence -- and
every report prints that confidence next to the margin, because a 1.3x margin
against a ``low``-confidence number is not a pass.

Two kinds of limit, because two kinds of motor. A brushed servo's stall torque
scales with its supply, so it is stored per voltage. A stepper's is set by the
current its driver pushes, so keying it by volts would be a lie; it gets one
``holding_torque_Nm`` and the current it holds at.

A bench measurement beats both. ``measured_torque_Nm`` is stored per voltage, and
when an entry has one it is the only table used for that actuator: datasheet rows at
other voltages are not mixed in, because a part that measures 35% under its
datasheet at one voltage is not going to meet it at the next.
"""

from __future__ import annotations

import datetime as dt
from functools import lru_cache
from importlib import resources
from pathlib import Path
from typing import Any, Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

from mechlint.core.materials import Confidence

Kind = Literal["hobby_servo", "smart_servo", "stepper", "dc_gearmotor"]

#: The same names as a set, so a mistyped filter can be rejected rather than quietly
#: matching nothing -- an empty result reads as a real answer, which is the worst outcome.
KINDS: frozenset[str] = frozenset(("hobby_servo", "smart_servo", "stepper", "dc_gearmotor"))

#: Datasheets quote kgf*cm; mechlint stores N*m. Here so the conversion is written once.
KGFCM_TO_NM = 0.0980665


class TableNotFound(FileNotFoundError):
    """A named actuator YAML that is not there, with the reason it usually is not."""

    def __init__(self, message: str, hint: str = "") -> None:
        super().__init__(message)
        self.message = message
        self.hint = hint


class TorqueLimit(BaseModel):
    """The torque an actuator is judged against, and on what grounds."""

    model_config = ConfigDict(frozen=True)

    torque_Nm: float
    basis: str = Field(description="One line: what this number is and at what operating point.")
    voltage_V: float | None = None
    exact: bool = Field(
        default=True, description="Whether the voltage asked for is one the datasheet lists."
    )
    note: str | None = None
    overstates: bool = Field(
        default=False,
        description="True when no figure at or below the voltage asked for exists, so the "
        "one used is from a higher voltage and claims more than the part delivers there.",
    )


class Actuator(BaseModel):
    """One row of the actuator database."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    key: str
    name: str
    vendor: str
    kind: Kind

    stall_torque_Nm: dict[float, float] = Field(
        default_factory=dict, description="Supply voltage -> stall torque. Servos."
    )
    measured_torque_Nm: dict[float, float] = Field(
        default_factory=dict,
        description="Supply voltage -> torque measured on the bench. When present, used "
        "instead of every datasheet figure.",
    )
    holding_torque_Nm: float | None = Field(
        default=None, gt=0.0, description="Torque at the rated current. Steppers."
    )
    rated_torque_Nm: float | None = Field(
        default=None, gt=0.0, description="Continuous torque, where the vendor publishes one."
    )
    rated_current_A: float | None = Field(default=None, gt=0.0)
    no_load_speed_rpm: dict[float, float] = Field(default_factory=dict)
    recommended_voltage: float | None = Field(default=None, gt=0.0)
    voltage_range: tuple[float, float] | None = None
    gear_ratio: float | None = Field(default=None, gt=0.0)
    axes: int = Field(default=1, ge=1, description="Output shafts in one case; mass covers all.")

    mass_g: float = Field(gt=0.0)
    dims_mm: tuple[float, float, float] | None = None
    mounting: str | None = None
    interface: str | None = None
    price_hint: str | None = None

    source_url: str = Field(description="Where the numbers were read. Every entry needs one.")
    source_date: dt.date = Field(description="When they were read. Datasheets get revised.")
    source: str = Field(description="What the source actually said, in its own units.")
    confidence: Confidence = "medium"
    note: str | None = None

    @model_validator(mode="after")
    def _has_a_torque(self) -> Actuator:
        if (
            not self.stall_torque_Nm
            and self.holding_torque_Nm is None
            and not self.measured_torque_Nm
        ):
            raise ValueError(
                f"actuator {self.key!r} has no torque: give stall_torque_Nm "
                "{voltage: N*m} for a servo or DC gearmotor, holding_torque_Nm for a stepper, "
                "or measured_torque_Nm {voltage: N*m}"
            )
        return self

    @property
    def mass_kg(self) -> float:
        return self.mass_g * 1e-3

    def torque_limit(self, voltage: float | None = None) -> TorqueLimit:
        """The torque this actuator is good for on a supply of ``voltage``.

        Never interpolates. When the supply is not one of the listed voltages it
        takes the highest listed one *at or below* it, which under-states rather
        than invents -- and says so, so a report can print "at 4.8 V" next to a
        robot running on 5 V instead of a number nobody measured.

        A ``measured_torque_Nm`` table, when there is one, replaces the datasheet under
        the same never-interpolate rules. One exception: below every measured voltage the
        measurement would over-state the part, so a datasheet row at or below that voltage
        is used instead -- but only when it is the lower of the two.
        """
        if self.measured_torque_Nm:
            measured = _lookup(
                self.measured_torque_Nm, "measured torque", "measured", voltage, self
            )
            if not measured.overstates or not self.stall_torque_Nm:
                return measured
            datasheet = _lookup(self.stall_torque_Nm, "stall torque", "listed", voltage, self)
            if datasheet.overstates or datasheet.torque_Nm >= measured.torque_Nm:
                return measured
            return datasheet.model_copy(
                update={
                    "note": f"below every measured voltage; the datasheet's {datasheet.basis} "
                    f"is used because it is lower than the {measured.voltage_V:g} V measurement"
                }
            )
        if self.holding_torque_Nm is not None:
            current = f" at {self.rated_current_A:g} A" if self.rated_current_A else ""
            return TorqueLimit(
                torque_Nm=self.holding_torque_Nm,
                basis=f"holding torque{current}",
                note=(
                    None
                    if voltage is None
                    else "a stepper's torque follows its driver current, not the supply voltage"
                ),
            )

        return _lookup(self.stall_torque_Nm, "stall torque", "listed", voltage, self)

    def voltage_note(self, voltage: float | None) -> str | None:
        """Whether a supply is outside what the vendor says the part tolerates."""
        if voltage is None or self.voltage_range is None:
            return None
        low, high = self.voltage_range
        if voltage < low:
            return f"{voltage:g} V is below the {low:g}-{high:g} V operating range"
        if voltage > high:
            return f"{voltage:g} V is above the {low:g}-{high:g} V operating range"
        return None

    def runs_on(self, voltage: float | None) -> bool:
        """Whether this supply gives a torque figure that can be trusted.

        False when the supply is outside the vendor's operating range, or when the
        only figure available is from a higher voltage and so OVER-states the actuator.
        Either way, ranking it would recommend a
        part that cannot deliver on that supply.
        """
        if voltage is None:
            return True
        if self.voltage_note(voltage) is not None:
            return False
        return not self.torque_limit(voltage).overstates


def _lookup(
    table: dict[float, float],
    label: str,
    listed_as: str,
    voltage: float | None,
    actuator: Actuator,
) -> TorqueLimit:
    """One voltage-keyed table read without interpolating: the exact row, else the
    highest one below (under-states), else the lowest one (OVER-states, and says so)."""
    wanted = voltage if voltage is not None else actuator.recommended_voltage
    listed = sorted(table)
    if wanted is None:
        wanted = listed[-1]

    for candidate in listed:
        if abs(candidate - wanted) < 1e-9:
            return TorqueLimit(
                torque_Nm=table[candidate],
                basis=f"{label} at {candidate:g} V",
                voltage_V=candidate,
            )

    below = [v for v in listed if v < wanted]
    if below:
        used = below[-1]
        return TorqueLimit(
            torque_Nm=table[used],
            basis=f"{label} at {used:g} V",
            voltage_V=used,
            exact=False,
            note=f"{wanted:g} V is not a {listed_as} voltage; the {used:g} V figure is used, "
            "which under-states this actuator",
        )

    used = listed[0]
    return TorqueLimit(
        torque_Nm=table[used],
        basis=f"{label} at {used:g} V",
        voltage_V=used,
        exact=False,
        overstates=True,
        note=f"{wanted:g} V is below every {listed_as} voltage; the {used:g} V figure is used, "
        "which OVER-states what this actuator delivers on that supply",
    )


class ActuatorDatabase(BaseModel):
    """The whole table, keyed by the short name used in ``mechlint.yaml``."""

    model_config = ConfigDict(frozen=True)

    actuators: dict[str, Actuator]

    @classmethod
    def from_yaml(cls, path: str | Path) -> ActuatorDatabase:
        source = Path(path)
        if not source.is_file():
            # A bare "[Errno 2] No such file" is a stack trace by another name: it says
            # nothing about which of the three places named this file, or what to do.
            raise TableNotFound(
                f"actuator table not found: {source}",
                "a path in mechlint.yaml's actuator_db is relative to that file; "
                "--actuator-db and the actuator_db argument are relative to where you are.",
            )
        return cls._from_text(source.read_text(), source=str(source))

    @classmethod
    def _from_text(cls, text: str, *, source: str) -> ActuatorDatabase:
        raw: Any = yaml.safe_load(text) or {}
        entries = raw.get("actuators")
        if not isinstance(entries, dict):
            raise ValueError(f"{source} must have a top-level 'actuators' mapping")
        return cls(actuators={key: Actuator(key=key, **value) for key, value in entries.items()})

    def __getitem__(self, key: str) -> Actuator:
        try:
            return self.actuators[key]
        except KeyError:
            known = ", ".join(sorted(self.actuators))
            raise KeyError(f"unknown actuator {key!r}; known actuators: {known}") from None

    def __contains__(self, key: str) -> bool:
        return key in self.actuators

    def __iter__(self) -> Any:
        return iter(sorted(self.actuators.values(), key=lambda a: a.key))

    def __len__(self) -> int:
        return len(self.actuators)

    def merged_with(self, other: ActuatorDatabase) -> ActuatorDatabase:
        """This table plus another one, the other winning on shared keys."""
        return ActuatorDatabase(actuators={**self.actuators, **other.actuators})

    def matching(
        self,
        *,
        min_torque_Nm: float | None = None,
        voltage: float | None = None,
        max_mass_g: float | None = None,
        kind: Kind | None = None,
    ) -> list[Actuator]:
        """The entries that meet a requirement, strongest first.

        This is the "what else would fit?" query -- the one an LLM asks after a
        joint fails T001, and the reason the database is worth curating at all.
        With a ``voltage``, entries that cannot run on that supply are left out
        rather than ranked on a figure they would never reach.
        """
        if kind is not None and kind not in KINDS:
            raise ValueError(f"unknown kind {kind!r}; use {', '.join(sorted(KINDS))}")
        found = []
        for actuator in self:
            if kind is not None and actuator.kind != kind:
                continue
            if max_mass_g is not None and actuator.mass_g > max_mass_g:
                continue
            if not actuator.runs_on(voltage):
                continue
            if (
                min_torque_Nm is not None
                and actuator.torque_limit(voltage).torque_Nm < min_torque_Nm
            ):
                continue
            found.append(actuator)
        return sorted(found, key=lambda a: -a.torque_limit(voltage).torque_Nm)


@lru_cache(maxsize=1)
def bundled() -> ActuatorDatabase:
    """The table that ships with mechlint."""
    text = (resources.files("mechlint.data") / "actuators.yaml").read_text()
    return ActuatorDatabase._from_text(text, source="mechlint/data/actuators.yaml")


def load_database(path: str | Path | None = None) -> ActuatorDatabase:
    """The bundled table, extended by a project's own YAML if it has one."""
    if path is None:
        return bundled()
    return bundled().merged_with(ActuatorDatabase.from_yaml(path))
