"""Schema for ``mechlint.yaml`` — the facts that are in neither the CAD nor the URDF.

Motors, materials, measured weights, payload and how the robot is mounted live
here, next to the robot description, the way ``package.xml`` sits next to a ROS
package. Nothing is ever asked interactively: what an LLM learns from you in
conversation it *writes into this file*, so the knowledge is versioned and
reviewable instead of trapped in a chat.

Unknown keys are rejected rather than ignored — a typo in a config that decides
whether a servo is strong enough should fail loudly.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, PrivateAttr, model_validator

Vec3 = tuple[float, float, float]

#: Only the mountings a single name can actually pin down. A wall mount leaves gravity
#: free to point any horizontal direction in the base frame, depending on how the robot
#: is rolled about its mounting axis, so it is spelled as an explicit vector -- or, far
#: better, left to the world-to-base rotation in the model, which already says it.
Mount = Literal["table", "ceiling"]


class _Strict(BaseModel):
    model_config = ConfigDict(extra="forbid")


class RobotConfig(_Strict):
    """Where the description is and how to read it."""

    description: Path = Field(description="Path to the URDF or xacro, relative to this file.")
    xacro_args: dict[str, str] = Field(
        default_factory=dict, description="Arguments passed to xacro, e.g. {is_sim: 'true'}."
    )
    package_paths: dict[str, Path] = Field(
        default_factory=dict,
        description="Resolves package:// without a sourced ROS shell: {package: path}.",
    )
    model_scale: float | str = Field(
        default=1.0,
        description="Metres per URDF unit, or a unit name ('dm'). DAST-1 is modelled in "
        "decimetres, so 0.1. Check U001 flags a model whose reach looks implausible.",
    )


class MaterialSpec(_Strict):
    """How a link is made, or what it actually weighed."""

    material: str | None = Field(default=None, description="Key into the material density table.")
    infill: float | None = Field(
        default=None, ge=0.0, le=1.0, description="Printed infill fraction, 0..1."
    )
    measured_mass_g: float | None = Field(
        default=None,
        gt=0.0,
        description="Mass from a scale. Wins over the estimate; the report says which was used.",
    )


class MaterialsConfig(_Strict):
    default: MaterialSpec = Field(default_factory=lambda: MaterialSpec(material="pla", infill=1.0))
    links: dict[str, MaterialSpec] = Field(default_factory=dict)


class Component(_Strict):
    """Mass that is not in the mesh — a servo, a battery, a gripper motor.

    An ``actuator`` is looked up in the actuator database for its mass. List it
    under the link it is bolted to -- the parent or the child of the joint it
    ``drives`` -- and it sits on that joint, in the frame of the link you list it
    under, unless ``at`` says otherwise. Listed under any other link, it needs an
    ``at``: mechlint cannot know where it is (check D004).
    """

    actuator: str | None = None
    drives: str | None = Field(default=None, description="Joint this actuator drives.")
    mass_g: float | None = Field(default=None, gt=0.0)
    at: Vec3 | None = Field(
        default=None,
        description="Position in the frame of the link it is listed under, URDF units.",
    )
    note: str | None = None

    @model_validator(mode="after")
    def _needs_a_mass_source(self) -> Component:
        if self.actuator is None and self.mass_g is None:
            raise ValueError("a component needs either 'actuator' (mass from the db) or 'mass_g'")
        if self.actuator is None and self.drives is not None:
            raise ValueError("'drives' names the joint an actuator turns; it needs 'actuator' too")
        if self.actuator is not None and self.drives is None and self.at is None:
            raise ValueError(
                f"actuator {self.actuator!r} needs 'drives' (a joint) or 'at' (a position) "
                "so mechlint knows where to place its mass"
            )
        return self


class Transmission(_Strict):
    """A gear stage, belt or capstan between the actuator's shaft and the joint."""

    ratio: float = Field(
        gt=0.0,
        description="Joint-side teeth / motor-side teeth (or pulley diameters). Above 1 is a "
        "reduction: a small gear on the motor driving a big one on the joint is ratio 2.0 for "
        "a 1:2 stage, giving the joint twice the torque at half the speed.",
    )
    efficiency: float | None = Field(
        default=None,
        gt=0.0,
        le=1.0,
        description="Fraction of the torque that survives the stage, 0..1. Left out, the "
        "stage is treated as ideal (1.0) and every report says so.",
    )

    @property
    def factor(self) -> float:
        """What the actuator's torque is multiplied by on its way to the joint."""
        return self.ratio * (1.0 if self.efficiency is None else self.efficiency)

    def describe(self) -> str:
        """The stage in a few words, for a report: ``x 2:1 stage x 0.95 efficiency``."""
        if self.efficiency is None:
            return f"x {self.ratio:g}:1 stage, ideal (no efficiency given)"
        return f"x {self.ratio:g}:1 stage x {self.efficiency:g} efficiency"


def joint_torque_limit(motor_Nm: float, transmission: Transmission | None) -> float:
    """What reaches the joint: the actuator's figure times any gear stage."""
    return motor_Nm if transmission is None else motor_Nm * transmission.factor


def joint_torque_basis(basis: str, motor_Nm: float, transmission: Transmission | None) -> str:
    """The basis line, naming the motor figure and the stage so neither is mistaken
    for the other: a bare motor number must never read as the joint's."""
    if transmission is None:
        return basis
    return f"{basis} ({motor_Nm:.3g} N*m) {transmission.describe()}"


class JointDrive(_Strict):
    """The long form of one ``actuators.joints`` entry: the actuator and how it is driven."""

    actuator: str = Field(description="Key into the actuator database.")
    voltage: float | None = Field(
        default=None,
        gt=0.0,
        description="This joint's supply voltage. Wins over actuators.voltage.",
    )
    transmission: Transmission | None = Field(
        default=None, description="A gear stage between the actuator and the joint, if any."
    )


class ActuatorsConfig(_Strict):
    voltage: float | None = Field(
        default=None,
        gt=0.0,
        description="Default supply voltage for every joint that does not give its own; stall "
        "torque is quoted per voltage.",
    )
    joints: dict[str, str | JointDrive | None] = Field(
        default_factory=dict,
        description="Actuator per joint: a key (`joint_1: mg996r`), or the long form "
        "`{actuator, voltage, transmission}` for a joint on its own supply rail or behind a "
        "gear stage. An explicit null means 'no servo yet' — torque is reported for that "
        "joint but not judged.",
    )

    def drive(self, joint: str) -> JointDrive | None:
        """One joint's drive in the long form, with the default voltage filled in.

        The one place both spellings are read, so no caller parses them twice. None
        when the joint has no actuator (absent, or an explicit null).
        """
        entry = self.joints.get(joint)
        if entry is None:
            return None
        if isinstance(entry, str):
            return JointDrive(actuator=entry, voltage=self.voltage) if entry else None
        if entry.voltage is None:
            return entry.model_copy(update={"voltage": self.voltage})
        return entry

    def drives(self) -> dict[str, JointDrive | None]:
        """Every joint the config names, resolved by :meth:`drive`."""
        return {joint: self.drive(joint) for joint in self.joints}


class Scenario(_Strict):
    """The load case every torque number is computed for."""

    mount: Mount | Vec3 = Field(
        default="table",
        description="Named mounting, or an explicit gravity vector in the base frame. Only "
        "consulted when the model is not grounded: a URDF with a world frame already states "
        "its mounting in the world-to-base rotation, and check D002 reports a disagreement.",
    )
    payload_g: float = Field(default=0.0, ge=0.0, description="Mass held at the tip.")
    safety_factor: float = Field(
        default=2.0,
        gt=0.0,
        description="Required margin on stall torque. A convention, not physics — every report "
        "prints it so nobody mistakes it for one.",
    )


class MechlintConfig(_Strict):
    """A whole ``mechlint.yaml``."""

    robot: RobotConfig
    actuator_db: Path | None = Field(
        default=None,
        description="This project's own actuator YAML, merged over the bundled table. "
        "Relative to this file. Put a servo mechlint has never heard of here.",
    )
    materials: MaterialsConfig = Field(default_factory=MaterialsConfig)
    components: dict[str, list[Component]] = Field(default_factory=dict)
    actuators: ActuatorsConfig = Field(default_factory=ActuatorsConfig)
    scenario: Scenario = Field(default_factory=Scenario)

    _source_path: Path | None = PrivateAttr(default=None)

    @classmethod
    def from_yaml(cls, path: str | Path) -> MechlintConfig:
        path = Path(path)
        if not path.is_file():
            raise FileNotFoundError(f"config not found: {path}")
        raw: Any = yaml.safe_load(path.read_text()) or {}
        if not isinstance(raw, dict):
            raise ValueError(f"{path} must contain a YAML mapping, got {type(raw).__name__}")
        config = cls.model_validate(raw)
        config._source_path = path.resolve()
        return config

    @property
    def source_path(self) -> Path | None:
        """The file this config was read from, if any."""
        return self._source_path

    @property
    def base_dir(self) -> Path:
        """Directory that relative paths in this config are resolved against."""
        return self._source_path.parent if self._source_path else Path.cwd()

    def resolve(self, path: str | Path) -> Path:
        """Make a config-relative path absolute."""
        candidate = Path(path)
        return candidate if candidate.is_absolute() else (self.base_dir / candidate).resolve()

    @property
    def description_path(self) -> Path:
        return self.resolve(self.robot.description)

    @property
    def actuator_db_path(self) -> Path | None:
        return None if self.actuator_db is None else self.resolve(self.actuator_db)


def applied_overrides(**values: object) -> dict[str, object]:
    """The per-call overrides that were actually given, for a report to echo back.

    A what-if answer is only quotable if it carries its own assumptions: "2.25 N*m"
    means nothing without "at 200 g, on the ceiling, at 4.8 V". Callers pass every
    override they accepted, and the ones left unset drop out here rather than
    filling the result with nulls that read like decisions.
    """
    kept: dict[str, object] = {}
    for name, value in values.items():
        if value is None or (isinstance(value, dict | list | tuple) and not value):
            continue
        kept[name] = value
    return kept
