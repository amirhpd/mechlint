"""The check registry and the URDF sanity checks (U0xx, M0xx).

Check IDs are the contract -- with CI, with ``docs/checks.md``, and with the
``urdfdom`` proposal in M5 -- so they are never reused and never renumbered.
The registry here is the single place the human-readable title and fix hint
live, which is why a :class:`Finding` message can be one short line: the ID
carries the rest.
"""

from __future__ import annotations

from collections import defaultdict
from collections.abc import Iterator
from enum import Enum
from pathlib import Path
from typing import NamedTuple

import numpy as np
from pydantic import BaseModel, ConfigDict, Field, computed_field

from mechlint.core import chain, units
from mechlint.core.actuators import ActuatorDatabase
from mechlint.core.actuators import bundled as bundled_actuators
from mechlint.core.config import MechlintConfig
from mechlint.core.geometry import mass_properties
from mechlint.core.urdf import RobotModel


class Severity(str, Enum):
    """FAIL sets a non-zero exit code; WARN reports; INFO is context."""

    FAIL = "fail"
    WARN = "warn"
    INFO = "info"


class CheckSpec(NamedTuple):
    id: str
    title: str
    fix: str


CHECKS: dict[str, CheckSpec] = {
    spec.id: spec
    for spec in (
        CheckSpec(
            "U001",
            "Model scale suspicious",
            "Set robot.model_scale, or convert the model to metres. mechlint can scale; "
            "a physics engine reads the numbers as metres either way.",
        ),
        CheckSpec(
            "U002",
            "Inertia tensor not positive definite",
            "No rigid body has such a tensor. Recompute it -- `mechlint inertia` writes one.",
        ),
        CheckSpec(
            "U003",
            "Triangle inequality violated",
            "Principal moments must satisfy I1 + I2 >= I3. Recompute the tensor.",
        ),
        CheckSpec(
            "U004",
            "Tensor inconsistent with the mass and the geometry",
            "Usually a tensor computed in the wrong length unit, or for a different part.",
        ),
        CheckSpec(
            "U005",
            "Placeholder inertia tensor",
            "Identical tensors mean nobody has computed the physics yet. Run `mechlint inertia`.",
        ),
        CheckSpec(
            "U006",
            "Mass missing, non-positive, or a placeholder",
            "Weigh the part into materials.links.<link>.measured_mass_g, or let "
            "`mechlint inertia` estimate it from the mesh.",
        ),
        CheckSpec(
            "U007",
            "Joint effort limit exceeds the actuator's stall torque",
            "Lower <limit effort> to what the servo delivers; a controller tuned against "
            "fiction is untuned.",
        ),
        CheckSpec(
            "M001",
            "Mesh not watertight",
            "An open mesh has no defined inside. mechlint uses the convex hull and says so -- "
            "an over-estimate. Fix the CAD export, or supply a STEP file.",
        ),
        CheckSpec(
            "T001",
            "Static torque exceeds the actuator's stall torque over the safety factor",
            "Stronger servo, mass moved inboard, or a shorter link.",
        ),
        CheckSpec(
            "D002",
            "Declared mounting disagrees with the model",
            "scenario.mount and the world-to-base rotation say different things about which "
            "way is down. The model wins, because it is what the simulator runs -- fix "
            "whichever of the two is stale.",
        ),
        CheckSpec(
            "D003",
            "mechlint.yaml names something the model does not have",
            "A link or joint in the config does not exist in the description. Usually a "
            "rename on one side only -- and a silently ignored motor is a torque number "
            "that is wrong in the safe-looking direction.",
        ),
        CheckSpec(
            "D001",
            "URDF joint spacing disagrees with the CAD",
            "One of the two is stale. Re-export, or re-measure.",
        ),
    )
}


class Finding(BaseModel):
    """One thing a check found, about one subject."""

    model_config = ConfigDict(frozen=True)

    check: str
    severity: Severity
    subject: str = Field(description="Link, joint, mesh or 'model' -- what the finding is about.")
    message: str = Field(description="One line. The ID carries the explanation.")
    subjects: list[str] = Field(
        default_factory=list,
        description="The links a grouped finding covers, when 'subject' is only a count.",
    )

    @computed_field  # type: ignore[prop-decorator]
    @property
    def title(self) -> str:
        return CHECKS[self.check].title

    @computed_field  # type: ignore[prop-decorator]
    @property
    def fix(self) -> str:
        return CHECKS[self.check].fix


def finding(
    check: str,
    severity: Severity,
    subject: str,
    message: str,
    subjects: list[str] | None = None,
) -> Finding:
    if check not in CHECKS:
        raise KeyError(f"unregistered check id {check!r}")
    return Finding(
        check=check,
        severity=severity,
        subject=subject,
        message=message,
        subjects=subjects or [],
    )


class CheckReport(BaseModel):
    """Everything one ``urdf-check`` run found."""

    robot: str
    source: Path
    model_scale: float
    findings: list[Finding] = Field(default_factory=list)
    skipped: dict[str, str] = Field(
        default_factory=dict,
        description="Check id -> why it did not run. Silence is not a pass.",
    )

    @computed_field  # type: ignore[prop-decorator]
    @property
    def ok(self) -> bool:
        return not any(f.severity is Severity.FAIL for f in self.findings)

    def counts(self) -> dict[Severity, int]:
        tally = dict.fromkeys(Severity, 0)
        for item in self.findings:
            tally[item.severity] += 1
        return tally


# --------------------------------------------------------------------------- checks


def check_urdf(
    model: RobotModel,
    config: MechlintConfig | None = None,
    *,
    actuators: ActuatorDatabase | None = None,
) -> CheckReport:
    """Every check that can be answered without computing torque."""
    database = actuators or bundled_actuators()
    findings = [
        *_u001_scale(model),
        *_tensor_checks(model),
        *_u005_placeholder_tensors(model),
        *_u006_masses(model),
        *_u007_effort_limits(model, config, database),
        *_m001_meshes(model),
        *_d002_mounting(model, config),
        *_d003_names(model, config),
    ]
    skipped = {
        "T001": "static torque is its own command: run `mechlint torque`",
        "D001": "drift arrives in M4",
    }
    if config is None or not any(config.actuators.joints.values()):
        skipped["U007"] = "needs actuators.joints in mechlint.yaml to say which servo is where"
    return CheckReport(
        robot=model.name,
        source=model.path,
        model_scale=model.model_scale,
        findings=findings,
        skipped=skipped,
    )


def _u007_effort_limits(
    model: RobotModel, config: MechlintConfig | None, actuators: ActuatorDatabase
) -> Iterator[Finding]:
    """Does ``<limit effort>`` claim more torque than the assigned servo can produce?

    The effort limit is what a controller clamps its command to, so a fictional one
    means the clamp never engages and the joint is tuned against a motor nobody owns.
    URDF states effort in N*m whatever length unit the model uses, so this comparison
    needs no scaling -- and stall torque is the right side to compare against, not
    rated torque: the limit is a ceiling, and stall is the ceiling.
    """
    if config is None:
        return
    voltage = config.actuators.voltage
    for name, key in config.actuators.joints.items():
        joint = model.urdf.joint_map.get(name)
        if not key or joint is None:
            continue
        effort = getattr(getattr(joint, "limit", None), "effort", None)
        if effort is None:
            continue
        try:
            actuator = actuators[key]
        except KeyError as error:
            yield finding("U007", Severity.FAIL, name, str(error))
            continue
        limit = actuator.torque_limit(voltage)
        if float(effort) > limit.torque_Nm * (1.0 + 1e-9):
            yield finding(
                "U007",
                Severity.FAIL,
                name,
                f"<limit effort> is {float(effort):g} N*m, but {actuator.name} gives "
                f"{limit.torque_Nm:.3g} N*m ({limit.basis}) -- "
                f"{float(effort) / limit.torque_Nm:.3g}x what the servo can deliver",
            )


def _d003_names(model: RobotModel, config: MechlintConfig | None) -> Iterator[Finding]:
    """Every link and joint ``mechlint.yaml`` mentions has to exist in the model.

    ``extra="forbid"`` on the schema catches a mistyped *key*; nothing catches a
    mistyped link *name*, and a servo attached to a link that does not exist is
    simply dropped -- quietly making the robot lighter than it is.
    """
    if config is None:
        return
    links = set(model.urdf.link_map)
    joints = set(model.urdf.joint_map)

    for where, names, known, kind in (
        ("materials.links", config.materials.links, links, "link"),
        ("components", config.components, links, "link"),
        ("actuators.joints", config.actuators.joints, joints, "joint"),
    ):
        for name in names:
            if name not in known:
                yield finding(
                    "D003",
                    Severity.FAIL,
                    f"{where}.{name}",
                    f"the model has no {kind} called {name!r}",
                )

    for link, entries in config.components.items():
        for entry in entries:
            if entry.drives is not None and entry.drives not in joints:
                yield finding(
                    "D003",
                    Severity.FAIL,
                    f"components.{link}",
                    f"drives {entry.drives!r}, which is not a joint in the model",
                )


def _d002_mounting(model: RobotModel, config: MechlintConfig | None) -> Iterator[Finding]:
    """Does ``scenario.mount`` agree with how the model is actually bolted down?

    Only a grounded model can disagree with anything: without a world frame there is
    nothing to compare the config against, and the config is simply the answer.
    """
    if config is None or not chain.is_grounded(model.urdf):
        return
    derived = chain.gravity(model.urdf)
    try:
        declared = chain.mount_gravity(config.scenario.mount)
    except ValueError as error:
        yield finding("D002", Severity.FAIL, "scenario.mount", str(error))
        return

    if np.allclose(derived.array, declared, atol=1e-6):
        return
    yield finding(
        "D002",
        Severity.WARN,
        "scenario.mount",
        f"config says {_mount_text(config.scenario.mount)} = {_fmt(declared)} m/s^2, but the "
        f"model mounts {chain.mounted_link(model.urdf)} so that gravity is "
        f"{_fmt(derived.array)}{f' ({derived.implied_mount})' if derived.implied_mount else ''}; "
        "mechlint used the model",
    )


def _mount_text(mount: object) -> str:
    return mount if isinstance(mount, str) else f"{tuple(mount)}"


def _u001_scale(model: RobotModel) -> Iterator[Finding]:
    reach_units, link = chain.reach(model.urdf)
    if reach_units <= 0.0:
        return
    scale = model.model_scale
    reach_m = reach_units * scale
    low, high = units.PLAUSIBLE_REACH_M
    mesh_note = _mesh_unit_note(model)

    if not low <= reach_m <= high:
        yield finding(
            "U001",
            Severity.FAIL,
            "model",
            f"reach to {link} is {reach_units:.3g} URDF units = {reach_m:.3g} m at "
            f"model_scale {scale:g}, outside the plausible {low}-{high} m{mesh_note}",
        )
    elif scale != 1.0:
        yield finding(
            "U001",
            Severity.WARN,
            "model",
            f"model is not in metres: one URDF unit is {scale:g} m "
            f"({_unit_name(scale)}), so reach to {link} is {reach_units:.3g} units "
            f"= {reach_m:.3g} m{mesh_note}",
        )


def _unit_name(scale: float) -> str:
    for name in ("mm", "cm", "dm", "m", "inch"):
        if np.isclose(units.NAMED_SCALES[name], scale):
            return {
                "mm": "millimetres",
                "cm": "centimetres",
                "dm": "decimetres",
                "m": "metres",
                "inch": "inches",
            }[name]
    return f"{scale:g} m per unit"


def _mesh_unit_note(model: RobotModel) -> str:
    """What the ``<mesh scale>`` values imply about the units the CAD was exported in."""
    scales = set()
    for link in model.urdf.robot.links:
        for element in [*link.visuals, *link.collisions]:
            mesh = getattr(element.geometry, "mesh", None)
            if mesh is not None and mesh.scale is not None:
                values = np.asarray(mesh.scale, dtype=float)
                if np.allclose(values, values[0]):
                    scales.add(round(float(values[0]) * model.model_scale, 12))
    if len(scales) != 1:
        return ""
    per_mesh_unit = scales.pop()
    return f"; meshes are in {_unit_name(per_mesh_unit)}"


def _tensor_checks(model: RobotModel) -> Iterator[Finding]:
    """U002, U003 and U004 -- all three read the same declared tensor."""
    for name in model.link_names:
        inertial = model.declared_inertial(name)
        if inertial is None or inertial.inertia is None:
            continue
        tensor = np.asarray(inertial.inertia, dtype=float)

        if not np.allclose(tensor, tensor.T):
            yield finding("U002", Severity.FAIL, name, "inertia tensor is not symmetric")
            continue

        moments = np.linalg.eigvalsh(tensor)
        if moments.min() <= 0.0:
            yield finding(
                "U002",
                Severity.FAIL,
                name,
                f"principal moments {_fmt(moments)} are not all positive",
            )
            continue

        i1, i2, i3 = sorted(moments)
        if i1 + i2 < i3 * (1.0 - 1e-9):
            yield finding(
                "U003",
                Severity.FAIL,
                name,
                f"principal moments {_fmt(moments)} violate I1 + I2 >= I3",
            )

        yield from _u004_matches_the_geometry(
            model, name, tensor, float(inertial.mass or 0.0), inertial.origin
        )


#: How far a declared tensor may sit from what the link's own shape implies before
#: U004 calls it wrong. A printed shell carries its mass nearer the surface than a
#: solid part of the same outline, so a factor of a few is ordinary; a factor of ten
#: is not, and a tensor worked out in the wrong length unit is off by a million.
U004_TOLERANCE = 10.0


def _u004_matches_the_geometry(
    model: RobotModel, name: str, tensor: np.ndarray, mass: float, origin: np.ndarray | None
) -> Iterator[Finding]:
    """Compare the declared tensor with the link's own geometry at the declared mass.

    Both sides are in URDF units, so the ratio is dimensionless whatever those
    units are -- which is the point: this check finds tensors computed for a
    part in millimetres and pasted into a model in metres.

    The comparison is between principal moments, about the point the declared
    ``<origin>`` names. Both matter. Principal moments because they do not
    depend on how the inertial frame is rotated; that point because a tensor
    about the link origin and the same tensor about the centre of mass differ
    by ``m*d^2``, which for an arm is most of the number.
    """
    if mass <= 0.0:
        return
    reference = _geometry_tensor(model, name, mass, origin)
    if reference is None:
        return

    declared = np.linalg.eigvalsh(tensor)
    expected = np.linalg.eigvalsh(reference)
    if declared.min() <= 0.0 or expected.min() <= 0.0:
        return

    ratios = np.maximum(declared / expected, expected / declared)
    if ratios.max() > U004_TOLERANCE:
        yield finding(
            "U004",
            Severity.FAIL,
            name,
            f"principal moments {_fmt(declared)} are {ratios.max():.3g}x off the "
            f"{_fmt(expected)} its own geometry implies at {mass:g} kg",
        )


def _geometry_tensor(
    model: RobotModel, name: str, mass: float, origin: np.ndarray | None
) -> np.ndarray | None:
    """The tensor the link's shape would have at ``mass``, about the declared origin."""
    bodies = model.geometries(name, which="visual-or-collision")
    if not bodies:
        return None

    parts = [mass_properties(body.mesh) for body in bodies]
    volume = sum(part.volume_m3 for part in parts)
    if volume <= 0.0:
        return None

    density = mass / volume
    centre = sum(part.mass_kg * np.asarray(part.com_m) for part in parts) / sum(
        part.mass_kg for part in parts
    )
    tensor = sum(part.inertia_array for part in parts) * density

    about = np.zeros(3) if origin is None else np.asarray(origin, dtype=float)[:3, 3]
    offset = centre - about
    return tensor + mass * (offset @ offset * np.eye(3) - np.outer(offset, offset))


def _shape(model: RobotModel, name: str) -> tuple[float, ...] | None:
    """A link's bounding box, rounded -- two links with the same one are the same part."""
    size = model.extents(name)
    return None if size is None else tuple(round(v, 9) for v in size)


def _differently_shaped(model: RobotModel, names: list[str]) -> bool:
    """Whether a shared value spans parts that are not copies of each other.

    A gripper's two fingers really do have the same mass and the same tensor,
    and saying so every run would train people to ignore U005 and U006. What is
    suspicious is one number covering parts of different sizes -- which is what
    a ``default_inertial`` macro produces.
    """
    return len({_shape(model, name) for name in names}) > 1


def _u005_placeholder_tensors(model: RobotModel) -> Iterator[Finding]:
    groups: dict[tuple[float, ...], list[str]] = defaultdict(list)
    for name in model.link_names:
        inertial = model.declared_inertial(name)
        if inertial is None or inertial.inertia is None:
            continue
        key = tuple(round(v, 12) for v in np.asarray(inertial.inertia, dtype=float).ravel())
        groups[key].append(name)

    for key, names in groups.items():
        if len(names) > 1 and _differently_shaped(model, names):
            diagonal = (key[0], key[4], key[8])
            yield finding(
                "U005",
                Severity.WARN,
                f"{len(names)} links",
                f"{', '.join(names)} all declare the tensor diag {_fmt(np.array(diagonal))}",
                subjects=names,
            )


def _u006_masses(model: RobotModel) -> Iterator[Finding]:
    by_mass: dict[float, list[str]] = defaultdict(list)
    for name in model.link_names:
        inertial = model.declared_inertial(name)
        has_geometry = bool(model.geometries(name, which="visual-or-collision"))

        if inertial is None or inertial.mass is None:
            if has_geometry:
                yield finding(
                    "U006", Severity.FAIL, name, "link has geometry but no <inertial> mass"
                )
            continue

        mass = float(inertial.mass)
        if mass <= 0.0:
            yield finding("U006", Severity.FAIL, name, f"mass is {mass:g} kg")
            continue
        by_mass[round(mass, 12)].append(name)

    for mass, names in by_mass.items():
        if len(names) > 1 and _differently_shaped(model, names):
            yield finding(
                "U006",
                Severity.WARN,
                f"{len(names)} links",
                f"{', '.join(names)} all declare the mass {mass:g} kg",
                subjects=names,
            )


def _m001_meshes(model: RobotModel) -> Iterator[Finding]:
    for name in model.link_names:
        for body in model.geometries(name, which="visual-or-collision"):
            if body.kind == "mesh" and not body.watertight:
                yield finding(
                    "M001",
                    Severity.WARN,
                    f"{name}: {Path(body.source).name}",
                    "not a closed volume; mass properties come from its convex hull",
                )


def _fmt(values: np.ndarray) -> str:
    return "[" + ", ".join(f"{v:.4g}" for v in np.asarray(values).ravel()) + "]"
