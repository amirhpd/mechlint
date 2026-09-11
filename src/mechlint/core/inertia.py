"""Composite mass properties per link, and the ``<inertial>`` blocks they become.

A link is not one body. It is a printed shell, at whatever infill the slicer
used, plus the discrete things bolted to it -- a servo is a known mass at a
known place. This module puts those together with the parallel-axis theorem and
reports, per link, whether the mass was *measured*, *estimated* from the mesh,
or simply the value already in the URDF.

Everything is computed in SI. The generated xacro converts back to the model's
own length unit at the last moment, so a decimetre model stays self-consistent.
"""

from __future__ import annotations

from collections.abc import Mapping
from datetime import datetime, timezone
from pathlib import Path
from typing import Literal
from xml.sax.saxutils import escape

import numpy as np
from pydantic import BaseModel, ConfigDict, Field, computed_field

from mechlint import __version__
from mechlint.core import chain
from mechlint.core.actuators import ActuatorDatabase
from mechlint.core.actuators import bundled as bundled_actuators
from mechlint.core.checks import Finding, Severity, finding
from mechlint.core.config import MaterialSpec, MechlintConfig, applied_overrides
from mechlint.core.geometry import MassProperties, Mat3, Vec3, mass_properties
from mechlint.core.materials import MaterialDatabase, density_of
from mechlint.core.urdf import RobotModel

MassSource = Literal["measured", "estimated", "urdf", "components", "none", "override"]

#: A body whose mass, centre and tensor are all known: (kg, metres, kg*m^2).
_Body = tuple[float, np.ndarray, np.ndarray]


class ComponentMass(BaseModel):
    """One entry of ``components:`` in ``mechlint.yaml``, resolved or not.

    A component that cannot be resolved is reported, never dropped: a servo
    silently missing from a link is the difference between a torque number that
    is right and one that is comfortably wrong.

    Resolved ones are treated as **point masses**. Mass and centre of mass are
    then exact; the link's tensor is under-stated by each component's own
    moment about its centre, which for a 40 mm servo on a 200 mm arm is a low
    single-digit percentage. Torque does not use the tensor at all.
    """

    model_config = ConfigDict(frozen=True)

    label: str
    mass_kg: float | None = None
    at_m: Vec3 | None = None
    source: str | None = Field(
        default=None, description="Where the mass came from: mechlint.yaml, or the actuator db."
    )
    ok: bool = True
    error: str | None = None
    hint: str | None = None


class LinkInertia(BaseModel):
    """What one link weighs, where its centre is, and how it resists rotation."""

    model_config = ConfigDict(frozen=True)

    link: str
    mass_kg: float
    com_m: Vec3
    inertia_kg_m2: Mat3
    mass_source: MassSource
    carried: bool = Field(
        default=True,
        description="Whether some joint has to hold this mass up. False for the base and for "
        "anything bolted to the world beside the robot -- a tripod-mounted camera is in the "
        "model because its frame matters, but no actuator carries it.",
    )

    material: str | None = None
    infill: float | None = None
    density_kg_m3: float | None = None
    volume_m3: float | None = None
    shell_mass_kg: float | None = None
    watertight: bool | None = None
    convex_hull_fallback: bool = False

    components: list[ComponentMass] = Field(default_factory=list)
    urdf_mass_kg: float | None = Field(
        default=None, description="What the URDF declares today, for comparison."
    )
    notes: list[str] = Field(default_factory=list)

    @computed_field  # type: ignore[prop-decorator]
    @property
    def writable(self) -> bool:
        """Whether mechlint computed enough to emit an ``<inertial>`` for this link."""
        return self.mass_source in ("measured", "estimated", "components") and self.mass_kg > 0.0


class InertiaReport(BaseModel):
    """One ``mechlint inertia`` run."""

    robot: str
    source: Path
    model_scale: float
    links: list[LinkInertia] = Field(default_factory=list)
    findings: list[Finding] = Field(default_factory=list)
    skipped: dict[str, str] = Field(default_factory=dict)
    overrides: dict[str, object] = Field(
        default_factory=dict,
        description="What this call changed about mechlint.yaml, echoed back. Empty means "
        "the report describes the project as committed.",
    )

    @computed_field  # type: ignore[prop-decorator]
    @property
    def total_mass_kg(self) -> float:
        """Every link added up. Rarely the number you want -- see ``carried_mass_kg``."""
        return sum(link.mass_kg for link in self.links)

    @computed_field  # type: ignore[prop-decorator]
    @property
    def carried_mass_kg(self) -> float:
        """The mass the joints have to hold. This is what torque is computed from."""
        return sum(link.mass_kg for link in self.links if link.carried)

    @computed_field  # type: ignore[prop-decorator]
    @property
    def uncarried_mass_kg(self) -> float:
        """The rest: the base, and anything bolted to the world next to the robot."""
        return sum(link.mass_kg for link in self.links if not link.carried)

    @property
    def uncarried_links(self) -> list[str]:
        return [link.link for link in self.links if not link.carried and link.mass_kg > 0.0]

    @property
    def writable_links(self) -> list[LinkInertia]:
        return [link for link in self.links if link.writable]

    @property
    def pending_components(self) -> list[ComponentMass]:
        return [c for link in self.links for c in link.components if not c.ok]


# --------------------------------------------------------------------------- compute


def compute_inertia(
    model: RobotModel,
    config: MechlintConfig | None = None,
    *,
    materials: MaterialDatabase | None = None,
    actuators: ActuatorDatabase | None = None,
    link_masses_g: Mapping[str, float] | None = None,
    geometry: str = "visual-or-collision",
) -> InertiaReport:
    """Mass, centre of mass and inertia tensor for every link, in the link frame.

    ``link_masses_g`` is the what-if door: "suppose arm_2 came out at 80 g". It
    replaces the *whole* link mass -- shell and components together -- keeping the
    shape, so the centre of mass stays put and the tensor scales with the mass. A
    link overridden this way is deliberately not writable: a hypothetical must
    never reach the generated xacro.
    """
    findings: list[Finding] = []
    overrides = _link_mass_overrides(model, link_masses_g)
    # Which links a joint actually holds up. Derived from the chain, never declared:
    # a link reachable from the root through only fixed joints is bolted to the world,
    # not carried, whatever it is called.
    carried = chain.moving_links(model.urdf)
    database = actuators or bundled_actuators()
    links = [
        _link_inertia(
            model,
            name,
            config,
            materials,
            database,
            geometry,
            findings,
            name in carried,
            overrides.get(name),
        )
        for name in model.link_names
    ]
    return InertiaReport(
        robot=model.name,
        source=model.path,
        model_scale=model.model_scale,
        links=links,
        findings=findings,
        overrides=applied_overrides(link_mass_g=overrides),
    )


def _link_mass_overrides(
    model: RobotModel, link_masses_g: Mapping[str, float] | None
) -> dict[str, float]:
    """Validate the what-if masses. A typo here is refused, not quietly ignored.

    Silently dropping an override for a misspelt link would answer a question
    nobody asked -- and answer it with the unmodified robot, which looks right.
    """
    overrides: dict[str, float] = {}
    for name, grams in (link_masses_g or {}).items():
        if name not in model.link_names:
            known = ", ".join(model.link_names)
            raise ValueError(f"no link called {name!r} in {model.name}; it has: {known}")
        if grams < 0.0:
            raise ValueError(f"link mass for {name!r} cannot be negative, got {grams:g} g")
        overrides[name] = float(grams)
    return overrides


def _link_inertia(
    model: RobotModel,
    name: str,
    config: MechlintConfig | None,
    materials: MaterialDatabase | None,
    actuators: ActuatorDatabase,
    geometry: str,
    findings: list[Finding],
    carried: bool,
    override_mass_g: float | None = None,
) -> LinkInertia:
    spec = _material_spec(config, name)
    material = spec.material or "pla"
    infill = 1.0 if spec.infill is None else spec.infill
    density = density_of(material, database=materials)

    shell = _shell(model, name, density * infill, geometry, findings)
    components = _components(model, name, config, actuators)
    declared = model.declared_inertial(name)
    urdf_mass = None if declared is None or declared.mass is None else float(declared.mass)

    notes: list[str] = []
    if shell is not None and spec.measured_mass_g is not None:
        shell = shell.with_mass(spec.measured_mass_g * 1e-3)
        notes.append("shell mass measured; components below are added on top")

    bodies: list[_Body] = []
    if shell is not None:
        bodies.append((shell.mass_kg, np.asarray(shell.com_m), shell.inertia_array))
    for component in components:
        if component.ok and component.mass_kg is not None and component.at_m is not None:
            bodies.append((component.mass_kg, np.asarray(component.at_m), np.zeros((3, 3))))

    if bodies:
        mass, com, tensor = _combine(bodies)
        source: MassSource = (
            "measured"
            if shell is not None and spec.measured_mass_g is not None
            else "estimated"
            if shell is not None
            else "components"
        )
    elif urdf_mass is not None and urdf_mass > 0.0:
        mass, com, tensor = urdf_mass, np.zeros(3), np.zeros((3, 3))
        source = "urdf"
        notes.append("no geometry and no components: mechlint has nothing to compute from")
    else:
        mass, com, tensor = 0.0, np.zeros(3), np.zeros((3, 3))
        source = "none"

    if override_mass_g is not None:
        wanted = override_mass_g * 1e-3
        # Same shape, different mass: the centre stays where the geometry put it and
        # the tensor scales with the mass. With nothing to scale, the tensor is left
        # at zero rather than invented, and the note says so.
        tensor = tensor * (wanted / mass) if mass > 0.0 else np.zeros((3, 3))
        notes.append(
            f"mass overridden for this call: {wanted * 1e3:g} g instead of "
            f"{mass * 1e3:.1f} g ({source}); not written to the xacro"
            if mass > 0.0
            else f"mass overridden for this call: {wanted * 1e3:g} g, with no geometry to "
            "scale, so the inertia tensor stays zero"
        )
        mass, source = wanted, "override"

    return LinkInertia(
        link=name,
        mass_kg=mass,
        com_m=_vec3(com),
        inertia_kg_m2=_mat3(tensor),
        mass_source=source,
        carried=carried,
        material=material if shell is not None else None,
        infill=infill if shell is not None else None,
        density_kg_m3=density if shell is not None else None,
        volume_m3=None if shell is None else shell.volume_m3,
        shell_mass_kg=None if shell is None else shell.mass_kg,
        watertight=None if shell is None else shell.watertight,
        convex_hull_fallback=bool(shell is not None and shell.convex_hull_fallback),
        components=components,
        urdf_mass_kg=urdf_mass,
        notes=notes,
    )


def _material_spec(config: MechlintConfig | None, link: str) -> MaterialSpec:
    """Per-link material over the project default, field by field."""
    if config is None:
        return MaterialSpec(material="pla", infill=1.0)
    default = config.materials.default
    override = config.materials.links.get(link)
    if override is None:
        return default
    return MaterialSpec(
        material=override.material or default.material,
        infill=default.infill if override.infill is None else override.infill,
        measured_mass_g=override.measured_mass_g,
    )


def _shell(
    model: RobotModel,
    name: str,
    density: float,
    geometry: str,
    findings: list[Finding],
) -> MassProperties | None:
    """The printed body of a link: every geometry it carries, combined."""
    bodies = model.geometries(name, which=geometry)
    if not bodies:
        return None

    parts = [
        mass_properties(body.mesh, scale=model.model_scale, density_kg_m3=density)
        for body in bodies
    ]
    for body, part in zip(bodies, parts, strict=True):
        if part.convex_hull_fallback:
            findings.append(
                finding(
                    "M001",
                    Severity.WARN,
                    f"{name}: {Path(body.source).name}",
                    "not a closed volume; mass properties come from its convex hull",
                )
            )

    mass, com, tensor = _combine([(p.mass_kg, np.asarray(p.com_m), p.inertia_array) for p in parts])
    volume = sum(p.volume_m3 for p in parts)
    return MassProperties(
        source=" + ".join(Path(p.source).name if p.source != "<mesh>" else p.source for p in parts),
        volume_m3=volume,
        mass_kg=mass,
        density_kg_m3=density,
        com_m=_vec3(com),
        inertia_kg_m2=_mat3(tensor),
        extents_m=_vec3(np.max([p.extents_m for p in parts], axis=0)),
        watertight=all(p.watertight for p in parts),
        convex_hull_fallback=any(p.convex_hull_fallback for p in parts),
    )


def _components(
    model: RobotModel,
    name: str,
    config: MechlintConfig | None,
    actuators: ActuatorDatabase,
) -> list[ComponentMass]:
    """Resolve one link's ``components:`` entries to masses at places.

    An explicit ``mass_g`` wins over the database: it is what somebody weighed,
    and a clone servo rarely matches the figure its datasheet claims.
    """
    if config is None:
        return []
    resolved: list[ComponentMass] = []
    for entry in config.components.get(name, []):
        label = entry.actuator or entry.note or "component"
        position = _component_position(model, name, entry)
        if entry.mass_g is not None:
            resolved.append(
                ComponentMass(
                    label=label,
                    mass_kg=entry.mass_g * 1e-3,
                    at_m=position,
                    source="mechlint.yaml",
                )
            )
            continue
        try:
            actuator = actuators[entry.actuator or ""]
        except KeyError as error:
            resolved.append(
                ComponentMass(
                    label=label,
                    at_m=position,
                    ok=False,
                    error=str(error),
                    hint="add it to your own actuator YAML, or give this component a mass_g",
                )
            )
            continue
        resolved.append(
            ComponentMass(
                label=label,
                mass_kg=actuator.mass_kg,
                at_m=position,
                source=f"actuator db: {actuator.name}, {actuator.mass_g:g} g "
                f"({actuator.confidence} confidence)",
            )
        )
    return resolved


def _component_position(model: RobotModel, link: str, entry: object) -> Vec3 | None:
    """Where a component sits in its link frame, in metres.

    ``at`` is given in URDF units, like everything else a user types into a
    description. A component that only says which joint it ``drives`` sits at
    that joint's origin -- which is where a servo physically is.
    """
    at = getattr(entry, "at", None)
    if at is not None:
        return _vec3(np.asarray(at, dtype=float) * model.model_scale)
    drives = getattr(entry, "drives", None)
    if drives is None:
        return None
    joint = model.urdf.joint_map.get(drives)
    if joint is None or joint.origin is None:
        return (0.0, 0.0, 0.0)
    return _vec3(np.asarray(joint.origin, dtype=float)[:3, 3] * model.model_scale)


def _combine(bodies: list[_Body]) -> tuple[float, np.ndarray, np.ndarray]:
    """Total mass, common centre of mass, and the tensor about it (parallel axis)."""
    total = float(sum(mass for mass, _, _ in bodies))
    if total <= 0.0:
        return 0.0, np.zeros(3), np.zeros((3, 3))
    com = sum(mass * centre for mass, centre, _ in bodies) / total
    tensor = np.zeros((3, 3))
    for mass, centre, own in bodies:
        offset = np.asarray(centre, dtype=float) - com
        tensor = tensor + own + mass * (offset @ offset * np.eye(3) - np.outer(offset, offset))
    return total, np.asarray(com, dtype=float), tensor


def _vec3(values: np.ndarray) -> Vec3:
    a = np.asarray(values, dtype=float).reshape(3)
    return (float(a[0]), float(a[1]), float(a[2]))


def _mat3(values: np.ndarray) -> Mat3:
    a = np.asarray(values, dtype=float).reshape(3, 3)
    return (_vec3(a[0]), _vec3(a[1]), _vec3(a[2]))


# --------------------------------------------------------------------------- emit

#: Every generated file starts with this. It is what tells mechlint that a file it is
#: about to overwrite is its own output and not something a human wrote.
GENERATED_MARKER = "Generated by mechlint"

#: What a directory target is filled in with.
DEFAULT_FILENAME = "inertials.xacro"

#: Why a link got no macro. Keyed by ``mass_source``, so the reason is the same
#: sentence wherever it is shown.
_NOT_WRITABLE = {
    "urdf": "mass comes from the URDF itself; mechlint computed nothing to write back",
    "none": "no geometry and no components: nothing to compute from",
    "override": "mass was overridden for this call; a hypothetical is not written",
}


class WriteRefused(Exception):
    """A write mechlint will not do, with the thing to do instead."""

    def __init__(self, message: str, hint: str = "") -> None:
        super().__init__(message)
        self.message = message
        self.hint = hint


class WriteResult(BaseModel):
    """What ``write_inertials`` did, in enough detail to be reviewed."""

    model_config = ConfigDict(frozen=True)

    path: Path
    created: bool = Field(description="False when an existing generated file was replaced.")
    unchanged: bool = Field(
        default=False,
        description="The file already said exactly this. Re-running is a no-op, so a host "
        "may repeat the call without touching the working tree.",
    )
    links: list[str] = Field(default_factory=list, description="Links that got a macro.")
    skipped: dict[str, str] = Field(
        default_factory=dict, description="Links that did not, and why. Not an error."
    )
    bytes_written: int = 0

    @computed_field  # type: ignore[prop-decorator]
    @property
    def include(self) -> str:
        """The line to paste into the description, which is the next thing to do."""
        return f'<xacro:include filename="{self.path.name}"/>'


def target_path(path: Path) -> Path:
    """Where the xacro goes: a directory means ``inertials.xacro`` inside it."""
    path = Path(path)
    return path / DEFAULT_FILENAME if path.is_dir() else path


def write_inertials(report: InertiaReport, path: Path, *, force: bool = False) -> WriteResult:
    """Write the generated xacro, and refuse in the two cases where writing is wrong.

    mechlint overwrites exactly one kind of file: one it generated itself, marked
    as such in its header. Anything else is the user's, and needs ``force``. A
    report carrying per-call overrides is refused outright -- a what-if answer that
    ends up in the robot description is worse than no answer.
    """
    if report.overrides:
        names = ", ".join(sorted(report.overrides))
        raise WriteRefused(
            f"this report was computed with overrides ({names}), so it describes a "
            "hypothetical robot, not the one in the description",
            hint="re-run without the override, then write.",
        )

    target = target_path(path)
    existing = target.read_text() if target.is_file() else None
    if existing is not None and not force and GENERATED_MARKER not in existing:
        raise WriteRefused(
            f"{target} was not generated by mechlint",
            hint="mechlint only overwrites its own output. Pass force, or write elsewhere.",
        )

    content = inertials_xacro(report)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(content)
    written = [link.link for link in report.writable_links]
    return WriteResult(
        path=target,
        created=existing is None,
        unchanged=existing == content,
        links=written,
        skipped={
            link.link: _NOT_WRITABLE.get(link.mass_source, "nothing computed")
            for link in report.links
            if not link.writable
        },
        bytes_written=len(content.encode()),
    )


def inertials_xacro(report: InertiaReport) -> str:
    """One xacro macro per link, ready to be included by the description.

    Macros rather than a drop-in replacement, because mechlint never edits the
    file you wrote: you swap ``<xacro:default_inertial mass="0.1"/>`` for
    ``<xacro:mechlint_inertial_arm_1_link/>`` link by link, and can see in the
    diff exactly which links you accepted.
    """
    scale = report.model_scale
    unit = "metre" if scale == 1.0 else f"{scale:g} m"
    generated = datetime.now(timezone.utc).strftime("%Y-%m-%d")

    lines = [
        '<?xml version="1.0"?>',
        f"<!-- Generated by mechlint {__version__} on {generated} from",
        f"     {escape(str(report.source))}",
        "     DO NOT EDIT: re-run `mechlint inertia` instead.",
        "",
        f"     Lengths below are in URDF units (1 unit = {unit}), matching the rest of the",
        "     description, so mass is kg and inertia is kg*unit^2. Include this file and",
        "     replace a link's existing <inertial> with its macro, one link at a time:",
        "",
        '       <xacro:include filename="inertials.xacro"/>',
        "       ...",
        "       <xacro:mechlint_inertial_base_link/>",
        "",
        "     Masses marked estimated assume the material and infill in mechlint.yaml.",
        "     Weigh the part and set measured_mass_g when you want the real number.",
        "-->",
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">',
    ]

    for link in report.writable_links:
        xyz = np.asarray(link.com_m) / scale
        tensor = np.asarray(link.inertia_kg_m2) / scale**2
        note = f"{link.mass_source}"
        if link.convex_hull_fallback:
            note += ", convex hull (M001)"
        lines += [
            "",
            f"  <!-- {escape(link.link)}: {link.mass_kg * 1e3:.1f} g, {note} -->",
            f'  <xacro:macro name="mechlint_inertial_{link.link}">',
            "    <inertial>",
            f'      <origin xyz="{_xyz(xyz)}" rpy="0 0 0"/>',
            f'      <mass value="{link.mass_kg:.6g}"/>',
            "      <inertia "
            f'ixx="{tensor[0][0]:.6g}" ixy="{tensor[0][1]:.6g}" ixz="{tensor[0][2]:.6g}" '
            f'iyy="{tensor[1][1]:.6g}" iyz="{tensor[1][2]:.6g}" izz="{tensor[2][2]:.6g}"/>',
            "    </inertial>",
            "  </xacro:macro>",
        ]

    lines += ["", "</robot>", ""]
    return "\n".join(lines)


def _xyz(values: np.ndarray) -> str:
    return " ".join(f"{v:.6g}" for v in np.asarray(values).reshape(3))
