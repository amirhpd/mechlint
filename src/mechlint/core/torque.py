"""Worst-case static joint torque, and whether the assigned actuator can hold it.

This is the question mechlint exists for. Every other command is groundwork: the
chain says what is downstream of a joint, ``inertia`` says what it weighs, the
actuator database says what the motor delivers, and this module puts the three
together into one number per joint -- with the pose it happens at, so the answer
is checkable on the bench rather than only on screen.

For joint *i* with axis **a** through point **p**, and every link *j* in the
subtree below it at centre **c**_j with mass *m*_j::

    tau_i(q) = a . SUM_j ( (c_j - p) x m_j g )  +  a . ( (c_tip - p) x m_payload g )

Static only: gravity holding, nothing accelerating. That understates a moving
robot, which is exactly why every report prints the safety factor it applied.
"""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path

import numpy as np
from pydantic import BaseModel, ConfigDict, Field, computed_field

from mechlint.core import chain
from mechlint.core.actuators import Actuator, ActuatorDatabase, bundled
from mechlint.core.checks import Finding, Severity, finding
from mechlint.core.config import (
    JointDrive,
    MechlintConfig,
    Transmission,
    applied_overrides,
    joint_torque_basis,
    joint_torque_limit,
)
from mechlint.core.geometry import Vec3
from mechlint.core.inertia import InertiaReport, compute_inertia
from mechlint.core.materials import Confidence
from mechlint.core.urdf import RobotModel


def _or_unbounded(margin: float | None) -> float:
    """``None`` means an unbounded margin, which sorts last -- not first."""
    return float("inf") if margin is None else margin


#: Poses searched by default. The joint-limit corners are always in there, so this
#: number only controls how finely the interior is filled; it is cheap to raise.
DEFAULT_SAMPLES = 2048


class TorqueCase(BaseModel):
    """One joint, one payload: the worst pose found and what it demands."""

    model_config = ConfigDict(frozen=True)

    payload_kg: float
    max_torque_Nm: float
    pose: dict[str, float] = Field(
        default_factory=dict, description="The configuration the maximum occurs at, in rad or m."
    )
    required_Nm: float | None = Field(
        default=None,
        description="max_torque_Nm * safety_factor: what must be available at the joint.",
    )
    motor_required_Nm: float | None = Field(
        default=None,
        description="required_Nm divided by the joint's transmission (ratio x efficiency): "
        "what a replacement actuator must deliver at its own shaft. Equal to required_Nm when "
        "the joint has no transmission. This, not required_Nm, is what to search the "
        "actuator database with.",
    )
    available_Nm: float | None = None
    margin: float | None = Field(
        default=None,
        description="available / required. Below 1.0 is a fail. Null with ok=true means an "
        "unbounded margin: gravity applies no torque about this axis at any pose, so any "
        "actuator holds it. Null with ok=null means no actuator was assigned -- read `ok` "
        "to tell the two apart.",
    )
    ok: bool | None = Field(
        default=None, description="None when no actuator is assigned: reported, not judged."
    )


class JointTorque(BaseModel):
    """Everything the torque budget knows about one joint."""

    model_config = ConfigDict(frozen=True)

    joint: str
    type: str
    axis: Vec3
    downstream_links: list[str] = Field(default_factory=list)
    downstream_mass_kg: float = 0.0

    actuator: str | None = None
    actuator_name: str | None = None
    torque_basis: str | None = Field(
        default=None,
        description="What the available torque is, e.g. 'stall torque at 6 V', or with a "
        "transmission 'stall torque at 5 V (0.059 N*m) x 2:1 stage x 0.95 efficiency'.",
    )
    supply_V: float | None = Field(
        default=None,
        description="The supply this joint is judged at: a per-call override, else its own "
        "voltage in mechlint.yaml, else actuators.voltage. Null means none was stated and the "
        "actuator's recommended voltage was used.",
    )
    voltage_V: float | None = Field(
        default=None,
        description="The datasheet row the torque was read from. Differs from supply_V when "
        "the supply is not a listed voltage; the notes then say which way that errs.",
    )
    motor_torque_Nm: float | None = Field(
        default=None,
        description="The actuator's own figure at its shaft, before any transmission.",
    )
    transmission: Transmission | None = Field(
        default=None, description="The gear stage between actuator and joint, if any."
    )
    confidence: Confidence | None = Field(
        default=None, description="Of the datasheet, not of the arithmetic."
    )
    source_url: str | None = None
    effort_limit_Nm: float | None = Field(
        default=None, description="What the URDF's <limit effort> claims. See U007."
    )

    cases: list[TorqueCase] = Field(default_factory=list)
    notes: list[str] = Field(default_factory=list)

    @computed_field  # type: ignore[prop-decorator]
    @property
    def judged(self) -> bool:
        return any(case.ok is not None for case in self.cases)

    @computed_field  # type: ignore[prop-decorator]
    @property
    def ok(self) -> bool | None:
        """Whether this joint holds at every payload asked for."""
        verdicts = [case.ok for case in self.cases if case.ok is not None]
        return all(verdicts) if verdicts else None

    @property
    def worst(self) -> TorqueCase | None:
        return max(self.cases, key=lambda case: case.max_torque_Nm) if self.cases else None


class TorqueReport(BaseModel):
    """One ``mechlint torque`` run."""

    robot: str
    source: Path
    model_scale: float

    gravity_m_s2: Vec3
    gravity_source: str = Field(description="'model' or 'config' -- who decided which way is down.")
    implied_mount: str | None = None
    safety_factor: float
    voltage_V: float | None = Field(
        default=None,
        description="The supply every joint is judged at. Null when joints are on different "
        "rails: read each joint's own supply_V then.",
    )
    payloads_g: list[float] = Field(default_factory=list)
    payload_at: str | None = Field(default=None, description="The link a payload hangs from.")
    samples: int = 0
    overrides: dict[str, object] = Field(
        default_factory=dict,
        description="What this call changed about mechlint.yaml, echoed back, so a what-if "
        "answer carries its own assumptions. Empty means the project as committed.",
    )

    joints: list[JointTorque] = Field(default_factory=list)
    unjudged: list[str] = Field(
        default_factory=list,
        description="Joints whose torque is reported but not judged: no actuator assigned.",
    )
    findings: list[Finding] = Field(default_factory=list)
    skipped: dict[str, str] = Field(default_factory=dict)

    @computed_field  # type: ignore[prop-decorator]
    @property
    def ok(self) -> bool:
        return not any(f.severity is Severity.FAIL for f in self.findings)

    @property
    def worst_joint(self) -> JointTorque | None:
        """The joint with the least margin -- the one that decides the motor choice."""
        judged = [j for j in self.joints if j.judged]
        if judged:
            return min(judged, key=lambda j: min(_or_unbounded(c.margin) for c in j.cases))
        return max(
            self.joints, key=lambda j: j.worst.max_torque_Nm if j.worst else 0.0, default=None
        )


# --------------------------------------------------------------------------- compute


def torque_budget(
    model: RobotModel,
    config: MechlintConfig | None = None,
    *,
    inertia: InertiaReport | None = None,
    actuators: ActuatorDatabase | None = None,
    payloads_g: list[float] | None = None,
    voltage: float | Mapping[str, float] | None = None,
    assignments: Mapping[str, str | None] | None = None,
    mount: object | None = None,
    payload_at: str | None = None,
    link_masses_g: Mapping[str, float] | None = None,
    samples: int = DEFAULT_SAMPLES,
) -> TorqueReport:
    """Worst-case static torque per joint, and the margin against its actuator.

    Every keyword is an override of what ``mechlint.yaml`` says, so an LLM can ask
    "what if the payload were 200 g?" without editing the file -- and the report
    echoes the values it used, so the answer carries its own assumptions.

    ``voltage`` is either one number -- every joint on that supply -- or
    ``{joint: volts}`` for only the joints named. ``assignments`` swaps the actuator
    and keeps the joint's own supply and transmission: the gear stage is part of
    the arm, not of the motor.
    """
    if inertia is not None and link_masses_g:
        raise ValueError(
            "link_masses_g cannot be applied to an inertia report that was already computed; "
            "pass one or the other"
        )
    database = actuators or bundled()
    scenario = config.scenario if config is not None else None
    urdf = model.urdf

    default_supply = config.actuators.voltage if config is not None else None
    factor = scenario.safety_factor if scenario is not None else 2.0
    drives = _drives(model, config, assignments, voltage)

    loads = _payloads(payloads_g, scenario)
    # An explicit mount overrides even the model: "what if it were on the ceiling?" is a
    # question about a robot that is not the one in the URDF, and answering it is the point.
    down = chain.gravity_in_root(
        urdf,
        mount if mount is not None else _mount(scenario),
        override=mount is not None,
    )
    report = (
        inertia
        if inertia is not None
        else compute_inertia(model, config, actuators=database, link_masses_g=link_masses_g)
    )
    bodies = {link.link: (link.mass_kg, np.asarray(link.com_m)) for link in report.links}

    joints = chain.dof_joints(urdf)
    poses = chain.forward_kinematics(
        urdf, chain.sample_configurations(urdf, samples), joints=joints
    )
    count = len(next(iter(poses.values())))

    hand = _payload_link(model, payload_at) or chain.tip(urdf)
    tip_centre = _origin(poses, hand, model.model_scale) if hand else None

    findings: list[Finding] = []
    results = [
        _joint_torque(
            model,
            name,
            poses,
            bodies,
            down.array,
            tip_centre,
            loads,
            factor,
            drives.get(name),
            database,
            findings,
        )
        for name in joints
    ]
    # A servo the config places on no side of its joint is not in any link's mass, so
    # every torque below is light by its weight. The inertia report found it; say it here too.
    findings.extend(f for f in report.findings if f.check == "D004")

    return TorqueReport(
        robot=model.name,
        source=model.path,
        model_scale=model.model_scale,
        gravity_m_s2=down.vector,
        gravity_source=down.source,
        implied_mount=down.implied_mount,
        safety_factor=factor,
        voltage_V=_common_supply(results, voltage, default_supply),
        payloads_g=[load * 1e3 for load in loads],
        payload_at=hand,
        samples=count,
        overrides=applied_overrides(
            payload_g=payloads_g,
            voltage_V=dict(voltage) if isinstance(voltage, Mapping) else voltage,
            joint_actuators=dict(assignments or {}),
            mount=mount,
            payload_at=payload_at,
            samples=samples if samples != DEFAULT_SAMPLES else None,
            **report.overrides,
        ),
        joints=results,
        unjudged=[j.joint for j in results if not j.judged],
        findings=findings,
        skipped={"D001": "drift arrives in M4"},
    )


def _drives(
    model: RobotModel,
    config: MechlintConfig | None,
    assignments: Mapping[str, str | None] | None,
    voltage: float | Mapping[str, float] | None,
) -> dict[str, JointDrive | None]:
    """Every joint's actuator, supply and transmission, with this call's overrides applied.

    A voltage override naming a joint that does not exist is refused rather than
    ignored: answering "joint_7 at 12 V" with the unchanged robot would look right.
    """
    default = config.actuators.voltage if config is not None else None
    drives = config.actuators.drives() if config is not None else {}

    for joint, key in (assignments or {}).items():
        current = drives.get(joint)
        if not key:
            drives[joint] = None
        elif current is None:
            drives[joint] = JointDrive(actuator=key, voltage=default)
        else:
            drives[joint] = current.model_copy(update={"actuator": key})

    if isinstance(voltage, Mapping):
        for joint, volts in voltage.items():
            if joint not in model.urdf.joint_map:
                known = ", ".join(chain.dof_joints(model.urdf))
                raise ValueError(f"no joint called {joint!r} to set a voltage for; it has: {known}")
            if float(volts) <= 0.0:
                raise ValueError(f"voltage for {joint!r} must be positive, got {volts:g} V")
            current = drives.get(joint)
            if current is not None:
                drives[joint] = current.model_copy(update={"voltage": float(volts)})
    elif voltage is not None:
        if float(voltage) <= 0.0:
            raise ValueError(f"voltage must be positive, got {voltage:g} V")
        drives = {
            joint: None if drive is None else drive.model_copy(update={"voltage": float(voltage)})
            for joint, drive in drives.items()
        }
    return drives


def _common_supply(
    results: list[JointTorque],
    voltage: float | Mapping[str, float] | None,
    default: float | None,
) -> float | None:
    """The one supply the whole report is at, or None when the joints disagree."""
    if voltage is not None and not isinstance(voltage, Mapping):
        return float(voltage)
    supplies = {j.supply_V for j in results if j.actuator is not None}
    if len(supplies) > 1:
        return None
    return supplies.pop() if supplies else default


def _payloads(payloads_g: list[float] | None, scenario: object) -> list[float]:
    """The load cases, in kg. Zero is always one of them: it is the floor.

    A joint that cannot hold the arm up empty will not hold anything, so the
    no-payload case belongs in every report even when nobody asked for it.
    """
    asked = payloads_g if payloads_g is not None else [getattr(scenario, "payload_g", 0.0) or 0.0]
    values = sorted({0.0} | {round(float(grams), 6) for grams in asked})
    return [value * 1e-3 for value in values]


def _payload_link(model: RobotModel, payload_at: str | None) -> str | None:
    """The link a payload hangs from, checked before it becomes a KeyError deep in the FK.

    Left unset, the chain decides: the deepest moving leaf is where a gripper is. Set
    to a link that does not exist, the answer would silently be about a different arm.
    """
    if payload_at is None:
        return None
    if payload_at not in model.link_names:
        known = ", ".join(model.link_names)
        raise ValueError(f"no link called {payload_at!r} to hang a payload from; it has: {known}")
    return payload_at


def _mount(scenario: object) -> object:
    return getattr(scenario, "mount", "table")


def _origin(poses: dict[str, np.ndarray], link: str, scale: float) -> np.ndarray:
    """A link's origin in the root frame, in metres, for every sampled pose."""
    return poses[link][:, :3, 3] * scale


def _joint_torque(
    model: RobotModel,
    name: str,
    poses: dict[str, np.ndarray],
    bodies: dict[str, tuple[float, np.ndarray]],
    gravity: np.ndarray,
    tip_centre: np.ndarray | None,
    loads: list[float],
    factor: float,
    drive: JointDrive | None,
    database: ActuatorDatabase,
    findings: list[Finding],
) -> JointTorque:
    urdf = model.urdf
    joint = urdf.joint_map[name]
    scale = model.model_scale

    transform = poses[joint.child]
    axis = np.asarray(joint.axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    axis_root = transform[:, :3, :3] @ axis
    pivot = transform[:, :3, 3] * scale

    downstream = sorted(chain.subtree(urdf, joint.child))
    # The moment gravity applies about the pivot, once, for all sampled poses. Split
    # from the payload term because only the payload changes between load cases: the
    # links weigh the same whatever is in the gripper.
    moment = np.zeros_like(pivot)
    held = 0.0
    for link in downstream:
        mass, com = bodies.get(link, (0.0, np.zeros(3)))
        if mass <= 0.0:
            continue
        held += mass
        centre = poses[link][:, :3, :3] @ com + poses[link][:, :3, 3] * scale
        moment += np.cross(centre - pivot, mass * gravity)
    lever = np.cross(tip_centre - pivot, gravity) if tip_centre is not None else None

    supply = drive.voltage if drive is not None else None
    stage = drive.transmission if drive is not None else None
    actuator, reason = _actuator(
        database, drive.actuator if drive is not None else None, findings, name
    )
    limit = actuator.torque_limit(supply) if actuator is not None else None
    available = joint_torque_limit(limit.torque_Nm, stage) if limit is not None else None
    basis = None if limit is None else joint_torque_basis(limit.basis, limit.torque_Nm, stage)
    effort = _effort(joint)

    notes: list[str] = []
    if reason is not None:
        notes.append(reason)
    if actuator is not None:
        if limit is not None and limit.note:
            notes.append(limit.note)
        outside = actuator.voltage_note(supply)
        if outside:
            notes.append(outside)
    if tip_centre is None and any(load > 0.0 for load in loads):
        notes.append("no tip link found: payload cases carry no payload")

    cases = []
    for load in loads:
        total = moment if lever is None else moment + load * lever
        magnitude = np.abs(np.einsum("ij,ij->i", axis_root, total))
        peak = int(np.argmax(magnitude))
        worst = float(magnitude[peak])
        required = worst * factor
        cases.append(
            TorqueCase(
                payload_kg=load,
                max_torque_Nm=worst,
                pose=_pose(urdf, poses, peak, scale),
                required_Nm=required,
                motor_required_Nm=required / stage.factor if stage is not None else required,
                available_Nm=available,
                margin=None
                if available is None
                else available / required
                if required > 0
                else None,
                ok=None if available is None else available >= required,
            )
        )

    if all(case.max_torque_Nm < 1e-12 for case in cases):
        notes.append(
            "axis is parallel to gravity at every sampled pose, so static torque is zero: "
            "this joint is sized by friction and acceleration, which mechlint does not model"
        )

    result = JointTorque(
        joint=name,
        type=joint.type,
        axis=(float(axis[0]), float(axis[1]), float(axis[2])),
        downstream_links=downstream,
        downstream_mass_kg=held,
        actuator=None if actuator is None else actuator.key,
        actuator_name=None if actuator is None else actuator.name,
        torque_basis=basis,
        supply_V=None if actuator is None else supply,
        voltage_V=None if limit is None else limit.voltage_V,
        motor_torque_Nm=None if limit is None else limit.torque_Nm,
        transmission=stage if actuator is not None else None,
        confidence=None if actuator is None else actuator.confidence,
        source_url=None if actuator is None else actuator.source_url,
        effort_limit_Nm=effort,
        cases=cases,
        notes=notes,
    )
    findings.extend(_t001(result))
    return result


def _actuator(
    database: ActuatorDatabase, key: str | None, findings: list[Finding], joint: str
) -> tuple[Actuator | None, str | None]:
    """The joint's actuator, plus a note when a name was given and did not resolve.

    "No servo assigned" and "the servo you named is not in the database" both end
    with an unjudged joint, and a report that showed them the same way would let a
    typo read as a deliberate ``null``. The first needs no note -- the report already
    lists every unjudged joint -- but the second is a mistake, and says so.
    """
    if not key:
        return None, None
    try:
        return database[key], None
    except KeyError as error:
        findings.append(finding("T001", Severity.FAIL, joint, str(error)))
        return None, f"actuator {key!r} is not in the database, so this joint is not judged"


def _effort(joint: object) -> float | None:
    limit = getattr(joint, "limit", None)
    effort = None if limit is None else getattr(limit, "effort", None)
    return None if effort is None else float(effort)


def _pose(urdf: object, poses: dict[str, np.ndarray], index: int, scale: float) -> dict[str, float]:
    """Recover the joint values of one sampled pose, for the report.

    Read back out of the transforms rather than carried alongside them, so what
    is printed is provably the pose the maximum was computed from.
    """
    values = {}
    for name in chain.dof_joints(urdf):  # type: ignore[arg-type]
        joint = urdf.joint_map[name]  # type: ignore[attr-defined]
        origin = np.eye(4) if joint.origin is None else np.asarray(joint.origin, dtype=float)
        parent = poses[joint.parent][index]
        local = np.linalg.inv(parent @ origin) @ poses[joint.child][index]
        axis = np.asarray(joint.axis, dtype=float)
        axis = axis / np.linalg.norm(axis)
        if joint.type == "prismatic":
            values[name] = float(local[:3, 3] @ axis * scale)
        else:
            rotation = local[:3, :3]
            values[name] = float(
                np.arctan2(
                    np.array(
                        [
                            rotation[2, 1] - rotation[1, 2],
                            rotation[0, 2] - rotation[2, 0],
                            rotation[1, 0] - rotation[0, 1],
                        ]
                    )
                    @ axis
                    / 2.0,
                    (np.trace(rotation) - 1.0) / 2.0,
                )
            )
    return values


def _t001(result: JointTorque) -> list[Finding]:
    """One finding per joint, at the lightest payload it already fails under.

    The lightest, not the heaviest: a joint that cannot hold the arm up empty is a
    different problem from one that runs out at 200 g, and the report should say
    which of the two it is without the reader comparing columns.
    """
    failing = [case for case in result.cases if case.ok is False]
    if not failing:
        return []
    case = min(failing, key=lambda c: c.payload_kg)
    short = (case.required_Nm or 0.0) / (case.available_Nm or 1.0)
    payload = "no payload" if case.payload_kg == 0.0 else f"{case.payload_kg * 1e3:g} g payload"
    return [
        finding(
            "T001",
            Severity.FAIL,
            result.joint,
            f"needs {case.max_torque_Nm:.3g} N*m at {payload}, but {result.actuator_name} "
            f"gives {case.available_Nm:.3g} N*m ({result.torque_basis}, "
            f"{result.confidence} confidence) -- short by {short:.3g}x at safety factor "
            f"{(case.required_Nm or 0.0) / case.max_torque_Nm:g}"
            + (
                ""
                if result.transmission is None
                else f"; a replacement must give {case.motor_required_Nm:.3g} N*m at its own "
                f"shaft, before the {result.transmission.ratio:g}:1 stage"
            ),
        )
    ]
