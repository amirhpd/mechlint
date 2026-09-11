"""mechlint's tools over MCP: the same functions the CLI calls, as JSON.

Six tools, in the order ``docs/llms.md`` tells a host to call them:
``inspect_robot`` to see the chain, ``check_urdf`` to find out whether its
numbers can be trusted, ``compute_inertia`` to get real ones, ``torque_budget``
to find out whether the motors hold, and ``list_actuators`` to find one that
would. Five of the six are read-only and say so; ``write_inertials`` is the only
one that touches a file, and the only file it touches is one mechlint generated.

Every what-if -- a heavier payload, another servo, a different supply, a link
that came out lighter than the model says -- is an argument, never a file edit,
and every result echoes the overrides it was given so the answer carries its own
assumptions.

Run it with ``mechlint-mcp``; the README has the ``.mcp.json`` that points a
host at it.
"""

from __future__ import annotations

from pathlib import Path
from typing import Annotated, Any

from pydantic import Field

from mechlint import __version__
from mechlint.core import units
from mechlint.core.actuators import load_database
from mechlint.core.checks import check_urdf as run_check_urdf
from mechlint.core.config import MechlintConfig
from mechlint.core.inertia import DEFAULT_FILENAME
from mechlint.core.inertia import compute_inertia as run_compute_inertia
from mechlint.core.inertia import write_inertials as run_write_inertials
from mechlint.core.torque import DEFAULT_SAMPLES
from mechlint.core.torque import torque_budget as run_torque_budget
from mechlint.core.urdf import DescriptionError, PackageResolver, RobotModel

try:
    from mcp.server.mcpserver import MCPServer
    from mcp.types import ToolAnnotations
except ImportError as error:  # pragma: no cover - depends on how mechlint was installed
    # ModuleNotFoundError and not a bare ImportError, because that is what it is:
    # tools that probe for an optional import (pytest.importorskip among them) treat
    # the two differently, and only this one means "not installed" rather than "broken".
    raise ModuleNotFoundError(
        "the MCP server needs the optional dependency: pip install 'mechlint[mcp]'",
        name="mcp",
    ) from error

INSTRUCTIONS = """
mechlint computes the mechanics of a robot from its CAD and URDF. Every number
you report about mass, inertia or scale must come from one of these tools, not
from arithmetic of your own and not from a picture.

Call inspect_robot first, then check_urdf. Inertia computed from a model that
check_urdf flags with U005 or U006 is arithmetic on placeholders: say so before
quoting it. Quote check IDs (U001, M001) rather than describing findings in
prose; docs/checks.md explains each one.

Torque answers "does this servo hold this joint?". Never answer it from the
inertia table by hand -- torque_budget searches thousands of poses for the worst
one, and the pose is most of the answer. It is static: gravity holding, nothing
accelerating, which is why it applies a safety factor and prints it. Report the
margin and the actuator's confidence together; a 1.2x margin against a `low`
confidence datasheet is not a pass. When a joint fails, call list_actuators with
min_torque_Nm set to the required torque to say what would fit instead.

Answer what-if questions with arguments, never by editing mechlint.yaml: payload_g,
voltage, joint_actuators, actuator_db, mount, payload_at and link_mass_g all override
the file for one call and come back in the result's `overrides`. Quote them alongside
the numbers. What the user tells you that is not a hypothesis -- "joint_2 is an MG996R
at 6 V" -- is a fact about their robot, and belongs in mechlint.yaml: offer to write it
there.

write_inertials is the only tool that writes. It writes one generated file that the
description includes, never the description itself, and refuses a report computed
with overrides. Ask before calling it.
""".strip()

server: MCPServer = MCPServer(
    name="mechlint",
    version=__version__,
    instructions=INSTRUCTIONS,
)

#: Set on every tool here. The wire form is readOnlyHint/openWorldHint; mcp 2.x takes
#: the snake_case names in Python. "Not an open world" means the answers come from
#: files on this machine, so a host may cache and repeat a call without side effects.
READ_ONLY = ToolAnnotations(read_only_hint=True, open_world_hint=False)

Config = Annotated[
    str | None,
    Field(description="Path to mechlint.yaml. Everything else defaults to what it says."),
]
Description = Annotated[
    str | None,
    Field(description="URDF or xacro to read, if not the one the config names."),
]
ModelScale = Annotated[
    float | str | None,
    Field(description="Metres per URDF unit, or a unit name such as 'dm'. Overrides the config."),
]
PackagePaths = Annotated[
    dict[str, str] | None,
    Field(description="{package: directory}, to resolve package:// without a ROS shell."),
]
XacroArgs = Annotated[dict[str, str] | None, Field(description="{name: value} passed to xacro.")]
ActuatorDb = Annotated[
    str | None,
    Field(
        description="Extra actuator YAML, merged over the bundled table. Overrides the "
        "config's own actuator_db for this call."
    ),
]
LinkMasses = Annotated[
    dict[str, float] | None,
    Field(
        description="{link: grams} for this call only: suppose a link weighed this instead. "
        "Replaces the whole link mass, shell and components together."
    ),
]


def _load(
    config: str | None,
    description: str | None,
    model_scale: float | str | None,
    package_paths: dict[str, str] | None,
    xacro_args: dict[str, str] | None,
) -> tuple[MechlintConfig | None, RobotModel]:
    """The same precedence the CLI uses: call argument, then config, then default."""
    project = MechlintConfig.from_yaml(config) if config else None

    if description is None:
        if project is None:
            raise ValueError("give either 'config' (a mechlint.yaml) or 'description' (a URDF)")
        source: Path = project.description_path
    else:
        source = Path(description)

    packages: dict[str, Path] = {}
    if project is not None:
        packages = {
            name: project.resolve(path) for name, path in project.robot.package_paths.items()
        }
    packages.update({name: Path(path) for name, path in (package_paths or {}).items()})

    arguments = dict(project.robot.xacro_args) if project is not None else {}
    arguments.update(xacro_args or {})

    raw = (
        model_scale
        if model_scale is not None
        else (project.robot.model_scale if project is not None else 1.0)
    )
    return project, RobotModel.load(
        source,
        xacro_args=arguments,
        resolver=PackageResolver(packages),
        model_scale=units.resolve_scale(raw),
    )


def _database(project: MechlintConfig | None, actuator_db: str | None) -> Any:
    """The bundled table, extended by the project's own YAML, then by this call's.

    Every tool that judges a motor goes through here, so the CLI and the chat see the
    same servos: a project that ships its own YAML must not be told, by one door out
    of two, that its own servo is unknown.
    """
    if actuator_db is not None:
        return load_database(Path(actuator_db))
    return load_database(project.actuator_db_path if project else None)


def _soft(call: Any) -> dict[str, Any]:
    """A failure is a result, not a traceback: ``{ok: false, error, hint}``."""
    try:
        return {"ok": True, **call()}
    except DescriptionError as error:
        return {"ok": False, "error": error.message, "hint": error.hint}
    except Exception as error:
        # Anything carrying its own hint (WriteRefused, and DescriptionError above)
        # has one worth passing on: a model can act on "write elsewhere", not on a
        # bare exception string.
        return {"ok": False, "error": str(error), "hint": getattr(error, "hint", "")}


@server.tool(annotations=READ_ONLY)
def inspect_robot(
    config: Config = None,
    description: Description = None,
    model_scale: ModelScale = None,
    package_paths: PackagePaths = None,
    xacro_args: XacroArgs = None,
) -> dict[str, Any]:
    """List a robot's links and joints, its reach in metres, and the scale it is modelled in.

    Read this before anything else: it says which links carry geometry, which
    already declare a mass, and how far the arm reaches. It computes no physics.
    """

    def run() -> dict[str, Any]:
        _, model = _load(config, description, model_scale, package_paths, xacro_args)
        return model.summary().model_dump(mode="json")

    return _soft(run)


@server.tool(annotations=READ_ONLY)
def check_urdf(
    config: Config = None,
    description: Description = None,
    actuator_db: ActuatorDb = None,
    model_scale: ModelScale = None,
    package_paths: PackagePaths = None,
    xacro_args: XacroArgs = None,
) -> dict[str, Any]:
    """Check a URDF's physical sanity: units, inertia tensors, masses, meshes (U0xx, M0xx).

    Returns one finding per problem, each with a stable check ID and a fix.
    ``skipped`` lists the checks that need a milestone which has not landed --
    they are not passes. Run this before quoting any number from the URDF.
    """

    def run() -> dict[str, Any]:
        project, model = _load(config, description, model_scale, package_paths, xacro_args)
        database = _database(project, actuator_db)
        return run_check_urdf(model, project, actuators=database).model_dump(mode="json")

    return _soft(run)


@server.tool(annotations=READ_ONLY)
def compute_inertia(
    config: Config = None,
    description: Description = None,
    link_mass_g: LinkMasses = None,
    actuator_db: ActuatorDb = None,
    model_scale: ModelScale = None,
    package_paths: PackagePaths = None,
    xacro_args: XacroArgs = None,
) -> dict[str, Any]:
    """Mass, centre of mass and inertia tensor per link, in SI, from the meshes and mechlint.yaml.

    Each link says whether its mass was ``measured``, ``estimated`` from the
    mesh, or simply taken from the URDF. ``pending_components`` lists masses
    mechlint could not resolve -- a servo missing from that list is a torque
    number that will be wrong later. ``link_mass_g`` is the what-if door -- "suppose
    arm_2 came out at 80 g" -- and comes back in ``overrides``. This tool writes
    nothing: ``write_inertials`` is what turns the same numbers into a file.
    """

    def run() -> dict[str, Any]:
        project, model = _load(config, description, model_scale, package_paths, xacro_args)
        report = run_compute_inertia(
            model,
            project,
            actuators=_database(project, actuator_db),
            link_masses_g=link_mass_g,
        )
        return report.model_dump(mode="json")

    return _soft(run)


@server.tool(annotations=READ_ONLY)
def torque_budget(
    config: Config = None,
    description: Description = None,
    payload_g: Annotated[
        list[float] | None,
        Field(description="Payloads at the tip, in grams. One result column per value."),
    ] = None,
    voltage: Annotated[
        float | None, Field(description="Supply volts. Overrides the config.")
    ] = None,
    joint_actuators: Annotated[
        dict[str, str] | None,
        Field(description="{joint: actuator key} for this call only, to try a different servo."),
    ] = None,
    mount: Annotated[
        str | list[float] | None,
        Field(description="'table', 'ceiling' or [gx, gy, gz]. Overrides the model itself."),
    ] = None,
    payload_at: Annotated[
        str | None,
        Field(
            description="Link the payload hangs from. Default: the deepest moving link, "
            "which is where a gripper is."
        ),
    ] = None,
    link_mass_g: LinkMasses = None,
    actuator_db: ActuatorDb = None,
    samples: Annotated[int | None, Field(description="Poses to search. Default 2048.")] = None,
    model_scale: ModelScale = None,
    package_paths: PackagePaths = None,
    xacro_args: XacroArgs = None,
) -> dict[str, Any]:
    """Worst-case static joint torque, and whether the assigned actuator holds it (T001).

    For each joint: the largest gravity torque over the sampled joint space, the
    pose it happens at, the actuator's stall torque at the supply voltage, and the
    margin against the safety factor. Static only -- gravity holding, nothing
    accelerating -- so it under-states a moving robot; the safety factor is in the
    result, quote it. ``margin`` is null where gravity applies no torque about the
    axis at all, which is not a failure but an unbounded margin. Every argument here
    overrides mechlint.yaml for this call only and is echoed back in the result, so
    a what-if answer carries its own assumptions.
    """

    def run() -> dict[str, Any]:
        project, model = _load(config, description, model_scale, package_paths, xacro_args)
        database = _database(project, actuator_db)
        result = run_torque_budget(
            model,
            project,
            actuators=database,
            payloads_g=payload_g,
            voltage=voltage,
            assignments=joint_actuators,
            mount=mount,
            payload_at=payload_at,
            link_masses_g=link_mass_g,
            samples=samples or DEFAULT_SAMPLES,
        )
        return result.model_dump(mode="json")

    return _soft(run)


@server.tool(annotations=READ_ONLY)
def list_actuators(
    actuator: Annotated[
        str | None, Field(description="One key, to get that entry in full with its source.")
    ] = None,
    min_torque_Nm: Annotated[
        float | None, Field(description="Only entries that reach this torque at `voltage`.")
    ] = None,
    max_mass_g: Annotated[float | None, Field(description="Only entries no heavier.")] = None,
    kind: Annotated[str | None, Field(description="hobby_servo, smart_servo or stepper.")] = None,
    voltage: Annotated[
        float | None, Field(description="Supply volts. Without it, each entry's recommended one.")
    ] = None,
    config: Config = None,
    actuator_db: ActuatorDb = None,
) -> dict[str, Any]:
    """The actuator database: what each motor delivers, how heavy it is, and where that came from.

    This is the "what would fit instead?" tool. Every torque is a datasheet figure,
    never a measurement, and every entry carries ``confidence`` plus the ``source_url``
    it was read from -- cite them. Torque is stored per supply voltage for servos and
    as one holding torque for steppers, so a comparison across kinds only means
    something at a stated voltage.
    """

    def run() -> dict[str, Any]:
        project = MechlintConfig.from_yaml(config) if config else None
        database = _database(project, actuator_db)
        if actuator is not None:
            return {"actuator": database[actuator].model_dump(mode="json")}
        found = database.matching(
            min_torque_Nm=min_torque_Nm,
            voltage=voltage,
            max_mass_g=max_mass_g,
            kind=kind,  # type: ignore[arg-type]
        )
        return {
            "voltage_V": voltage,
            "count": len(found),
            "actuators": [
                {
                    **entry.model_dump(mode="json"),
                    "torque_at_voltage": entry.torque_limit(voltage).model_dump(mode="json"),
                }
                for entry in found
            ],
        }

    return _soft(run)


#: The one tool that is not read-only. It is not destructive either: it writes a
#: single generated file and refuses to overwrite anything it did not write, so a
#: host can present it as an ordinary write rather than a warning. Idempotent
#: because the same model produces the same file, byte for byte.
WRITES = ToolAnnotations(
    read_only_hint=False,
    destructive_hint=False,
    idempotent_hint=True,
    open_world_hint=False,
)


@server.tool(annotations=WRITES)
def write_inertials(
    config: Config = None,
    description: Description = None,
    output_path: Annotated[
        str | None,
        Field(
            description="Where to write. A directory gets inertials.xacro inside it. "
            f"Default: {DEFAULT_FILENAME} beside the description."
        ),
    ] = None,
    force: Annotated[
        bool,
        Field(description="Overwrite a file mechlint did not generate. Ask the user first."),
    ] = False,
    actuator_db: ActuatorDb = None,
    model_scale: ModelScale = None,
    package_paths: PackagePaths = None,
    xacro_args: XacroArgs = None,
) -> dict[str, Any]:
    """Write the computed <inertial> blocks as a generated xacro the description can include.

    The only tool here that touches a file, and it touches exactly one: a
    generated ``inertials.xacro`` full of ``<xacro:macro>`` definitions. It never
    edits the URDF -- you include the file and swap one link's ``<inertial>`` for
    its macro at a time, so the diff shows what was accepted. It refuses to
    overwrite a file it did not generate, and refuses any report computed with
    overrides, because a what-if must not end up in a robot description. Ask
    before calling it: the numbers should have been reviewed first.
    """

    def run() -> dict[str, Any]:
        project, model = _load(config, description, model_scale, package_paths, xacro_args)
        report = run_compute_inertia(model, project, actuators=_database(project, actuator_db))
        target = Path(output_path) if output_path else model.path.parent / DEFAULT_FILENAME
        result = run_write_inertials(report, target, force=force)
        return result.model_dump(mode="json")

    return _soft(run)


def main() -> None:
    """Entry point for ``mechlint-mcp``. Speaks stdio, the transport hosts expect."""
    server.run(transport="stdio")


if __name__ == "__main__":  # pragma: no cover
    main()
