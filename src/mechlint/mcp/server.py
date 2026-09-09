"""mechlint's tools over MCP: the same functions the CLI calls, as JSON.

Three tools, all read-only, in the order ``docs/llms.md`` tells a host to call
them: ``inspect_robot`` to see the chain, ``check_urdf`` to find out whether its
numbers can be trusted, ``compute_inertia`` to get real ones. Torque arrives in
M2 and the one writing tool, ``write_inertials``, in M3 -- until then nothing
here touches a file.

Run it with ``mechlint-mcp``; the README has the ``.mcp.json`` that points a
host at it.
"""

from __future__ import annotations

from pathlib import Path
from typing import Annotated, Any

from pydantic import Field

from mechlint import __version__
from mechlint.core import units
from mechlint.core.checks import check_urdf as run_check_urdf
from mechlint.core.config import MechlintConfig
from mechlint.core.inertia import compute_inertia as run_compute_inertia
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


def _soft(call: Any) -> dict[str, Any]:
    """A failure is a result, not a traceback: ``{ok: false, error, hint}``."""
    try:
        return {"ok": True, **call()}
    except DescriptionError as error:
        return {"ok": False, "error": error.message, "hint": error.hint}
    except Exception as error:
        return {"ok": False, "error": str(error), "hint": ""}


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
        return run_check_urdf(model, project).model_dump(mode="json")

    return _soft(run)


@server.tool(annotations=READ_ONLY)
def compute_inertia(
    config: Config = None,
    description: Description = None,
    model_scale: ModelScale = None,
    package_paths: PackagePaths = None,
    xacro_args: XacroArgs = None,
) -> dict[str, Any]:
    """Mass, centre of mass and inertia tensor per link, in SI, from the meshes and mechlint.yaml.

    Each link says whether its mass was ``measured``, ``estimated`` from the
    mesh, or simply taken from the URDF. ``pending_components`` lists masses
    mechlint could not resolve -- a servo missing from that list is a torque
    number that will be wrong later. This tool writes nothing; the CLI's
    ``mechlint inertia --write`` generates the xacro.
    """

    def run() -> dict[str, Any]:
        project, model = _load(config, description, model_scale, package_paths, xacro_args)
        return run_compute_inertia(model, project).model_dump(mode="json")

    return _soft(run)


def main() -> None:
    """Entry point for ``mechlint-mcp``. Speaks stdio, the transport hosts expect."""
    server.run(transport="stdio")


if __name__ == "__main__":  # pragma: no cover
    main()
