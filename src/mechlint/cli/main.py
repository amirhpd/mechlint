"""``mechlint`` command line.

One subcommand per core function, plus ``check`` which runs them all and sets
the exit code. Inspect subcommands never write; the only thing mechlint ever
writes is a *generated* ``inertials.xacro`` that the description includes.

Exit codes:
    0   everything checked passed
    1   a check failed (this is what turns a CI run red)
    2   usage error, or a subcommand that is not implemented yet
"""

from __future__ import annotations

from typing import Annotated

import typer

from mechlint import __version__

app = typer.Typer(
    name="mechlint",
    help="Lint and CI for robot mechanics: mass, inertia and joint torque from CAD + URDF, "
    "checked against actuator and printing limits.",
    no_args_is_help=True,
    add_completion=False,
)

#: Which milestone each subcommand lands in. Printed by the stubs so nobody has
#: to read the plan to find out whether a command is missing or just unbuilt.
_MILESTONE = {
    "inertia": "M1",
    "urdf-check": "M1",
    "torque": "M2",
    "actuators": "M2",
    "drift": "M4",
    "render": "M4",
    "check": "M1",
}


def _not_implemented(command: str) -> None:
    milestone = _MILESTONE[command]
    typer.secho(
        f"mechlint {command}: not implemented yet (planned for {milestone}).",
        err=True,
        fg=typer.colors.YELLOW,
    )
    raise typer.Exit(code=2)


def _version_callback(value: bool) -> None:
    if value:
        typer.echo(f"mechlint {__version__}")
        raise typer.Exit()


@app.callback()
def main(
    version: Annotated[
        bool,
        typer.Option("--version", callback=_version_callback, is_eager=True, help="Show version."),
    ] = False,
) -> None:
    """Robot mechanics, checked."""


@app.command()
def inertia() -> None:
    """Mass, centre of mass and inertia tensor per link, in the link frame."""
    _not_implemented("inertia")


@app.command("urdf-check")
def urdf_check() -> None:
    """Physical sanity of the URDF: units, tensors, masses, meshes (U0xx, M0xx)."""
    _not_implemented("urdf-check")


@app.command()
def torque() -> None:
    """Worst-case static joint torque, and the margin against the assigned actuator (T0xx)."""
    _not_implemented("torque")


@app.command()
def actuators() -> None:
    """Query the actuator database — which servos meet a torque requirement."""
    _not_implemented("actuators")


@app.command()
def drift() -> None:
    """Disagreements between the CAD and the URDF that describes it (D0xx)."""
    _not_implemented("drift")


@app.command()
def render() -> None:
    """Multi-view PNG of a pose, with labelled joint frames. For the human, not for numbers."""
    _not_implemented("render")


@app.command()
def check() -> None:
    """Run every check and exit non-zero if any fails. This is the CI entry point."""
    _not_implemented("check")


if __name__ == "__main__":
    app()
