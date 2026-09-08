"""The CLI surface. M0 only promises that it exists and is honest about what it cannot do yet."""

from __future__ import annotations

import pytest
from typer.testing import CliRunner

from mechlint import __version__
from mechlint.cli.main import app

SUBCOMMANDS = ["inertia", "urdf-check", "torque", "actuators", "drift", "render", "check"]

runner = CliRunner()


def test_help_lists_every_subcommand() -> None:
    result = runner.invoke(app, ["--help"])

    assert result.exit_code == 0
    for name in SUBCOMMANDS:
        assert name in result.output


def test_version() -> None:
    result = runner.invoke(app, ["--version"])

    assert result.exit_code == 0
    assert __version__ in result.output


def test_bare_invocation_shows_help_rather_than_a_traceback() -> None:
    result = runner.invoke(app, [])

    assert result.exit_code != 0
    assert "Usage" in result.output


@pytest.mark.parametrize("name", SUBCOMMANDS)
def test_each_subcommand_has_help(name: str) -> None:
    result = runner.invoke(app, [name, "--help"])

    assert result.exit_code == 0


@pytest.mark.parametrize("name", SUBCOMMANDS)
def test_unbuilt_subcommands_exit_2_and_name_their_milestone(name: str) -> None:
    """Exit 2 is 'not implemented', distinct from 1, which means a check failed."""
    result = runner.invoke(app, [name])

    assert result.exit_code == 2
    assert "not implemented yet" in result.output


def test_an_unknown_subcommand_is_an_error() -> None:
    assert runner.invoke(app, ["levitate"]).exit_code != 0
