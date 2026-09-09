"""The CLI surface: which commands exist, what they exit with, and what they never do."""

from __future__ import annotations

from pathlib import Path

import pytest
from typer.testing import CliRunner

from mechlint import __version__
from mechlint.cli.main import app
from tests.unit.urdf_builder import box, inertial, write_urdf

SUBCOMMANDS = ["inertia", "urdf-check", "torque", "actuators", "drift", "render", "check"]
BUILT = ["inertia", "urdf-check", "check"]
UNBUILT = ["torque", "actuators", "drift", "render"]

#: A model with a sane tensor for its 0.1 x 0.1 x 0.4 box, so nothing is flagged.
SANE = f'<link name="base">{inertial(0.5, (0.00708, 0.00708, 0.000833))}{box("0.1 0.1 0.4")}</link>'

runner = CliRunner()


@pytest.fixture
def sane_urdf(tmp_path: Path) -> Path:
    return write_urdf(tmp_path, SANE)


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
    assert runner.invoke(app, [name, "--help"]).exit_code == 0


@pytest.mark.parametrize("name", UNBUILT)
def test_unbuilt_subcommands_exit_2_and_name_their_milestone(name: str) -> None:
    """Exit 2 is 'not implemented', distinct from 1, which means a check failed."""
    result = runner.invoke(app, [name])

    assert result.exit_code == 2
    assert "not implemented yet" in result.output


def test_an_unknown_subcommand_is_an_error() -> None:
    assert runner.invoke(app, ["levitate"]).exit_code != 0


# --------------------------------------------------------------------------- exit codes


def test_a_clean_model_exits_zero(sane_urdf: Path) -> None:
    assert runner.invoke(app, ["urdf-check", str(sane_urdf)]).exit_code == 0


def test_a_failed_check_exits_one(tmp_path: Path) -> None:
    """Exit 1 is what turns a CI run red, and it has to be distinct from exit 2."""
    path = write_urdf(tmp_path, SANE.replace('value="0.5"', 'value="0"'))

    assert runner.invoke(app, ["urdf-check", str(path)]).exit_code == 1


def test_check_is_the_ci_entry_point(tmp_path: Path) -> None:
    path = write_urdf(tmp_path, SANE.replace('value="0.5"', 'value="0"'))

    assert runner.invoke(app, ["check", str(path)]).exit_code == 1


@pytest.mark.parametrize("name", BUILT)
def test_a_missing_description_is_a_usage_error_not_a_traceback(name: str, tmp_path: Path) -> None:
    result = runner.invoke(app, [name, str(tmp_path / "nope.urdf")])

    assert result.exit_code == 2
    assert "not found" in result.output


def test_an_unknown_option_value_is_reported(sane_urdf: Path) -> None:
    result = runner.invoke(app, ["urdf-check", str(sane_urdf), "--model-scale", "furlong"])

    assert result.exit_code == 2
    assert "unknown unit" in result.output


# --------------------------------------------------------------------------- formats


@pytest.mark.parametrize("output_format", ["table", "json", "markdown"])
def test_every_format_renders(sane_urdf: Path, output_format: str) -> None:
    result = runner.invoke(app, ["urdf-check", str(sane_urdf), "-f", output_format])

    assert result.exit_code == 0
    assert result.output.strip()


def test_json_output_is_machine_readable(sane_urdf: Path) -> None:
    import json

    result = runner.invoke(app, ["inertia", str(sane_urdf), "-f", "json"])
    payload = json.loads(result.output)

    assert payload["robot"] == "test"
    assert payload["links"][0]["mass_kg"] > 0


def test_a_unit_name_works_as_a_model_scale(sane_urdf: Path) -> None:
    result = runner.invoke(app, ["urdf-check", str(sane_urdf), "-s", "dm", "-f", "json"])

    assert '"model_scale": 0.1' in result.output


# --------------------------------------------------------------------------- writing


def test_inertia_writes_nothing_unless_asked(sane_urdf: Path, tmp_path: Path) -> None:
    """Inspect commands never write. That is the rule the MCP layer leans on."""
    before = sorted(p.name for p in tmp_path.iterdir())
    runner.invoke(app, ["inertia", str(sane_urdf)])

    assert sorted(p.name for p in tmp_path.iterdir()) == before


def test_write_puts_inertials_in_a_directory(sane_urdf: Path, tmp_path: Path) -> None:
    result = runner.invoke(app, ["inertia", str(sane_urdf), "-w", str(tmp_path)])

    assert result.exit_code == 0
    assert "mechlint_inertial_base" in (tmp_path / "inertials.xacro").read_text()


def test_write_refuses_to_clobber_a_file_it_did_not_generate(
    sane_urdf: Path, tmp_path: Path
) -> None:
    """The only file mechlint writes is its own; everything else is the user's."""
    target = tmp_path / "handwritten.xacro"
    target.write_text("<robot><!-- mine --></robot>")

    result = runner.invoke(app, ["inertia", str(sane_urdf), "-w", str(target)])

    assert result.exit_code == 2
    assert target.read_text() == "<robot><!-- mine --></robot>"


def test_force_overwrites_and_regenerating_is_idempotent(sane_urdf: Path, tmp_path: Path) -> None:
    target = tmp_path / "handwritten.xacro"
    target.write_text("<robot><!-- mine --></robot>")

    assert (
        runner.invoke(app, ["inertia", str(sane_urdf), "-w", str(target), "--force"]).exit_code == 0
    )
    first = target.read_text()
    runner.invoke(app, ["inertia", str(sane_urdf), "-w", str(target)])

    assert target.read_text() == first
