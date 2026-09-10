"""The CLI surface: which commands exist, what they exit with, and what they never do."""

from __future__ import annotations

from pathlib import Path

import pytest
from typer.testing import CliRunner

from mechlint import __version__
from mechlint.cli.main import app
from tests.unit.urdf_builder import box, inertial, write_urdf

SUBCOMMANDS = ["inertia", "urdf-check", "torque", "actuators", "drift", "render", "check"]
BUILT = ["inertia", "urdf-check", "torque", "check"]
UNBUILT = ["drift", "render"]

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
@pytest.mark.parametrize("name", BUILT)
def test_every_format_renders(sane_urdf: Path, name: str, output_format: str) -> None:
    """Three doors, one set of numbers: every command has to reach all three."""
    result = runner.invoke(app, [name, str(sane_urdf), "-f", output_format])

    assert result.exit_code == 0, result.output
    assert result.output.strip()


@pytest.mark.parametrize("name", [*BUILT, "actuators"])
def test_an_unknown_format_is_refused_rather_than_guessed(sane_urdf: Path, name: str) -> None:
    """Silently falling back to the table would make `-f yaml` look like it worked."""
    arguments = [name, "-f", "yaml"]
    if name != "actuators":
        arguments.insert(1, str(sane_urdf))
    result = runner.invoke(app, arguments)

    assert result.exit_code == 2
    assert "unknown format" in result.output


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


# --------------------------------------------------------------------------- actuators

EXTRA_DB = """
actuators:
  hs645:
    name: HS-645MG
    vendor: Hitec
    kind: hobby_servo
    stall_torque_Nm: { 6.0: 0.932 }
    recommended_voltage: 6.0
    mass_g: 55.2
    source_url: http://example.invalid/hs645
    source_date: 2026-09-10
    source: invented for a test
    confidence: low
"""


@pytest.fixture
def extra_db(tmp_path: Path) -> Path:
    path = tmp_path / "my_servos.yaml"
    path.write_text(EXTRA_DB)
    return path


def test_actuators_lists_the_bundled_table() -> None:
    result = runner.invoke(app, ["actuators"])

    assert result.exit_code == 0
    assert "mg996r" in result.output
    assert "confidence" not in result.output.splitlines()[0]  # the header is the compact one


def test_actuators_shows_one_entry_with_its_source() -> None:
    """The whole point of the database: a number you can chase back to a page."""
    result = runner.invoke(app, ["actuators", "mg996r"])

    assert result.exit_code == 0
    assert "servodatabase.com" in result.output
    assert "read 2026-09-09" in result.output
    assert "9.4 kgf*cm" in result.output


def test_actuators_json_is_a_parseable_list() -> None:
    import json

    result = runner.invoke(app, ["actuators", "-t", "2.0", "-f", "json"])
    payload = json.loads(result.output)

    assert [entry["key"] for entry in payload] == ["ds3225", "ds3218", "nema17_pg5"]
    assert all(entry["source_url"] for entry in payload)


def test_actuators_json_of_one_entry_is_an_object() -> None:
    import json

    payload = json.loads(runner.invoke(app, ["actuators", "xl430_w250", "-f", "json"]).output)

    assert payload["vendor"] == "ROBOTIS"
    assert payload["confidence"] == "high"


def test_actuators_markdown_links_every_datasheet() -> None:
    result = runner.invoke(app, ["actuators", "-f", "markdown"])

    assert result.output.count("[datasheet](http") == 7


def test_an_empty_result_says_so_rather_than_printing_a_bare_header() -> None:
    result = runner.invoke(app, ["actuators", "-t", "999"])

    assert result.exit_code == 0
    assert "no actuator in the database meets that requirement" in result.output


def test_an_unknown_actuator_lists_what_is_known() -> None:
    result = runner.invoke(app, ["actuators", "mg996rr"])

    assert result.exit_code == 2
    assert "mg996r" in result.output


def test_an_unknown_kind_is_refused_rather_than_matching_nothing(sane_urdf: Path) -> None:
    """An empty result reads as a real answer, which is the worst outcome for a typo."""
    result = runner.invoke(app, ["actuators", "--kind", "hobby-servo"])

    assert result.exit_code == 2
    assert "unknown kind" in result.output


# --------------------------------------------------------------------------- the extra table


def test_actuator_db_extends_the_bundled_table(extra_db: Path) -> None:
    result = runner.invoke(app, ["actuators", "--actuator-db", str(extra_db)])

    assert result.exit_code == 0
    assert "hs645" in result.output
    assert "mg996r" in result.output  # the bundled entries survive


def test_a_project_table_named_in_the_config_is_picked_up(
    tmp_path: Path, sane_urdf: Path, extra_db: Path
) -> None:
    """`mechlint actuators` takes no robot, but it still reads the config: a project that
    ships its own servos should see them listed here, not only inside `torque`."""
    config = tmp_path / "mechlint.yaml"
    config.write_text(f"robot: {{ description: {sane_urdf.name} }}\nactuator_db: {extra_db.name}\n")

    result = runner.invoke(app, ["actuators", "-c", str(config)])

    assert result.exit_code == 0
    assert "hs645" in result.output


def test_a_project_table_reaches_torque_too(tmp_path: Path, extra_db: Path) -> None:
    tool = f'<link name="tool">{inertial(0.5, (1, 1, 1))}{box("0.1 0.1 0.4")}</link>'
    arm = (
        '<link name="base"/>' + tool + '<joint name="j1" type="revolute">'
        '<parent link="base"/><child link="tool"/>'
        '<origin xyz="0 0 0.5"/><axis xyz="0 1 0"/>'
        '<limit lower="-1" upper="1" effort="5" velocity="1"/></joint>'
    )
    path = write_urdf(tmp_path, arm)
    config = tmp_path / "mechlint.yaml"
    config.write_text(
        f"robot: {{ description: {path.name} }}\n"
        f"actuator_db: {extra_db.name}\n"
        "actuators: { voltage: 6.0, joints: { j1: hs645 } }\n"
    )

    result = runner.invoke(app, ["torque", "-c", str(config), "-f", "json"])
    import json

    payload = json.loads(result.output)

    assert payload["joints"][0]["actuator_name"] == "HS-645MG"
    assert payload["joints"][0]["cases"][0]["available_Nm"] == pytest.approx(0.932)


def test_a_malformed_actuator_table_is_a_usage_error_not_a_traceback(tmp_path: Path) -> None:
    path = tmp_path / "broken.yaml"
    path.write_text("servos: {}\n")

    result = runner.invoke(app, ["actuators", "--actuator-db", str(path)])

    assert result.exit_code == 2
    assert "top-level 'actuators' mapping" in result.output
