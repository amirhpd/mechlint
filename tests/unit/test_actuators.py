"""The actuator database: the curated part, so the tests are about provenance as much
as about numbers.

Every entry has to carry where it came from and how much that source is worth. A
torque figure with no URL behind it is the one thing this file is here to prevent
from ever landing.
"""

from __future__ import annotations

import datetime as dt
from pathlib import Path

import pytest

from mechlint.core.actuators import (
    KGFCM_TO_NM,
    ActuatorDatabase,
    TableNotFound,
    bundled,
    load_database,
)

ENTRY = """
actuators:
  test_servo:
    name: Test Servo
    vendor: nobody
    kind: hobby_servo
    stall_torque_Nm: { 4.8: 1.0, 6.0: 1.5 }
    recommended_voltage: 6.0
    voltage_range: [4.8, 6.6]
    mass_g: 50
    source_url: http://example.invalid
    source_date: 2026-01-01
    source: invented for a test
    confidence: low
"""


def _database(tmp_path: Path, text: str = ENTRY) -> ActuatorDatabase:
    path = tmp_path / "actuators.yaml"
    path.write_text(text)
    return ActuatorDatabase.from_yaml(path)


# --------------------------------------------------------------------------- the bundled table


def test_every_bundled_entry_carries_a_source() -> None:
    """The database is the part of mechlint a model cannot regenerate. That is only true
    while every row says where it came from."""
    for actuator in bundled():
        assert actuator.source_url.startswith("http"), actuator.key
        assert isinstance(actuator.source_date, dt.date), actuator.key
        assert actuator.source, actuator.key
        assert actuator.confidence in ("high", "medium", "low"), actuator.key


def test_the_plans_first_seven_entries_are_all_there() -> None:
    assert set(bundled().actuators) == {
        "mg996r",
        "ds3218",
        "ds3225",
        "xl430_w250",
        "xc330_m288",
        "2xl430_w250",
        "nema17_pg5",
    }


def test_the_mg996r_matches_its_datasheet_in_kgf_cm() -> None:
    """The stored value is N*m; the source line quotes kgf*cm. This is the conversion
    between them, so a transcription slip cannot hide behind a unit change."""
    mg996r = bundled()["mg996r"]

    assert mg996r.stall_torque_Nm[4.8] == pytest.approx(9.4 * KGFCM_TO_NM, abs=5e-4)
    assert mg996r.stall_torque_Nm[6.0] == pytest.approx(11.0 * KGFCM_TO_NM, abs=5e-4)
    assert "9.4 kgf*cm" in mg996r.source


def test_a_hobby_servo_datasheet_is_never_high_confidence() -> None:
    """Nobody publishes a measured curve for a 4-dollar servo. Saying `high` about one
    would make the whole confidence column meaningless."""
    for actuator in bundled():
        if actuator.kind == "hobby_servo":
            assert actuator.confidence != "high", actuator.key


# --------------------------------------------------------------------------- torque lookup


def test_a_listed_voltage_is_used_exactly(tmp_path: Path) -> None:
    limit = _database(tmp_path)["test_servo"].torque_limit(4.8)

    assert limit.torque_Nm == 1.0
    assert limit.exact
    assert limit.basis == "stall torque at 4.8 V"


def test_an_unlisted_voltage_falls_back_rather_than_interpolating(tmp_path: Path) -> None:
    """5.5 V is between the two listed figures. Inventing a number for it would be the
    one thing mechlint promises never to do, so it takes the lower one and says so."""
    limit = _database(tmp_path)["test_servo"].torque_limit(5.5)

    assert limit.torque_Nm == 1.0
    assert limit.voltage_V == 4.8
    assert not limit.exact
    assert "under-states" in limit.note


def test_a_voltage_below_the_whole_table_is_flagged_as_optimistic(tmp_path: Path) -> None:
    """Running below every measured point means the real torque is lower than anything
    in the table, so the figure quoted is an over-estimate and has to say so."""
    limit = _database(tmp_path)["test_servo"].torque_limit(3.3)

    assert limit.torque_Nm == 1.0
    assert "OVER-states" in limit.note


def test_no_voltage_means_the_recommended_one(tmp_path: Path) -> None:
    assert _database(tmp_path)["test_servo"].torque_limit().torque_Nm == 1.5


def test_a_supply_outside_the_operating_range_is_reported(tmp_path: Path) -> None:
    actuator = _database(tmp_path)["test_servo"]

    assert "below" in actuator.voltage_note(3.3)
    assert "above" in actuator.voltage_note(9.0)
    assert actuator.voltage_note(6.0) is None


def test_a_stepper_ignores_voltage_because_current_is_what_sets_its_torque() -> None:
    limit = bundled()["nema17_pg5"].torque_limit(24.0)

    assert limit.torque_Nm == 2.0
    assert limit.basis == "holding torque at 1.68 A"
    assert "driver current" in limit.note


# --------------------------------------------------------------------------- queries


def test_matching_answers_what_would_fit_instead() -> None:
    found = bundled().matching(min_torque_Nm=2.0, voltage=6.8, max_mass_g=100)

    assert [a.key for a in found] == ["ds3225", "ds3218"]  # strongest first, stepper too heavy


def test_matching_can_be_narrowed_to_one_kind() -> None:
    assert all(a.kind == "smart_servo" for a in bundled().matching(kind="smart_servo"))


# --------------------------------------------------------------------------- loading


def test_an_entry_with_no_torque_at_all_is_rejected(tmp_path: Path) -> None:
    text = ENTRY.replace("    stall_torque_Nm: { 4.8: 1.0, 6.0: 1.5 }\n", "")

    with pytest.raises(ValueError, match="has no torque"):
        _database(tmp_path, text)


def test_an_unknown_field_is_rejected_rather_than_ignored(tmp_path: Path) -> None:
    """A typo in a file that decides whether a servo is strong enough should fail loudly."""
    with pytest.raises(ValueError, match="stall_torque_nm"):
        _database(tmp_path, ENTRY + "    stall_torque_nm: { 6.0: 99 }\n")


def test_an_unknown_key_lists_what_is_known() -> None:
    with pytest.raises(KeyError, match="mg996r"):
        bundled()["mg996rr"]


def test_a_project_table_wins_over_the_bundled_one(tmp_path: Path) -> None:
    path = tmp_path / "mine.yaml"
    path.write_text(ENTRY.replace("test_servo", "mg996r"))
    merged = load_database(path)

    assert merged["mg996r"].vendor == "nobody"
    assert "xl430_w250" in merged  # the rest of the bundled table survives


def test_a_missing_table_says_where_paths_are_resolved_from(tmp_path: Path) -> None:
    """A bare "[Errno 2] No such file" is a stack trace by another name: three places
    can name this file, and each resolves relative paths differently."""
    with pytest.raises(TableNotFound) as raised:
        ActuatorDatabase.from_yaml(tmp_path / "nope.yaml")

    assert "actuator table not found" in raised.value.message
    assert "relative" in raised.value.hint
