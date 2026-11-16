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
    Actuator,
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
    # A superset, not equality: adding an actuator to the table must not break this.
    assert set(bundled().actuators) >= {
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
    keys = [a.key for a in found]

    # Membership, not positions: a new entry or a new measurement can reorder the list.
    assert {"ds3240", "ds3225", "ds3218"} <= set(keys)
    assert "nema17_pg5" not in keys  # strong enough, but 520 g is over the 100 g limit
    torques = [a.torque_limit(6.8).torque_Nm for a in found]
    assert torques == sorted(torques, reverse=True)


def test_matching_leaves_out_what_cannot_run_on_the_supply() -> None:
    """A 10-12.6 V servo is not "the strongest thing at 6 V" -- it does not run at 6 V."""
    keys = [a.key for a in bundled().matching(voltage=6.0)]

    assert "ds51150_12v" not in keys  # 6 V is below its 10-12.6 V range
    assert "xl430_w250" not in keys  # 6 V is below its 6.5-12 V range
    assert "ds3225" in keys  # 6 V is inside its range, between two listed voltages
    assert "nema17_pg5" in keys  # a stepper's torque does not depend on the supply


def test_runs_on_refuses_a_voltage_below_the_stall_table_without_a_range() -> None:
    """With no vendor range, the lowest listed voltage is the floor: below it, the only
    figure available over-states the part."""
    servo = bundled()["mg996r"].model_copy(update={"voltage_range": None})

    assert servo.runs_on(6.0)
    assert servo.runs_on(None)
    assert not servo.runs_on(4.0)


def test_the_n20_5v_row_is_its_12v_datasheet_row_scaled() -> None:
    """The 5 V figures are calculated, not quoted: a brushed DC motor's stall torque and
    no-load speed scale with its supply. This keeps the calculation checkable."""
    n20 = bundled()["n20_50"]

    assert n20.kind == "dc_gearmotor"
    assert n20.stall_torque_Nm[12.0] == pytest.approx(20 * 0.00706155, abs=5e-4)  # 20 oz*in
    assert n20.stall_torque_Nm[5.0] == pytest.approx(n20.stall_torque_Nm[12.0] * 5 / 12, abs=5e-4)
    assert n20.no_load_speed_rpm[5.0] == pytest.approx(460.0 * 5 / 12, abs=0.05)
    assert "n20_50" in [a.key for a in bundled().matching(kind="dc_gearmotor")]


def _servo(**changes: object):
    """A made-up servo, so these rules are tested without leaning on bundled figures."""
    base = bundled()["mg996r"].model_copy(
        update={
            "stall_torque_Nm": {4.8: 1.0, 6.0: 1.2},
            "measured_torque_Nm": {6.0: 0.9},
            "voltage_range": (4.8, 6.6),
        }
    )
    return base.model_copy(update=changes)


def test_a_measured_torque_replaces_the_datasheet() -> None:
    limit = _servo().torque_limit(6.0)

    assert limit.torque_Nm == 0.9
    assert limit.basis == "measured torque at 6 V"
    assert limit.overstates is False


def test_above_the_measured_voltage_the_measurement_under_states() -> None:
    """6.6 V has a datasheet row nowhere and a measurement only below it: use 6 V."""
    limit = _servo().torque_limit(6.6)

    assert limit.torque_Nm == 0.9
    assert limit.exact is False
    assert limit.overstates is False


def test_below_the_measured_voltage_a_lower_datasheet_row_is_used() -> None:
    """At 4.8 V the 6 V measurement over-states the part; the 4.8 V datasheet row is lower."""
    servo = _servo(measured_torque_Nm={6.0: 1.1})
    limit = servo.torque_limit(4.8)

    assert limit.torque_Nm == 1.0
    assert limit.basis == "stall torque at 4.8 V"
    assert servo.runs_on(4.8)


def test_below_the_measured_voltage_a_higher_datasheet_row_is_not_trusted() -> None:
    """The datasheet says 1.0 at 4.8 V, but the part measured 0.9 at 6 V: the datasheet
    is already shown to be optimistic, so the measurement is kept and flagged."""
    servo = _servo()
    limit = servo.torque_limit(4.8)

    assert limit.torque_Nm == 0.9
    assert limit.overstates is True
    assert not servo.runs_on(4.8)


def test_a_measured_table_alone_is_enough_torque() -> None:
    servo = Actuator(
        **{
            **_servo().model_dump(exclude={"stall_torque_Nm"}),
            "measured_torque_Nm": {6.0: 0.9},
        }
    )

    assert servo.torque_limit(6.0).basis == "measured torque at 6 V"


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
