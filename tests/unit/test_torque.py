"""Static torque, against a two-link arm that can be solved on paper.

The plan makes this milestone's first condition "the 2-link analytic test passes
to 1e-9", and that is what the first test here is. Everything after it checks the
parts that a hand solution cannot: which links a joint holds up, what an override
does, and what happens when gravity applies no torque about an axis at all.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from mechlint.core import units
from mechlint.core.actuators import ActuatorDatabase, bundled
from mechlint.core.config import MechlintConfig
from mechlint.core.torque import torque_budget
from mechlint.core.urdf import RobotModel
from tests.unit.urdf_builder import write_urdf

G = units.G

#: A planar arm in the x-z plane, both joints turning about +y, laid out along +x at
#: the zero pose. Link 1 is 1 m, link 2 is 0.5 m, and a fixed `tip` marks the end.
#: No geometry anywhere: the masses come from `components`, so every number below is
#: one the reader can put on paper.
TWO_LINK = """
  <link name="base"/>
  <link name="link_1"/>
  <link name="link_2"/>
  <link name="tip"/>
  <joint name="joint_1" type="revolute">
    <parent link="base"/><child link="link_1"/>
    <origin xyz="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-1.5708" upper="1.5708" effort="20" velocity="1"/>
  </joint>
  <joint name="joint_2" type="revolute">
    <parent link="link_1"/><child link="link_2"/>
    <origin xyz="1 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-1.5708" upper="1.5708" effort="20" velocity="1"/>
  </joint>
  <joint name="joint_tip" type="fixed">
    <parent link="link_2"/><child link="tip"/><origin xyz="0.5 0 0"/>
  </joint>
"""

#: 1 kg at the middle of link 1, 0.5 kg at the middle of link 2.
MASSES = """
components:
  link_1: [{ mass_g: 1000, at: [0.5, 0, 0], note: m1 }]
  link_2: [{ mass_g: 500,  at: [0.25, 0, 0], note: m2 }]
scenario: { mount: table, safety_factor: 1.0 }
"""


def _model(tmp_path: Path, body: str = TWO_LINK) -> RobotModel:
    return RobotModel.load(write_urdf(tmp_path, body))


def _config(tmp_path: Path, yaml: str = MASSES) -> MechlintConfig:
    path = tmp_path / "mechlint.yaml"
    path.write_text("robot: { description: test.urdf }\n" + yaml)
    return MechlintConfig.from_yaml(path)


def _joints(report: object) -> dict[str, object]:
    return {joint.joint: joint for joint in report.joints}  # type: ignore[attr-defined]


# --------------------------------------------------------------------------- the hand solution


def test_the_two_link_arm_matches_the_hand_solution(tmp_path: Path) -> None:
    """Fully extended: joint 1 carries 1 kg at 0.5 m and 0.5 kg at 1.25 m, joint 2
    carries 0.5 kg at 0.25 m. Nothing here is approximate, so neither is the tolerance."""
    report = torque_budget(_model(tmp_path), _config(tmp_path), payloads_g=[0.0])
    joints = _joints(report)

    assert joints["joint_1"].cases[0].max_torque_Nm == pytest.approx(
        (1.0 * 0.5 + 0.5 * 1.25) * G, abs=1e-9
    )
    assert joints["joint_2"].cases[0].max_torque_Nm == pytest.approx(0.5 * 0.25 * G, abs=1e-9)


def test_the_payload_adds_exactly_its_own_moment(tmp_path: Path) -> None:
    """200 g at the tip: 1.5 m from joint 1, 0.5 m from joint 2."""
    report = torque_budget(_model(tmp_path), _config(tmp_path), payloads_g=[200.0])
    joints = _joints(report)

    assert joints["joint_1"].cases[-1].max_torque_Nm == pytest.approx(
        (1.0 * 0.5 + 0.5 * 1.25 + 0.2 * 1.5) * G, abs=1e-9
    )
    assert joints["joint_2"].cases[-1].max_torque_Nm == pytest.approx(
        (0.5 * 0.25 + 0.2 * 0.5) * G, abs=1e-9
    )


def test_the_worst_pose_is_the_extended_one(tmp_path: Path) -> None:
    """An arm holds most torque straight out, and the report has to say so: the pose is
    what a person takes to the bench to check the number."""
    report = torque_budget(_model(tmp_path), _config(tmp_path))
    pose = _joints(report)["joint_1"].cases[0].pose

    assert pose == pytest.approx({"joint_1": 0.0, "joint_2": 0.0}, abs=1e-9)


def test_zero_payload_is_always_a_case(tmp_path: Path) -> None:
    """A joint that cannot hold the arm up empty will not hold anything, so the floor
    belongs in every report even when nobody asked for it."""
    report = torque_budget(_model(tmp_path), _config(tmp_path), payloads_g=[200.0])

    assert report.payloads_g == [0.0, 200.0]


def test_the_payload_hangs_from_the_derived_tip(tmp_path: Path) -> None:
    report = torque_budget(_model(tmp_path), _config(tmp_path))

    assert report.payload_at == "tip"


# --------------------------------------------------------------------------- the chain


def test_a_joint_only_carries_what_is_below_it(tmp_path: Path) -> None:
    report = torque_budget(_model(tmp_path), _config(tmp_path))
    joints = _joints(report)

    assert joints["joint_1"].downstream_links == ["link_1", "link_2", "tip"]
    assert joints["joint_2"].downstream_links == ["link_2", "tip"]
    assert joints["joint_1"].downstream_mass_kg == pytest.approx(1.5)
    assert joints["joint_2"].downstream_mass_kg == pytest.approx(0.5)


def test_gravity_along_an_axis_produces_no_torque_about_it(tmp_path: Path) -> None:
    """A shoulder yaw under a table mount holds nothing statically. That is an unbounded
    margin, not a pass with a number, and calling it 0.0 would read as a failure."""
    body = TWO_LINK.replace('<origin xyz="0 0 0"/><axis xyz="0 1 0"/>', '<axis xyz="0 0 1"/>')
    report = torque_budget(_model(tmp_path, body), _config(tmp_path))
    case = _joints(report)["joint_1"].cases[0]

    assert case.max_torque_Nm == pytest.approx(0.0, abs=1e-12)
    assert case.margin is None
    assert "parallel to gravity" in _joints(report)["joint_1"].notes[0]


# --------------------------------------------------------------------------- judging


def test_an_assigned_actuator_is_judged_against_its_stall_torque(tmp_path: Path) -> None:
    config = _config(
        tmp_path, MASSES + "\nactuators: { voltage: 6.0, joints: { joint_2: mg996r } }"
    )
    joint = _joints(torque_budget(_model(tmp_path), config))["joint_2"]

    assert joint.actuator_name == "MG996R"
    assert joint.torque_basis == "stall torque at 6 V"
    assert joint.confidence == "low"
    # 0.5 kg at 0.25 m is 1.226 N*m against 1.079 N*m of stall, at safety factor 1.0.
    assert joint.cases[0].available_Nm == pytest.approx(1.079)
    assert joint.cases[0].margin == pytest.approx(1.079 / (0.5 * 0.25 * G))
    assert joint.ok is False


def test_a_joint_with_no_actuator_is_reported_but_not_judged(tmp_path: Path) -> None:
    """`joint: null` means "no servo yet". Silence would read as a pass."""
    config = _config(tmp_path, MASSES + "\nactuators: { joints: { joint_1: null } }")
    report = torque_budget(_model(tmp_path), config)

    assert report.unjudged == ["joint_1", "joint_2"]
    assert all(case.ok is None for case in _joints(report)["joint_1"].cases)
    assert report.ok  # nothing judged means nothing failed


def test_t001_fires_at_the_lightest_payload_that_already_fails(tmp_path: Path) -> None:
    config = _config(
        tmp_path, MASSES + "\nactuators: { voltage: 6.0, joints: { joint_2: mg996r } }"
    )
    report = torque_budget(_model(tmp_path), config, payloads_g=[500.0])
    (t001,) = [f for f in report.findings if f.check == "T001"]

    assert t001.subject == "joint_2"
    assert "no payload" in t001.message  # it fails empty; the 500 g case is not the news
    assert "low confidence" in t001.message
    assert not report.ok


def test_an_unknown_actuator_fails_loudly(tmp_path: Path) -> None:
    config = _config(tmp_path, MASSES + "\nactuators: { joints: { joint_1: nope } }")
    report = torque_budget(_model(tmp_path), config)

    assert not report.ok
    assert "unknown actuator" in report.findings[0].message


# --------------------------------------------------------------------------- overrides


def test_a_per_call_actuator_answers_what_if_without_editing_the_file(tmp_path: Path) -> None:
    config = _config(
        tmp_path, MASSES + "\nactuators: { voltage: 6.8, joints: { joint_2: mg996r } }"
    )
    report = torque_budget(_model(tmp_path), config, assignments={"joint_2": "ds3225"})
    joint = _joints(report)["joint_2"]

    assert joint.actuator == "ds3225"
    assert joint.cases[0].ok is True


def test_a_mount_override_beats_even_the_model(tmp_path: Path) -> None:
    """Hanging the same arm from the ceiling flips gravity; the magnitudes are unchanged
    because a mirror image of a static load case is still that load case."""
    upright = torque_budget(_model(tmp_path), _config(tmp_path))
    upside_down = torque_budget(_model(tmp_path), _config(tmp_path), mount="ceiling")

    assert upside_down.gravity_m_s2 == pytest.approx((0.0, 0.0, G))
    assert _joints(upside_down)["joint_1"].cases[0].max_torque_Nm == pytest.approx(
        _joints(upright)["joint_1"].cases[0].max_torque_Nm
    )


def test_a_voltage_override_changes_what_the_servo_is_credited_with(tmp_path: Path) -> None:
    config = _config(
        tmp_path, MASSES + "\nactuators: { voltage: 6.0, joints: { joint_2: mg996r } }"
    )
    report = torque_budget(_model(tmp_path), config, voltage=4.8)
    joint = _joints(report)["joint_2"]

    assert joint.voltage_V == 4.8
    assert joint.cases[0].available_Nm == pytest.approx(0.922)


def test_a_project_actuator_table_extends_the_bundled_one(tmp_path: Path) -> None:
    (tmp_path / "servos.yaml").write_text(
        "actuators:\n"
        "  monster:\n"
        "    name: Monster\n    vendor: nobody\n    kind: hobby_servo\n"
        "    stall_torque_Nm: { 6.0: 50.0 }\n    mass_g: 100\n"
        "    source_url: http://example.invalid\n    source_date: 2026-01-01\n"
        "    source: invented for a test\n    confidence: low\n"
    )
    config = _config(
        tmp_path, MASSES + "\nactuators: { voltage: 6.0, joints: { joint_1: monster } }"
    )
    database = ActuatorDatabase.from_yaml(tmp_path / "servos.yaml")
    report = torque_budget(_model(tmp_path), config, actuators=bundled().merged_with(database))

    assert _joints(report)["joint_1"].cases[0].ok is True
    assert len(database) == 1


# --------------------------------------------------------------------------- sampling


def test_more_samples_never_find_a_smaller_worst_case(tmp_path: Path) -> None:
    """The corners and the zero pose are always searched, so raising --samples can only
    ever refine the interior. A number that moved down would mean the sweep is unstable."""
    model, config = _model(tmp_path), _config(tmp_path)
    coarse = torque_budget(model, config, samples=64)
    fine = torque_budget(model, config, samples=4096)

    for name, joint in _joints(coarse).items():
        assert _joints(fine)[name].cases[0].max_torque_Nm >= joint.cases[0].max_torque_Nm - 1e-12


def test_the_sweep_is_the_same_on_every_run(tmp_path: Path) -> None:
    """No seed anywhere: two people reading the same report see the same worst pose."""
    model, config = _model(tmp_path), _config(tmp_path)
    first = torque_budget(model, config).model_dump_json()

    assert torque_budget(model, config).model_dump_json() == first


def test_a_prismatic_joint_slides_rather_than_turns(tmp_path: Path) -> None:
    """FK has to handle the other single-DOF joint type, or a gantry silently reads as fixed."""
    body = """
      <link name="base"/><link name="carriage"/>
      <joint name="slide" type="prismatic">
        <parent link="base"/><child link="carriage"/>
        <origin xyz="0 0 0"/><axis xyz="1 0 0"/>
        <limit lower="0" upper="2" effort="10" velocity="1"/>
      </joint>
    """
    from mechlint.core import chain

    poses = chain.forward_kinematics(_model(tmp_path, body).urdf, np.array([[0.0], [1.5]]))

    assert poses["carriage"][0, :3, 3] == pytest.approx([0.0, 0.0, 0.0])
    assert poses["carriage"][1, :3, 3] == pytest.approx([1.5, 0.0, 0.0])


def test_a_mistyped_actuator_name_does_not_read_as_a_deliberate_null(tmp_path: Path) -> None:
    """`joint: null` means "no servo yet"; `joint: nope` is a typo. Both end unjudged, so
    the typo has to be told apart from the choice."""
    config = _config(tmp_path, MASSES + "\nactuators: { joints: { joint_1: null } }")
    deliberate = _joints(torque_budget(_model(tmp_path), config))["joint_1"]
    mistyped = _joints(torque_budget(_model(tmp_path), config, assignments={"joint_1": "nope"}))[
        "joint_1"
    ]

    assert deliberate.notes == []
    assert "not in the database" in mistyped.notes[0]


# --------------------------------------------------------------------------- overrides


def test_a_link_mass_override_lands_exactly_where_the_hand_solution_says(tmp_path: Path) -> None:
    """Halve link 1: joint 1 loses 0.5 kg at 0.5 m and joint 2 does not move at all."""
    report = torque_budget(
        _model(tmp_path), _config(tmp_path), payloads_g=[0.0], link_masses_g={"link_1": 500.0}
    )
    joints = _joints(report)

    assert joints["joint_1"].cases[0].max_torque_Nm == pytest.approx(
        (0.5 * 0.5 + 0.5 * 1.25) * G, abs=1e-9
    )
    assert joints["joint_2"].cases[0].max_torque_Nm == pytest.approx(0.5 * 0.25 * G, abs=1e-9)


def test_the_report_echoes_only_what_was_overridden(tmp_path: Path) -> None:
    report = torque_budget(_model(tmp_path), _config(tmp_path), payloads_g=[200.0], mount="ceiling")

    assert report.overrides == {"payload_g": [200.0], "mount": "ceiling"}


def test_a_run_straight_from_the_config_overrides_nothing(tmp_path: Path) -> None:
    assert torque_budget(_model(tmp_path), _config(tmp_path)).overrides == {}


def test_a_non_default_sample_count_is_echoed_because_it_changes_the_answer(
    tmp_path: Path,
) -> None:
    """Two runs at different sample counts can disagree; the result has to say which."""
    report = torque_budget(_model(tmp_path), _config(tmp_path), samples=64)

    assert report.overrides == {"samples": 64}


def test_a_mass_override_on_an_already_computed_inertia_report_is_refused(tmp_path: Path) -> None:
    """Applying it would be silent: the masses were fixed before this call was made."""
    from mechlint.core.inertia import compute_inertia

    model, config = _model(tmp_path), _config(tmp_path)
    with pytest.raises(ValueError, match="pass one or the other"):
        torque_budget(
            model, config, inertia=compute_inertia(model, config), link_masses_g={"link_1": 500.0}
        )
