"""Forward kinematics only far enough to answer 'is this model in metres?'."""

from __future__ import annotations

import numpy as np
import pytest

from mechlint.core import chain, units
from mechlint.core.urdf import RobotModel
from tests.unit.urdf_builder import write_urdf

TWO_LINK = """
  <link name="base"/>
  <link name="upper"/>
  <link name="fore"/>
  <link name="sensor"/>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="upper"/>
    <origin xyz="0 0 0.3"/><axis xyz="0 1 0"/>
    <limit lower="-1" upper="1" effort="5" velocity="1"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="upper"/><child link="fore"/>
    <origin xyz="0 0 0.4"/><axis xyz="0 1 0"/>
    <limit lower="-1" upper="1" effort="5" velocity="1"/>
  </joint>
  <joint name="mount" type="fixed">
    <parent link="base"/><child link="sensor"/>
    <origin xyz="0 0 5.0"/>
  </joint>
"""


@pytest.fixture
def two_link(tmp_path) -> RobotModel:
    return RobotModel.load(write_urdf(tmp_path, TWO_LINK))


def test_a_link_behind_only_fixed_joints_does_not_move(two_link: RobotModel) -> None:
    assert chain.moving_links(two_link.urdf) == {"upper", "fore"}


def test_reach_ignores_a_sensor_bolted_to_the_base(two_link: RobotModel) -> None:
    """The sensor is 5 m away and never moves; the arm is 0.7 m long. Reach is 0.7."""
    distance, link = chain.reach(two_link.urdf)

    assert distance == pytest.approx(0.7)
    assert link == "fore"


def test_link_origins_are_measured_from_the_root(two_link: RobotModel) -> None:
    origins = chain.link_origins(two_link.urdf)

    assert origins["upper"] == pytest.approx([0, 0, 0.3])
    assert origins["fore"] == pytest.approx([0, 0, 0.7])


# --------------------------------------------------------------------------- gravity

GROUNDED = (
    '<link name="world"/>'
    '<joint name="anchor" type="fixed">'
    '<parent link="world"/><child link="base"/><origin xyz="0 0 0" rpy="{rpy}"/>'
    "</joint>" + TWO_LINK
)


def _gravity(tmp_path, rpy: str = "0 0 0", mount: object = "table"):
    model = RobotModel.load(write_urdf(tmp_path, GROUNDED.format(rpy=rpy)))
    return chain.gravity(model.urdf, mount)


def test_a_model_without_a_world_frame_falls_back_to_the_config(tmp_path) -> None:
    """Nothing to derive from: the root *is* the robot -- a revolute joint hangs straight
    off it -- so mechlint.yaml is the answer rather than a second opinion."""
    model = RobotModel.load(write_urdf(tmp_path, TWO_LINK))

    assert not chain.is_grounded(model.urdf)
    assert chain.gravity(model.urdf, "ceiling").source == "config"
    assert chain.gravity(model.urdf, "ceiling").vector == pytest.approx((0, 0, units.G))


def test_a_grounded_model_says_which_way_is_down_itself(tmp_path) -> None:
    """The world-to-base rotation already encodes the mounting, and it is the rotation
    the simulator runs -- so it wins over the config rather than the other way round."""
    derived = _gravity(tmp_path, rpy="0 0 0", mount="ceiling")

    assert derived.source == "model"
    assert derived.implied_mount == "table"
    assert derived.vector == pytest.approx((0, 0, -units.G))


def test_a_robot_hung_upside_down_is_read_off_the_model(tmp_path) -> None:
    upside_down = _gravity(tmp_path, rpy="3.14159265359 0 0")

    assert upside_down.implied_mount == "ceiling"
    assert upside_down.vector == pytest.approx((0, 0, units.G), abs=1e-6)


def test_a_wall_mount_needs_no_name_because_the_model_carries_it(tmp_path) -> None:
    """This is why there is no 'wall': rolled 90 degrees about x, down is -y in the base
    frame. Rolled the other way it would be +y. One word cannot say which."""
    on_a_wall = _gravity(tmp_path, rpy="1.5707963268 0 0")
    other_way = _gravity(tmp_path, rpy="-1.5707963268 0 0")

    assert on_a_wall.implied_mount is None
    assert on_a_wall.vector == pytest.approx((0, -units.G, 0), abs=1e-6)
    assert other_way.vector == pytest.approx((0, units.G, 0), abs=1e-6)


def test_the_dropped_wall_name_is_rejected_rather_than_guessed(tmp_path) -> None:
    with pytest.raises(ValueError, match="unknown mount 'wall'"):
        chain.mount_gravity("wall")


def test_an_explicit_vector_is_taken_as_given(tmp_path) -> None:
    assert chain.mount_gravity([0.0, -9.81, 0.0]) == pytest.approx([0, -9.81, 0])


# --------------------------------------------------------------------------- kinematics

BRANCHED = """
  <link name="base"/>
  <link name="shoulder"/>
  <link name="elbow"/>
  <link name="hand"/>
  <link name="camera"/>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="shoulder"/>
    <origin xyz="0 0 0.2"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="2" effort="5" velocity="1"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="shoulder"/><child link="elbow"/>
    <origin xyz="0.3 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-1" upper="1" effort="5" velocity="1"/>
  </joint>
  <joint name="wrist" type="fixed">
    <parent link="elbow"/><child link="hand"/><origin xyz="0.1 0 0"/>
  </joint>
  <joint name="bracket" type="fixed">
    <parent link="base"/><child link="camera"/><origin xyz="0 0 3.0"/>
  </joint>
"""


@pytest.fixture
def branched(tmp_path) -> RobotModel:
    return RobotModel.load(write_urdf(tmp_path, BRANCHED))


def test_a_subtree_is_everything_one_joint_holds_up(branched: RobotModel) -> None:
    assert chain.subtree(branched.urdf, "shoulder") == {"shoulder", "elbow", "hand"}
    assert chain.subtree(branched.urdf, "elbow") == {"elbow", "hand"}


def test_the_tip_is_the_end_of_the_longest_actuated_branch(branched: RobotModel) -> None:
    """`camera` is a leaf, is further from the base than `hand`, and is still not the tip:
    a payload hangs from the end effector, and depth in moving joints is what says which
    link that is."""
    assert chain.tip(branched.urdf) == "hand"


def test_forward_kinematics_agrees_with_yourdfpy(branched: RobotModel) -> None:
    """The batched implementation exists for speed, so it has to be checked against the
    one it replaced -- on a pose where every joint is somewhere awkward."""
    urdf = branched.urdf
    sample = np.array([0.7, -0.4])
    mine = chain.forward_kinematics(urdf, sample)

    urdf.update_cfg(sample)
    for link in urdf.link_map:
        reference = np.asarray(urdf.get_transform(link, urdf.base_link), dtype=float)
        assert mine[link][0] == pytest.approx(reference, abs=1e-12), link


def test_a_configuration_of_the_wrong_width_is_rejected(branched: RobotModel) -> None:
    with pytest.raises(ValueError, match="2 joint values"):
        chain.forward_kinematics(branched.urdf, [[0.0, 0.0, 0.0]])


def test_every_sample_lands_inside_the_joint_limits(branched: RobotModel) -> None:
    lower, upper = chain.joint_limits(branched.urdf)
    samples = chain.sample_configurations(branched.urdf, 256)

    assert samples.shape == (256, 2)
    assert np.all(samples >= lower - 1e-12)
    assert np.all(samples <= upper + 1e-12)


def test_the_sweep_always_contains_the_zero_pose_and_every_corner(branched: RobotModel) -> None:
    """An arm holds most torque fully extended, and a quasi-random fill of six dimensions
    can miss that badly. The corners are cheap, so they are never left to chance."""
    lower, upper = chain.joint_limits(branched.urdf)
    samples = chain.sample_configurations(branched.urdf, 64)
    rows = {tuple(row) for row in samples}

    assert (0.0, 0.0) in rows
    for corner in ((lower[0], lower[1]), (lower[0], upper[1]), (upper[0], upper[1])):
        assert tuple(np.asarray(corner)) in rows


def test_a_continuous_joint_is_swept_a_full_turn(tmp_path) -> None:
    """It has no limits, so the honest range is every orientation it can be left in."""
    body = BRANCHED.replace(
        '<joint name="j1" type="revolute">', '<joint name="j1" type="continuous">'
    )
    urdf = RobotModel.load(write_urdf(tmp_path, body)).urdf

    assert chain.joint_limits(urdf)[:, 0] == pytest.approx([-np.pi, np.pi])


def test_the_sampling_is_the_same_every_run(branched: RobotModel) -> None:
    first = chain.sample_configurations(branched.urdf, 200)

    assert (chain.sample_configurations(branched.urdf, 200) == first).all()


def test_gravity_in_the_fk_root_frame_and_the_base_frame_agree(tmp_path) -> None:
    """The two differ only by the world-to-base rotation, and a torque computed in one
    frame with the other's gravity would be wrong in a way nothing else would catch."""
    urdf = RobotModel.load(write_urdf(tmp_path, GROUNDED.format(rpy="1.5708 0 0"))).urdf
    rotation = np.asarray(urdf.get_transform("base", urdf.base_link), dtype=float)[:3, :3]

    assert chain.gravity_in_root(urdf).array == pytest.approx([0.0, 0.0, -units.G])
    assert rotation @ chain.gravity(urdf).array == pytest.approx(chain.gravity_in_root(urdf).array)


def test_an_explicit_mount_can_override_even_a_grounded_model(tmp_path) -> None:
    """What-if questions are about a robot that is not the one in the URDF. Answering them
    is the point, and `override` is the only way past the model's own rotation."""
    urdf = RobotModel.load(write_urdf(tmp_path, GROUNDED.format(rpy="0 0 0"))).urdf

    assert chain.gravity(urdf, "ceiling").vector == pytest.approx((0.0, 0.0, -units.G))
    assert chain.gravity(urdf, "ceiling", override=True).vector == pytest.approx(
        (0.0, 0.0, units.G)
    )
