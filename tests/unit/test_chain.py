"""Forward kinematics only far enough to answer 'is this model in metres?'."""

from __future__ import annotations

import pytest

from mechlint.core import chain
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
