"""Loading a description without ROS: `package://` and `$(find pkg)` by hand."""

from __future__ import annotations

from pathlib import Path

import pytest

from mechlint.core.urdf import DescriptionError, PackageResolver, RobotModel, expand_xacro
from tests.unit.urdf_builder import box, inertial, urdf_text, write_urdf

XACRO = """<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:arg name="tall" default="false"/>
  <xacro:property name="height" value="$(arg tall)"/>
  <link name="base">
    <visual><geometry><mesh filename="package://parts/cube.stl"/></geometry></visual>
  </link>
  <xacro:if value="${height}"><link name="mast"/></xacro:if>
  <xacro:include filename="$(find parts)/extra.xacro"/>
</robot>
"""

EXTRA = """<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><link name="included"/></robot>
"""


@pytest.fixture
def package(tmp_path: Path) -> Path:
    (tmp_path / "extra.xacro").write_text(EXTRA)
    (tmp_path / "robot.urdf.xacro").write_text(XACRO)
    return tmp_path


def test_package_paths_win_over_a_sourced_ros_workspace(tmp_path: Path) -> None:
    """A project pins its checkout; it should not have to unset the environment first."""
    resolver = PackageResolver({"description": tmp_path})

    assert resolver.share("description") == tmp_path


def test_an_unresolvable_package_says_what_to_do(tmp_path: Path) -> None:
    with pytest.raises(DescriptionError) as error:
        PackageResolver({"known": tmp_path}).share("missing")

    assert "--package-path missing=" in error.value.hint
    assert "known" in error.value.hint


@pytest.mark.parametrize(
    ("uri", "expected"),
    [
        ("package://parts/meshes/a.stl", "PKG/meshes/a.stl"),
        ("meshes/a.stl", "HERE/meshes/a.stl"),
        ("file:///abs/a.stl", "/abs/a.stl"),
    ],
)
def test_mesh_uris_resolve(tmp_path: Path, uri: str, expected: str) -> None:
    resolver = PackageResolver({"parts": tmp_path / "pkg"})
    here = tmp_path / "here"

    resolved = resolver.resolve_uri(uri, relative_to=here)

    assert str(resolved) == expected.replace("PKG", str(tmp_path / "pkg")).replace(
        "HERE", str(here)
    )


def test_xacro_expands_without_ros_on_the_path(package: Path) -> None:
    """`$(find)` normally imports ament_index_python, which is not installed here."""
    xml = expand_xacro(
        package / "robot.urdf.xacro",
        xacro_args={"tall": "true"},
        resolver=PackageResolver({"parts": package}),
    )

    assert "included" in xml
    assert "mast" in xml


def test_a_xacro_argument_changes_what_is_built(package: Path) -> None:
    xml = expand_xacro(
        package / "robot.urdf.xacro", xacro_args={}, resolver=PackageResolver({"parts": package})
    )

    assert "mast" not in xml


def test_an_unresolvable_package_inside_xacro_keeps_its_own_message(package: Path) -> None:
    """xacro rewraps every exception; the hint the user can act on has to survive that."""
    with pytest.raises(DescriptionError) as error:
        expand_xacro(package / "robot.urdf.xacro", resolver=PackageResolver())

    assert "cannot resolve package 'parts'" in error.value.message
    assert "--package-path parts=" in error.value.hint


def test_a_missing_description_is_not_a_traceback(tmp_path: Path) -> None:
    with pytest.raises(DescriptionError, match="description not found"):
        RobotModel.load(tmp_path / "nope.urdf")


def test_primitives_become_geometry_with_analytic_volume(tmp_path: Path) -> None:
    body = f'<link name="a">{inertial(1.0, (1, 1, 1))}{box("1 2 3")}</link>'
    model = RobotModel.load(write_urdf(tmp_path, body))
    (geometry,) = model.geometries("a")

    assert geometry.kind == "box"
    assert geometry.mesh.volume == pytest.approx(6.0)
    assert geometry.watertight


def test_a_visual_origin_places_geometry_in_the_link_frame(tmp_path: Path) -> None:
    """Mass properties are wanted in the link frame, not the mesh's own."""
    body = (
        '<link name="a"><visual><origin xyz="0 0 5" rpy="0 0 0"/>'
        '<geometry><box size="1 1 1"/></geometry></visual></link>'
    )
    model = RobotModel.load(write_urdf(tmp_path, body))
    (geometry,) = model.geometries("a")

    assert geometry.mesh.centroid == pytest.approx([0, 0, 5])


def test_collision_geometry_is_the_fallback_not_the_default(tmp_path: Path) -> None:
    body = (
        '<link name="a">'
        '<visual><geometry><box size="1 1 1"/></geometry></visual>'
        '<collision><geometry><box size="2 2 2"/></geometry></collision>'
        "</link>"
        '<link name="b"><collision><geometry><box size="3 3 3"/></geometry></collision></link>'
    )
    model = RobotModel.load(write_urdf(tmp_path, body))

    assert model.geometries("a", which="visual-or-collision")[0].mesh.volume == pytest.approx(1.0)
    assert model.geometries("b", which="visual-or-collision")[0].mesh.volume == pytest.approx(27.0)


def test_a_urdf_string_and_a_xacro_produce_the_same_model(tmp_path: Path) -> None:
    body = f'<link name="a">{inertial(1.0, (1, 1, 1))}{box("1 1 1")}</link>'
    (tmp_path / "plain.urdf").write_text(urdf_text(body))
    (tmp_path / "same.urdf.xacro").write_text(
        urdf_text(body).replace("<robot ", '<robot xmlns:xacro="http://www.ros.org/wiki/xacro" ')
    )

    plain = RobotModel.load(tmp_path / "plain.urdf")
    expanded = RobotModel.load(tmp_path / "same.urdf.xacro")

    assert plain.link_names == expanded.link_names
    assert plain.declared_inertial("a").mass == expanded.declared_inertial("a").mass
