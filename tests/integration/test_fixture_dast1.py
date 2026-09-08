"""The pinned DAST-1 fixture is the thing every later milestone is measured against.

These tests do not check mechlint's own logic so much as the fixture's integrity:
if the geometry or the chain silently changes, the M1 and M2 expectations built
on top of it become fiction. See tests/fixtures/dast1/PROVENANCE.md.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
import yourdfpy

from mechlint.core import units
from mechlint.core.config import MechlintConfig
from mechlint.core.geometry import mass_properties

PLA_DENSITY = 1240.0

#: Recorded from these exact bytes with trimesh 5.1.0. Meshes are in millimetres.
#: (watertight, volume cm^3 or None, extents mm)
MESH_BASELINE = {
    "arm_1.stl": (False, None, (208.500, 26.000, 74.000)),
    "arm_2.stl": (True, 104.535, (248.500, 26.000, 74.000)),
    "arm_3.stl": (True, 96.725, (127.500, 61.500, 52.500)),
    "base.stl": (False, None, (80.000, 80.000, 81.500)),
    "gripper_3.stl": (True, 35.692, (67.773, 156.292, 78.258)),
    "rotary.stl": (True, 32.012, (26.000, 74.000, 64.000)),
}

#: Link origins in base_link at all-zero joints, in URDF units (decimetres).
FK_ZERO = {
    "base_link": (0.0, 0.0, 0.0),
    "rotary_link": (0.0, 0.0, 1.47),
    "arm_1_link": (0.0, 0.0, 1.47),
    "arm_2_link": (0.0, 0.0, 3.015),
    "arm_3_link": (0.0, 0.0, 4.96),
    "wrist_link": (0.0, 0.0, 5.84),
    "gripper_link": (0.0, 0.0, 5.84),
    "tip": (0.0, 0.0, 7.59),
}


def test_the_fixture_is_complete(dast1_dir: Path) -> None:
    for name in ("dast1.urdf", "mechlint.yaml", "PROVENANCE.md"):
        assert (dast1_dir / name).is_file(), name
    for name in ("description.urdf.xacro", "control.xacro"):
        assert (dast1_dir / "urdf" / name).is_file(), name
    assert len(list((dast1_dir / "meshes").glob("*.stl"))) == 12


def test_no_absolute_paths_leaked_into_the_expanded_urdf(dast1_urdf: Path) -> None:
    """The fixture has to work on a machine that has never heard of dast_1."""
    text = dast1_urdf.read_text()
    assert "/home/" not in text
    assert "/opt/ros" not in text


@pytest.mark.parametrize("name", sorted(MESH_BASELINE))
def test_mesh_baseline_still_holds(dast1_dir: Path, name: str) -> None:
    watertight, volume_cm3, extents_mm = MESH_BASELINE[name]
    mp = mass_properties(dast1_dir / "meshes" / name, scale=units.MM, density_kg_m3=PLA_DENSITY)

    assert mp.watertight is watertight
    assert mp.convex_hull_fallback is not watertight
    assert mp.extents_m == pytest.approx(tuple(e * units.MM for e in extents_mm), abs=1e-5)

    if volume_cm3 is not None:
        assert mp.volume_m3 * 1e6 == pytest.approx(volume_cm3, abs=1e-3)
        # Solid PLA: an upper bound on the printed shell, before infill.
        assert mp.mass_kg == pytest.approx(volume_cm3 * 1e-6 * PLA_DENSITY, rel=1e-3)


def test_two_meshes_are_not_watertight(dast1_dir: Path) -> None:
    """The reason this robot is a good fixture: M001 has something to find (arm_1, base)."""
    open_meshes = {
        p.name
        for p in sorted((dast1_dir / "meshes").glob("*.stl"))
        if "collision" not in p.name and not mass_properties(p, scale=units.MM).watertight
    }
    assert open_meshes == {"arm_1.stl", "base.stl"}


def _load_urdf(dast1_dir: Path) -> yourdfpy.URDF:
    def resolve(fname: str) -> str:
        return str(dast1_dir / fname.removeprefix("package://description/"))

    return yourdfpy.URDF.load(str(dast1_dir / "dast1.urdf"), filename_handler=resolve)


def test_the_chain_is_the_one_the_plan_describes(dast1_dir: Path) -> None:
    urdf = _load_urdf(dast1_dir)

    assert urdf.robot.name == "dast_1"
    assert urdf.actuated_joint_names == [f"joint_{i}" for i in range(1, 7)]
    for name in FK_ZERO:
        assert name in urdf.link_map


def test_forward_kinematics_at_zero(dast1_dir: Path) -> None:
    urdf = _load_urdf(dast1_dir)
    urdf.update_cfg(np.zeros(6))

    for link, expected in FK_ZERO.items():
        origin = urdf.get_transform(link, "base_link")[:3, 3]
        assert origin == pytest.approx(expected, abs=1e-6), link


def test_the_model_is_in_decimetres(dast1_dir: Path) -> None:
    """7.59 URDF units of reach is 0.759 m only if one unit is a decimetre. Nothing in
    the URDF itself says so -- which is exactly what check U001 is for."""
    urdf = _load_urdf(dast1_dir)
    urdf.update_cfg(np.zeros(6))
    reach_units = float(np.linalg.norm(urdf.get_transform("tip", "base_link")[:3, 3]))

    assert reach_units == pytest.approx(7.59, abs=1e-6)

    scale = MechlintConfig.from_yaml(dast1_dir / "mechlint.yaml").robot.model_scale
    low, high = units.PLAUSIBLE_REACH_M
    assert low < reach_units * float(scale) < high
    assert not low < reach_units < high  # unscaled, it is out of the plausible band


def test_every_link_carries_the_same_placeholder_tensor(dast1_urdf: Path) -> None:
    """U005/U006 fodder: a default_inertial macro applied to all eight links."""
    text = dast1_urdf.read_text()

    assert text.count('ixx="1.0"') == 8
    assert text.count('izz="1.0"') == 8
    assert text.count('effort="20.0"') == 6  # U007: an MG996R stalls near 1.1 N*m
