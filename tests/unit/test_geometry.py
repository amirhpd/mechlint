"""Analytic checks on mass properties.

A unit cube is the whole point: its volume, mass and inertia tensor are known
in closed form, so any drift in the geometry pipeline shows up here rather than
in a robot-shaped number nobody can verify by hand.
"""

from __future__ import annotations

import math

import numpy as np
import pytest
import trimesh

from mechlint.core import units
from mechlint.core.geometry import load_mesh, mass_properties


def unit_cube() -> trimesh.Trimesh:
    return trimesh.creation.box(extents=(1.0, 1.0, 1.0))


def test_unit_cube_volume_mass_and_inertia() -> None:
    """Side 1, density 1: V = 1, m = 1, I = diag(m*a^2/6) = diag(1/6)."""
    mp = mass_properties(unit_cube())

    assert mp.volume_m3 == pytest.approx(1.0, abs=1e-12)
    assert mp.mass_kg == pytest.approx(1.0, abs=1e-12)
    assert mp.com_m == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)
    assert mp.extents_m == pytest.approx((1.0, 1.0, 1.0), abs=1e-12)
    assert mp.watertight is True
    assert mp.convex_hull_fallback is False

    expected = np.eye(3) / 6.0
    assert np.allclose(mp.inertia_array, expected, atol=1e-12)


def test_inertia_of_a_rectangular_box() -> None:
    """I_xx = m/12 * (b^2 + c^2) for a solid box of sides a, b, c."""
    a, b, c = 0.2, 0.3, 0.5
    density = 1240.0  # PLA
    mp = mass_properties(trimesh.creation.box(extents=(a, b, c)), density_kg_m3=density)

    mass = density * a * b * c
    assert mp.mass_kg == pytest.approx(mass, rel=1e-12)

    expected = np.diag(
        [
            mass * (b**2 + c**2) / 12.0,
            mass * (a**2 + c**2) / 12.0,
            mass * (a**2 + b**2) / 12.0,
        ]
    )
    assert np.allclose(mp.inertia_array, expected, rtol=1e-12, atol=1e-15)


def test_scale_converts_model_units_to_metres() -> None:
    """A cube drawn 1000 units wide in a millimetre model is a 1 m cube."""
    mp = mass_properties(trimesh.creation.box(extents=(1000.0, 1000.0, 1000.0)), scale=units.MM)

    assert mp.volume_m3 == pytest.approx(1.0, rel=1e-12)
    assert mp.extents_m == pytest.approx((1.0, 1.0, 1.0), rel=1e-12)
    assert np.allclose(mp.inertia_array, np.eye(3) / 6.0, atol=1e-12)


def test_mass_and_inertia_are_linear_in_density() -> None:
    plain = mass_properties(unit_cube())
    dense = mass_properties(unit_cube(), density_kg_m3=1240.0)

    assert dense.mass_kg == pytest.approx(plain.mass_kg * 1240.0, rel=1e-12)
    assert np.allclose(dense.inertia_array, plain.inertia_array * 1240.0, rtol=1e-12)


def test_with_mass_rescales_the_tensor_and_leaves_the_shape_alone() -> None:
    """A link that was weighed on a scale keeps its geometry, gains a real mass."""
    estimated = mass_properties(unit_cube(), density_kg_m3=1240.0)
    measured = estimated.with_mass(1.0)

    assert measured.mass_kg == pytest.approx(1.0)
    assert measured.volume_m3 == pytest.approx(estimated.volume_m3)
    assert measured.com_m == pytest.approx(estimated.com_m)
    assert measured.density_kg_m3 == pytest.approx(1.0)
    assert np.allclose(measured.inertia_array, np.eye(3) / 6.0, atol=1e-12)


def test_com_offset_is_reported_in_the_geometry_frame() -> None:
    cube = unit_cube()
    cube.apply_translation((2.0, -3.0, 0.5))
    mp = mass_properties(cube)

    assert mp.com_m == pytest.approx((2.0, -3.0, 0.5), abs=1e-12)
    # The tensor is taken about the COM, so moving the part does not change it.
    assert np.allclose(mp.inertia_array, np.eye(3) / 6.0, atol=1e-12)


def test_open_mesh_falls_back_to_the_convex_hull_and_says_so() -> None:
    """Not watertight is a warning, never a crash and never a silent skip (M001)."""
    cube = unit_cube()
    holed = trimesh.Trimesh(vertices=cube.vertices, faces=cube.faces[:-2], process=False)
    assert not holed.is_watertight

    mp = mass_properties(holed)

    assert mp.watertight is False
    assert mp.convex_hull_fallback is True
    assert mp.volume_m3 == pytest.approx(1.0, rel=1e-9)


@pytest.mark.parametrize(
    ("kwargs", "message"),
    [
        ({"scale": 0.0}, "scale must be positive"),
        ({"scale": -1.0}, "scale must be positive"),
        ({"density_kg_m3": 0.0}, "density must be positive"),
    ],
)
def test_nonsense_arguments_are_rejected(kwargs: dict[str, float], message: str) -> None:
    with pytest.raises(ValueError, match=message):
        mass_properties(unit_cube(), **kwargs)


def test_with_mass_rejects_a_non_positive_mass() -> None:
    with pytest.raises(ValueError, match="measured mass must be positive"):
        mass_properties(unit_cube()).with_mass(0.0)


def test_missing_file_names_the_file() -> None:
    with pytest.raises(FileNotFoundError, match="no_such_part.stl"):
        load_mesh("no_such_part.stl")


def test_resolve_scale_accepts_names_and_numbers() -> None:
    assert units.resolve_scale("dm") == pytest.approx(0.1)
    assert units.resolve_scale("MM") == pytest.approx(1e-3)
    assert units.resolve_scale(0.25) == pytest.approx(0.25)
    assert math.isclose(units.resolve_scale("inch"), 0.0254)

    with pytest.raises(ValueError, match="unknown unit"):
        units.resolve_scale("furlong")
    with pytest.raises(ValueError, match="must be positive"):
        units.resolve_scale(0.0)


def test_a_step_file_is_refused_with_a_hint_not_mis_parsed(tmp_path) -> None:
    """STEP needs the OCCT kernel and lands in M4; until then say so precisely."""
    path = tmp_path / "part.STEP"
    path.write_text("ISO-10303-21;\n")

    with pytest.raises(NotImplementedError, match="M4"):
        load_mesh(path)
