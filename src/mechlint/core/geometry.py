"""Mass properties from geometry.

Meshes (STL, OBJ, PLY -- anything trimesh reads) today. STEP is recognised and
refused with a hint rather than mis-parsed: reading it needs the OCCT kernel
behind the ``[step]`` extra, and it earns its keep in M4, where ``drift``
compares hole positions that a mesh simply does not carry. Results are SI:
metres, kilograms, kg*m^2.

The caller says how big one mesh unit is, in metres, via ``scale``. A part
exported from CAD in millimetres is ``scale=units.MM``. Nothing here guesses.

Mass and the inertia tensor are both linear in density, so a part whose mass
was measured on a scale does not need recomputing — see
:meth:`MassProperties.with_mass`.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import trimesh
from pydantic import BaseModel, ConfigDict, Field

Vec3 = tuple[float, float, float]
Mat3 = tuple[Vec3, Vec3, Vec3]

#: Recognised so the failure says "not yet" instead of "could not parse".
STEP_SUFFIXES = frozenset({".step", ".stp"})


class MassProperties(BaseModel):
    """What a single piece of geometry weighs and how it resists rotation.

    ``inertia`` is taken about ``com``, aligned with the axes of the frame the
    geometry was given in — which is what URDF's ``<inertial>`` wants once
    ``<origin xyz>`` is set to ``com``.
    """

    model_config = ConfigDict(frozen=True)

    source: str = Field(description="File the geometry came from, or '<mesh>' for an object.")
    volume_m3: float
    mass_kg: float
    density_kg_m3: float
    com_m: Vec3 = Field(description="Centre of mass in the geometry's own frame.")
    inertia_kg_m2: Mat3 = Field(description="Inertia tensor about com_m, frame-aligned.")
    extents_m: Vec3 = Field(description="Axis-aligned bounding box side lengths.")
    watertight: bool
    convex_hull_fallback: bool = Field(
        default=False,
        description="True when the mesh was not a closed volume and its convex hull was used "
        "instead. Mass properties are then an over-estimate — check M001.",
    )

    @property
    def inertia_array(self) -> np.ndarray:
        return np.asarray(self.inertia_kg_m2, dtype=float)

    def with_mass(self, mass_kg: float) -> MassProperties:
        """Same shape, different mass — for a link that was actually weighed.

        Mass and inertia are both proportional to density, so a measured mass
        rescales the whole tensor by one factor. Volume and COM do not move.
        """
        if mass_kg <= 0.0:
            raise ValueError(f"measured mass must be positive, got {mass_kg}")
        factor = mass_kg / self.mass_kg
        scaled = (self.inertia_array * factor).tolist()
        return self.model_copy(
            update={
                "mass_kg": mass_kg,
                "density_kg_m3": mass_kg / self.volume_m3,
                "inertia_kg_m2": _as_mat3(scaled),
            }
        )


def load_mesh(path: str | Path) -> trimesh.Trimesh:
    """Load a mesh file as a single concatenated :class:`trimesh.Trimesh`.

    Multi-body STLs and scenes are merged, because a URDF ``<mesh>`` is one
    rigid body no matter how many shells the file holds.
    """
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"mesh not found: {path}")
    if path.suffix.lower() in STEP_SUFFIXES:
        raise NotImplementedError(
            f"{path.name} is a STEP file, which mechlint cannot read yet (planned for M4, "
            "with the [step] extra). Export the same part as STL and point the URDF at that."
        )
    loaded: Any = trimesh.load(path, force="mesh")
    if not isinstance(loaded, trimesh.Trimesh):
        raise ValueError(f"{path} did not load as a mesh (got {type(loaded).__name__})")
    return loaded


def mass_properties(
    geometry: str | Path | trimesh.Trimesh,
    *,
    scale: float = 1.0,
    density_kg_m3: float = 1.0,
) -> MassProperties:
    """Volume, mass, COM and inertia of one piece of geometry.

    Args:
        geometry: A mesh file path, or an already-loaded trimesh.
        scale: Metres per mesh unit (``units.MM`` for a millimetre export).
        density_kg_m3: Material density. The default of 1.0 makes the result a
            per-unit-density tensor, which is what the analytic tests check.

    A mesh that is not a closed volume has no well-defined inside, so its
    convex hull is used and ``convex_hull_fallback`` is set. The number is then
    an upper bound, never silently wrong.
    """
    if scale <= 0.0:
        raise ValueError(f"scale must be positive, got {scale}")
    if density_kg_m3 <= 0.0:
        raise ValueError(f"density must be positive, got {density_kg_m3}")

    source = str(geometry) if isinstance(geometry, str | Path) else "<mesh>"
    mesh = load_mesh(geometry) if isinstance(geometry, str | Path) else geometry.copy()

    watertight = bool(mesh.is_watertight)
    body = mesh
    fallback = False
    if not watertight or mesh.volume <= 0.0:
        body = mesh.convex_hull
        fallback = True

    body = body.copy()
    body.apply_scale(scale)
    body.density = density_kg_m3

    return MassProperties(
        source=source,
        volume_m3=float(body.volume),
        mass_kg=float(body.mass),
        density_kg_m3=density_kg_m3,
        com_m=_as_vec3(body.center_mass),
        inertia_kg_m2=_as_mat3(body.moment_inertia),
        extents_m=_as_vec3(mesh.extents * scale),
        watertight=watertight,
        convex_hull_fallback=fallback,
    )


def _as_vec3(values: Any) -> Vec3:
    a = np.asarray(values, dtype=float).reshape(3)
    return (float(a[0]), float(a[1]), float(a[2]))


def _as_mat3(values: Any) -> Mat3:
    a = np.asarray(values, dtype=float).reshape(3, 3)
    return (_as_vec3(a[0]), _as_vec3(a[1]), _as_vec3(a[2]))
