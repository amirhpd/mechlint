"""The kinematic chain: what moves, and how far out it reaches.

M1 needs only enough of this to answer "is this model in metres?" -- which is a
question about distances, so it is a question about forward kinematics. The
downstream-mass sets and joint-space sampling that ``torque`` needs are M2.
"""

from __future__ import annotations

import numpy as np
import yourdfpy

MOVING_JOINT_TYPES = frozenset({"revolute", "continuous", "prismatic", "planar", "floating"})


def parent_of(urdf: yourdfpy.URDF) -> dict[str, tuple[str, str]]:
    """``child link -> (parent link, joint name)`` for every joint in the model."""
    return {joint.child: (joint.parent, joint.name) for joint in urdf.robot.joints}


def moving_links(urdf: yourdfpy.URDF) -> set[str]:
    """Links whose pose depends on at least one joint value.

    A sensor bolted to the base is *not* part of the arm no matter how far off
    to the side it is mounted, so reach is measured over this set only.
    """
    parents = parent_of(urdf)
    moving_joints = {joint.name for joint in urdf.robot.joints if joint.type in MOVING_JOINT_TYPES}

    def is_moving(link: str, seen: frozenset[str] = frozenset()) -> bool:
        entry = parents.get(link)
        if entry is None or link in seen:
            return False
        parent, joint = entry
        return joint in moving_joints or is_moving(parent, seen | {link})

    return {link for link in urdf.link_map if is_moving(link)}


def link_origins(urdf: yourdfpy.URDF, *, root: str | None = None) -> dict[str, np.ndarray]:
    """Every link's origin in the root frame, at the zero configuration.

    In URDF units -- the caller multiplies by the model scale to get metres.
    """
    root = root or urdf.base_link
    urdf.update_cfg(np.zeros(urdf.num_actuated_joints))
    return {
        link: np.asarray(urdf.get_transform(link, root)[:3, 3], dtype=float)
        for link in urdf.link_map
    }


def reach(urdf: yourdfpy.URDF) -> tuple[float, str]:
    """Distance to the furthest moving link at the zero pose, and which link that is.

    The zero pose, not the worst case: this feeds the unit check (U001), where
    an order of magnitude is what matters, and a folded-up arm is still within
    a factor of two of its extension. ``torque`` sweeps the joint space instead.
    """
    origins = link_origins(urdf)
    candidates = {link: origins[link] for link in moving_links(urdf) if link in origins}
    if not candidates:
        return 0.0, ""
    furthest = max(candidates, key=lambda link: float(np.linalg.norm(candidates[link])))
    return float(np.linalg.norm(candidates[furthest])), furthest
