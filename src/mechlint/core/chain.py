"""The kinematic chain: what moves, and how far out it reaches.

M1 needed only enough of this to answer "is this model in metres?" -- a question
about distances, so a question about forward kinematics. M2 added the rest: the
subtree a joint holds up, batched FK over thousands of poses at once, and the
deterministic joint-space sweep ``torque`` searches for its worst case.
"""

from __future__ import annotations

from typing import Final, Literal, NamedTuple

import numpy as np
import yourdfpy

from mechlint.core import units

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


#: REP-103: in a world frame, +z is up. Everything below rests on that one convention.
GROUND_UP: Final = np.array([0.0, 0.0, 1.0])

#: The mountings that are a pure flip of the base's +z, and so mean one vector each.
#: There is deliberately no "wall": bolt a robot to a vertical surface and gravity is
#: perpendicular to its +z, but *which* horizontal direction depends on how the robot is
#: rolled about its mounting axis, and no name can say. Wall mounts are expressed either
#: by the model -- the world-to-base rpy already says it -- or by an explicit vector.
MOUNT_GRAVITY: Final[dict[str, tuple[float, float, float]]] = {
    "table": (0.0, 0.0, -1.0),
    "ceiling": (0.0, 0.0, 1.0),
}


class Gravity(NamedTuple):
    """Which way is down, in the robot base's own frame, and who said so."""

    vector: tuple[float, float, float]
    source: Literal["model", "config"]
    implied_mount: str | None = None

    @property
    def array(self) -> np.ndarray:
        return np.asarray(self.vector, dtype=float)


def is_grounded(urdf: yourdfpy.URDF) -> bool:
    """Whether the tree's root is a world frame rather than the robot's own body.

    A root that carries no geometry and no inertial is not a part, it is a place --
    conventionally the link called ``world``. It also has to be still: every joint
    leaving it must be fixed, because the ground does not rotate. Without that second
    half, a robot whose root is a bare ``base_link`` with a revolute joint straight off
    it would be mistaken for a world frame, and its mounting read from a rotation that
    is really its first axis.

    When a world frame is present, how the robot is bolted down is already in the
    model, in the fixed joint below it.
    """
    root = urdf.link_map.get(urdf.base_link)
    if root is None:
        return False
    if root.visuals or root.collisions or root.inertial is not None:
        return False
    leaving = [joint for joint in urdf.robot.joints if joint.parent == urdf.base_link]
    return bool(leaving) and all(joint.type == "fixed" for joint in leaving)


def mounted_link(urdf: yourdfpy.URDF) -> str | None:
    """The robot's own base: the link the world frame is bolted to."""
    if not is_grounded(urdf) or not urdf.actuated_joint_names:
        return None
    parents = parent_of(urdf)
    root = urdf.base_link
    link = urdf.joint_map[urdf.actuated_joint_names[0]].parent
    seen: set[str] = set()
    while link in parents and parents[link][0] != root and link not in seen:
        seen.add(link)
        link = parents[link][0]
    return link if parents.get(link, ("",))[0] == root else None


def gravity(urdf: yourdfpy.URDF, mount: object = "table", *, override: bool = False) -> Gravity:
    """Gravity in the robot base's frame, derived from the model where the model says.

    Precedence here is deliberately the reverse of mechlint's usual "config beats the
    URDF": a grounded model already states its mounting in the world-to-base rotation,
    and that is the rotation the simulator runs. Believing ``mechlint.yaml`` over it
    would produce torques for a mounting nobody is simulating. Check D002 reports the
    disagreement instead. ``override=True`` is how a per-call argument still wins over
    both -- that is what what-if questions are for, and the report echoes which of the
    three decided.
    """
    base = mounted_link(urdf)
    if base is not None and not override:
        rotation = np.asarray(urdf.get_transform(base, urdf.base_link), dtype=float)[:3, :3]
        down = rotation.T @ (-GROUND_UP * units.G)
        return Gravity(_as_tuple(down), "model", _name_for(down))
    return Gravity(_as_tuple(mount_gravity(mount)), "config", _name_for(mount_gravity(mount)))


def mount_gravity(mount: object) -> np.ndarray:
    """A ``scenario.mount`` value as a gravity vector, in m/s^2."""
    if isinstance(mount, str):
        try:
            direction = MOUNT_GRAVITY[mount]
        except KeyError:
            known = ", ".join(sorted(MOUNT_GRAVITY))
            raise ValueError(
                f"unknown mount {mount!r}; use {known}, or an explicit [gx, gy, gz]"
            ) from None
        return np.asarray(direction, dtype=float) * units.G
    return np.asarray(mount, dtype=float).reshape(3)


def _name_for(vector: np.ndarray) -> str | None:
    """The named mounting a vector corresponds to, if any."""
    for name, direction in MOUNT_GRAVITY.items():
        if np.allclose(vector, np.asarray(direction) * units.G, atol=1e-6):
            return name
    return None


def _as_tuple(vector: np.ndarray) -> tuple[float, float, float]:
    a = np.asarray(vector, dtype=float).reshape(3)
    return (float(a[0]), float(a[1]), float(a[2]))


# --------------------------------------------------------------------------- kinematics

#: Joints ``torque`` sweeps: one scalar each, and gravity produces a torque about them.
#: ``planar`` and ``floating`` are excluded deliberately -- they are 3- and 6-DOF, so a
#: single "joint value" and a single axis do not describe them, and a static torque
#: about them is not one number.
DOF_JOINT_TYPES = frozenset({"revolute", "continuous", "prismatic"})

#: How far a continuous joint is swept when sampling. It has no limits, so a full turn
#: is the honest range: every orientation the link can be left in.
CONTINUOUS_RANGE: Final = (-np.pi, np.pi)


def dof_joints(urdf: yourdfpy.URDF) -> list[str]:
    """The single-DOF joints, in the order a configuration vector lists them."""
    return [
        name for name in urdf.actuated_joint_names if urdf.joint_map[name].type in DOF_JOINT_TYPES
    ]


def joint_limits(urdf: yourdfpy.URDF, joints: list[str] | None = None) -> np.ndarray:
    """``(2, n)`` array of lower and upper bounds for each joint, in URDF units."""
    names = joints if joints is not None else dof_joints(urdf)
    bounds = []
    for name in names:
        joint = urdf.joint_map[name]
        limit = getattr(joint, "limit", None)
        lower = None if limit is None else getattr(limit, "lower", None)
        upper = None if limit is None else getattr(limit, "upper", None)
        if joint.type == "continuous" or lower is None or upper is None:
            bounds.append(CONTINUOUS_RANGE)
        else:
            bounds.append((float(lower), float(upper)))
    return np.asarray(bounds, dtype=float).reshape(len(names), 2).T


def children_of(urdf: yourdfpy.URDF) -> dict[str, list[str]]:
    """``parent link -> child links``, the tree the other way round."""
    tree: dict[str, list[str]] = {}
    for joint in urdf.robot.joints:
        tree.setdefault(joint.parent, []).append(joint.child)
    return tree


def subtree(urdf: yourdfpy.URDF, link: str) -> set[str]:
    """``link`` and everything hanging off it -- the mass one joint has to hold up."""
    tree = children_of(urdf)
    found: set[str] = set()
    stack = [link]
    while stack:
        current = stack.pop()
        if current in found:
            continue
        found.add(current)
        stack.extend(tree.get(current, ()))
    return found


def tip(urdf: yourdfpy.URDF) -> str | None:
    """Where a payload hangs: the end of the longest actuated branch.

    Derived, not declared -- the chain already says which link is furthest out.
    Depth in *moving joints* comes first and distance only breaks ties, so a long
    fixed stalk (a camera bracket) never outranks the actual end effector.
    """
    moving = moving_links(urdf)
    tree = children_of(urdf)
    leaves = [link for link in moving if not tree.get(link)]
    if not leaves:
        return None
    origins = link_origins(urdf)
    parents = parent_of(urdf)
    moving_joints = {joint.name for joint in urdf.robot.joints if joint.type in MOVING_JOINT_TYPES}

    def depth(link: str) -> int:
        count, seen = 0, set()
        while link in parents and link not in seen:
            seen.add(link)
            parent, joint = parents[link]
            count += joint in moving_joints
            link = parent
        return count

    return max(leaves, key=lambda link: (depth(link), float(np.linalg.norm(origins[link]))))


def forward_kinematics(
    urdf: yourdfpy.URDF, configurations: np.ndarray, *, joints: list[str] | None = None
) -> dict[str, np.ndarray]:
    """Every link's pose in the root frame, for a batch of configurations at once.

    ``configurations`` is ``(n_samples, n_joints)``; the result maps each link to
    an ``(n_samples, 4, 4)`` array, in URDF units. Written out here rather than
    looped through :mod:`yourdfpy` because ``torque`` evaluates thousands of poses
    and a Python loop over each of them is the whole runtime.
    """
    names = joints if joints is not None else dof_joints(urdf)
    q = np.atleast_2d(np.asarray(configurations, dtype=float))
    if q.shape[1] != len(names):
        raise ValueError(f"expected {len(names)} joint values per sample, got {q.shape[1]}")
    index = {name: column for column, name in enumerate(names)}
    samples = q.shape[0]

    poses = {urdf.base_link: np.broadcast_to(np.eye(4), (samples, 4, 4)).copy()}
    for joint in _root_first(urdf):
        origin = np.eye(4) if joint.origin is None else np.asarray(joint.origin, dtype=float)
        local = np.broadcast_to(origin, (samples, 4, 4)).copy()
        column = index.get(joint.name)
        if column is not None:
            values = q[:, column]
            axis = np.asarray(joint.axis, dtype=float)
            if joint.type == "prismatic":
                slide = local[:, :3, :3] @ (axis / np.linalg.norm(axis))
                local[:, :3, 3] += slide * values[:, None]
            else:
                local[:, :3, :3] = local[:, :3, :3] @ _rotate_about(axis, values)
        poses[joint.child] = poses[joint.parent] @ local
    return poses


def _root_first(urdf: yourdfpy.URDF) -> list[yourdfpy.Joint]:
    """Joints ordered so a parent's pose is always known before its child's."""
    by_parent = {joint.child: joint for joint in urdf.robot.joints}
    tree = children_of(urdf)
    ordered: list[yourdfpy.Joint] = []
    stack = [urdf.base_link]
    while stack:
        link = stack.pop()
        for child in tree.get(link, ()):
            ordered.append(by_parent[child])
            stack.append(child)
    return ordered


def _rotate_about(axis: np.ndarray, angles: np.ndarray) -> np.ndarray:
    """Rodrigues, batched: ``(n,)`` angles about one axis -> ``(n, 3, 3)``."""
    unit = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    skew = np.array([[0.0, -unit[2], unit[1]], [unit[2], 0.0, -unit[0]], [-unit[1], unit[0], 0.0]])
    sin = np.sin(angles)[:, None, None]
    cos = np.cos(angles)[:, None, None]
    return np.eye(3) + sin * skew + (1.0 - cos) * (skew @ skew)


# --------------------------------------------------------------------------- sampling

#: Beyond this many joints the limit box has more corners than it is worth evaluating,
#: and the quasi-random fill carries the search on its own. 2^12 poses is milliseconds.
MAX_CORNERS = 4096

_PRIMES: Final = (2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53)


def sample_configurations(urdf: yourdfpy.URDF, count: int = 2048) -> np.ndarray:
    """Poses to search for the worst case, deterministically.

    Three parts, in order of how likely each is to *be* the worst case:

    * the zero pose, so a report always contains the one everybody pictures;
    * every corner of the joint-limit box, because an arm holds most torque fully
      extended and a purely random sweep of six dimensions can miss that by a lot;
    * a Halton fill for the interior, which is low-discrepancy and needs no seed --
      the same model gives the same worst-case pose on every machine, every run.
    """
    bounds = joint_limits(urdf)
    lower, upper = bounds[0], bounds[1]
    n = len(lower)
    if n == 0:
        return np.zeros((1, 0))

    poses = [np.zeros((1, n))]
    if 2**n <= MAX_CORNERS:
        corners = np.array(np.meshgrid(*([[0.0, 1.0]] * n), indexing="ij")).reshape(n, -1).T
        poses.append(lower + corners * (upper - lower))

    used = sum(len(part) for part in poses)
    if count > used:
        poses.append(lower + _halton(count - used, n) * (upper - lower))
    return np.vstack(poses)


def _halton(count: int, dimensions: int) -> np.ndarray:
    """A Halton sequence in ``[0, 1)``: deterministic, and it fills gaps evenly."""
    if dimensions > len(_PRIMES):
        raise ValueError(f"sampling supports up to {len(_PRIMES)} joints, got {dimensions}")
    points = np.empty((count, dimensions))
    for axis, base in enumerate(_PRIMES[:dimensions]):
        indices = np.arange(1, count + 1, dtype=float)
        value = np.zeros(count)
        fraction = 1.0 / base
        remaining = indices.copy()
        while np.any(remaining >= 1.0):
            value += (remaining % base) * fraction
            remaining = np.floor(remaining / base)
            fraction /= base
        points[:, axis] = value
    return points


def gravity_in_root(
    urdf: yourdfpy.URDF, mount: object = "table", *, override: bool = False
) -> Gravity:
    """:func:`gravity`, expressed in the frame forward kinematics runs in.

    :func:`gravity` answers "which way is down for the robot"; this answers "which
    way is down in the frame every link pose is computed in". They are the same
    vector seen from two frames, and they differ exactly when the model has a world
    frame that the base is rotated relative to.
    """
    base = mounted_link(urdf)
    down = gravity(urdf, mount, override=override)
    if base is None:
        return down
    rotation = np.asarray(urdf.get_transform(base, urdf.base_link), dtype=float)[:3, :3]
    return Gravity(_as_tuple(rotation @ down.array), down.source, down.implied_mount)
