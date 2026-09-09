"""Loading a robot description without needing a sourced ROS shell.

Two things stand between a ``.urdf.xacro`` on disk and a kinematic chain in
memory, and both are ROS conventions rather than XML: ``$(find pkg)`` inside
xacro, and ``package://pkg/...`` inside ``<mesh filename>``. mechlint resolves
both through one :class:`PackageResolver` -- from ``robot.package_paths`` in
``mechlint.yaml`` first, and from ``ament_index`` only if a ROS shell happens
to be sourced. That order is what keeps the core ROS-free: it works without
ROS, and it does not fight ROS when ROS is there.

Everything here is read-only. mechlint never modifies the description it loads.
"""

from __future__ import annotations

from collections.abc import Iterator, Mapping
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import trimesh
import yourdfpy
from pydantic import BaseModel, ConfigDict, Field

from mechlint.core import chain
from mechlint.core.geometry import Vec3, load_mesh

XACRO_SUFFIXES = (".xacro",)


class DescriptionError(RuntimeError):
    """A description could not be read. Carries a hint about what to do."""

    def __init__(self, message: str, hint: str = "") -> None:
        super().__init__(message if not hint else f"{message}\n  hint: {hint}")
        self.message = message
        self.hint = hint


class PackageResolver:
    """Turns ROS package names into directories.

    ``package_paths`` wins over ``ament_index`` so that a project can pin a
    checkout without unsetting its environment; ``ament_index`` is tried only
    as a fallback, and its absence is never an error by itself.
    """

    def __init__(self, package_paths: Mapping[str, Path] | None = None) -> None:
        self.package_paths = {name: Path(path) for name, path in (package_paths or {}).items()}

    def share(self, package: str) -> Path:
        """The directory ``package://<package>/`` and ``$(find <package>)`` refer to."""
        explicit = self.package_paths.get(package)
        if explicit is not None:
            return explicit
        found = self._from_ament(package)
        if found is not None:
            return found
        known = ", ".join(sorted(self.package_paths)) or "none"
        raise DescriptionError(
            f"cannot resolve package {package!r}",
            f"add it to robot.package_paths in mechlint.yaml, or pass "
            f"--package-path {package}=<dir>. Known packages: {known}. "
            "Sourcing a ROS workspace would also work.",
        )

    @staticmethod
    def _from_ament(package: str) -> Path | None:
        try:
            from ament_index_python.packages import get_package_share_directory
        except ImportError:
            return None
        try:
            return Path(get_package_share_directory(package))
        except Exception:  # PackageNotFoundError, and whatever else ament raises
            return None

    def resolve_uri(self, uri: str, *, relative_to: Path) -> Path:
        """A ``<mesh filename>`` value as a path on this machine."""
        if uri.startswith("package://"):
            package, _, rest = uri.removeprefix("package://").partition("/")
            return self.share(package) / rest
        if uri.startswith("file://"):
            return Path(uri.removeprefix("file://"))
        path = Path(uri)
        return path if path.is_absolute() else relative_to / path

    def filename_handler(self, relative_to: Path):
        """A handler for :meth:`yourdfpy.URDF.load`, which calls it as ``f(fname=...)``."""

        def handler(fname: str) -> str:
            return str(self.resolve_uri(fname, relative_to=relative_to))

        return handler


@contextmanager
def _xacro_finds_packages(resolver: PackageResolver) -> Iterator[list[DescriptionError]]:
    """Teach ``$(find pkg)`` about ``package_paths`` for the duration of one expansion.

    xacro hard-codes ``ament_index_python`` for ``$(find)``, which raises
    ImportError outside a ROS install. The substitution table is module-level
    state, so it is patched in place and restored -- two entries, because
    ``$(find x)`` and ``$(eval find('x'))`` read from different ones.

    Yields the list an unresolvable package lands in. xacro rewraps whatever a
    substitution raises into its own exception, which would bury the one message
    the user can act on, so the original is kept aside and re-raised instead.
    """
    from xacro import substitution_args

    unresolved: list[DescriptionError] = []

    def find(package: str) -> str:
        try:
            return str(resolver.share(package))
        except DescriptionError as error:
            unresolved.append(error)
            raise

    original_fn = substitution_args._eval_find
    original_entry = substitution_args._eval_dict["find"]
    substitution_args._eval_find = find
    substitution_args._eval_dict["find"] = find
    try:
        yield unresolved
    finally:
        substitution_args._eval_find = original_fn
        substitution_args._eval_dict["find"] = original_entry


def expand_xacro(
    path: Path,
    *,
    xacro_args: Mapping[str, str] | None = None,
    resolver: PackageResolver | None = None,
) -> str:
    """Run xacro over a file and return the URDF XML it produces."""
    import xacro

    resolver = resolver or PackageResolver()
    unresolved: list[DescriptionError] = []
    try:
        with _xacro_finds_packages(resolver) as unresolved:
            document = xacro.process_file(str(path), mappings=dict(xacro_args or {}))
        return str(document.toxml())
    except DescriptionError:
        raise
    except Exception as error:
        if unresolved:
            raise unresolved[0] from error
        raise DescriptionError(
            f"xacro failed on {path}: {error}",
            "check robot.xacro_args -- an unset $(arg ...) fails here the same way it would "
            "in a launch file.",
        ) from error


def is_xacro(path: Path) -> bool:
    return path.suffix in XACRO_SUFFIXES


def load_description(
    path: str | Path,
    *,
    xacro_args: Mapping[str, str] | None = None,
    resolver: PackageResolver | None = None,
) -> yourdfpy.URDF:
    """Load a ``.urdf`` or ``.urdf.xacro`` into a :class:`yourdfpy.URDF`.

    Meshes are *not* loaded here. yourdfpy would read every STL just to build a
    scene, and mechlint reads them itself, once, with the scale it needs.
    """
    path = Path(path)
    if not path.is_file():
        raise DescriptionError(
            f"description not found: {path}",
            "robot.description in mechlint.yaml is relative to that file.",
        )
    resolver = resolver or PackageResolver()
    handler = resolver.filename_handler(path.parent)

    try:
        if is_xacro(path):
            return yourdfpy.URDF.load(
                _StringPath(expand_xacro(path, xacro_args=xacro_args, resolver=resolver)),
                filename_handler=handler,
                load_meshes=False,
                build_scene_graph=True,
            )
        return yourdfpy.URDF.load(
            str(path),
            filename_handler=handler,
            load_meshes=False,
            build_scene_graph=True,
        )
    except DescriptionError:
        raise
    except Exception as error:
        raise DescriptionError(f"could not parse {path}: {error}") from error


class _StringPath:
    """XML text where yourdfpy expects a file. It accepts any file-like object."""

    def __init__(self, xml: str) -> None:
        from io import BytesIO

        self._buffer = BytesIO(xml.encode())

    def read(self, *args: object) -> bytes:
        return self._buffer.read(*args)  # type: ignore[arg-type]


@dataclass(frozen=True)
class LinkGeometry:
    """One ``<visual>`` or ``<collision>`` body, already placed in its link's frame.

    ``mesh`` is in URDF units, not metres: the model scale is applied once,
    later, where mass properties are computed.
    """

    link: str
    kind: str
    source: str
    mesh: trimesh.Trimesh
    watertight: bool


def _primitive(geometry: yourdfpy.Geometry) -> tuple[str, str, trimesh.Trimesh] | None:
    """A URDF primitive as a trimesh. Primitives carry analytic volume and inertia."""
    if geometry.box is not None:
        return (
            "box",
            f"box {tuple(geometry.box.size)}",
            trimesh.primitives.Box(extents=np.asarray(geometry.box.size, dtype=float)),
        )
    if geometry.cylinder is not None:
        return (
            "cylinder",
            "cylinder",
            trimesh.primitives.Cylinder(
                radius=float(geometry.cylinder.radius), height=float(geometry.cylinder.length)
            ),
        )
    if geometry.sphere is not None:
        return "sphere", "sphere", trimesh.primitives.Sphere(radius=float(geometry.sphere.radius))
    return None


class RobotModel:
    """A loaded description, plus the two things the XML does not say.

    Those two are where its packages live (``resolver``) and how big one URDF
    length unit is (``model_scale``, metres per unit). Both come from
    ``mechlint.yaml`` or the command line -- never guessed.
    """

    def __init__(
        self,
        urdf: yourdfpy.URDF,
        *,
        path: Path,
        resolver: PackageResolver | None = None,
        model_scale: float = 1.0,
    ) -> None:
        self.urdf = urdf
        self.path = path
        self.resolver = resolver or PackageResolver()
        self.model_scale = model_scale
        self._geometry_cache: dict[tuple[str, str], list[LinkGeometry]] = {}

    @classmethod
    def load(
        cls,
        path: str | Path,
        *,
        xacro_args: Mapping[str, str] | None = None,
        resolver: PackageResolver | None = None,
        model_scale: float = 1.0,
    ) -> RobotModel:
        path = Path(path)
        urdf = load_description(path, xacro_args=xacro_args, resolver=resolver)
        return cls(urdf, path=path, resolver=resolver, model_scale=model_scale)

    @property
    def name(self) -> str:
        return str(self.urdf.robot.name)

    @property
    def link_names(self) -> list[str]:
        return [link.name for link in self.urdf.robot.links]

    def geometries(self, link_name: str, *, which: str = "visual") -> list[LinkGeometry]:
        """The bodies of one link, in the link frame, in URDF units.

        ``which`` is ``"visual"``, ``"collision"``, or ``"visual-or-collision"``
        -- the last being what mass properties use, because a visual mesh is
        the real shape and a collision mesh is usually a coarsened stand-in.

        Results are cached, because several checks ask for the same link and
        reading an STL four times is four times too many. The meshes handed
        back are shared: read them, do not modify them in place.
        """
        link = self.urdf.link_map.get(link_name)
        if link is None:
            raise KeyError(f"no link named {link_name!r}")

        cached = self._geometry_cache.get((link_name, which))
        if cached is not None:
            return cached

        if which == "visual-or-collision":
            bodies = self.geometries(link_name, which="visual") or self.geometries(
                link_name, which="collision"
            )
            self._geometry_cache[(link_name, which)] = bodies
            return bodies
        elements = link.visuals if which == "visual" else link.collisions

        bodies: list[LinkGeometry] = []
        for element in elements:
            built = self._build(link_name, element)
            if built is not None:
                bodies.append(built)
        self._geometry_cache[(link_name, which)] = bodies
        return bodies

    def _build(self, link_name: str, element: object) -> LinkGeometry | None:
        geometry = getattr(element, "geometry", None)
        if geometry is None:
            return None

        primitive = _primitive(geometry)
        if primitive is not None:
            kind, source, mesh = primitive
            mesh = trimesh.Trimesh(vertices=mesh.vertices.copy(), faces=mesh.faces.copy())
        elif geometry.mesh is not None:
            path = self.resolver.resolve_uri(geometry.mesh.filename, relative_to=self.path.parent)
            kind, source = "mesh", str(path)
            mesh = load_mesh(path)
            if geometry.mesh.scale is not None:
                mesh.apply_scale(np.asarray(geometry.mesh.scale, dtype=float))
        else:
            return None

        watertight = bool(mesh.is_watertight)
        origin = getattr(element, "origin", None)
        if origin is not None:
            mesh.apply_transform(np.asarray(origin, dtype=float))
        return LinkGeometry(
            link=link_name, kind=kind, source=source, mesh=mesh, watertight=watertight
        )

    def extents(self, link_name: str) -> tuple[float, float, float] | None:
        """Bounding box of everything the link carries, in URDF units, or None."""
        bodies = self.geometries(link_name, which="collision") or self.geometries(
            link_name, which="visual"
        )
        if not bodies:
            return None
        corners = np.vstack([body.mesh.bounds for body in bodies])
        size = corners.max(axis=0) - corners.min(axis=0)
        return (float(size[0]), float(size[1]), float(size[2]))

    def declared_inertial(self, link_name: str) -> yourdfpy.Inertial | None:
        """The ``<inertial>`` already in the URDF, if the link has one."""
        link = self.urdf.link_map.get(link_name)
        return None if link is None else link.inertial

    def summary(self) -> RobotSummary:
        """The chain as data -- what an LLM should read before asking for physics."""
        reach_units, reach_link = chain.reach(self.urdf)
        moving = chain.moving_links(self.urdf)
        return RobotSummary(
            robot=self.name,
            source=self.path,
            model_scale=self.model_scale,
            reach_m=reach_units * self.model_scale,
            reach_link=reach_link,
            actuated_joints=list(self.urdf.actuated_joint_names),
            links=[self._link_summary(name, name in moving) for name in self.link_names],
            joints=[_joint_summary(joint) for joint in self.urdf.robot.joints],
        )

    def _link_summary(self, name: str, moves: bool) -> LinkSummary:
        bodies = self.geometries(name, which="visual-or-collision")
        inertial = self.declared_inertial(name)
        return LinkSummary(
            name=name,
            moves=moves,
            geometry=[body.kind for body in bodies],
            watertight=all(body.watertight for body in bodies) if bodies else None,
            declared_mass_kg=None
            if inertial is None or inertial.mass is None
            else float(inertial.mass),
            has_inertial=inertial is not None,
        )


def _joint_summary(joint: yourdfpy.Joint) -> JointSummary:
    limit = joint.limit
    origin = None if joint.origin is None else np.asarray(joint.origin, dtype=float)[:3, 3]
    return JointSummary(
        name=joint.name,
        type=joint.type,
        parent=joint.parent,
        child=joint.child,
        axis=None if joint.axis is None else _vec3(joint.axis),
        origin_xyz=None if origin is None else _vec3(origin),
        lower=None if limit is None else limit.lower,
        upper=None if limit is None else limit.upper,
        effort=None if limit is None else limit.effort,
        velocity=None if limit is None else limit.velocity,
    )


def _vec3(values: object) -> Vec3:
    a = np.asarray(values, dtype=float).reshape(3)
    return (float(a[0]), float(a[1]), float(a[2]))


class JointSummary(BaseModel):
    """One joint, as an LLM needs to see it: what it connects and how far it turns."""

    model_config = ConfigDict(frozen=True)

    name: str
    type: str
    parent: str
    child: str
    axis: Vec3 | None = None
    origin_xyz: Vec3 | None = Field(default=None, description="In the parent frame, URDF units.")
    lower: float | None = None
    upper: float | None = None
    effort: float | None = Field(default=None, description="<limit effort>, N*m. Check U007.")
    velocity: float | None = None


class LinkSummary(BaseModel):
    """One link: whether mechlint has something to weigh, and what the URDF claims."""

    model_config = ConfigDict(frozen=True)

    name: str
    moves: bool = Field(description="Whether at least one joint changes this link's pose.")
    geometry: list[str] = Field(default_factory=list, description="Kinds: mesh, box, ...")
    watertight: bool | None = None
    declared_mass_kg: float | None = None
    has_inertial: bool = False


class RobotSummary(BaseModel):
    """What ``inspect_robot`` answers: the chain, before any physics is computed."""

    robot: str
    source: Path
    model_scale: float = Field(description="Metres per URDF unit.")
    reach_m: float
    reach_link: str
    actuated_joints: list[str] = Field(default_factory=list)
    links: list[LinkSummary] = Field(default_factory=list)
    joints: list[JointSummary] = Field(default_factory=list)
