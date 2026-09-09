"""A URDF from a string, for tests that need a chain with exactly one thing wrong."""

from __future__ import annotations

from pathlib import Path

TEMPLATE = '<?xml version="1.0"?>\n<robot name="{name}">{body}</robot>\n'


def urdf_text(body: str, *, name: str = "test") -> str:
    return TEMPLATE.format(name=name, body=body)


def write_urdf(directory: Path, body: str, *, name: str = "test") -> Path:
    path = directory / f"{name}.urdf"
    path.write_text(urdf_text(body, name=name))
    return path


def inertial(mass: float, diagonal: tuple[float, float, float], xyz: str = "0 0 0") -> str:
    ixx, iyy, izz = diagonal
    return (
        f'<inertial><origin xyz="{xyz}" rpy="0 0 0"/><mass value="{mass}"/>'
        f'<inertia ixx="{ixx}" ixy="0" ixz="0" iyy="{iyy}" iyz="0" izz="{izz}"/></inertial>'
    )


def box(size: str) -> str:
    return f'<visual><geometry><box size="{size}"/></geometry></visual>'
