"""Density table: what a printed or machined part is made of.

Densities ship with mechlint in ``data/materials.yaml``, every entry with the
source it came from. A project extends the table with its own YAML rather than
editing the bundled file.

Infill is deliberately *not* a material property: it is a slicer setting, so it
lives per link in ``mechlint.yaml`` and is applied by :mod:`mechlint.core.inertia`.
"""

from __future__ import annotations

from functools import lru_cache
from importlib import resources
from pathlib import Path
from typing import Any, Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field

Confidence = Literal["high", "medium", "low"]


class Material(BaseModel):
    """One row of the density table."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    key: str
    name: str
    density_kg_m3: float = Field(gt=0.0)
    source: str = Field(description="Where the number came from. Every entry needs one.")
    confidence: Confidence = "medium"


class MaterialDatabase(BaseModel):
    """The whole table, keyed by the short name used in ``mechlint.yaml``."""

    model_config = ConfigDict(frozen=True)

    materials: dict[str, Material]

    @classmethod
    def from_yaml(cls, path: str | Path) -> MaterialDatabase:
        return cls._from_text(Path(path).read_text(), source=str(path))

    @classmethod
    def _from_text(cls, text: str, *, source: str) -> MaterialDatabase:
        raw: Any = yaml.safe_load(text) or {}
        entries = raw.get("materials")
        if not isinstance(entries, dict):
            raise ValueError(f"{source} must have a top-level 'materials' mapping")
        return cls(materials={key: Material(key=key, **value) for key, value in entries.items()})

    def __getitem__(self, key: str) -> Material:
        try:
            return self.materials[key]
        except KeyError:
            known = ", ".join(sorted(self.materials))
            raise KeyError(f"unknown material {key!r}; known materials: {known}") from None

    def __contains__(self, key: str) -> bool:
        return key in self.materials

    def merged_with(self, other: MaterialDatabase) -> MaterialDatabase:
        """This table plus another one, the other winning on shared keys."""
        return MaterialDatabase(materials={**self.materials, **other.materials})


@lru_cache(maxsize=1)
def bundled() -> MaterialDatabase:
    """The table that ships with mechlint."""
    text = (resources.files("mechlint.data") / "materials.yaml").read_text()
    return MaterialDatabase._from_text(text, source="mechlint/data/materials.yaml")


def density_of(material: str, *, database: MaterialDatabase | None = None) -> float:
    """Density in kg/m^3, from the bundled table unless another one is given."""
    return (database or bundled())[material].density_kg_m3
