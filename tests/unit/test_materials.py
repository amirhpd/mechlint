"""The bundled density table. Every entry has to carry where its number came from."""

from __future__ import annotations

import pytest

from mechlint.core.materials import MaterialDatabase, bundled, density_of


def test_the_bundled_table_ships_with_the_package() -> None:
    """It lives inside mechlint/data/, so it is there in an installed wheel too."""
    assert density_of("pla") == pytest.approx(1240.0)
    assert density_of("steel_mild") > density_of("aluminium_6061") > density_of("pla")


@pytest.mark.parametrize("key", sorted(bundled().materials))
def test_every_entry_is_sourced(key: str) -> None:
    entry = bundled()[key]

    assert entry.source.strip(), f"{key} has no source"
    assert entry.density_kg_m3 > 0


def test_an_unknown_material_says_what_it_knows() -> None:
    with pytest.raises(KeyError, match="unknown material 'unobtainium'"):
        density_of("unobtainium")


def test_a_project_table_overrides_the_bundled_one(tmp_path) -> None:
    extra = tmp_path / "materials.yaml"
    extra.write_text(
        "materials:\n"
        "  pla: { name: 'PLA, weighed', density_kg_m3: 1200, source: 'kitchen scale' }\n"
        "  nylon: { name: Nylon, density_kg_m3: 1140, source: datasheet }\n"
    )
    merged = bundled().merged_with(MaterialDatabase.from_yaml(extra))

    assert density_of("pla", database=merged) == pytest.approx(1200.0)
    assert density_of("nylon", database=merged) == pytest.approx(1140.0)
    assert density_of("petg", database=merged) == pytest.approx(1270.0)


def test_a_table_without_a_materials_key_is_rejected(tmp_path) -> None:
    path = tmp_path / "materials.yaml"
    path.write_text("pla: {density_kg_m3: 1240}\n")

    with pytest.raises(ValueError, match="top-level 'materials' mapping"):
        MaterialDatabase.from_yaml(path)
