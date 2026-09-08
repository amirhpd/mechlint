from __future__ import annotations

from pathlib import Path

import pytest

FIXTURES = Path(__file__).parent / "fixtures"


@pytest.fixture(scope="session")
def dast1_dir() -> Path:
    """The pinned DAST-1 fixture. See its PROVENANCE.md."""
    return FIXTURES / "dast1"


@pytest.fixture(scope="session")
def dast1_urdf(dast1_dir: Path) -> Path:
    return dast1_dir / "dast1.urdf"


@pytest.fixture(scope="session")
def dast1_config(dast1_dir: Path) -> Path:
    return dast1_dir / "mechlint.yaml"
