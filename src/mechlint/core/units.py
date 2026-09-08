"""SI inside, explicit at the boundary.

Every number that crosses into `mechlint.core` is metres, kilograms, seconds.
Models authored in other units (DAST-1 is in decimetres) are converted once, at
load time, using a single scale factor — never by sprinkling factors of ten
through the physics.
"""

from __future__ import annotations

from typing import Final

#: Standard gravity, m/s^2.
G: Final = 9.80665

#: Metres per unit, for the length units a CAD export is likely to be in.
#: A mesh authored in millimetres has ``scale = MM``.
MM: Final = 1e-3
CM: Final = 1e-2
DM: Final = 1e-1
M: Final = 1.0
INCH: Final = 0.0254

#: Named scales accepted by ``--model-scale`` and ``robot.model_scale``.
NAMED_SCALES: Final[dict[str, float]] = {
    "mm": MM,
    "millimetre": MM,
    "cm": CM,
    "centimetre": CM,
    "dm": DM,
    "decimetre": DM,
    "m": M,
    "metre": M,
    "in": INCH,
    "inch": INCH,
}

#: A serial manipulator whose total reach falls outside this band, once scaled,
#: is almost certainly modelled in the wrong unit. Drives check U001.
PLAUSIBLE_REACH_M: Final[tuple[float, float]] = (0.05, 3.0)


def resolve_scale(value: float | str) -> float:
    """Metres per model unit, from a number or a unit name ('dm', 'mm', ...)."""
    if isinstance(value, str):
        key = value.strip().lower()
        if key not in NAMED_SCALES:
            known = ", ".join(sorted(NAMED_SCALES))
            raise ValueError(f"unknown unit {value!r}; use a number or one of: {known}")
        return NAMED_SCALES[key]
    scale = float(value)
    if scale <= 0.0:
        raise ValueError(f"model scale must be positive, got {scale}")
    return scale


def g_to_kg(grams: float) -> float:
    return grams * 1e-3


def kg_to_g(kilograms: float) -> float:
    return kilograms * 1e3
