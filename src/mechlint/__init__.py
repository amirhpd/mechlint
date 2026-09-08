"""mechlint — lint and CI for robot mechanics.

Reads a robot's CAD and URDF, computes the physics (mass, inertia, joint
torque), checks them against actuator and printing limits, and exposes all of
it to an LLM over MCP.

The library contains no AI: same inputs, same numbers, no network, no API key.
"""

from mechlint.core.geometry import MassProperties, mass_properties

__version__ = "0.0.1"

__all__ = ["MassProperties", "__version__", "mass_properties"]
