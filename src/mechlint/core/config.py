"""Schema for ``mechlint.yaml`` — the facts that are in neither the CAD nor the URDF.

Motors, materials, measured weights, payload and how the robot is mounted live
here, next to the robot description, the way ``package.xml`` sits next to a ROS
package. Nothing is ever asked interactively: what an LLM learns from you in
conversation it *writes into this file*, so the knowledge is versioned and
reviewable instead of trapped in a chat.

Unknown keys are rejected rather than ignored — a typo in a config that decides
whether a servo is strong enough should fail loudly.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, PrivateAttr, model_validator

Vec3 = tuple[float, float, float]

#: Only the mountings a single name can actually pin down. A wall mount leaves gravity
#: free to point any horizontal direction in the base frame, depending on how the robot
#: is rolled about its mounting axis, so it is spelled as an explicit vector -- or, far
#: better, left to the world-to-base rotation in the model, which already says it.
Mount = Literal["table", "ceiling"]


class _Strict(BaseModel):
    model_config = ConfigDict(extra="forbid")


class RobotConfig(_Strict):
    """Where the description is and how to read it."""

    description: Path = Field(description="Path to the URDF or xacro, relative to this file.")
    xacro_args: dict[str, str] = Field(
        default_factory=dict, description="Arguments passed to xacro, e.g. {is_sim: 'true'}."
    )
    package_paths: dict[str, Path] = Field(
        default_factory=dict,
        description="Resolves package:// without a sourced ROS shell: {package: path}.",
    )
    model_scale: float | str = Field(
        default=1.0,
        description="Metres per URDF unit, or a unit name ('dm'). DAST-1 is modelled in "
        "decimetres, so 0.1. Check U001 flags a model whose reach looks implausible.",
    )


class MaterialSpec(_Strict):
    """How a link is made, or what it actually weighed."""

    material: str | None = Field(default=None, description="Key into the material density table.")
    infill: float | None = Field(
        default=None, ge=0.0, le=1.0, description="Printed infill fraction, 0..1."
    )
    measured_mass_g: float | None = Field(
        default=None,
        gt=0.0,
        description="Mass from a scale. Wins over the estimate; the report says which was used.",
    )


class MaterialsConfig(_Strict):
    default: MaterialSpec = Field(default_factory=lambda: MaterialSpec(material="pla", infill=1.0))
    links: dict[str, MaterialSpec] = Field(default_factory=dict)


class Component(_Strict):
    """Mass that is not in the mesh — a servo, a battery, a gripper motor.

    An ``actuator`` is looked up in the actuator database for its mass. It sits
    at the joint it ``drives``, in the parent link, unless ``at`` says otherwise.
    """

    actuator: str | None = None
    drives: str | None = Field(default=None, description="Joint this actuator drives.")
    mass_g: float | None = Field(default=None, gt=0.0)
    at: Vec3 | None = Field(default=None, description="Position in the link frame, URDF units.")
    note: str | None = None

    @model_validator(mode="after")
    def _needs_a_mass_source(self) -> Component:
        if self.actuator is None and self.mass_g is None:
            raise ValueError("a component needs either 'actuator' (mass from the db) or 'mass_g'")
        if self.actuator is None and self.drives is not None:
            raise ValueError("'drives' names the joint an actuator turns; it needs 'actuator' too")
        if self.actuator is not None and self.drives is None and self.at is None:
            raise ValueError(
                f"actuator {self.actuator!r} needs 'drives' (a joint) or 'at' (a position) "
                "so mechlint knows where to place its mass"
            )
        return self


class ActuatorsConfig(_Strict):
    voltage: float | None = Field(
        default=None, gt=0.0, description="Supply voltage; stall torque is quoted per voltage."
    )
    joints: dict[str, str | None] = Field(
        default_factory=dict,
        description="Actuator per joint. An explicit null means 'no servo yet' — torque is "
        "reported for that joint but not judged.",
    )


class Scenario(_Strict):
    """The load case every torque number is computed for."""

    mount: Mount | Vec3 = Field(
        default="table",
        description="Named mounting, or an explicit gravity vector in the base frame. Only "
        "consulted when the model is not grounded: a URDF with a world frame already states "
        "its mounting in the world-to-base rotation, and check D002 reports a disagreement.",
    )
    payload_g: float = Field(default=0.0, ge=0.0, description="Mass held at the tip.")
    safety_factor: float = Field(
        default=2.0,
        gt=0.0,
        description="Required margin on stall torque. A convention, not physics — every report "
        "prints it so nobody mistakes it for one.",
    )


class MechlintConfig(_Strict):
    """A whole ``mechlint.yaml``."""

    robot: RobotConfig
    actuator_db: Path | None = Field(
        default=None,
        description="This project's own actuator YAML, merged over the bundled table. "
        "Relative to this file. Put a servo mechlint has never heard of here.",
    )
    materials: MaterialsConfig = Field(default_factory=MaterialsConfig)
    components: dict[str, list[Component]] = Field(default_factory=dict)
    actuators: ActuatorsConfig = Field(default_factory=ActuatorsConfig)
    scenario: Scenario = Field(default_factory=Scenario)

    _source_path: Path | None = PrivateAttr(default=None)

    @classmethod
    def from_yaml(cls, path: str | Path) -> MechlintConfig:
        path = Path(path)
        if not path.is_file():
            raise FileNotFoundError(f"config not found: {path}")
        raw: Any = yaml.safe_load(path.read_text()) or {}
        if not isinstance(raw, dict):
            raise ValueError(f"{path} must contain a YAML mapping, got {type(raw).__name__}")
        config = cls.model_validate(raw)
        config._source_path = path.resolve()
        return config

    @property
    def source_path(self) -> Path | None:
        """The file this config was read from, if any."""
        return self._source_path

    @property
    def base_dir(self) -> Path:
        """Directory that relative paths in this config are resolved against."""
        return self._source_path.parent if self._source_path else Path.cwd()

    def resolve(self, path: str | Path) -> Path:
        """Make a config-relative path absolute."""
        candidate = Path(path)
        return candidate if candidate.is_absolute() else (self.base_dir / candidate).resolve()

    @property
    def description_path(self) -> Path:
        return self.resolve(self.robot.description)

    @property
    def actuator_db_path(self) -> Path | None:
        return None if self.actuator_db is None else self.resolve(self.actuator_db)
