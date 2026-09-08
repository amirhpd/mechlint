"""The mechlint.yaml schema, checked against the DAST-1 example from the plan."""

from __future__ import annotations

from pathlib import Path

import pytest
from pydantic import ValidationError

from mechlint.core.config import Component, MechlintConfig


def test_the_dast1_example_parses(dast1_config: Path) -> None:
    cfg = MechlintConfig.from_yaml(dast1_config)

    assert cfg.robot.description == Path("urdf/description.urdf.xacro")
    assert cfg.robot.xacro_args == {"is_sim": "true"}
    assert cfg.robot.package_paths == {"description": Path(".")}
    assert cfg.robot.model_scale == pytest.approx(0.1)

    assert cfg.materials.default.material == "pla"
    assert cfg.materials.default.infill == pytest.approx(0.30)
    assert cfg.materials.links["base_link"].measured_mass_g == pytest.approx(210)

    assert cfg.actuators.voltage == pytest.approx(6.0)
    assert cfg.actuators.joints["joint_2"] == "mg996r"
    # An explicit null means "no servo yet": reported, not judged.
    assert cfg.actuators.joints["joint_6"] is None
    assert "joint_6" in cfg.actuators.joints

    assert cfg.scenario.mount == "table"
    assert cfg.scenario.payload_g == pytest.approx(100)
    assert cfg.scenario.safety_factor == pytest.approx(2.0)

    assert cfg.components["arm_2_link"][0].actuator == "mg996r"
    assert cfg.components["arm_2_link"][0].drives == "joint_4"
    gripper = cfg.components["gripper_link"][0]
    assert gripper.mass_g == pytest.approx(30)
    assert gripper.at == pytest.approx((0.0, 0.0, 0.05))


def test_paths_resolve_against_the_config_file(dast1_config: Path) -> None:
    cfg = MechlintConfig.from_yaml(dast1_config)

    assert cfg.source_path == dast1_config.resolve()
    assert cfg.description_path == (dast1_config.parent / "urdf/description.urdf.xacro").resolve()
    assert cfg.description_path.is_file()


def test_a_typo_is_rejected_not_ignored(tmp_path: Path) -> None:
    """Hard rules, not prompts: a misspelt key in a config that decides whether a
    servo is strong enough must fail loudly."""
    bad = tmp_path / "mechlint.yaml"
    bad.write_text("robot:\n  description: r.urdf\nscenario:\n  payload_gram: 100\n")

    with pytest.raises(ValidationError, match="payload_gram"):
        MechlintConfig.from_yaml(bad)


def test_a_component_needs_a_mass_source() -> None:
    with pytest.raises(ValidationError, match="either 'actuator'"):
        Component(note="a mystery lump")


def test_an_actuator_needs_somewhere_to_sit() -> None:
    with pytest.raises(ValidationError, match="needs 'drives'"):
        Component(actuator="mg996r")

    assert Component(actuator="mg996r", drives="joint_2").drives == "joint_2"
    assert Component(actuator="mg996r", at=(0.0, 0.0, 0.1)).at == (0.0, 0.0, 0.1)


def test_infill_is_a_fraction() -> None:
    with pytest.raises(ValidationError):
        MechlintConfig.model_validate(
            {"robot": {"description": "r.urdf"}, "materials": {"default": {"infill": 30}}}
        )


def test_mount_takes_a_name_or_an_explicit_gravity_vector() -> None:
    base = {"robot": {"description": "r.urdf"}}

    assert MechlintConfig.model_validate(base).scenario.mount == "table"
    assert (
        MechlintConfig.model_validate({**base, "scenario": {"mount": "ceiling"}}).scenario.mount
        == "ceiling"
    )
    explicit = MechlintConfig.model_validate({**base, "scenario": {"mount": [0, 9.81, 0]}})
    assert explicit.scenario.mount == (0.0, 9.81, 0.0)

    with pytest.raises(ValidationError):
        MechlintConfig.model_validate({**base, "scenario": {"mount": "underwater"}})


def test_missing_config_names_the_file(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError, match="mechlint.yaml"):
        MechlintConfig.from_yaml(tmp_path / "mechlint.yaml")
