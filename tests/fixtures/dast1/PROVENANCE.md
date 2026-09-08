# DAST-1 fixture — provenance

Geometry and robot description copied from [amirhpd/dast_1](https://github.com/amirhpd/dast_1),
the dogfooding target for mechlint. This is a **pinned snapshot**, not a live link: mechlint's
tests assert on numbers computed from these exact bytes, so updating the fixture is a deliberate
act that comes with re-recording the expected values.

| | |
|---|---|
| Source repo | `amirhpd/dast_1` |
| Commit | `2678b1bbb8ff706a2ed6fb36e987f03fcd8a3a38` ("Merge pull request #19 from amirhpd/kinect") |
| Source package | `src/description` |
| Copied on | 2026-09-08 |
| ROS distro used for expansion | Lyrical Luth |
| xacro version | 2.1.1 |

## What is here

```
dast1.urdf                     xacro-expanded, is_sim:=true   <- the main fixture
urdf/description.urdf.xacro    the source xacro, verbatim     <- exercises the xacro input path
urdf/control.xacro             included by the above
meshes/*.stl                   6 visual + 6 collision meshes, verbatim
mechlint.yaml                  the project config from the plan, section 4
```

`dast1.urdf` was produced with:

```bash
xacro src/description/urdf/description.urdf.xacro is_sim:=true > dast1.urdf
```

**One edit was made to the expansion.** `$(find controller)` resolved to an absolute path in the
author's colcon install space; that single `<parameters>` line was rewritten back to
`package://controller/config/controller.yaml` so the fixture is machine-independent. Nothing else
was touched, and mechlint does not read that element.

## Why this robot is a good fixture

It is wrong in useful ways — every one of these is a check mechlint has to catch:

- **The model is in decimetres.** Meshes are authored in millimetres and the xacro scales them by
  `0.01`, so one URDF unit is one decimetre and the arm's 0.759 m reach reads as `7.59`. The
  xacro says so in a comment; nothing in the URDF itself does. → **U001**
- **Every inertia tensor is a placeholder**, `ixx = iyy = izz = 1.0`, from a `default_inertial`
  macro, on all eight links. → **U005**
- **Masses are round guesses**, `{0.2, 0.05, 0.1, 0.01, 0.5}` kg, none measured. → **U006**
- **Every joint declares `effort="20.0"` N·m** while the MG996R behind it stalls near 1.1 N·m. A
  controller tuned against that limit is tuned against fiction. → **U007**
- **Two visual meshes are not closed volumes**, `arm_1.stl` and `base.stl`, which is ordinary for
  professional-CAD STL exports and makes their mass properties unreliable. → **M001**
- `joint_6` and `wrist_link` are a simulation-only placeholder with no servo and no geometry
  behind them, so `actuators.joints.joint_6` is explicitly `null` in `mechlint.yaml`: torque is
  reported for that joint, not judged.

## Baseline measured from these bytes

trimesh 5.1.0, visual meshes, solid PLA at 1.24 g/cm³ — an upper bound on the printed shell:

| Mesh | Watertight | Volume | Mass (solid PLA) | Extents (mm) |
|---|---|---|---|---|
| `arm_1.stl` | no | — | — | 208.5 × 26.0 × 74.0 |
| `arm_2.stl` | yes | 104.535 cm³ | 129.6 g | 248.5 × 26.0 × 74.0 |
| `arm_3.stl` | yes | 96.725 cm³ | 119.9 g | 127.5 × 61.5 × 52.5 |
| `base.stl` | no | — | — | 80.0 × 80.0 × 81.5 |
| `gripper_3.stl` | yes | 35.692 cm³ | 44.3 g | 67.8 × 156.3 × 78.3 |
| `rotary.stl` | yes | 32.012 cm³ | 39.7 g | 26.0 × 74.0 × 64.0 |

These exact values are asserted in `tests/integration/test_fixture_dast1.py`; the plan's
section 9 quotes them rounded.

Forward kinematics at all-zero joints, in `base_link`, URDF units (× 0.1 for metres):

```
rotary_link  [0, 0, 1.47]     arm_3_link   [0, 0, 4.96]
arm_1_link   [0, 0, 1.47]     wrist_link   [0, 0, 5.84]
arm_2_link   [0, 0, 3.015]    gripper_link [0, 0, 5.84]
                              tip          [0, 0, 7.59]
```

## Checksums

```
6686cdb955d4cef2e1f3bc82a65d1cac93050a141c85ea238c32b84a80badaee  meshes/arm_1_collision.stl
479a6debca6d7e382bb497f7088590b2e8bfb9ad908b36f5b179758ac954e242  meshes/arm_1.stl
4848215c4e3a9920b83d479e199fae0c66e12135293dbcc45b087029644d3037  meshes/arm_2_collision.stl
ff1b66a7040e176d2b81063f2e046eb0b209a9d4c8fe5d95d5c368033fcd5fb2  meshes/arm_2.stl
b50f92f8d92fc57b090b37bb1d883e6b503acb7ffe337f99ef2fa35ae4f9005c  meshes/arm_3_collision.stl
3dca12a901d504a14a16c87a37e70fe66772fdf77286a1d4e5ec33817c7af79b  meshes/arm_3.stl
05c3b07fa70316a0807e66f5c4da9ad91c5ced1dce550adbb9272e715e9a54e8  meshes/base_collision.stl
b2a3c4188a066ac48ed28c034a48fa9b480e02d50c48d0dfcab64774a84cd9c5  meshes/base.stl
21758be188519dbd2f97b91b086f9c8dbcc765235078b083b35cd17cc431537f  meshes/gripper_3_collision.stl
ca1007ce440a87ed3367eae41b15c2e55dac364c58e17ee5ccf8d6807b19a088  meshes/gripper_3.stl
47ad58d8eba4248bbf97c0d440299c184c976663ad53e66dc394dcca172bc843  meshes/rotary_collision.stl
5d093facc9a6be638db7a204e77c6260935ac6ca7ae6d8e2fedd276a99b083ef  meshes/rotary.stl
0c9fee0dd7b92711d8410e8fab5a16a031fd3d1400bc2bd22c0491352fb5373b  dast1.urdf
27189a04a7351268e16086c1616eaf11305b33919ba1358bcbb6d585f72139c8  urdf/control.xacro
a25605865c933131a415202cbb20c81b6c3fc88ab756973ddc46ac160b17ffa6  urdf/description.urdf.xacro
```

## Refreshing this fixture

0. Source ROS, which `dast_1` needs for `$(find ...)` but mechlint itself never does:
   `source /opt/ros/<distro>/setup.bash && source install/setup.bash` from the dast_1 root.
1. Re-copy the meshes and xacro from the dast_1 commit you want to pin.
2. Re-run the `xacro` command above and re-apply the `package://controller/...` substitution.
3. Re-run `pytest tests/` and update the recorded numbers that legitimately changed.
4. Update the commit hash, date and checksums in this file. Never do 3 without 4.

## Licence

DAST-1 is Apache-2.0, the same licence as mechlint. See `LICENSE` in the dast_1 repository.
