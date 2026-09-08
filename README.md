# mechlint

**mechlint reads a robot's CAD and URDF, computes the physics — mass, inertia, joint torque —
checks them against actuator and printing limits, and exposes all of it to an LLM, so the model
answers *"does this servo have enough torque?"* with numbers from a tool, not a guess.**

> **Status: pre-alpha, M0.** The skeleton, the test fixture and the CLI surface exist. The
> commands themselves are being built — each one tells you which milestone it lands in. Nothing
> here is on PyPI yet.

The unit of work is **the joint, not the part**. Every existing CAD-agent tool works on one
part; robot questions need the whole kinematic chain — what is downstream of this joint, how
heavy it is, how far out, and what motor is holding it. That is the gap mechlint fills.

It sits **downstream of CAD and upstream of the simulator**. It creates no geometry and runs no
physics engine.

```
 INPUTS                                 MECHLINT                     OUTPUTS                  USED BY
 ──────                                 ────────                     ───────                  ───────
 geometry   STEP / STL       ──┐                                ┌──► <inertial> blocks    →  URDF, Gazebo, MoveIt
 chain      URDF / xacro     ──┼──►   inertia · urdf-check      ├──► pass/fail report     →  CI (exit code, JSON)
 config     mechlint.yaml    ──┘      torque · drift · render ──┼──► JSON over MCP        →  Claude Code, Cursor, …
            (motors, materials,                                 └──► PNG renders          →  the human
             payload, mounting)
```

## Works with any CAD

mechlint reads files, not CAD sessions, so where the geometry came from is irrelevant: CATIA,
SolidWorks, Fusion, FreeCAD or build123d. The professional-CAD workflow — model in CATIA, export
STL for the printer and the URDF — is the primary one.

| You export | mechlint can do | Cannot do |
|---|---|---|
| STL only | mass, COM, inertia, watertightness, `torque`, mass drift | hole-to-hole `drift` (a mesh has no features) |
| STL **+ STEP** of the same part (recommended) | all of the above, plus exact volume and hole/axis positions from the STEP's cylindrical faces → full `drift` | — |

## Three doors into the same functions

| Who | How | Gets |
|---|---|---|
| A human at a terminal | `mechlint torque --payload-g 100` | A table and an exit code |
| A human in a chat | Natural language; the LLM calls the MCP tools | An explanation that quotes tool numbers |
| CI | `mechlint check` in GitHub Actions or `colcon test` | Red or green on the PR |

The library itself contains **no AI**: same inputs, same numbers, no network, no API key. The
conversation happens outside it, in whatever LLM host you already use. That is what makes the
maths testable without a model, reproducible, and separable — you can always tell which part of
an answer is physics (the tool output) and which is interpretation (the model's prose).
[`docs/llms.md`](docs/llms.md) makes it a rule: every number in the answer must be traceable to
a tool result.

## Commands

| Question | Command | Lands in |
|---|---|---|
| How heavy is each link, and what is its inertia tensor in the link frame? | `mechlint inertia` | M1 |
| Is my URDF physically sane — units, tensors, masses, meshes? | `mechlint urdf-check` | M1 |
| Run everything and fail CI if anything fails. | `mechlint check` | M1 |
| Does servo X hold joint N at the worst pose, with payload P, at voltage V? | `mechlint torque` | M2 |
| Which servos in the database would? | `mechlint actuators` | M2 |
| Does the URDF still match the CAD after I changed a part? | `mechlint drift` | M4 |
| Show me the arm at the worst-case pose. | `mechlint render` | M4 |

Exit codes: `0` all checks passed, `1` a check failed, `2` usage error or unimplemented command.

## Non-goals

mechlint deliberately does **not** do these, and will point you at the tool that does:

- **Generic printability** (walls, overhangs, manifold) — [build123d-mcp](https://github.com/pzfreo/build123d-mcp) does it. It owns the *part*; mechlint owns the *mechanism*. The interop contract is a STEP or STL file on disk.
- **FEA / stress analysis** — out of scope.
- **Motion planning, gripper force, BOM/cost, GUI.**
- **Editing CAD.** mechlint never writes geometry. The one file it generates is an
  `inertials.xacro` that your description includes — your URDF, meshes and config are never
  touched in place.

## Landscape

| Tool | What it does | Relation |
|---|---|---|
| [build123d](https://github.com/gumyr/build123d) | Parametric CAD as Python (OCCT) | Dependency for STEP input, optional extra `[step]` |
| [build123d-mcp](https://github.com/pzfreo/build123d-mcp) | MCP: run build123d code, render, measure, printability, fit | Recommended companion, not wrapped, not required |
| [trimesh](https://github.com/mikedh/trimesh) | Mesh loading, watertightness, volume, inertia | Dependency; the STL path needs nothing else |
| [yourdfpy](https://github.com/clemense/yourdfpy) | URDF parsing with mesh loading | Dependency |
| [xacro](https://pypi.org/project/xacro/) | xacro without a sourced ROS shell | Dependency |
| [Pinocchio](https://github.com/stack-of-tasks/pinocchio) | Rigid-body dynamics (RNEA), collision | Optional extra `[dyn]`, roadmap |
| STEP-to-URDF converters | One-shot export: STEP in, URDF with inertia out | Complementary — they run once, mechlint re-runs on every change and catches the drift |
| `check_urdf` (urdfdom) | Structural URDF validation, no physics | `mechlint urdf-check` is a superset; the IDs are designed to be proposed upstream |

## ROS

The core is **ROS-free** and installs from pip; a separate `mechlint_ros` ament package (M5)
adds `ros2 run`, `ament_index` resolution of `package://`, and a `colcon test` hook. Inputs are
exactly what ROS uses — URDF or xacro, `package://` URIs, mesh `scale`, joint `<limit>`s — and
`package://` resolves via `ament_index` when a ROS shell is sourced, else via
`--package-path description=/path`, so the core never imports `rclpy`.

## Development

```bash
uv sync              # create the venv and install everything, including dev tools
uv run pytest        # the suite
uv run ruff check .  # lint
uv run ruff format . # format
uv run mechlint --help
```

There is no CI workflow yet; run the three commands above before pushing.

Requires Python 3.10–3.14 — the floor is what ROS 2 Humble ships, so mechlint installs on
Humble (3.10), Jazzy (3.12) and Lyrical (3.14) alike. The test suite runs the same whether or
not a ROS setup.bash has been sourced.

`tests/fixtures/dast1/` is a pinned snapshot of
[DAST-1](https://github.com/amirhpd/dast_1), a 3D-printed 6-DOF arm that is wrong in exactly the
ways mechlint exists to catch — see its
[`PROVENANCE.md`](tests/fixtures/dast1/PROVENANCE.md).

## Licence

Apache-2.0.
