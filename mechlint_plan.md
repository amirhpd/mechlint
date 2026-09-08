# mechlint — project plan

*Written 2026-09-06. Status: plan, nothing built yet. Lives in the DAST-1 repo because DAST-1
is the dogfooding target; the project itself gets its own repo.*

## 0. Decisions already taken

| Decision | Choice | Why |
|---|---|---|
| Name | **mechlint** | Free on PyPI and GitHub (checked 2026-09-06). Says what it is: lint/CI for robot mechanics. |
| First interface | **Library + CLI first; MCP as a thin layer** | Testable without an LLM, runs in CI, Claude Code can call the CLI via Bash from day one. |
| DAST-1 CAD source | **Existing STLs + URDF now; re-model only the parts the 6-DOF upgrade touches, in build123d** | First numbers today; the parametric loop arrives with the parts that actually change. |
| ROS depth | **ROS-free pip core + separate `mechlint_ros` ament package** | Hobbyists without ROS can use it; ROS users get `ros2 run` and a `colcon test` hook. |
| License | Apache-2.0 (assumed, easy to change) | ROS-ecosystem norm; same as build123d-mcp and agentcad. |
| Repo | `~/projects/mechlint`, GitHub `amirhpd/mechlint` | Sibling of dast_1 and NeuraPlatform. |
| Python | 3.11 – 3.14 | `cadquery-ocp` 8.0.1 ships cp314 wheels; this machine runs 3.14.4. |

## 1. What it is

**One sentence.** mechlint reads a robot's CAD and URDF, computes the physics (mass, inertia,
joint torque), checks them against actuator and printing limits, and exposes all of it to an
LLM — so the model answers *"does this servo have enough torque?"* with numbers from a tool, not
a guess.

**The unit of work is the joint, not the part.** Every existing CAD-agent tool works on one
part. Robot questions need the whole kinematic chain: what is downstream of this joint, how
heavy is it, how far out is it, what motor is holding it. That is the gap mechlint fills.

**Positioning: downstream of CAD, upstream of the simulator.** mechlint creates no geometry
and runs no physics engine. It sits between the tools that *make parts* and the tools that
*move the robot*, and turns the output of the first into numbers the second — and an LLM — can
trust.

```
 INPUTS                                 MECHLINT                     OUTPUTS                  USED BY
 ──────                                 ────────                     ───────                  ───────
 geometry   STEP / STL       ──┐                                ┌──► <inertial> blocks    →  URDF, Gazebo, MoveIt
 chain      URDF / xacro     ──┼──►   inertia · urdf-check      ├──► pass/fail report     →  CI (exit code, JSON)
 config     mechlint.yaml    ──┘      torque · drift · render ──┼──► JSON over MCP        →  Claude Code, Cursor, …
            (motors, materials,                                 └──► PNG renders          →  the human
             payload, mounting)
```

**Works with any CAD.** mechlint reads files, not CAD sessions, so where the geometry comes
from is irrelevant: CATIA, SolidWorks, Fusion, FreeCAD, or build123d. The professional-CAD
workflow — model in CATIA, export STL for the printer and the URDF — is the *primary* one; the
build123d path is the add-on for when an LLM authors a small part itself. What each export
enables:

| You export | mechlint can do | Cannot do |
|---|---|---|
| STL only (what DAST-1 has today) | mass, COM, inertia, watertightness, `torque`, mass drift | hole-to-hole `drift` (a mesh has no features) |
| STL **+ STEP** of the same part (recommended) | all of the above, plus exact volume and hole/axis positions found from the STEP's cylindrical faces → full `drift` | — |

A CATIA user changes nothing in how they model; they add "export STEP too" to the routine.
Physics simulation stays in Gazebo. mechlint is the calculator in between.

**Example — one pass through the loop, the 6th-joint redesign** (numbers illustrative).
Steps 2–5 are the same whether you type them in a terminal or an LLM runs them for you in a
chat; step 7 is what the chat adds on top.

```
1. Design   You model wrist_link in CATIA and export wrist_link.stl (+ wrist_link.step).
            (Or an LLM writes it in build123d via build123d-mcp — mechlint does not care.)

2. Weigh    $ mechlint inertia
            wrist_link: 38 g PLA @ 30 % infill + 55 g MG996R = 93 g; COM and tensor in the
            wrist_link frame → written to inertials.xacro (the URDF includes that file).

3. Load     $ mechlint torque --payload-g 100
            joint_2  max 1.71 N·m at q = [0, 90°, 0, 0, 0, 0]   MG996R @ 6 V: 1.08 N·m
                     margin 0.63× (required 2.0×)                                 T001 FAIL
            joint_3  max 0.98 N·m …                              margin 1.10×      T001 WARN

4. Decide   $ mechlint actuators --min-stall-nm 3.4        # 1.71 × safety factor 2.0
            lists the servos in the database that qualify, with mass and interface.
            Edit mechlint.yaml → joint_2: <chosen servo>, re-run step 3 → PASS.
            (Or move mass inboard / shorten arm_2 and re-run — the tool does not care which.)

5. Guard    $ mechlint drift
            joint_3 origin in URDF: 154.5 mm; hole-to-hole in wrist_link.step: 152.0 mm
                                                                                  D001 FAIL
            → fix the URDF, re-run → PASS.

6. CI       `mechlint check` runs on every PR to dast_1. A red X means "the mechanics and the
            model disagree", not "the code is broken".

7. Ask      From now on the robot has a mechanics-aware chat. In Claude Code:
            "What if I put a 200 g camera at the tip?"       → torque_budget(payload_g=200)
            "Which joint is the most marginal?"              → torque_budget(), sorted by margin
            "How much do I gain moving the joint_4 servo
             40 mm inboard?"                                 → torque_budget(overrides={...})
            "Why is arm_2 heavier than arm_3?"               → compute_inertia(), per-link breakdown
            None of these edit a file; tool arguments override mechlint.yaml for the question,
            and the answer says which values were overridden.
```

## 2. Landscape (researched 2026-09-06) and what we do with each

| Tool | What it does | mechlint's relation |
|---|---|---|
| [build123d](https://github.com/gumyr/build123d) | Parametric CAD as Python (OCCT kernel) | **Dependency** for STEP input (optional extra, OCP wheels are big). |
| [build123d-mcp](https://github.com/pzfreo/build123d-mcp) | MCP: run build123d code, render, measure volume/COM, find holes, **printability**, **fit/alignment**, STEP/STL import + compare. Py 3.11–3.14, Apache-2.0. | **Recommended companion when an LLM authors parts; not wrapped, not required.** It owns the *part*; we own the *mechanism*. We do not re-implement printability or hole detection. Interop contract: STEP/STL files on disk (from any CAD), plus an optional JSON sidecar of named features. |
| [agentcad](https://agentcad.dev/) | CLI + MCP: execute, render, export, validate, diff with per-view overlap score. Py 3.10–3.12 only, part-level, not a library. | **Not used.** Borrow one idea: a *diff report with a score* — ours diffs CAD against URDF, not two CAD versions. |
| [CADGenBench](https://github.com/huggingface/cadgenbench), BenchCAD | Benchmarks for CAD generation behind a validity gate | **Model for our own eval:** a small mechanics Q&A benchmark with numeric expected answers (M3). |
| STEP-to-URDF converters ([robosimtools](https://robosimtools.com/tools/step-to-urdf/), Jointly, [JupyterCAD robotics](https://blog.jupyter.org/a-robotics-workbench-for-jupytercad-6eb498a22178)) | One-shot export: STEP in, URDF with inertia out | **Complementary.** They run once; mechlint re-runs on every change and catches drift. Their output is valid mechlint input. |
| [trimesh](https://github.com/mikedh/trimesh) 5.x | Mesh loading, watertightness, volume, inertia | **Dependency** (the STL path needs nothing else). |
| [yourdfpy](https://github.com/clemense/yourdfpy) | URDF parsing with mesh loading | **Dependency.** |
| [xacro](https://pypi.org/project/xacro/) 2.1.1 on PyPI | xacro without ROS | **Dependency**, so `.xacro` inputs work outside a sourced ROS shell. |
| [Pinocchio](https://github.com/stack-of-tasks/pinocchio) (`pin` 4.1.0, cp314 wheels exist) | Rigid-body dynamics from URDF (RNEA), collision via coal | **v2 optional extra** for dynamic torques and self-collision. v1 static torque is plain numpy so the core stays light and analytically testable. |
| `check_urdf` (urdfdom) | Structural URDF validation, no physics | `mechlint urdf-check` is a **superset**; check IDs are designed to be proposed upstream (see §8). |

## 3. Scope

### The questions v1 must answer

| Question the user (or the LLM) asks | Command / MCP tool |
|---|---|
| How heavy is each link and what is its inertia tensor, in the link frame? | `mechlint inertia` |
| Is my URDF physically sane — units, tensors, masses, meshes? | `mechlint urdf-check` |
| Does servo X hold joint N at the worst pose, with payload P, at voltage V? With what margin? | `mechlint torque` |
| Which servos in the database would? | `mechlint actuators` |
| Does the URDF still match the CAD after I changed a part? | `mechlint drift` |
| What if the payload were 200 g / the servo were X / the arm 40 mm shorter? | Any tool with argument overrides (no file edits) |
| Show me the arm at the worst-case pose. | `mechlint render` (lowest priority) |
| Run everything and fail CI if anything fails. | `mechlint check` |

### Explicit non-goals for v1 (and where they live instead)

- Generic printability (walls, overhangs, manifold) — build123d-mcp does it.
- FEA / stress — out of scope; there is already research on FEA-in-the-loop CAD agents.
- Motion planning, gripper force, BOM/cost, GUI.
- Editing CAD. mechlint never writes geometry.

### Roadmap after v1

- Layer orientation vs. load direction for the bracket carrying each joint's torque (needs the torque result, so it belongs here rather than in build123d-mcp).
- Dynamic torques from a trajectory (Pinocchio RNEA), self-collision over the workspace.
- SARIF output so GitHub shows failed checks as annotations on the URDF.
- Servo mounting-geometry library (STEP of MG996R, XL430, …) for fit/drift checks.

## 4. Architecture

```
mechlint/                          # GitHub amirhpd/mechlint
  pyproject.toml                   # uv-managed; extras: [step] (build123d/OCP), [dyn] (pin), [mcp]
  src/mechlint/
    core/
      units.py        # SI internally; model-scale detection + explicit override
      geometry.py     # STL (trimesh) and STEP (build123d) -> volume, COM, inertia, watertightness
      materials.py    # density table: PLA 1.24, PETG 1.27, ABS 1.04 g/cm3, ...; infill factor
      inertia.py      # composite bodies (shell + servos), parallel-axis, frame transforms, <inertial> emit
      urdf.py         # load URDF/xacro, resolve package://, chain extraction, patch-free
      chain.py        # FK, downstream sets per joint, joint-space sampling (grid / Sobol)
      torque.py       # static gravity + payload torque per joint; margins vs actuator
      actuators.py    # YAML db loader, pydantic schema, query
      drift.py        # CAD vs URDF consistency
      checks.py       # check registry: id, severity, message, fix hint
      report.py       # pydantic result models -> JSON / table / markdown
      config.py       # mechlint.yaml schema (pydantic): materials, components, actuators, scenario
    cli/              # typer; one subcommand per core function + `check` (all)
    mcp/              # FastMCP server; same functions, readOnlyHint on inspect tools
    render/           # multi-view PNG (matplotlib Agg, headless) with labeled joint frames
  data/
    actuators/*.yaml  # one file per actuator, each entry with source URL + date + confidence
    materials.yaml
  ros/mechlint_ros/   # ament_python: `ros2 run mechlint_ros check`, ament_index resolution, colcon test hook
  docs/
    llms.md           # how an LLM should use the tools (the "policy" doc; see §7)
    checks.md         # every check ID, what it means, how to fix
  tests/
    unit/             # analytic cases: 2-link arm, point masses, known tensors
    integration/      # CLI + MCP called end to end on fixtures
    fixtures/dast1/   # expanded URDF + meshes, pinned with a provenance note
    bench/            # M3 mechanics Q&A benchmark (question, expected number, tolerance)
```

### Inputs — everything mechlint needs, and where it comes from

"CAD and URDF" is only the geometry. Motors, materials, measured weights, payload and how the
robot is mounted are in **neither** file, so they live in one project config, `mechlint.yaml`,
next to the robot description — the way `package.xml` sits next to a ROS package. Nothing is
asked interactively. What an LLM learns from you in conversation ("joint_2 is an MG996R at
6 V") it **writes into this file**, so the knowledge is versioned and reviewable instead of
trapped in a chat.

| Input | Carries | Comes from | Needed for |
|---|---|---|---|
| URDF / xacro | The chain: links, joints, axes, limits, effort, mesh references, existing `<inertial>` | The robot repo | everything |
| Meshes (STL) or parts (STEP) | Volume → mass, COM, inertia; watertightness; hole positions (STEP only) | Referenced by the URDF, or `parts:` in the config | `inertia`, `drift` |
| `mechlint.yaml` | Material and infill per link, measured masses, components, actuator per joint, voltage, payload, mount orientation, safety factor, model scale | You, or the LLM on your behalf | `torque` (required); others (optional) |
| Bundled databases | Actuator torque/mass/dims; material densities | Ship with mechlint; extend with your own YAML | `torque`, `inertia` |
| STEP (optional but recommended) | Exact volume; hole and axis positions found from cylindrical faces — works from CATIA, SolidWorks, Fusion exports | Your CAD tool | `drift` (full), better `inertia` |
| Named-feature sidecar (JSON, optional) | Hole and axis positions by *name*, when a part is authored in build123d | Exported by the build123d part script | `drift` (named matches) |
| Previous report | Baseline for "what changed since" | Committed `mechlint-report.json` | `drift` |

Example `mechlint.yaml` for DAST-1:

```yaml
robot:
  description: src/description/urdf/description.urdf.xacro
  xacro_args: { is_sim: "true" }
  package_paths: { description: src/description }
  model_scale: 0.1                 # DAST-1 is modelled in decimetres (U001 would tell you)

materials:
  default: { material: pla, infill: 0.30 }
  links:
    base_link: { measured_mass_g: 210 }          # weighed on a scale → overrides the estimate

components:                        # mass that is not in the meshes; servo sits at the joint it
  base_link:    [ { actuator: mg996r, drives: joint_1 } ]   # drives, in the parent link, unless
  rotary_link:  [ { actuator: mg996r, drives: joint_2 } ]   # `at: [x, y, z]` says otherwise
  arm_1_link:   [ { actuator: mg996r, drives: joint_3 } ]
  arm_2_link:   [ { actuator: mg996r, drives: joint_4 } ]
  arm_3_link:   [ { actuator: mg996r, drives: joint_5 } ]
  gripper_link: [ { mass_g: 30, at: [0, 0, 0.05], note: gripper motor } ]

actuators:
  voltage: 6.0
  joints:
    joint_1: mg996r
    joint_2: mg996r
    joint_3: mg996r
    joint_4: mg996r
    joint_5: mg996r
    joint_6: null                  # simulation placeholder, no servo yet → torque reported, not judged

scenario:
  mount: table                     # gravity = -Z of base_link; also wall | ceiling | [gx, gy, gz]
  payload_g: 100
  safety_factor: 2.0
```

Precedence when the same fact appears twice: tool/CLI argument › `mechlint.yaml` › value already
in the URDF › bundled default. A URDF mass is used when the config has none — and U005/U006 say
so if it looks like a placeholder.

### Design rules

1. **Numbers come from tools, never from pictures.** A render is for the human. Every number an
   LLM reports must be quoted from a tool result. (Lifted from NeuraPlatform's `prompt_builder`
   "CRITICAL — coordinates come from data, never from the image".)
2. **Inspect tools never write. Writing tools never overwrite the user's file.** The one thing
   mechlint writes is a *generated* `inertials.xacro` the description `xacro:include`s — same
   convention as DAST-1's generated RViz configs and NeuraPlatform's `output_path`.
3. **Every result is a pydantic model.** One object becomes the CLI table, the MCP JSON, the
   markdown report, and the thing tests assert on.
4. **Soft errors with hints.** `{ok: false, error, hint}` — a mesh that is not watertight tells
   you which one and what to do, it does not crash the run.
5. **Deterministic and offline.** The library contains no AI: same inputs, same numbers, no
   network, no API key. The conversation happens *outside* it — see *How a user talks to it*
   below.
6. **SI inside, explicit at the boundary.** Non-metre models (DAST-1!) are detected, warned
   about (check U001), and handled via `--model-scale`.
7. **Stable check IDs.** `U0xx` URDF sanity, `T0xx` torque, `D0xx` drift, `M0xx` mesh. They are
   the contract with CI and with the upstream proposal.

### How a user talks to it

"Deterministic and offline" describes the library, not the experience. mechlint is a
calculator; the intelligence sits in whatever LLM host you already use, which supplies the
conversation and calls mechlint's tools over MCP. A typical exchange in Claude Code:

```
1. You       "Would an MG996R hold joint_2 with a 100 g payload?"

2. Claude    reads mechlint.yaml (joint_2: mg996r @ 6 V) and calls
             torque_budget(joint="joint_2", payload_g=100)

3. mechlint  loads the URDF + config, sweeps the joint space, returns
             { max_torque_Nm: 1.71, worst_q: [0, 1.57, 0, 0, 0, 0],
               actuator: "mg996r", stall_Nm: 1.08, margin: 0.63,
               required_margin: 2.0, check: "T001", status: "FAIL" }

4. Claude    "No. At full horizontal extension joint_2 needs 1.71 N·m; an MG996R at 6 V
             stalls at 1.08 N·m — margin 0.63×, you asked for 2×. Options: a stronger servo
             (list_actuators(min_stall_nm=3.4) gives …), moving the joint_4 servo inboard,
             or shortening arm_2."

5. You       "Use a DS3225."  →  Claude edits mechlint.yaml and re-runs torque_budget.

6. You       "And with a 200 g camera at the tip?"  →  torque_budget(payload_g=200); the config
             is untouched, the answer notes "payload overridden: 100 → 200 g".
```

Three doors into the same functions:

| Who | How | Gets |
|---|---|---|
| A human at a terminal | `mechlint torque --payload-g 100` | A table and an exit code |
| A human in a chat | Natural language; the LLM calls the MCP tools | An explanation that quotes tool numbers |
| CI | `mechlint check` in GitHub Actions or `colcon test` | Red or green on the PR |

Why keep the AI outside: the maths is testable without a model and without cost (the same
goal as NeuraPlatform's `DEMO_MODE`, reached here by construction); results are reproducible;
and you can always tell which part of an answer is physics (the tool output) and which is
interpretation (the model's prose). `docs/llms.md` makes that a rule: every number in the
answer must be traceable to a tool result.

## 5. The physics of v1, precisely

**Mass properties per link.** watertight? → volume → shell mass = ρ · V · infill_factor →
plus discrete components placed in the link frame (a servo is a box of known mass at a known
pose) → composite COM and inertia via parallel-axis → `<inertial>` in the link frame. Links
with a measured mass (`masses.yaml`) use it and scale the tensor accordingly; the report says
per link whether the mass is *measured* or *estimated*. Non-watertight meshes fall back to the
convex hull with a warning (M001).

**URDF sanity checks (first set).**

| ID | Check |
|---|---|
| U001 | Model scale suspicious: total reach outside 0.05–3 m, or mesh `scale` implies non-metre units |
| U002 | Inertia tensor not positive definite |
| U003 | Triangle inequality violated (`ixx + iyy >= izz` and permutations) |
| U004 | Tensor inconsistent with mass × collision bounding box (more than 10× off either way) |
| U005 | Placeholder detected: identical tensors on several links |
| U006 | Mass zero, negative, or identical placeholder across links |
| U007 | `<limit effort>` exceeds the assigned actuator's stall torque (needs an actuator map) |
| M001 | Mesh not watertight (mass properties unreliable) |

**Static torque.** For each sample `q` inside the joint limits (grid, or Sobol for >4 joints):
run FK; for joint *i* with axis **a**ᵢ at point **p**ᵢ,

```
τ_i(q) = a_i · Σ_{links j downstream of i} ( (c_j − p_i) × m_j g )  +  a_i · ((p_tip − p_i) × m_payload g)
```

Pure numpy. Unit-tested against a hand-solved 2-link planar arm. Report per joint: max |τ|, the
pose where it occurs, stall and rated torque of the assigned actuator at the given voltage,
margin, pass/fail against a safety factor (default 2.0 on stall; configurable and printed so
nobody mistakes it for physics).

**Actuator database.** YAML, pydantic-validated. Fields: `name, vendor, stall_torque_Nm{voltage: value}, rated_torque_Nm, no_load_speed, mass_g, dims_mm, mounting, interface (PWM/TTL/RS485/…), price_hint, source_url, source_date, confidence`. First entries: MG996R, DS3218, DS3225, Dynamixel XL430-W250, XC330-M288, 2XL430, NEMA17 + 5:1 planetary. **Every entry needs a source** — the curated, sourced database is the part of the project a model cannot regenerate, i.e. the moat.

## 6. Milestones

Each milestone ends with a DAST-1 dogfood step and a green `pytest`.

### M0 — Skeleton (1–2 sessions)
- `uv init`, `pyproject.toml` with extras, Apache-2.0, ruff, pytest, GitHub Actions matrix 3.11–3.14.
- `tests/fixtures/dast1/`: xacro-expanded URDF + visual/collision meshes + `PROVENANCE.md` (commit hash of dast_1).
- `docs/llms.md` stub, README with the one-sentence pitch and the non-goals.
- **Done when:** `uv run pytest` green, `mechlint --help` lists the subcommands.

### M1 — `inertia` + `urdf-check`
- STL path (trimesh) first; STEP path behind `[step]`.
- **Done when**, on the DAST-1 fixture, it reports U001 (decimetre model), U005/U006 (placeholder
  `mass=0.1, ixx=iyy=izz=1.0`), M001 (`arm_1.stl`, `base.stl` not watertight) and writes
  `inertials.xacro` for every link.
- **Dogfood:** `mechlint.yaml` (the §4 example) is committed to dast_1; `description.urdf.xacro`
  includes the generated `inertials.xacro`; headless sim (`sim_robot.launch.py gui:=False
  rviz:=False`) still runs; behaviour differences noted.
- **MCP from day one:** `inspect_robot`, `compute_inertia`, `check_urdf` are exposed over MCP as
  soon as they exist, and `.mcp.json` goes into dast_1. The chat layer is the headline outcome,
  so it is dogfooded from M1 — tool names, descriptions and hints get corrected while they are
  cheap to change (NeuraPlatform lesson: the prompt-side wording matters as much as the code).

### M2 — `torque` + actuator db
- **Done when:** the 2-link analytic test passes to 1e-9; on DAST-1 it prints per-joint worst-case
  torque for 0 / 100 / 200 g payload and a pass/fail per joint against MG996R @ 6 V.
- **Dogfood:** this output *is* the motor decision for the 6-DOF upgrade. Record it in the DAST-1
  repo next to the mechanics. `torque_budget` and `list_actuators` join the MCP server; the
  first real "does this servo hold joint_2?" conversation happens here.

### M3 — Complete the chat layer: `llms.md`, argument overrides, mini benchmark
- All tools support per-call overrides (payload, actuator, voltage, mount, link mass) so the
  LLM can answer what-if questions without editing `mechlint.yaml`; results echo every override.
- Inspect tools carry `readOnlyHint`; the only writing tool is `write_inertials`.
- `docs/llms.md`: the policy the host model follows (numbers from tools, inspect before torque,
  record facts learned in chat into `mechlint.yaml`, quote check IDs).
- `tests/bench/`: ~10 questions with numeric expected answers and tolerances, run by a script
  that drives the MCP tools directly (no LLM) — the validity gate — plus an optional manual run
  through Claude Code.
- **Done when:** with `.mcp.json` in dast_1, Claude Code answers *"does MG996R suffice at joint_2
  with 100 g payload?"* by calling `torque_budget` and quoting its numbers.

### M4 — `drift` + `render`
- Drift: joint-to-joint distance in URDF vs. hole-to-hole distance in CAD. Hole axes come from
  the STEP's cylindrical faces (works with CATIA/SolidWorks exports); the build123d sidecar
  only adds *names* for unambiguous matching. Also mass mismatch and mesh hash changes since the
  last accepted report.
- Render: iso + 3 orthos of the worst-case pose with labeled joint frames; the result carries
  an `overlay` note listing what is drawn that is not geometry.
- **Dogfood:** the new wrist/sixth-joint part — modelled in CATIA, exported as STL + STEP — goes
  through `inertia` → `torque` → URDF → `drift` → sim. A small adapter authored in build123d via
  build123d-mcp goes through the same path to prove both routes.

### M5 — `mechlint_ros` + upstream
- `ament_python` package: `ros2 run mechlint_ros check <package> <xacro>`; `package://` via
  `ament_index`; a pytest-based ament test that runs `urdf-check` at `colcon test`.
- **Dogfood:** DAST-1 `description` package gets that test.
- **Upstream** (from `contribute.md`, "URDF inertia: no guidance on units, no sanity check"):
  docs PR to the ROS 2 URDF tutorial (units, conversion factors, triangle inequality), and a
  `urdfdom` issue proposing U002–U004 in `check_urdf`, citing mechlint as reference
  implementation with the same IDs.

### M6 — Publish
- PyPI release, GitHub release, MCP registry listing, README case study with the DAST-1 numbers,
  ROS Discourse post.

## 7. What transfers from NeuraPlatform

Mostly patterns, little code — the two products share a thesis (from `NOTES.md`: *"encode
discipline-specific validation rules as hard rules, not prompts"* and *"pick one workflow
end-to-end"*), not a stack.

| NeuraPlatform | mechlint |
|---|---|
| `ToolRegistry.tool_phase()` — `context` vs `action` tools | Inspect vs. write split; MCP `readOnlyHint`; CLI subcommands never mix the two |
| `prompt_builder`: "coordinates come from data, never from the image"; "read before write"; "if `already_rendered`, stop retrying" | `docs/llms.md` policy, near-verbatim: numbers from tools; run `inspect_robot` before `torque_budget`; results carry a content hash so a repeat call is a no-op |
| `approval_reason()` — human sign-off before AutoCAD draws | Nothing overwrites the user's URDF; generated `inertials.xacro` + `output_path`; the MCP host's own permission prompt gates writes |
| Soft tool errors `{ok:false, error}` + redirect hints (`parse_pdf_for_llm` rejecting `.dxf`) | Same shape, with a `hint` field on every failed check |
| Render overlays in colours absent from the drawing + "these are NOT entities" note | Joint frames / axes overlay + `overlay` note in the render result |
| `snap_dxf_point`, `relative_to_handle` — exact placement resolved server-side | All lever arms, COMs, frames computed server-side; the LLM never supplies a coordinate it read off an image |
| Integration tests calling the tool entrypoint on a tmp workspace and asserting on the result | Same, minus the scripted-LLM layer (there is no LLM in the loop) |
| Pydantic everywhere; `DXFService` render-to-PNG with matplotlib Agg and bundled-font registration | Same choice; the Agg/font plumbing can be lifted directly into `render/` |

**Deliberately not carried over:** FastAPI, DB, auth, billing, SSE, `FileBackend`/Azure (local
files only), the OpenAI client (the MCP host brings the model), pipenv (a publishable library
uses `uv` + `pyproject.toml`), and the single large hand-written JSON-schema dict in
`tool_registry.py` (FastMCP derives schemas from typed signatures and pydantic models).

## 8. ROS compatibility, specifically

- **Inputs are exactly what ROS uses:** URDF or xacro, `package://` URIs, mesh `scale`, joint
  `<limit>`s. `package://` resolves via `ament_index` when a ROS shell is sourced, else via
  `--package-path description=/path` so the core never imports `rclpy`.
- **Outputs are ROS-native:** `<inertial>` blocks and an includable `inertials.xacro`; a JSON
  report for CI; SARIF later.
- **`urdf-check` is a superset of `check_urdf`,** and the check IDs are stable so the same
  table can go into the `urdfdom` issue. If upstream adopts U002–U004, mechlint keeps the rest.
- **U007 connects the URDF to reality:** DAST-1 declares `effort="20.0"` N·m on every joint
  while an MG996R stalls at about 1.1 N·m — a controller tuned against that limit is tuned
  against fiction.
- **Simulation consequence of U001:** a model in decimetres under 9.81 m/s² gravity has wrong
  dynamics in Gazebo by construction; the check explains why, not just that.
- **`mechlint_ros`** is the only ROS-dependent code: `ros2 run`, `ament_index`, a `colcon test`
  hook. Released on PyPI as well so it does not need a bloom release to be useful.

## 9. Dogfooding baseline: DAST-1 as measured today

Numbers from the checked-in meshes (trimesh, PLA 1.24 g/cm³, **solid infill** — an upper
bound on the printed shell):

| Mesh | Watertight | Volume | Shell mass (solid PLA) | Extents (mm) |
|---|---|---|---|---|
| `arm_1.stl` | no | — | — | 208 × 26 × 74 |
| `arm_2.stl` | yes | 104.5 cm³ | 130 g | 249 × 26 × 74 |
| `arm_3.stl` | yes | 96.7 cm³ | 120 g | 128 × 62 × 53 |
| `gripper_3.stl` | yes | 35.7 cm³ | 44 g | 68 × 156 × 78 |
| `rotary.stl` | yes | 32.0 cm³ | 40 g | 26 × 74 × 64 |
| `base.stl` | no | — | — | 80 × 80 × 82 |

URDF facts mechlint would flag on day one: model is in **decimetres** (the xacro says so in a
comment; `find_reachable_workspace` and the Kinect mount already work around it); every link has
`ixx=iyy=izz=1.0` and `mass ∈ {0.2, 0.05, 0.1, 0.01}`; `effort="20.0"` everywhere; two meshes
are not watertight.

**Back-of-envelope torque at joint_2, arm horizontal** (to be replaced by M2's output; treat as
motivation, not a result). Lever arms from the joint origins: joint_3 at 154.5 mm, joint_4 at
349 mm, joint_5 at 437 mm, tip at 612 mm. Using the solid-PLA shell masses above, ~0.1 kg for
`arm_1`, and one 55 g MG996R at each of joints 3, 4, 5:

```
τ ≈ g · Σ m·r ≈ 9.81 · (0.1·0.077 + 0.13·0.25 + 0.12·0.39 + 0.044·0.52 + 0.055·(0.155+0.349+0.437))
  ≈ 9.81 · 0.16  ≈ 1.6 N·m        (no payload)
```

MG996R stall torque is roughly 0.92 N·m @ 4.8 V / 1.08 N·m @ 6 V (datasheet values, to be
sourced into the actuator db). Even if real infill halves the shell masses, joint_2 sits at or
above stall with zero margin when fully extended. That single number is the argument for the
project, and it is exactly the question that blocks the DAST-1 motor upgrade.

## 10. Risks and mitigations

| Risk | Mitigation |
|---|---|
| Actuator specs are marketing numbers | Source + date + confidence per entry; PR template requires a source; report prints the confidence next to the margin |
| Printed mass depends on infill and slicer settings | `--infill` factor, per-link `masses.yaml` override, report marks *measured* vs *estimated*; the docs tell people to weigh parts |
| Non-watertight meshes (professional-CAD STL exports often are — DAST-1's `arm_1`, `base`) | Convex-hull fallback with M001 warning, never silently skip; docs give CATIA/SolidWorks export settings; STEP input avoids it entirely |
| Static torque understates dynamic loads | Safety factor printed in every report; Pinocchio RNEA on the roadmap |
| OCP/build123d wheels are large | STEP support is an optional extra; the STL path needs only trimesh |
| Scope creep into a CAD tool / duplicating build123d-mcp | Non-goals in the README; the interop contract is a STEP file |
| Non-metre models (DAST-1) | U001 + explicit `--model-scale`; SI everywhere inside |

## 11. Open questions (not blocking)

- Should DAST-1 be converted to metres? mechlint will flag it; converting touches recorded
  poses, `points.pcd`, the Kinect mount, and the task server's joint vectors. Separate DAST-1
  task, decide after M1.
- Actuator mounting-geometry library (STEP per servo) — probably yes, after M4.
- SARIF output for GitHub annotations — cheap, after M5.

## 12. First session (M0) checklist

1. `cd ~/projects && uv init mechlint --lib && cd mechlint`
2. `pyproject.toml`: deps `numpy trimesh yourdfpy xacro pydantic typer`; extras `step=[build123d]`, `dyn=[pin]`, `mcp=[mcp]`; dev `pytest ruff`.
3. `LICENSE` (Apache-2.0), `README.md` (pitch, non-goals, landscape table), `docs/llms.md` stub.
4. `tests/fixtures/dast1/`: `xacro description.urdf.xacro > dast1.urdf` (from a sourced DAST-1
   shell), copy meshes, write `PROVENANCE.md` with the dast_1 commit hash.
5. `src/mechlint/core/geometry.py`: load STL → `MassProperties(volume, com, inertia, watertight)`;
   first unit test against a unit cube (V=1, I=1/6·diag). `core/config.py`: pydantic schema
   for `mechlint.yaml`, with the DAST-1 example from §4 as a fixture.
6. GitHub Actions: `uv sync && uv run pytest` on 3.11–3.14.
7. Open `amirhpd/mechlint`, push, done.
