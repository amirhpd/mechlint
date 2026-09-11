# mechlint for LLMs

*This file exists because a model with a calculator available will still guess if nothing tells
it not to: these are the house rules for a host that has mechlint's MCP tools.*

Six tools: `inspect_robot`, `check_urdf`, `compute_inertia`, `torque_budget` and
`list_actuators` are read-only; `write_inertials` is the only one that writes. `drift` and
`render` arrive in M4. The policy was written before the tools on purpose: tool wording shapes
model behaviour as much as the code behind it does.

1. **Every number you report comes from a tool result.** Not from a render, not from memory, not
   from arithmetic you did yourself. Renders are for the human to look at.
2. **Call in order:** `inspect_robot` (links, joints, scale) → `check_urdf` → `compute_inertia` →
   `torque_budget`. Torque computed from placeholder masses is fiction, and `U005` tells you the
   masses are placeholders.
3. **What-if questions use per-call overrides, never file edits.** `payload_g`, `voltage`,
   `joint_actuators`, `actuator_db`, `mount`, `payload_at` and `link_mass_g` each override
   `mechlint.yaml` for one call. Every
   result carries an `overrides` block listing exactly what was changed — quote it, so nobody
   mistakes a hypothetical for their robot. An empty `overrides` means the answer is about the
   project as committed.
4. **Facts learned in chat go into `mechlint.yaml`.** "joint_2 is an MG996R at 6 V" is knowledge,
   not a hypothesis: offer to write it into the config. A chat is gone tomorrow, a versioned file
   is not. An override answers a question; the config records an answer.
5. **Quote the check ID.** `T001`, not "the torque check failed". Every ID is in
   [checks.md](checks.md).
6. **A margin is worth what its datasheet is worth.** Every actuator carries `confidence` and a
   `source_url`. Report them next to the margin: 1.2× against a `low`-confidence hobby-servo
   figure is not a pass, and saying so is the difference between a number and an answer.
7. **Torque here is static** — gravity holding, nothing accelerating. Never present it as the
   whole load case; quote the `safety_factor` the result carries. A `null` margin means gravity
   applies no torque about that axis at all, which is an unbounded margin, not a zero one.
8. **When a joint fails, say what would fit.** Call `list_actuators` with `min_torque_Nm` set to
   the failing case's `required_Nm`. An empty result is itself the answer: the arm needs a
   redesign, not a bigger servo.
9. **Say no to what mechlint does not do:** modelling parts (build123d-mcp, or their CAD),
   simulating motion (Gazebo), printability, stress. Point at the right tool; do not approximate.
10. **`write_inertials` is the only tool that writes,** and only to a generated file the
    description includes — never to the URDF itself. Ask before calling it, and only after the
    numbers have been looked at. It refuses to overwrite a file mechlint did not generate, and
    refuses any report computed with overrides. Everything else carries `readOnlyHint`.

## The shape of a good answer

> **joint_2 does not hold 100 g.** `torque_budget` puts the worst-case static torque at
> **1.65 N·m** (`T001`), against **1.079 N·m** of MG996R stall at 6 V — a margin of **0.33×**
> at the project's safety factor of 2. The datasheet is `low` confidence
> ([servodatabase.com](https://servodatabase.com/servo/towerpro/mg996r)), and this is gravity
> alone: nothing accelerating. `list_actuators` with `min_torque_Nm: 3.30` returns nothing, so
> no servo in the database fixes it — the arm is too long for its class.

Every number in that paragraph came out of a tool result, each one carries what it is worth, and
the last sentence answers the question the user was actually asking.

## The questions this is measured against

[`tests/bench/`](../tests/bench/) holds the questions above as a runnable benchmark: each one is
a tool call plus the number the answer must contain. `uv run python -m tests.bench.run` checks
the tools with no model in the loop; the README there describes the manual half, where the thing
being measured is whether the model called a tool at all.
