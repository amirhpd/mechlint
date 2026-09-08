# mechlint for LLMs

*This file exists because a model with a calculator available will still guess if nothing tells
it not to: these are the house rules for a host that has mechlint's MCP tools.*

Stub — the tools land in M1–M3. The policy is written first on purpose: tool wording shapes
model behaviour as much as the code behind it does.

1. **Every number you report comes from a tool result.** Not from a render, not from memory, not
   from arithmetic you did yourself. Renders are for the human to look at.
2. **Call in order:** `inspect_robot` (links, joints, scale) → `check_urdf` → `compute_inertia` →
   `torque_budget`. Torque computed from placeholder masses is fiction, and `U005` tells you the
   masses are placeholders.
3. **What-if questions use per-call overrides, never file edits.** Echo what was overridden
   ("payload 100 → 200 g") so nobody mistakes a hypothetical for their robot.
4. **Facts learned in chat go into `mechlint.yaml`.** "joint_2 is an MG996R at 6 V" is knowledge;
   offer to write it down. A chat is gone tomorrow, a versioned file is not.
5. **Quote the check ID.** `T001`, not "the torque check failed". Every ID is in
   [checks.md](checks.md).
6. **Say no to what mechlint does not do:** modelling parts (build123d-mcp, or their CAD),
   simulating motion (Gazebo), printability, stress. Point at the right tool; do not approximate.
7. **`write_inertials` is the only tool that writes,** and only to a generated file the
   description includes. Everything else carries `readOnlyHint`.
