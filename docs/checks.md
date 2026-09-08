# Checks

*This file exists so a failure like `U005` can be one short line in the terminal instead of a
paragraph: the message names the ID, and this table says what it means and how to fix it.*

Stub — IDs are fixed now because they are the contract with CI, with `mechlint.yaml`'s severity
overrides (M3), and with the `urdfdom` proposal (M5). Implementations land in M1 (`U`, `M`),
M2 (`T`), M4 (`D`). IDs are never reused or renumbered.

| ID | Means | Fix |
|---|---|---|
| U001 | Reach outside 0.05–3 m, or mesh `scale` implies non-metre units | Set `robot.model_scale`, or convert the model. A decimetre model has wrong Gazebo dynamics by construction. |
| U002 | Inertia tensor not positive definite | Recompute it — no rigid body has such a tensor. `mechlint inertia` writes a correct one. |
| U003 | Triangle inequality violated (`ixx + iyy >= izz`, and permutations) | Same. |
| U004 | Tensor off by >10× from mass × collision bounding box | Usually a tensor computed in the wrong units, or for the wrong part. |
| U005 | Identical tensors on several links | A `default_inertial` macro: nobody has computed the physics yet. Run `mechlint inertia`. |
| U006 | Mass zero, negative, or an identical placeholder across links | Same, for mass. Weigh the part, or let `inertia` estimate it. |
| U007 | `<limit effort>` exceeds the assigned actuator's stall torque | Lower the limit to what the servo delivers; a controller tuned against fiction is untuned. Needs `actuators.joints`. |
| M001 | Mesh not watertight | An open mesh has no defined inside, so mass properties are unreliable. mechlint uses the convex hull and marks it — an over-estimate. Export STEP, or fix the export settings. |
| T001 | Worst-case static torque exceeds stall torque ÷ safety factor | Stronger servo, mass inboard, or shorter link. The safety factor is a convention, not physics, and every report prints it. |
| D001 | URDF joint-to-joint distance disagrees with CAD hole-to-hole | One of the two is stale. This is why mechlint re-runs on every change instead of converting STEP to URDF once. |

Prefixes: `U` URDF sanity · `M` mesh · `T` torque · `D` drift.

Severities: **fail** sets a non-zero exit code from `mechlint check`; **warn** reports only;
**info** is context, such as which masses were measured and which estimated. Per-project
overrides are M3.
