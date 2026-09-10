# Checks

*This file exists so a failure like `U005` can be one short line in the terminal instead of a
paragraph: the message names the ID, and this table says what it means and how to fix it.*

IDs are the contract with CI, with `mechlint.yaml`'s severity overrides (M3), and with the
`urdfdom` proposal (M5), so they are never reused or renumbered. Only `U0xx` is proposable
upstream: those are judgements about the URDF alone, while a `D0xx` needs something outside it
to disagree with.

`U0xx`, `M0xx`, `T001`, `D002` and `D003` run today; `D001` needs M4, and every report lists
what it did *not* run rather than passing it silently. `U007` and `T001` need
`actuators.joints` in `mechlint.yaml` to say which servo is on which joint, and `T001` is its
own command — `mechlint torque` — because it searches thousands of poses rather than reading
the file once. `mechlint check` runs both.

| ID | Means | Fix |
|---|---|---|
| U001 | Reach outside 0.05–3 m, or mesh `scale` implies non-metre units | Set `robot.model_scale`, or convert the model. A decimetre model has wrong Gazebo dynamics by construction. |
| U002 | Inertia tensor not positive definite | Recompute it — no rigid body has such a tensor. `mechlint inertia` writes a correct one. |
| U003 | Triangle inequality violated (`ixx + iyy >= izz`, and permutations) | Same. |
| U004 | Principal moments off by >10× from what the link's own geometry implies at the declared mass, about the declared `<origin>` | Usually a tensor computed in the wrong length unit, for a different part, or about the link origin when it belongs about the centre of mass. |
| U005 | Identical tensors on links of different sizes | A `default_inertial` macro: nobody has computed the physics yet. Run `mechlint inertia`. Two copies of the same part are not flagged. |
| U006 | Mass zero or negative, geometry with no `<inertial>` at all, or one mass shared by links of different sizes | Same, for mass. Weigh the part, or let `inertia` estimate it. |
| U007 | `<limit effort>` exceeds the assigned actuator's stall torque at `actuators.voltage` | Lower the limit to what the servo delivers; a controller tuned against fiction is untuned. Effort is N·m whatever length unit the model uses, so this comparison needs no scaling. Stall torque, not rated: the limit is a ceiling, and stall is the ceiling. Needs `actuators.joints`. |
| M001 | Mesh not watertight | An open mesh has no defined inside, so mass properties are unreliable. mechlint uses the convex hull and marks it — an over-estimate. Export STEP, or fix the export settings. |
| T001 | Worst-case static torque exceeds stall torque ÷ safety factor | Stronger servo, mass inboard, or shorter link. Fires at the *lightest* payload that already fails, because a joint that cannot hold the arm up empty is a different problem from one that runs out at 200 g. Static only — gravity holding, nothing accelerating — which is why the safety factor exists; it is a convention, not physics, and every report prints it. `mechlint actuators --min-torque` says what would fit instead. |
| D001 | URDF joint-to-joint distance disagrees with CAD hole-to-hole | One of the two is stale. This is why mechlint re-runs on every change instead of converting STEP to URDF once. |
| D002 | `scenario.mount` disagrees with the world-to-base rotation in the model | The model wins — it is what the simulator runs. Fix whichever of the two is stale. A grounded URDF already states its mounting, so `mount` is only read when there is no world frame. |
| D003 | `mechlint.yaml` names a link or joint the model does not have | A rename on one side only. `extra: forbid` catches a mistyped *key*; nothing else catches a mistyped link *name*, and a servo attached to a link that does not exist is silently dropped — making the robot lighter than it is. |

Prefixes: `U` URDF sanity · `M` mesh · `T` torque · `D` drift.

Severities: **fail** sets a non-zero exit code from `mechlint check`; **warn** reports only;
**info** is context, such as which masses were measured and which estimated. Per-project
overrides are M3.
