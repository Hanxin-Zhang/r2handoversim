# Replay protocol v1

Each `handover.trial.v1` JSON stores all inputs needed for one standalone trial.
Lengths are metres, angles radians, coordinates right-handed, Z up. Homogeneous
transforms are row-major JSON arrays used with column vectors.

| Field | Meaning |
|---|---|
| `id`, `object_id`, `variant`, `provenance` | Identifiers and source description |
| `split` | S0 or S1; controls whether affordance contributes to success |
| `T_object_gripper` | Maps gripper/TCP coordinates into canonical object coordinates |
| `target_T_world_gripper` | Expected endpoint of the supplied plan |
| `object_boxes`, `usage_boxes` | Oriented box unions in object coordinates |
| `hand_boxes_world` | Static receiving-hand collision geometry in world coordinates |
| `palm_position_world`, `palm_normal_world` | Reach-region origin and direction |
| `reach_offset_m`, `reach_radius_m` | Default 0.12 and 0.10 |
| `max_opening_m` | Default 0.085 |
| `planned_joints`, `executed_joints` | Nonempty arrays of six-angle configurations; these may differ |
| `dt_s` | Time interval between replay samples; default 1/60 |

UR5e FK uses a fixed base at `[0,0,0.75]` and a 0.12 m tool offset from flange
to the simplified gripper TCP. These are explicit demo choices. The object pose
at each frame is `T_world_gripper @ inverse(T_object_gripper)`.

Boxes contain `center`, positive `half_extents`, proper `rotation`, and optional
`label`. The same proxy dimensions are used by the CPU evaluator, Isaac Sim
visuals and PhysX overlap queries. The static hand boxes are registered as
PhysX colliders. Moving robot boxes are queried against those colliders; no
dynamic robot or hand grasp controller is required.

Each result contains five flags, first failure, success, width, contact-frame
indices, safety source and explicit evaluation scope. The paper names are
retained, but **Plan and geometry fidelity are simplified**. The supplied path
check is limited to endpoints, joint limits, and hand clearance; passing it does
not certify self-collision or environmental collision freedom. Missing or empty
paths are rejected rather than silently reported as successful.

Stability checks global projected object width. This can be conservative for
multi-part objects. The method's approach-point usage constraint and the
benchmark's finger-volume affordance constraint are different: for example,
the region-agnostic hammer can pass the finger-volume proxy check even though
its selected approach point lies in the reserved region. See the JSON flags
rather than assuming every ablation must fail.

## Demo generation

The assets were generated from the companion method's procedural scenes. Normal
trials interpolate a fixed UR5e home/goal pair with a smooth cubic time profile.
Execution-deviation fixtures insert a position-IK waypoint inside the palm;
missed-delivery fixtures keep the executed robot at home. All trajectories are
stored explicitly, so the simulator does not need the companion package or a
random seed to replay them. The complete release set has 12 trials.

## Extending

Replace boxes with your own geometric approximation and provide trajectories
from any planner. Keep the transform convention unchanged. A full MoveIt
backend, robot CAD/articulation or neural policy can be added
later without changing failure-order semantics. Do not compare demo success
rates to the paper's table: assets, splits, geometry and planner differ.

## Predicted hand and execution status (0.2.0)

Optional `hand_mesh_world` contains `vertices` (N x 3 finite coordinates) and
`faces` (M x 3 integer vertex indices). By default it is used for rendering,
with `hand_boxes_world` as the collision approximation. In 0.3.0,
`--hand-collision mesh` uses the triangles as the static PhysX collider for Safe.
The CPU evaluator and planner continue to use the box approximation. Conversion from
the companion method now preserves a supplied `palm_normal` instead of always
substituting the procedural default.

`run.json` includes a unique run id, status, expected/completed trial counts and
an error message when relevant. The parent process checks the matching run id
and fresh result count after the Kit worker exits. A metric failure is a valid
completed trial and does not make the CLI fail; simulator/infrastructure errors
do produce a nonzero exit status.

`--animation` produces an additional self-contained USD with 60 Hz samples for
the default fixtures (or the rate defined by `dt_s`). These are kinematic
transform samples, not a saved dynamic articulation simulation. `report.html`
links only artifacts recorded as produced by the current run; stale screenshots
from an earlier run in the same directory are not presented as current outputs.

## Planned trials and table aggregation (0.3.0)

Optional `delivery` contains the companion method's computed world-frame target
and body keypoints. These keypoints are visual references, not colliders.
Optional `obstacle_boxes_world` specifies planner obstacle geometry.
Optional `planning` records status, algorithm, random seed, tolerances,
collision scope, failure reason and measured time. Such trials receive the
stronger Plan check described in [paper details](paper_details.md). The original
12 stored traces still use their original lightweight check.

Results add `execution_time_s` (simulated duration), `total_time_s` (null without
measured planning), and a measured `planning_time_s` for new plans. Isaac Sim
results record `hand_collision` and the actual collision source.
`paper_table.csv/json` follows Table I object and split weights; the existing
`summary.json` remains a pooled trial summary.
