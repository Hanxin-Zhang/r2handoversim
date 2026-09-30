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
backend, robot CAD/articulation, MANO geometry or neural policy can be added
later without changing failure-order semantics. Do not compare demo success
rates to the paper's table: assets, splits, geometry and planner differ.
