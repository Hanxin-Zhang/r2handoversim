# Changelog

## 0.3.0

- Add numerical full-pose IK and a portable RRT-Connect planner, with sampled
  robot, attached-object, receiving-hand and obstacle box collision checks.
- Consume ergonomic delivery targets without relocating the hand to a fixed goal.
- Add optional static triangle-mesh hand collisions in Isaac Sim.
- Export Table I object/split aggregation and measured planning times.
- Visualize supplied skeletal keypoints and target direction in Isaac Sim.

## 0.2.0

- Supervise Isaac Sim in a separate process and reject stale/incomplete run
  manifests even when Kit exits with code zero.
- Display decoded MANO meshes imported from the method while retaining explicit
  skeletal proxy collision evaluation and the predicted palm normal.
- Add self-contained result reports and optional time-sampled USD animation.
- Validate mesh indices, identifiers, duplicate trials and boolean contact traces.
- Bound screenshot capture time and check USD export success.

## 0.1.0

- Initial Isaac Sim replay examples, PhysX overlap evaluation, offline geometry
  checks and JSON/CSV results.
