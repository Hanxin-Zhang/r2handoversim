# Changelog

## 0.7.0

- Add configurable original UR5e/Robotiq USD and local OBJ meshes, calibrated tool frames, actual robot collider queries, and USD/MP4 replay exports.
- Add 8,000 deterministic reconstructed Table I replay records, source settings, exact aggregate verification and independent simulator evaluation.

## 0.6.0

- Record actual Isaac Sim viewport frames to H.264 MP4 with configurable playback
  speed, overview/receiving-hand cameras and report links.
- Wait for asynchronous PNG writes to finish after the viewport capture callback.
- Reject missing encoders/frames and preserve prior videos on encoding failure.

## 0.5.0

- Import a completed method pipeline directly, including optional delivery planning.
- Convert paired FS/A1/A2/A3 experiments while holding the receiving hand and
  object target fixed across modes, with shared tabletop clearance adjustment.
- Retain failed plans in batch evaluation and record infeasible selections separately.
- Reject duplicate trial IDs consistently in offline and Isaac Sim batch runs.

## 0.4.0

- Convert imported dataset manifests to batch trials and accept `--trials` arrays.
- Render original object point clouds in Isaac Sim and animate them with the grasp.
- Preserve source hashes, annotation status and explicitly supplied split labels.
- Validate all 16 locally configured objects and a real-cloud Text2HOI example.

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
