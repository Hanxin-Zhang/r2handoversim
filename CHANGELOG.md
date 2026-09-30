# Changelog

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
