# Release validation — 2026-09-30

Executed on Linux, NVIDIA RTX 4070 (12 GB), Isaac Sim 5.0.0, Python 3.11.

- Seven CPU tests passed, covering failure cases, S0 exclusion, failure-order
  attribution, endpoint validation, physics-trace coverage, aggregate-rate
  consistency and forward kinematics.
- All 12 bundled trials ran in headless Isaac Sim, with screenshots and final
  USD scenes exported. PhysX detected the intentionally unsafe trajectories
  (15 contact frames for hammer, 13 for screwdriver, 15 for bottle).
- Normal trials succeeded for all three objects. Missed deliveries failed Reach;
  the region-agnostic screwdriver failed Affordance. Detailed flags are in
  `demo_results.json`. These are release demo results, not paper measurements.
- A scene/selection pair exported by the separately installed method package
  was converted to a trial and replayed successfully through `scripts/run_isaac.py`.
- Editable installation, standalone wheel build and wheel-installed CPU demos
  were checked outside the source directory. No sibling repository was imported.
- Screenshots were visually inspected. The README image comes from an actual
  Isaac Sim run using the procedural assets.

The local Conda runtime logged missing GCC_12.0.0 symbols in unused RTX sensor
extensions. A second integration run with the process-scoped setting
`LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libgcc_s.so.1` completed without those
extension errors. This is a Linux/Conda troubleshooting option only; it is not
embedded in the portable entry point, and no environment libraries were replaced.

Only Isaac Sim 5.0 was exercised. The release does not validate MoveIt,
robot CAD collision geometry, articulated dynamics, MANO, physical grasping,
continuous collision detection or the original paper's numeric results.
