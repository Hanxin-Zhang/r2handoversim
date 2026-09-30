# Release validation — 2026-09-30

Executed on Linux, NVIDIA RTX 4070 (12 GB), Isaac Sim 5.0.0, Python 3.11.

## 0.1.0 initial release

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
robot CAD collision geometry, articulated dynamics, physical grasping,
continuous collision detection or the original paper's numeric results.

## 0.2.0 follow-up

- Fourteen CPU tests passed, including input validation, duplicate IDs, stale
  completion manifests and artifact links, and nonzero worker exits.
- All 12 bundled trials ran again in Isaac Sim. Every metric, first failure and
  contact-frame count matched `demo_results.json` from the initial release.
- A real H2O Text2HOI prediction decoded by the companion method with locally
  supplied MANO ran through `from-intent` and Isaac Sim. The decoded mesh was
  rendered, its skeleton proxies were evaluated, and the bottle trial passed
  all active criteria. The screenshot was visually inspected; generated MANO
  assets are excluded from Git.
- HTML reports, JSON/CSV results, final USD snapshots and animated USD exports
  were generated successfully.
- A deliberately blocked USD export raised an internal Kit exception while Kit
  returned exit code 0. The supervising CLI correctly returned exit code 2 and
  retained a failed run manifest. A missing Isaac Sim installation also returned
  exit code 2 with installation guidance.
- Reopened the animated USD through Isaac Sim's USD API: every robot part has
  90 time samples, transforms differ between first and last frames, and the
  timeline is configured at 60 frames per second.
- Version 0.2.0 wheel installed independently; all 12 CPU evaluations ran from
  outside the source directory and wrote the standalone HTML report.
