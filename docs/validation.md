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

## 0.3.0 paper-detail follow-up

- Twenty CPU tests pass. New coverage includes full-pose IK, RRT detouring around
  a blocking obstacle, attached-object collision, unreachable-target failure,
  mesh-mode input requirements, and unbalanced object/split aggregation.
- The skeletal ergonomic target produced by the companion method yielded an
  85-frame planned trajectory. Isaac Sim replay passed all five S1 criteria and
  exported a screenshot, report and animation. Supplied keypoint visualization
  was visually inspected; `ergonomic_demo.png` is from this actual run.
- The real coarse Text2HOI/MANO bottle output ran with triangle-mesh hand
  collision enabled and passed all active S0 criteria.
- A separate unsafe trajectory with a procedural triangulated hand generated
  15 contact frames and a Safe failure, confirming mesh colliders participate
  in PhysX queries rather than merely being rendered.
- All 12 original bundled traces ran again in Isaac Sim; metrics, first-failure
  labels and contact counts matched the recorded initial release results.
- Version 0.3.0 standalone wheels built successfully. Independently installed
  method delivery, benchmark IK/planning and offline evaluation ran outside
  both source directories.
- These checks do not validate MoveIt equivalence, mesh-accurate robot planning,
  original 16-object splits, or the paper's quantitative performance.

## 0.4.0 configured-data follow-up

- Twenty-two CPU tests pass, including source point-cloud/split preservation and
  malformed cloud rejection.
- All 16 configured han object demos completed in Isaac Sim 5.0, each exporting
  its original point-cloud display, result and screenshot. All active demo S0
  criteria passed; this is not a paper SR because annotations/trajectories are
  generated release examples.
- The actual binoculars cloud plus a left-hand Text2HOI/MANO prediction ran in
  triangle-hand-collision mode and passed all active criteria. Its screenshot
  was visually inspected. The animated USD was exported.
- Standalone 0.4.0 wheels were built, installed independently, and exercised
  through config import, dataset batch conversion and offline evaluation.
- Reopened the neural-object animation in Isaac Sim's USD API: all 8192 source
  points were retained and the object cloud had 90 animated transform samples.
