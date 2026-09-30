# Release 0.9.0

This release is ready for users to install and run kinematic handover replay.
Its supported workflows are procedural Isaac Sim demos, original local
UR5e/Robotiq and OBJ replay, method JSON interchange, and paper-reference replay.
The main benchmark entry point runs Isaac Sim; CPU evaluation is auxiliary.

## Release gates

- CPU suite exercises geometry, failure attribution, planning, converters,
  input rejection, recording failures, contact fitting and export consistency.
- CI builds both wheel and source distribution on Python 3.10, 3.11 and 3.12,
  checks package metadata, installs the wheel, runs tests outside package source
  imports, exercises all 12 offline fixtures and the 8,000-record paper replay,
  then rebuilds a wheel from the source archive.
- The actual GPU smoke check uses Isaac Sim 5.0 on Linux. It covers the 12 default
  fixtures, 16 original object meshes with UR5e/Robotiq, frame capture, animated
  USD, resolved JSON/NPZ export, and export/reload consistency. This local check
  is separate from CI, which has neither the simulator nor private assets.
- Every original-asset successful fit verifies both object contacts against the
  USD finger-pad surfaces with a 0.2 mm threshold. Failure-case examples remain
  separately labeled and checked against evaluated outcomes.
- `verify-output` checks completed-run counts, artifact presence, numeric
  trajectories and result consistency. Videos are rendered by Isaac Sim.
- Package files contain no local model weights, robot USDs, dataset meshes,
  MANO assets or participant records. External assets retain their source terms.

See [validation](validation.md) for executed checks and scope. A completed run
requires a successful `run.json`; a nonzero CLI exit or failed manifest must not
be treated as a measured benchmark success.

## Maintainer commands

Run tests in an environment with the optional `planning,assets` dependencies.

```bash
python -m pip install build twine
python -m build
python -m twine check dist/*
python -m pip install './dist/r2handoversim-0.9.0-py3-none-any.whl[planning,assets]'
python -m unittest discover -s tests -v
python scripts/release_smoke.py
```

For the simulator, follow the README's default and local-asset commands, then
run `verify-output` on each output directory. Check the movies for visible grip
and scene problems; geometric contact distances alone do not establish physical
force closure. Keep private generated artifacts out of source releases.

Versioned wheel and source archives are the distribution artifacts. A GitHub
release can attach those two files and their SHA-256 checksums. A source checkout
also works with `scripts/run_isaac.py` in an installed Isaac Sim environment.

## Scope carried into this release

The four baseline settings are offline authored/reconstructed inputs. Weights
and baseline inference implementations are not included. Table I source values
and reconstructed aggregates are distinguished from evaluated replay outcomes.
There are no claimed replications of original experiment logs or real-hardware
studies. The robot is driven kinematically, with a rigidly attached object;
frictional grasp dynamics are not simulated. The optional planner uses sampled
box proxies. Original-asset retargeting changes the receiving hand and therefore
does not preserve fixed-world paired method comparisons.

The original asset adapter supports the documented `danilab_ur5e` assembly;
arbitrary UR5e/Robotiq USDs may have different frames, joints and colliders and
are not interchangeable. Users without those assets can run the bundled
procedural demos immediately after installing Isaac Sim.
