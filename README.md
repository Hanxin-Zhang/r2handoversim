# R2HandoverSim

**Isaac Sim handover demos with offline method outputs and reproducible result files.**

This release contains three objects (hammer, screwdriver, bottle) and
four illustrative joint-trajectory variants. Isaac Sim renders the scene,
advances the simulation, and queries PhysX for robot/hand overlap at each frame.
No baseline model, ROS, MoveIt, external USD asset server, or MANO download is
needed to run the examples.

The robot uses nominal UR5e DH kinematics with **procedural link/parallel-gripper
proxies**, an 85 mm aperture, and a fixed receiving-hand proxy. The bundled examples
replay supplied joint trajectories; an optional planner can generate new paths
for independently supplied handover targets. They do not reproduce the paper's complete
UR5e/Robotiq/MANO/MoveIt experiments or the four published baselines.

## Run in Isaac Sim

Tested with Isaac Sim **5.0.0**, Python **3.11**, Linux, RTX 4070. Isaac Sim must
already be installed and its license accepted by the user. In its Python
environment:

```bash
python -m pip install --upgrade pip setuptools wheel
python -m pip install -e .
r2handoversim demo --object hammer --hold
```

For an Isaac Sim installation that supplies `python.sh`, from this repository:

```bash
/path/to/isaac-sim/python.sh scripts/run_isaac.py demo --object hammer --hold
```

The script resolves this repository relative to itself. There are no developer
machine paths in the runtime. Normal system Python is sufficient for the
offline `evaluate` command, but **not** for the main Isaac Sim `demo` command.

Run the complete set without a GUI:

```bash
r2handoversim demo --headless --object all --variant all \
  --render-every 6 --screenshot --animation --output outputs/isaac
```

Outputs include `report.html`, `results.json`, `results.csv`, `summary.json`, and
a final-scene `.usda` per trial. `--screenshot` adds PNGs; `--animation` adds
`*_animation.usda` files with time-sampled transforms that can be played directly
in a USD viewer. Open `report.html` to inspect metric flags and artifact links.
`--hold` keeps the GUI open after completion.

A supervisor process verifies the current run id and completion count after Kit
exits. Missing simulator dependencies, failed captures, crashes and incomplete
runs produce a nonzero CLI exit code instead of being mistaken for success.
`run.json` records the run status. Screenshot capture has a 60-second timeout.

![Isaac Sim demo](docs/isaac_demo.png)

## Demo variants

| Variant | Purpose |
|---|---|
| `intent_aware` | Present the free object region while leaving the intended human region available |
| `region_agnostic` | Use a candidate selected without the usage constraint |
| `execution_deviation` | Replay an unplanned deviation through the hand, demonstrating a safety failure |
| `missed_delivery` | Stop at home despite a valid plan, demonstrating a reach failure |

These are explicitly authored offline inputs, **not implementations or reported
results of FC-Handover, Handover-VA, Contact-Handover, or Intent-Handover baselines**.
The bundled S0/S1 assignment is illustrative: bottle is S0; hammer and screwdriver
are S1. It is not a recovered paper split. All sources and limitations are
recorded in the fixtures and result files.

## Metrics

| Metric | This release |
|---|---|
| Stability | Projected object-proxy width ≤ 85 mm; the object is then replayed rigidly with the gripper |
| Plan | Stored traces: joint limits, endpoint and sampled hand clearance. New planner: full pose IK plus RRT-Connect with robot/object/hand/obstacle box checks |
| Reach | Delivered object boxes intersect the palm-normal sphere (12 cm offset, 10 cm radius) |
| Affordance | Finger proxies do not intersect the intended region; omitted in S0 |
| Safe | No hand overlap for robot/gripper boxes at replay frames, using Isaac Sim PhysX; optional triangle-mesh hand collider |

First-failure attribution follows **Stability → Plan → Reach → Affordance → Safe**.
S0 excludes Affordance from success and reports it as null. Failure rates sum to
`1 - success_rate` within a split. Independently computed metric flags remain in
the result for diagnosis, even when an earlier criterion fails.

Stored traces retain the original lightweight Plan check. The new `plan`
command performs full-pose numerical IK and RRT-Connect, with attached-object,
hand, table/obstacle and nonadjacent arm-proxy checks. It is a portable release
implementation, not MoveIt or a CAD-accurate collision model. Safety is sampled,
not continuous collision detection. Scene visuals and box queries are kinematic
replay, not an articulated torque simulation. No frictional grasp stability is
claimed. Planning time is measured for newly planned trials and remains null for stored
traces. Simulation duration and simulator wall time have separate fields. See [protocol details](docs/protocol.md).

## Offline checks and your own traces

```bash
python -m pip install -e '.[planning]'
r2handoversim evaluate --object all --variant all --output outputs/offline
python -m unittest discover -s tests -v
```

The CPU evaluator uses oriented-box intersections in place of PhysX and is
useful for inspecting traces and testing installation. It is an auxiliary path;
the main demo remains Isaac Sim.

To use output from the separate `intent-handover` repository:

```bash
r2handoversim from-intent \
  --scene /path/to/intent-handover/outputs/demo/hammer_scene.json \
  --selection /path/to/intent-handover/outputs/demo/hammer_FS.json \
  --output outputs/custom_trial.json
r2handoversim demo --trial outputs/custom_trial.json --hold
```

Alternatively, edit one of `src/r2handoversim/assets/*.json` according to
[the protocol](docs/protocol.md). Both repositories install and run independently;
the integration exchanges JSON files, with no cross-repository imports.

If the method scene was produced with `from-prediction`, conversion preserves
the predicted palm normal and carries the decoded MANO mesh into Isaac Sim for
display. Add `--hand-collision mesh` to use its triangles for PhysX safety
queries. Without this flag, collision evaluation uses the skeletal box proxies.
The CPU planner/evaluator continues to use boxes in both cases. MANO-derived outputs remain local/generated assets
and are not bundled in the repository.

## Paper details and new targets

```bash
python -m pip install -e '.[planning]'
r2handoversim plan --trial outputs/custom_trial.json --seed 0 \
  --output outputs/planned_trial.json
r2handoversim demo --trial outputs/planned_trial.json --headless --animation
```

The method's skeletal delivery target can also be supplied with the
`from-intent --delivery` option, which runs IK/planning automatically. Failed planning saves a
failed trial for diagnosis and returns a nonzero exit code.

Each report now includes `paper_table.csv` and `paper_table.json`: trial means
per object, object means per split, then equal S0/S1 Avg. S0 Affordance is null;
only its contribution to Avg is zero, following Table I's footnote. Missing
timing values remain null. See [paper-to-code mapping and commands](docs/paper_details.md).

## Run all locally configured objects

After the method imports the original dataset config:

```bash
r2handoversim from-dataset \
  --manifest /path/to/intent-handover/outputs/han_dataset/dataset.json \
  --output outputs/han_trials
r2handoversim demo --trials outputs/han_trials/trials.json --headless \
  --screenshot --render-every 12 --output outputs/han_isaac
```

The 16 available original point clouds are displayed directly and move with the
object. Geometry metrics still use explicit occupied-cell box proxies. This
batch uses generated hand/grasp examples and S0 evaluation because the config
does not supply original functional-region labels or the paper split. It is
separate from the three synthetic objects. See [dataset protocol](docs/dataset.md).

## Sources

- Paper/project: [R2HandoverSim](https://robot-future.github.io/r2handoversim/).
- Robot dimensions: [Universal Robots nominal DH parameters](https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics).
- Simulator: [NVIDIA Isaac Sim documentation](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/index.html).

New release code and procedural fixtures are MIT. Isaac Sim is separately
installed under NVIDIA's terms. No NVIDIA robot USDs, original dataset meshes,
MANO models, or participant data are bundled.
