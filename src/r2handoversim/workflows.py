"""Interchange workflows for complete neural runs and paired method ablations."""
import json
from pathlib import Path
import numpy as np
from .demos import from_selection
from .geometry import corners, inverse, points, transform
from .robot import GOAL, tcp


def read_relative(root, name):
    path = (root/name).resolve()
    if not path.is_relative_to(root):
        raise ValueError("Input path escapes manifest directory")
    return json.loads(path.read_text())


def from_pipeline(path, seed=0):
    path = Path(path).resolve()
    run = json.loads(path.read_text())
    if run.get("schema_version") != "handover.pipeline.v1" or run.get("status") != "succeeded":
        raise ValueError("Expected a successfully completed method pipeline")
    files = run["files"]
    scene, selection = [read_relative(path.parent, files[k]) for k in ("scene", "selection")]
    delivery = read_relative(path.parent, files["delivery"]) if "delivery" in files else None
    trial = from_selection(scene, selection, delivery=delivery)
    if delivery is not None:
        from .planning import plan_trial
        trial = plan_trial(trial, seed=seed)
    return trial


def from_experiment(path, output, seed=0, iterations=200):
    """Keep the receiver/object target fixed within each object across all modes."""
    from .planning import plan_trial
    path, output = Path(path).resolve(), Path(output)
    experiment = json.loads(path.read_text())
    if experiment.get("schema_version") != "handover.experiment.v1":
        raise ValueError("Expected handover.experiment.v1")
    modes = experiment["modes"]
    if not modes or len(set(modes)) != len(modes) or any(m not in ("FS", "A1", "A2", "A3") for m in modes):
        raise ValueError("Expected distinct FS/A1/A2/A3 modes")
    trials, skipped, ids = [], [], set()
    for row in experiment["objects"]:
        if row["object_id"] in ids: raise ValueError("Duplicate experiment object")
        ids.add(row["object_id"])
        scene = read_relative(path.parent, row["scene"])
        if scene["object"]["id"] != row["object_id"]: raise ValueError("Experiment object id mismatch")
        selections = {m: read_relative(path.parent, row["selections"][m]) for m in modes}
        available = [m for m in modes if selections[m].get("status") == "ok" and selections[m].get("selected")]
        if not available:
            skipped.extend({"object_id": row["object_id"], "mode": m, "reason": "no_feasible_grasp"} for m in modes)
            continue
        reference = "FS" if "FS" in available else available[0]
        world_object = tcp(GOAL)@inverse(selections[reference]["selected"]["T_object_gripper"])
        # Stored replay fixtures predate table-aware planning. Raise the common
        # target until every object/hand proxy is 6 cm above the demo tabletop.
        local = np.concatenate([corners(b) for b in scene["object"]["boxes"]+scene["receiving_hand"]["boxes"]])
        clearance_lift = max(0., .735+.06-float(points(world_object, local)[:, 2].min()))
        world_object[2, 3] += clearance_lift
        for mode in modes:
            selection = selections[mode]
            if mode not in available:
                skipped.append({"object_id": row["object_id"], "mode": mode, "reason": selection.get("status")})
                continue
            if selection.get("mode") != mode: raise ValueError("Selection mode differs from experiment manifest")
            delivery = {"schema_version": "handover.delivery.v1", "units": "m", "object_id": row["object_id"],
                        "grasp_id": selection["selected"]["id"], "T_world_object": world_object.tolist(),
                        "T_world_gripper": (world_object@transform(selection["selected"]["T_object_gripper"])).tolist()}
            trial = from_selection(scene, selection, delivery=delivery, variant=mode)
            trial["experiment"] = {"reference_mode": reference, "receiver_policy": "fixed world hand and object target across modes",
                                   "table_clearance_lift_m": clearance_lift}
            trial = plan_trial(trial, seed=seed, iterations=iterations)
            trials.append(trial)
    if not trials:
        raise ValueError("No feasible grasps to convert; method experiment contains selection failures")
    output.mkdir(parents=True, exist_ok=True)
    (output/"trials.json").write_text(json.dumps(trials, allow_nan=False))
    state = {"converted": len(trials), "planning_failures": sum(t["planning"]["status"] == "failed" for t in trials),
             "skipped_selections": skipped, "receiver_policy": "fixed per object, shared by all modes",
             "scope": "Planning failures retained; infeasible grasp selections listed separately, not silently counted as benchmark successes"}
    (output/"conversion.json").write_text(json.dumps(state, indent=2))
    return trials, state
