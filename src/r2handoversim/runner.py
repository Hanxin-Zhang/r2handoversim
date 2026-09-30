"""Supervise Kit: shutdown may terminate its Python interpreter before returning."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import uuid
from .evaluation import validate_trial


def check_completion(output, run_id, expected, returncode):
    try:
        manifest = json.loads((Path(output) / "run.json").read_text())
        if (returncode != 0 or manifest.get("run_id") != run_id
                or manifest.get("status") != "succeeded"
                or manifest.get("completed_trials") != expected):
            raise RuntimeError(f"Isaac Sim run incomplete (exit {returncode}): {manifest.get('error') or manifest.get('status')}")
        results = json.loads((Path(output) / "results.json").read_text())
        if len(results) != expected:
            raise RuntimeError("Result count differs from completed trial count")
        return results
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"Isaac Sim did not produce a valid run manifest/result: {exc}") from exc


def replay(trials, output, headless=False, hold=False, render_every=1, screenshot=False, animation=False):
    if not trials:
        raise ValueError("At least one trial is required")
    if render_every < 1:
        raise ValueError("render_every must be at least 1")
    for trial in trials:
        validate_trial(trial)
    if len({trial["id"] for trial in trials}) != len(trials):
        raise ValueError("Duplicate trial ids would overwrite exported scenes")
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    run_id = str(uuid.uuid4())
    (output / "run.json").write_text(json.dumps({"run_id": run_id, "status": "starting",
        "expected_trials": len(trials), "completed_trials": 0}))
    job = {"trials": trials, "output": str(output), "headless": headless, "hold": hold,
           "render_every": render_every, "screenshot": screenshot, "animation": animation, "run_id": run_id}
    with tempfile.TemporaryDirectory(prefix="r2handover-job-") as tmp:
        path = Path(tmp) / "job.json"
        path.write_text(json.dumps(job, allow_nan=False))
        env = os.environ.copy()
        source = str(Path(__file__).resolve().parent.parent)
        env["PYTHONPATH"] = source + (os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")
        completed = subprocess.run([sys.executable, "-m", "r2handoversim.worker", str(path)], env=env)
    return check_completion(output, run_id, len(trials), completed.returncode)
