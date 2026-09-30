import argparse
import csv
import json
from pathlib import Path
from .demos import NAMES, VARIANTS, from_selection, load_demo
from .evaluation import evaluate, summarize, validate_trial
from .results import save_results


def main(argv=None):
    parser = argparse.ArgumentParser(description="R2HandoverSim: Isaac Sim replay demos and offline evaluation")
    sub = parser.add_subparsers(dest="command", required=True)
    for command in ("demo", "evaluate"):
        p = sub.add_parser(command, help="Run Isaac Sim" if command == "demo" else "Check traces without a simulator")
        p.add_argument("--object", choices=[*NAMES, "all"], default="hammer")
        p.add_argument("--variant", choices=[*VARIANTS, "all"], default="intent_aware")
        p.add_argument("--trial", type=Path, help="Custom handover.trial.v1 JSON instead of bundled demos")
        p.add_argument("--output", type=Path, default=Path(f"outputs/{command}"))
        if command == "demo":
            p.add_argument("--headless", action="store_true")
            p.add_argument("--hold", action="store_true", help="Keep GUI open after replay")
            p.add_argument("--render-every", type=int, default=1)
            p.add_argument("--screenshot", action="store_true")
            p.add_argument("--animation", action="store_true", help="Export a USD with sampled replay animation")
    convert = sub.add_parser("from-intent", help="Convert method scene + selection to a demo joint-replay trial")
    convert.add_argument("--scene", type=Path, required=True)
    convert.add_argument("--selection", type=Path, required=True)
    convert.add_argument("--output", type=Path, default=Path("outputs/intent_trial.json"))
    args = parser.parse_args(argv)
    try:
        if args.command == "from-intent":
            trial = from_selection(json.loads(args.scene.read_text()), json.loads(args.selection.read_text()))
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(trial, indent=2, allow_nan=False))
            print(args.output.resolve())
            return
        trials = ([json.loads(args.trial.read_text())] if args.trial else
                  [load_demo(n, v) for n in (NAMES if args.object == "all" else [args.object])
                   for v in (VARIANTS if args.variant == "all" else [args.variant])])
        for trial in trials:
            validate_trial(trial)
            import re
            if not re.fullmatch(r"[a-zA-Z0-9_-]+", trial["id"]):
                raise ValueError("Trial id must contain only letters, digits, underscores and hyphens")
        args.output.mkdir(parents=True, exist_ok=True)
        if args.command == "demo":
            from .runner import replay
            results = replay(trials, args.output, args.headless, args.hold, args.render_every, args.screenshot, args.animation)
        else:
            results = [evaluate(t) for t in trials]
            for result in results:
                print(f"{result['trial_id']}: {result['first_failure'] or 'success'} (offline geometry)")
        save_results(results, args.output)
        print(f"Results: {args.output.resolve()}")
    except (ValueError, KeyError, OSError, ImportError, RuntimeError) as exc:
        parser.exit(2, f"Error: {exc}\n")
