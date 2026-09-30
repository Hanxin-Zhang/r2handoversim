import csv
import json
from pathlib import Path
from .evaluation import summarize


def save_results(results, output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "results.json").write_text(json.dumps(results, indent=2, allow_nan=False))
    (output / "summary.json").write_text(json.dumps(summarize(results), indent=2, allow_nan=False))
    with (output / "results.csv").open("w", newline="") as stream:
        fields = ["trial_id", "object_id", "variant", "split", "success", "first_failure",
                  "stability", "plan", "reach", "affordance", "safe"]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for result in results:
            writer.writerow({k: result.get(k, result["metrics"].get(k)) for k in fields})
    from .report import write_report
    write_report(results, output)
