"""CLI for dry-run demo and condition-cue replay; no hardware support."""
import argparse
import json
from dataclasses import asdict
from pathlib import Path

from .models import CueVector
from .router import replay, route


def main() -> None:
    parser = argparse.ArgumentParser(prog="rcaer", description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    replay_cmd = sub.add_parser("replay", help="dry-run tier decisions on JSONL cue vectors")
    replay_cmd.add_argument("input", type=Path)
    replay_cmd.add_argument("--remove-cue", choices=["zD", "zH", "zT", "zE", "zM", "zA", "zC"])
    replay_cmd.add_argument("--material", type=float)
    args = parser.parse_args()
    if args.command == "replay":
        with args.input.open(encoding="utf-8") as f:
            for line_number, line in enumerate(f, 1):
                if not line.strip():
                    continue
                item = json.loads(line)
                row = item.get("cues", item)
                cues = CueVector(**{key: row[key] for key in
                                    ("zD", "zH", "zT", "zE", "zM", "zA", "zC")})
                original_risk, original_tier, _ = route(cues)
                comparison = replay(cues, remove_cue=args.remove_cue,
                                    replacement_material=args.material)
                print(json.dumps({"line": line_number, "original_risk": original_risk,
                                  "original_tier": original_tier,
                                  "replay_risk": comparison["risk"],
                                  "replay_tier": comparison["tier"],
                                  "route_changed": original_tier != comparison["tier"],
                                  "replay_profile": asdict(comparison["profile"]),
                                  "outcome": "NOT_ESTIMABLE_FROM_REPLAY"}))
