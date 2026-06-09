#!/usr/bin/env python3
"""Compare two lap-evaluation RESULT lines and render a delta table.

Parses a "current" RESULT and an optional "base" RESULT (e.g. from the target
branch) and prints a Markdown table showing each metric and its change versus
base. CI uses this to render a commit-to-commit delta on the GitHub Actions run
summary; it works the same way locally.

See tool_eval_common.py for the RESULT line format and field list.

Usage:
    # delta vs a base result
    tool_compare_eval.py --current cur.txt --base base.txt

    # current only (no base available - first run / new branch)
    tool_compare_eval.py --current cur.txt

Each input may be a file (containing the full evaluator output) or a literal
"RESULT ..." string.

Exit code is 0, unless --gate is passed and a regression beyond tolerance is
found (lap time worse than --lap-tol, or more off-track ticks than base).
"""

import argparse
import sys

from tool_eval_common import FIELDS, COUNT_KEYS, parse_result

# The table uses Unicode (Δ, ✅, ⚠️). Windows' default console encoding (cp1252)
# cannot encode these; force UTF-8 so the script works locally as well as in CI.
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8")
    except (AttributeError, ValueError):
        pass  # already UTF-8, or a stream that can't be reconfigured


def read_source(arg):
    """Return the text for a CLI argument that is a path or a literal RESULT."""
    if arg is None:
        return None
    if arg.lstrip().startswith("RESULT"):
        return arg
    try:
        with open(arg, "r", encoding="utf-8") as fh:
            return fh.read()
    except OSError:
        return None


def fmt_value(key, value):
    """Format a metric value: counts as ints, measurements to 3 decimals."""
    if value is None:
        return "n/a"
    return str(int(round(value))) if key in COUNT_KEYS else f"{value:.3f}"


def fmt_delta(key, lower_is_better, cur, base):
    """Format the change vs base with a ✅/⚠️ marker. '' if either is missing."""
    if base is None or cur is None:
        return ""
    delta = cur - base
    if abs(delta) < 1e-9:
        return "±0"
    sign = "+" if delta > 0 else ""
    num = f"{sign}{int(round(delta))}" if key in COUNT_KEYS else f"{sign}{delta:.3f}"
    improved = (delta < 0) if lower_is_better else (delta > 0)
    return f"{num}{' ✅' if improved else ' ⚠️'}"


def render_table(cur, base):
    have_base = base is not None
    rows = ["| Metric | Value |" + (" Δ vs base |" if have_base else ""),
            "| --- | --- |" + (" --- |" if have_base else "")]
    for key, label, unit, _is_count, lower_is_better in FIELDS:
        cur_v = cur.get(key) if cur else None
        unit_s = f" {unit}" if unit else ""
        row = f"| {label} | {fmt_value(key, cur_v)}{unit_s} |"
        if have_base:
            row += f" {fmt_delta(key, lower_is_better, cur_v, base.get(key))} |"
        rows.append(row)
    return "\n".join(rows)


def find_regressions(cur, base, lap_tol):
    """Return a list of human-readable regression strings (empty if none).

    Soft gate only: lap time must not worsen beyond `lap_tol`, and off-track
    ticks must not increase. Hard failures (no lap completed, gross off-track)
    are handled by the CI step itself, not here.
    """
    regressions = []
    cur_lap, base_lap = cur.get("lap_s"), base.get("lap_s")
    if cur_lap is not None and base_lap is not None and base_lap > 0:
        if cur_lap < 0:
            regressions.append("lap no longer completed")
        elif cur_lap - base_lap > lap_tol:
            regressions.append(
                f"lap time +{cur_lap - base_lap:.2f}s (> {lap_tol:.2f}s tolerance)")
    cur_off, base_off = cur.get("offtrack"), base.get("offtrack")
    if cur_off is not None and base_off is not None and cur_off > base_off:
        regressions.append(f"off-track +{int(cur_off - base_off)} ticks")
    return regressions


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--current", required=True,
                    help="Path or literal RESULT string for the current run")
    ap.add_argument("--base", default=None,
                    help="Path or literal RESULT string for the base run (optional)")
    ap.add_argument("--gate", action="store_true",
                    help="Exit non-zero if a regression beyond tolerance is found")
    ap.add_argument("--lap-tol", type=float, default=0.3,
                    help="Lap-time regression tolerance in seconds (default 0.3)")
    args = ap.parse_args(argv)

    cur = parse_result(read_source(args.current) or "")
    if cur is None:
        print("error: no RESULT line found in --current input", file=sys.stderr)
        return 2
    base = parse_result(read_source(args.base) or "") if args.base else None

    print(render_table(cur, base))

    if args.gate and base is not None:
        regressions = find_regressions(cur, base, args.lap_tol)
        if regressions:
            print("\nREGRESSION: " + "; ".join(regressions), file=sys.stderr)
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
