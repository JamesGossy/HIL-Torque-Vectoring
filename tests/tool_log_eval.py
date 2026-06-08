#!/usr/bin/env python3
"""Append a lap-evaluation RESULT to a local history CSV.

Reads the evaluator output (path as argv[1], or stdin), extracts the RESULT
line, and appends one row to the history CSV (argv[2], default
tests/eval_history.csv) tagged with an ISO timestamp, the current git SHA, and
whether the working tree was dirty. The header is written once, when the file is
created.

This is the local counterpart to tool_compare_eval.py's CI delta: it lets you
track how mean/worst CTE and lap time move across tuning iterations on your
machine. The CSV is gitignored.

See tool_eval_common.py for the RESULT line format and field list.

Usage:
    tool_log_eval.py [eval_output.txt] [history.csv]
"""

import csv
import datetime
import os
import subprocess
import sys

from tool_eval_common import FIELDS, FIELD_KEYS, COUNT_KEYS, parse_result

META_FIELDS = ["timestamp", "sha", "dirty"]


def git_info():
    """Return (short_sha, dirty_flag). Best-effort; '?'/'' if git unavailable."""
    def run(args):
        return subprocess.check_output(args, stderr=subprocess.DEVNULL).decode().strip()
    try:
        sha = run(["git", "rev-parse", "--short", "HEAD"])
    except (subprocess.CalledProcessError, OSError):
        sha = "?"
    try:
        dirty = "dirty" if run(["git", "status", "--porcelain"]) else "clean"
    except (subprocess.CalledProcessError, OSError):
        dirty = ""
    return sha, dirty


def fmt_csv(key, value):
    """Format a value for the CSV: counts as ints, measurements to 3 decimals."""
    if value is None:
        return ""
    return str(int(round(value))) if key in COUNT_KEYS else f"{value:.3f}"


def read_input(src):
    """Return the evaluator text. For stdin ('-'), echo it through to stdout so
    `eval_lap | tool_log_eval.py -` stays transparent (you still see the metrics)."""
    if src != "-":
        with open(src, encoding="utf-8") as fh:
            return fh.read()
    text = sys.stdin.read()
    sys.stdout.write(text)
    sys.stdout.flush()
    return text


def main(argv=None):
    argv = argv if argv is not None else sys.argv[1:]
    src = argv[0] if len(argv) >= 1 else "-"
    csv_path = argv[1] if len(argv) >= 2 else "tests/eval_history.csv"

    values = parse_result(read_input(src))
    if values is None:
        print("log_eval: no RESULT line found; nothing logged", file=sys.stderr)
        return 1

    sha, dirty = git_info()
    row = {
        "timestamp": datetime.datetime.now().isoformat(timespec="seconds"),
        "sha": sha,
        "dirty": dirty,
    }
    for key in FIELD_KEYS:
        row[key] = fmt_csv(key, values.get(key))

    write_header = not os.path.exists(csv_path) or os.path.getsize(csv_path) == 0
    with open(csv_path, "a", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=META_FIELDS + FIELD_KEYS)
        if write_header:
            writer.writeheader()
        writer.writerow(row)

    print(f"logged lap_s={row['lap_s']} offtrack={row['offtrack']} "
          f"mean_cte={row['mean_cte']} -> {csv_path}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
