"""Shared helpers for the lap-evaluation tools.

The headless evaluator (tools/tool_eval_lap.c) prints one machine-readable line:

    RESULT mean_cte=.. worst_cte=.. worst_cte_sharp=.. sharp_viol=.. \
           offtrack=.. laps=.. lap_s=..

compare_eval.py (the CI delta table) parses that line, so the field list and the
parser live here in one place to keep them in sync.
"""

# Canonical RESULT fields, in the order eval_lap.c emits them.
#   key: token name in the RESULT line
#   label: human-readable name for tables
#   unit: display unit ("" for dimensionless counts)
#   is_count: True for integer counts (laps, ticks), False for real measurements
#   lower_is_better: True if a smaller value is an improvement
FIELDS = [
    # key                label                              unit  is_count  lower_better
    ("lap_s",            "Lap time",                        "s",  False,    True),
    ("laps",             "Laps completed (in 50 s)",        "",   True,     False),
    ("mean_cte",         "Mean cross-track error",          "m",  False,    True),
    ("worst_cte",        "Worst cross-track error",         "m",  False,    True),
    ("worst_cte_sharp",  "Worst CTE @ sharp corner",        "m",  False,    True),
    ("sharp_viol",       "Sharp-corner violations (>2 m)",  "",   True,     True),
    ("offtrack",         "Off-track ticks (cone contact)",  "",   True,     True),
]

# Ordered field keys - handy for CSV columns.
FIELD_KEYS = [f[0] for f in FIELDS]

# Keys that are integer counts (used for int vs float formatting).
COUNT_KEYS = {f[0] for f in FIELDS if f[3]}


def parse_result(text):
    """Extract the RESULT line from `text` and return {key: float}.

    `text` may be the evaluator's full output or a bare "RESULT ..." string.
    Non-numeric tokens are skipped. Returns None if there is no RESULT line.
    """
    for line in text.splitlines():
        if not line.startswith("RESULT"):
            continue
        values = {}
        for token in line[len("RESULT"):].split():
            key, sep, val = token.partition("=")
            if not sep:
                continue
            try:
                values[key] = float(val)
            except ValueError:
                pass  # ignore non-numeric fields rather than crashing
        return values
    return None
