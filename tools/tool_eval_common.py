"""Shared helpers for the lap-evaluation tools. The RESULT field list and its
parser live here so compare_eval.py and eval_lap.c stay in sync."""

# Canonical RESULT fields, in the order eval_lap.c emits them.
FIELDS = [
    ("lap_s",            "Lap time",                        "s",  False,    True),
    ("laps",             "Laps completed (in 50 s)",        "",   True,     False),
    ("mean_cte",         "Mean cross-track error",          "m",  False,    True),
    ("worst_cte",        "Worst cross-track error",         "m",  False,    True),
    ("worst_cte_sharp",  "Worst CTE @ sharp corner",        "m",  False,    True),
    ("sharp_viol",       "Sharp-corner violations (>2 m)",  "",   True,     True),
    ("offtrack",         "Off-track ticks (cone contact)",  "",   True,     True),
]

FIELD_KEYS = [f[0] for f in FIELDS]  # ordered keys, handy for CSV columns

COUNT_KEYS = {f[0] for f in FIELDS if f[3]}  # integer counts, for int vs float formatting


# Extract the RESULT line from text and return {key: float}, or None if absent.
def parse_result(text):
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
                pass  # skip non-numeric fields
        return values
    return None
