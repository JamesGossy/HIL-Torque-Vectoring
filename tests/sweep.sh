#!/usr/bin/env bash
#
# tests/sweep.sh — parameter sweep over the motion-controller tuning gains.
#
# Recompiles the headless lap evaluator (tests/eval_lap.c) once per parameter
# combination, overriding the tuned defaults via -D, runs a 50 s evaluation, and
# prints a table sorted by lap time among the configs that completed a CLEAN lap
# (no cone contact).  The three gains below are the highest-leverage ones; edit
# the value lists to widen or narrow the search.
#
# Usage (from repo root):
#   bash tests/sweep.sh
#
# Requires gcc + the same source files as `make`.

set -u
cd "$(dirname "$0")/.."

OUT=HIL_Firmware/build/eval_sweep
SRCS="tests/eval_lap.c \
  HIL_Firmware/src/motion_control.c HIL_Firmware/src/vehicle_model.c \
  HIL_Firmware/src/track.c HIL_Firmware/src/path_planning.c \
  ECU_Firmware/src/torque_vectoring.c"
INC="-I HIL_Firmware/include -I shared -I ECU_Firmware/include"

# Sweep grids (the tuned defaults sit inside each list).  Keep values written
# with a decimal point so the 'f' suffix lands on a float constant, not an int.
ALAT_LIST="5.0 7.0 9.0 11.0"     # MAX_LATERAL_ACCEL_MS2 — corner speed budget
KLA_LIST="0.35 0.45 0.55"        # K_LOOKAHEAD           — look-ahead time, s
LMIN_LIST="1.8 2.2 2.8"          # LOOKAHEAD_MIN_M       — min look-ahead, m

mkdir -p HIL_Firmware/build
printf "%-6s %-5s %-5s | %-8s %-9s %-8s %-5s %s\n" \
       alat klah lmin "lap_s" "worstCTE" "meanCTE" "off" "verdict"
echo "---------------------------------------------------------------------"

best_lap=9999; best_cfg=""
for ALAT in $ALAT_LIST; do
 for KLA in $KLA_LIST; do
  for LMIN in $LMIN_LIST; do
    if ! gcc -std=c11 -O2 $INC \
        -DMAX_LATERAL_ACCEL_MS2=${ALAT}f \
        -DK_LOOKAHEAD=${KLA}f \
        -DLOOKAHEAD_MIN_M=${LMIN}f \
        -o "$OUT" $SRCS -lm 2>/dev/null; then
      echo "build failed $ALAT $KLA $LMIN"; continue
    fi

    RESULT=$("$OUT" | grep '^RESULT')
    # parse "key=val key=val ..." into EVAL_<key> shell variables
    unset EVAL_lap_s EVAL_worst_cte EVAL_mean_cte EVAL_offtrack EVAL_laps
    for kv in ${RESULT#RESULT }; do
      eval "EVAL_${kv%%=*}=${kv#*=}"
    done

    verdict="ok"
    [ "${EVAL_offtrack}" != "0" ] && verdict="OFF-TRACK"
    [ "${EVAL_laps}" = "0" ]      && verdict="NO-LAP"

    printf "%-6s %-5s %-5s | %-8s %-9s %-8s %-5s %s\n" \
           "$ALAT" "$KLA" "$LMIN" "$EVAL_lap_s" "$EVAL_worst_cte" \
           "$EVAL_mean_cte" "$EVAL_offtrack" "$verdict"

    if [ "$verdict" = "ok" ] && awk "BEGIN{exit !($EVAL_lap_s < $best_lap)}"; then
      best_lap=$EVAL_lap_s; best_cfg="alat=$ALAT klah=$KLA lmin=$LMIN"
    fi
  done
 done
done

echo "---------------------------------------------------------------------"
echo "FASTEST CLEAN LAP: ${best_lap}s  ($best_cfg)"
