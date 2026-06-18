#!/usr/bin/env bash
# Headless FAST-LIVO2-RTK (ROS 2 Humble) runner — runs INSIDE the container.
#
# Mirrors the ROS 1 docker/run_demo.sh: starts the node, plays the converted
# ROS 2 bag, then triggers the offline GTSAM/RTK back-end. The optimization
# thread blocks on std::cin.get() (optimization.cpp), so we give the node a FIFO
# as stdin and keep a writer open; a newline written to the FIFO triggers the
# batch optimization once the front-end has drained.
set -o pipefail

BAG="${1:-/bags/HH-LVGO-01-ros2}"
RATE="${RATE:-1.0}"
WS=/root/ros2_ws
PKG=$WS/src/fast_livo
PARAMS="${PARAMS:-$PKG/config/HH-LVGO.yaml}"
OUT="$PKG/output"
LOG="$OUT/run.log"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-71}"   # isolate from other ROS 2 nodes

mkdir -p "$OUT"/{TUM,vel,global_pcd,scan_pcd,debug} "$PKG/Log/result" "$PKG/Log/PCD"
: > "$LOG"
# Clear prior result artifacts so the success check cannot pass on STALE outputs.
rm -f "$OUT"/TUM/opt_trajectory_*.txt "$OUT"/global_pcd/*.pcd "$OUT"/vel/*.txt 2>/dev/null

NODE_PID=""
cleanup() {
  echo "[run_demo] cleaning up"
  { exec 3>&-; } 2>/dev/null || true
  [[ -n "$NODE_PID" ]] && kill "$NODE_PID" 2>/dev/null || true
  pkill -f fastlivo_mapping 2>/dev/null || true
  pkill -f "ros2 bag play"  2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [[ ! -e "$BAG" ]]; then
  echo "[run_demo] ERROR: bag not found: $BAG" >&2; exit 1
fi
echo "[run_demo] bag=$BAG rate=$RATE params=$PARAMS domain=$ROS_DOMAIN_ID"

# Controllable stdin: keep a writer (fd 3) open so std::cin.get() blocks.
FIFO=/tmp/livo_stdin
rm -f "$FIFO"; mkfifo "$FIFO"
exec 3<>"$FIFO"

echo "[run_demo] launching fastlivo_mapping (log -> $LOG)"
# Override outputfilepath so results land in the mounted $OUT regardless of the
# YAML default (decouples the run from the param file's path).
stdbuf -oL -eL ros2 run fast_livo fastlivo_mapping --ros-args --params-file "$PARAMS" \
  -p laserMapping.outputfilepath:="$OUT" \
  <"$FIFO" 2>&1 | tee -a "$LOG" &
NODE_PID=$!

# Readiness: wait until the node has subscribed to the input topics, so bag
# play does not drop the first messages (the GPS ENU origin is set from the
# first received fix).
echo "[run_demo] waiting for node subscriptions..."
ready=0
for _ in $(seq 1 90); do
  cnt=0
  for t in /livox/lidar /livox/imu /left_camera/image /ublox_driver/receiver_pvt; do
    n="$(ros2 topic info "$t" 2>/dev/null | awk -F': ' '/Subscription count/{print $2}')"
    [[ "${n:-0}" -ge 1 ]] && cnt=$((cnt+1))
  done
  if [[ $cnt -ge 4 ]]; then ready=1; break; fi
  sleep 1
done
[[ $ready -eq 1 ]] && echo "[run_demo] node subscribed to all inputs" \
                   || echo "[run_demo] WARN: subscription check timed out; playing anyway"

echo "[run_demo] playing bag (rate=$RATE)"
ros2 bag play "$BAG" --rate "$RATE"; PLAY_RC=$?
[[ $PLAY_RC -eq 0 ]] || echo "[run_demo] WARN: ros2 bag play exited with $PLAY_RC"

# Drain: wait until the front-end stops producing new LIO updates before
# triggering the back-end (avoids optimizing while callbacks still append).
echo "[run_demo] bag finished; draining front-end..."
prev=-1; stable=0
for _ in $(seq 1 120); do
  cur="$(grep -c 'LIO Update' "$LOG" 2>/dev/null)"; cur="${cur:-0}"
  if [[ "$cur" == "$prev" ]]; then
    stable=$((stable+1)); [[ $stable -ge 4 ]] && break
  else
    stable=0
  fi
  prev="$cur"; sleep 1
done
echo "[run_demo] front-end drained (~$prev LIO updates); triggering offline optimization"
printf '\n' >&3

echo "[run_demo] waiting for back-end completion..."
for _ in $(seq 1 900); do            # up to 30 min
  grep -q "\[Offline Optimization\] Finished\." "$LOG" 2>/dev/null && break
  sleep 2
done

# ---- verdict ----
finished=0; grep -q "\[Offline Optimization\] Finished\." "$LOG" 2>/dev/null && finished=1
out_ok=0; [[ -s "$OUT/TUM/opt_trajectory_after.txt" && -s "$OUT/global_pcd/after_optimization.pcd" ]] && out_ok=1

echo "[run_demo] ===== output trajectories ====="; ls -l "$OUT/TUM" 2>/dev/null
echo "[run_demo] ===== global map pcd =====";       ls -l "$OUT/global_pcd" 2>/dev/null

rc=1
if [[ $finished -eq 1 && $out_ok -eq 1 && ${PLAY_RC:-1} -eq 0 ]]; then
  echo "[run_demo] SUCCESS: back-end finished and fresh outputs written."; rc=0
else
  echo "[run_demo] FAILURE: finished=$finished outputs_ok=$out_ok play_rc=${PLAY_RC:-unset}" >&2
fi
exit $rc
