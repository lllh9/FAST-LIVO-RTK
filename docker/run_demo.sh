#!/usr/bin/env bash
# Headless FAST-LIVO2-RTK (ROS 2 Humble) runner — runs INSIDE the container.
#
# 启动 FAST-LIVO2 高频局部前端和在线 iSAM2 全局后端，播放 ROS 2 bag，
# 等待数据处理完成后通过 ROS 2 服务异步重建最终优化地图；不再使用 stdin/FIFO。
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
# 清理本次验证关心的旧输出，避免历史文件造成误判。
rm -f "$OUT"/TUM/global_optimized.txt "$OUT"/global_pcd/final_optimized_map.pcd 2>/dev/null

NODE_PID=""
cleanup() {
  echo "[run_demo] cleaning up"
  [[ -n "$NODE_PID" ]] && kill "$NODE_PID" 2>/dev/null || true
  pkill -f fastlivo_mapping 2>/dev/null || true
  pkill -f "ros2 bag play"  2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [[ ! -e "$BAG" ]]; then
  echo "[run_demo] ERROR: bag not found: $BAG" >&2; exit 1
fi
echo "[run_demo] bag=$BAG rate=$RATE params=$PARAMS domain=$ROS_DOMAIN_ID"

echo "[run_demo] launching fastlivo_mapping (log -> $LOG)"
# Override outputfilepath so results land in the mounted $OUT regardless of the
# YAML default (decouples the run from the param file's path).
stdbuf -oL -eL ros2 run fast_livo fastlivo_mapping --ros-args --params-file "$PARAMS" \
  -p laserMapping.outputfilepath:="$OUT" \
  2>&1 | tee -a "$LOG" &
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

# 等待前端停止产生新 LIO 更新，让异步 iSAM2 队列完成收尾。
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
echo "[run_demo] front-end drained (~$prev LIO updates); requesting final map rebuild"
ros2 service call /global_backend/save_map std_srvs/srv/Trigger "{}" >/dev/null

echo "[run_demo] waiting for final map reconstruction..."
for _ in $(seq 1 900); do            # 大数据集重建最多等待 30 分钟。
  grep -q "Final map rebuilt:" "$LOG" 2>/dev/null && break
  sleep 2
done

# ---- verdict ----
finished=0; grep -q "Final map rebuilt:" "$LOG" 2>/dev/null && finished=1
out_ok=0; [[ -s "$OUT/TUM/global_optimized.txt" && -s "$OUT/global_pcd/final_optimized_map.pcd" ]] && out_ok=1

echo "[run_demo] ===== output trajectories ====="; ls -l "$OUT/TUM" 2>/dev/null
echo "[run_demo] ===== global map pcd =====";       ls -l "$OUT/global_pcd" 2>/dev/null

rc=1
if [[ $finished -eq 1 && $out_ok -eq 1 && ${PLAY_RC:-1} -eq 0 ]]; then
  echo "[run_demo] SUCCESS: online iSAM2 and final map reconstruction completed."; rc=0
else
  echo "[run_demo] FAILURE: finished=$finished outputs_ok=$out_ok play_rc=${PLAY_RC:-unset}" >&2
fi
exit $rc
