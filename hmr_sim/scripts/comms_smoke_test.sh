#!/usr/bin/env bash
# Smoke test for hmr_comms_sim_node: relay, link gating, reliable backlog.
# Moved comments: docs/hmr_sim_code_notes.md
set -u
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v miniconda | paste -sd:)
unset PYTHONPATH CONDA_PREFIX CONDA_DEFAULT_ENV LD_LIBRARY_PATH 2>/dev/null || true
set +u
source /opt/ros/humble/setup.bash
source /home/kalhan/Projects/hmr_explo_ws/hmr_explo/ws/install/setup.bash
set -u
export ROS_DOMAIN_ID=$((60 + RANDOM % 30))  # fresh domain per run: strays from a previous run cannot interfere

OUT="$(mktemp -d /tmp/comms_smoke_XXXXXX)"
echo "logs: $OUT"
# ros2 run does not forward SIGTERM to its child: kill the node binary by its
# full path too (comm is truncated to 15 chars, so pkill -x can never match).
# Bracket pattern so pkill does not match this script's own cmdline.
pkill -f 'lib/hmr_sim/hmr_comms_sim_nod[e]' 2>/dev/null; sleep 1
PIDS=()
cleanup() { for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done
            pkill -f 'lib/hmr_sim/hmr_comms_sim_nod[e]' 2>/dev/null; sleep 1
            for p in "${PIDS[@]}"; do kill -9 "$p" 2>/dev/null; done; }
trap cleanup EXIT

start() { "$@" >/dev/null 2>&1 & PIDS+=($!); }

# --- emulator node (no world file -> pure distance model) -------------------
ros2 run hmr_sim hmr_comms_sim_node --ros-args \
  -p robot_names:="[alpha,beta]" \
  -p reliable_topics:="[chatter_rel]" \
  -p best_effort_topics:="[chatter_be]" \
  -p use_sim_time:=false -p stats_period_s:=5.0 \
  > "$OUT/node.log" 2>&1 &
NODE_PID=$!; PIDS+=($NODE_PID)

# --- poses: alpha at origin, beta 10 m away (good link) ---------------------
start ros2 topic pub -r 10 /alpha/odom_ground_truth nav_msgs/msg/Odometry \
  "{pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}}}}"
ros2 topic pub -r 10 /beta/odom_ground_truth nav_msgs/msg/Odometry \
  "{pose: {pose: {position: {x: 10.0, y: 0.0, z: 0.0}}}}" >/dev/null 2>&1 &
BETA_NEAR=$!; PIDS+=($BETA_NEAR)

# --- traffic: 2 Hz on both policies ----------------------------------------
# volatile like real sensor/map streams; ros2 topic pub's transient_local
# default would latch the last relayed msg on the rx topic and T4's echo
# (QoS auto-match) would replay it as a false "leak".
start ros2 topic pub --qos-durability volatile -r 2 /alpha/chatter_rel std_msgs/msg/String "{data: hello_rel}"
start ros2 topic pub --qos-durability volatile -r 2 /alpha/chatter_be  std_msgs/msg/String "{data: hello_be}"

sleep 8
echo "=== T1 link_states (near, expect bandwidth 72, connected 1) ==="
timeout 6 ros2 topic echo /hmr_comms_sim/link_states --once 2>/dev/null | head -20
echo "=== T2 reliable relayed while near (expect hello_rel) ==="
timeout 8 ros2 topic echo /beta/rx/alpha/chatter_rel --once 2>/dev/null
echo "=== T3 best-effort relayed while near (expect hello_be) ==="
timeout 8 ros2 topic echo /beta/rx/alpha/chatter_be --once 2>/dev/null

# --- take beta out of range -------------------------------------------------
kill $BETA_NEAR 2>/dev/null
# 2e6 m -> mean SNR ~ -44 dB: even the +/-4.8 dB fade cannot flap the link up.
ros2 topic pub -r 10 /beta/odom_ground_truth nav_msgs/msg/Odometry \
  "{pose: {pose: {position: {x: 2000000.0, y: 0.0, z: 0.0}}}}" >/dev/null 2>&1 &
BETA_FAR=$!; PIDS+=($BETA_FAR)
sleep 5   # tier-by-tier descent 72->28.9->7.2->0 takes ~1.5 s at 5 Hz

echo "=== T4 link down: reliable rx must be SILENT (expect timeout) ==="
if timeout 5 ros2 topic echo /beta/rx/alpha/chatter_rel --once >/dev/null 2>&1; then
  echo "FAIL: message leaked through a down link"
else
  echo "OK: no delivery while disconnected"
fi

sleep 10  # let reliable backlog build (~2 Hz * ~15 s total down)

# --- bring beta back --------------------------------------------------------
echo "=== T5 reconnect: backlog must arrive late, not be lost ==="
# The counter subscribes while the link is still down so the whole reconnect
# burst is captured; a count above steady state proves queueing. The reader
# needs a deep queue (200): the default depth 10 would shed the burst.
# (notes: smoke-t5-reconnect-burst)
( timeout 14 ros2 topic echo --qos-depth 200 /beta/rx/alpha/chatter_rel 2>/dev/null | grep -c hello_rel > "$OUT/t5_count" ) &
T5_PID=$!
sleep 2
kill $BETA_FAR 2>/dev/null
ros2 topic pub -r 10 /beta/odom_ground_truth nav_msgs/msg/Odometry \
  "{pose: {pose: {position: {x: 10.0, y: 0.0, z: 0.0}}}}" >/dev/null 2>&1 &
PIDS+=($!)
wait $T5_PID 2>/dev/null
COUNT=$(cat "$OUT/t5_count" 2>/dev/null || echo 0)
echo "received $COUNT reliable msgs in the 12 s spanning reconnect (steady-state alone: ~24)"
if [ "$COUNT" -gt 40 ]; then echo "OK: backlog delivered"; else echo "FAIL: backlog missing"; fi

echo "=== T6 node log (stats + startup) ==="
grep -a -E "hmr_comms_sim up|relay totals|Loaded|world_sdf" "$OUT/node.log" | tail -6
echo "=== T7 last stats JSON ==="
timeout 8 ros2 topic echo /hmr_comms_sim/stats --once 2>/dev/null | head -4

cleanup
trap - EXIT
PIDS=()

# --- T8: SDF tree parsing on the real flatforest world ----------------------
WORLD=/home/kalhan/Projects/hmr_explo_ws/hmr_explo/ws/src/hmr_sim/hmr_sim/worlds/flatforest/flatforestv2.sdf
ros2 run hmr_sim hmr_comms_sim_node --ros-args \
  -p robot_names:="[alpha,beta]" -p world_sdf:="$WORLD" -p use_sim_time:=false \
  > "$OUT/node_trees.log" 2>&1 &
TREE_PID=$!
sleep 4
kill $TREE_PID 2>/dev/null
pkill -f 'lib/hmr_sim/hmr_comms_sim_nod[e]' 2>/dev/null
# The expected count is derived from the world file (oak models plus pine
# includes of model://cmu_pine_tree), so this asserts the parser, not a
# remembered number. The pines guard include matching.
# (notes: smoke-t8-tree-parse-truth)
T8_OAKS=$(grep -c '<model name="Oak tree' "$WORLD")
T8_PINES=$(grep -c 'model://cmu_pine_tree' "$WORLD")
T8_EXPECT=$((T8_OAKS + T8_PINES))
T8_GOT=$(sed -n 's/.*Loaded \([0-9]\+\) tree positions.*/\1/p' "$OUT/node_trees.log" | head -1)
echo "=== T8 tree parse (expect $T8_EXPECT = $T8_OAKS oak + $T8_PINES pine) ==="
grep -E "Loaded|Not counted" "$OUT/node_trees.log" | head -3
if [ "$T8_GOT" = "$T8_EXPECT" ]; then
  echo "T8 PASS: parsed $T8_GOT trees"
else
  echo "T8 FAIL: parsed '${T8_GOT:-<no Loaded line>}', expected $T8_EXPECT"
  T8_FAILED=1
fi
echo "=== DONE ==="
[ -n "${T8_FAILED:-}" ] && exit 1
exit 0
