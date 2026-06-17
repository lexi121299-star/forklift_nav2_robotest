#!/bin/bash
# Run the full nav2 bringup (the configure-storm path that intermittently
# crashes recoveries_server with "failed to send response") N times under a
# given RMW. For each run, decide:
#   - recoveries_server reached "active" / navigation "Managed nodes are active"
#   - any crash signature appeared
# Reports per-run result and a tally.

RMW="${1:-rmw_fastrtps_cpp}"
RUNS="${2:-8}"
RUNTIME="${3:-30}"
MAP=/workspace/forklift_factory_big_map_clean.yaml
PARAMS=/workspace/forklift_nav2_demo/config/forklift_nav2_oru_test_foxy.yaml

source /opt/ros/foxy/setup.bash
source /workspace/install_foxy/setup.bash
export RMW_IMPLEMENTATION="$RMW"

echo "==================================================================="
echo "RMW=$RMW  runs=$RUNS  runtime=${RUNTIME}s"
echo "==================================================================="

ok=0; bad=0
for i in $(seq 1 "$RUNS"); do
  log=/tmp/camp_${RMW}_$i.log
  timeout "$RUNTIME" ros2 launch nav2_bringup bringup_launch.py \
    map:="$MAP" params_file:="$PARAMS" \
    use_sim_time:=false autostart:=true use_composition:=False \
    > "$log" 2>&1
  pkill -9 -f bringup_launch  2>/dev/null
  pkill -9 -f recoveries_server 2>/dev/null
  pkill -9 -f lifecycle_manager 2>/dev/null
  sleep 2

  # Crash signal only (no Gazebo/TF here, so full "active" isn't expected).
  send_fail=$(grep -ci "failed to send response" "$log")
  died=$(grep -ci "process has died" "$log")
  terminated=$(grep -ci "terminate called" "$log")

  if [[ "$send_fail" -eq 0 && "$died" -eq 0 && "$terminated" -eq 0 ]]; then
    echo "run $i: OK   (no crash)"
    ok=$((ok+1))
  else
    node=$(grep -iE "process has died|terminate called" "$log" | grep -oiE "\[[a-z_]+-[0-9]+\]" | head -1)
    echo "run $i: CRASH  send_fail=$send_fail died=$died terminate=$terminated  node=$node"
    grep -iE "failed to send response|process has died" "$log" | head -2 | sed 's/^/        /'
    bad=$((bad+1))
  fi
done

echo "-------------------------------------------------------------------"
echo "RMW=$RMW  clean=$ok  crashed=$bad / $RUNS runs"
