#!/usr/bin/env bash
# start_formation_sitl.sh — launch 3 PX4 SITL x500_lidar360 drones in the
# obstacles_swarm Gazebo world plus a single Micro-XRCE-DDS agent, for the
# ego_planner formation-flight test (px4_swarm_run.launch.py).
#
# Drone i becomes PX4 SITL instance i (instance 0 IS used here: drone_id 0
# maps to the un-namespaced /fmu/... topics, matching the launch file).
#
# SPAWN POSES MUST MATCH px4_swarm_run.launch.py's drone_configs
# (spawn_x/spawn_y, world ENU): the px4_bridge nodes shift each instance's
# spawn-origin local frame into the shared world frame using those values.
# PX4_GZ_MODEL_POSE is "x,y" in the Gazebo (ENU) world frame.
#
#   Usage:  start_formation_sitl.sh
#   Env:    PX4_DIR (~/PX4-Autopilot)  HEADLESS (set to run gz without a GUI)
set -euo pipefail

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
WORLD="obstacles_swarm"
MODEL="x500_lidar360"
AUTOSTART=4023   # SYS_AUTOSTART for gz_x500_lidar360
DDS_PORT="${DDS_PORT:-8888}"

# Equilateral triangle wedge, side 4 m, leader in front --
# keep in sync with px4_swarm_run.launch.py.
POSES=("0,0" "-3.464,-2" "-3.464,2")

PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
[ -x "$PX4_BIN" ] || {
    echo "ERROR: $PX4_BIN not found — build PX4 first:"
    echo "       cd $PX4_DIR && make px4_sitl"
    exit 1
}
command -v MicroXRCEAgent >/dev/null || {
    echo "ERROR: MicroXRCEAgent not on PATH (install eProsima Micro-XRCE-DDS-Agent)"
    exit 1
}
[ -n "${HEADLESS:-}" ] && export HEADLESS

cleanup() {
    echo
    echo "[formation] shutting down PX4 / agent / gazebo ..."
    pkill -x px4 2>/dev/null || true
    pkill -x MicroXRCEAgent 2>/dev/null || true
    pkill -f 'gz sim' 2>/dev/null || true
}
trap cleanup INT TERM

echo "[formation] stopping any previous PX4 / agent / gazebo ..."
pkill -x px4 2>/dev/null || true
pkill -x MicroXRCEAgent 2>/dev/null || true
pkill -f 'gz sim' 2>/dev/null || true
sleep 2

echo "[formation] starting Micro-XRCE-DDS agent on udp4 :$DDS_PORT"
MicroXRCEAgent udp4 -p "$DDS_PORT" >/tmp/px4_formation_agent.log 2>&1 &

cd "$PX4_DIR"
for i in 0 1 2; do
    pose="${POSES[$i]}"
    log="/tmp/px4_formation_instance_${i}.log"
    ns=$([ "$i" -eq 0 ] && echo "(none)" || echo "/px4_$i")
    echo "[formation] instance $i  ns=$ns  sys_id=$((i + 1))  pose(ENU x,y)=$pose  log=$log"
    PX4_SYS_AUTOSTART=$AUTOSTART \
    PX4_SIM_MODEL="gz_${MODEL}" \
    PX4_GZ_WORLD="$WORLD" \
    PX4_GZ_MODEL_POSE="$pose" \
    "$PX4_BIN" -i "$i" >"$log" 2>&1 &

    if [ "$i" -eq 0 ]; then
        echo "[formation] waiting for the Gazebo server to come up (first instance) ..."
        sleep 8
    else
        sleep 3
    fi
done

echo
echo "[formation] launched 3 drones in world '$WORLD'"
echo "[formation] agent log: /tmp/px4_formation_agent.log  |  px4 logs: /tmp/px4_formation_instance_*.log"
echo "[formation] check topics:  ros2 topic list | grep -E 'fmu'"
echo "[formation] Ctrl-C here to stop everything."
wait
