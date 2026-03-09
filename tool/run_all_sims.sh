#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

CONFIG_PATH="${ROOT_DIR}/config/common_config.json"
PID_FILE="${SCRIPT_DIR}/.sim_pids"
LOG_DIR="${SCRIPT_DIR}/logs"

ACTION="${1:-start}"

usage() {
  echo "Usage:"
  echo "  bash tool/run_all_sims.sh start [--config <path>]"
  echo "  bash tool/run_all_sims.sh restart [--config <path>]"
  echo "  bash tool/run_all_sims.sh stop"
  echo "  bash tool/run_all_sims.sh status"
}

parse_args() {
  shift || true
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --config)
        shift
        [[ $# -gt 0 ]] || { echo "error: --config requires path"; exit 1; }
        CONFIG_PATH="$1"
        ;;
      *)
        echo "error: unknown arg: $1"
        usage
        exit 1
        ;;
    esac
    shift || true
  done
}

is_pid_running() {
  local pid="$1"
  kill -0 "$pid" >/dev/null 2>&1
}

start_one() {
  local name="$1"
  local cmd="$2"
  local log_file="${LOG_DIR}/${name}.log"

  bash -lc "cd \"${SCRIPT_DIR}\" && ${cmd}" >"${log_file}" 2>&1 &
  local pid=$!
  echo "${name}:${pid}" >>"${PID_FILE}"
  echo "[start] ${name} pid=${pid} log=${log_file}"
}

start_all() {
  mkdir -p "${LOG_DIR}"

  if [[ -f "${PID_FILE}" ]]; then
    local any_running=0
    while IFS=: read -r name pid; do
      [[ -n "${name:-}" && -n "${pid:-}" ]] || continue
      if is_pid_running "${pid}"; then
        echo "[info] already running: ${name} pid=${pid}"
        any_running=1
      fi
    done <"${PID_FILE}"
    if [[ "${any_running}" -eq 1 ]]; then
      echo "[info] existing simulator process detected, skip start."
      echo "[hint] run: bash tool/run_all_sims.sh stop"
      exit 0
    fi
    rm -f "${PID_FILE}"
  fi

  local hook_transport
  hook_transport="$(python3 - "${CONFIG_PATH}" <<'PY'
import json, sys
cfg_path = sys.argv[1]
try:
    with open(cfg_path, "r", encoding="utf-8") as f:
        root = json.load(f)
    v = root.get("runtime", {}).get("hoist_hook", {}).get("transport", "tcp")
    print(str(v).strip().lower())
except Exception:
    print("tcp")
PY
)"

  start_one "modbus_gateway" "python3 modbus_gateway_sim.py --config \"${CONFIG_PATH}\""
  if [[ "${hook_transport}" == "rtu" ]]; then
    start_one "hoist_hook" "python3 hoist_hook_rtu_sim.py --config \"${CONFIG_PATH}\""
  else
    start_one "hoist_hook" "python3 hoist_hook_tcp_sim.py --config \"${CONFIG_PATH}\""
  fi
  start_one "multi_turn_encoder" "python3 multi_turn_encoder_rtu_sim.py --config \"${CONFIG_PATH}\""
  start_one "spd_lidar" "python3 spd_lidar_dual_sim.py --config \"${CONFIG_PATH}\""

  echo "[ok] all simulators started."
}

stop_all() {
  if [[ ! -f "${PID_FILE}" ]]; then
    echo "[info] pid file not found, nothing to stop."
    return
  fi

  while IFS=: read -r name pid; do
    [[ -n "${name:-}" && -n "${pid:-}" ]] || continue
    if is_pid_running "${pid}"; then
      kill "${pid}" >/dev/null 2>&1 || true
      echo "[stop] ${name} pid=${pid}"
    else
      echo "[info] already stopped: ${name} pid=${pid}"
    fi
  done <"${PID_FILE}"

  rm -f "${PID_FILE}"
  echo "[ok] stop done."
}

status_all() {
  if [[ ! -f "${PID_FILE}" ]]; then
    echo "[status] no pid file."
    return
  fi

  while IFS=: read -r name pid; do
    [[ -n "${name:-}" && -n "${pid:-}" ]] || continue
    if is_pid_running "${pid}"; then
      echo "[status] running ${name} pid=${pid}"
    else
      echo "[status] stopped ${name} pid=${pid}"
    fi
  done <"${PID_FILE}"
}

case "${ACTION}" in
  start)
    parse_args "$@"
    start_all
    ;;
  restart)
    parse_args "$@"
    stop_all
    start_all
    ;;
  stop)
    stop_all
    ;;
  status)
    status_all
    ;;
  *)
    usage
    exit 1
    ;;
esac

