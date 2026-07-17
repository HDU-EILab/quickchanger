#!/usr/bin/env bash
# Preempt fr10_web_teach.py while a pick-point script owns the Fairino SDK.
set -euo pipefail

WEB_TEACH_PATTERN="${WEB_TEACH_PATTERN:-fr10_web_teach.py --port 8890}"
WEB_TEACH_DIR="${WEB_TEACH_DIR:-/opt/fr10_web_teach/web_teach_192.168.58.2}"
WEB_TEACH_PY="${WEB_TEACH_PY:-/home/nvidia/miniconda3/envs/fr10-fairino/bin/python}"
WEB_TEACH_LOG="${WEB_TEACH_LOG:-/tmp/fr10_web_teach_preempt.log}"
WEB_TEACH_PORT="${WEB_TEACH_PORT:-8890}"
PREEMPT_WEB_TEACH="${PREEMPT_WEB_TEACH:-1}"

_web_pids() {
  pgrep -f "${WEB_TEACH_PATTERN}" || true
}

_wait_no_pids() {
  local deadline=$((SECONDS + 8))
  while [[ "${SECONDS}" -lt "${deadline}" ]]; do
    if [[ -z "$(_web_pids)" ]]; then
      return 0
    fi
    sleep 0.2
  done
  return 1
}

_start_web_teach() {
  if [[ -n "$(_web_pids)" ]]; then
    return 0
  fi
  if [[ ! -d "${WEB_TEACH_DIR}" ]]; then
    echo "[WARN] web_teach 目录不存在，无法恢复: ${WEB_TEACH_DIR}" >&2
    return 0
  fi
  echo "[PREEMPT] 恢复 fr10_web_teach.py --port ${WEB_TEACH_PORT}"
  (
    cd "${WEB_TEACH_DIR}"
    nohup "${WEB_TEACH_PY}" fr10_web_teach.py --port "${WEB_TEACH_PORT}" \
      >>"${WEB_TEACH_LOG}" 2>&1 &
  )
}

_stop_web_teach() {
  local pids
  pids="$(_web_pids)"
  if [[ -z "${pids}" ]]; then
    return 1
  fi
  echo "[PREEMPT] 暂停 fr10_web_teach.py: ${pids//$'\n'/ }"
  kill -TERM ${pids} 2>/dev/null || true
  if ! _wait_no_pids; then
    echo "[PREEMPT] TERM 后仍未退出，发送 KILL" >&2
    pids="$(_web_pids)"
    if [[ -n "${pids}" ]]; then
      kill -KILL ${pids} 2>/dev/null || true
    fi
    _wait_no_pids || true
  fi
  return 0
}

restart_needed=0
if [[ "${PREEMPT_WEB_TEACH}" == "1" ]]; then
  if _stop_web_teach; then
    restart_needed=1
    sleep 0.8
  fi
fi

cleanup() {
  local status=$?
  if [[ "${restart_needed}" == "1" && "${PREEMPT_WEB_TEACH}" == "1" ]]; then
    _start_web_teach
  fi
  exit "${status}"
}
trap cleanup EXIT INT TERM

"$@"
