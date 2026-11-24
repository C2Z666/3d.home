#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -ne 1 ]; then
  echo "Usage: $0 <jump-host-ip>" >&2
  exit 2
fi

pidfile="${TMPDIR:-/tmp}/robot-web-tunnel-$(id -u).pid"
if [ -f "$pidfile" ]; then
  old_pid=$(cat "$pidfile")
  if [[ "$old_pid" =~ ^[0-9]+$ ]] && kill -0 "$old_pid" 2>/dev/null; then
    old_cmd=$(ps -p "$old_pid" -o args= 2>/dev/null || true)
    if [[ "$old_cmd" == *"ssh"* && "$old_cmd" == *"127.0.0.1:8767:127.0.0.1:8767"* && "$old_cmd" == *"robot@192.168.168.100"* ]]; then
      kill "$old_pid"
    else
      echo "PID file points to another process; refusing to terminate it." >&2
      exit 1
    fi
  fi
  rm -f "$pidfile"
fi

echo 'Web: http://127.0.0.1:8767/ (Ctrl+C to close)'
ssh -N -o ExitOnForwardFailure=yes -L 127.0.0.1:8767:127.0.0.1:8767 -J "robot@$1" robot@192.168.168.100 &
ssh_pid=$!
printf '%s\n' "$ssh_pid" > "$pidfile"
trap 'kill "$ssh_pid" 2>/dev/null || true; rm -f "$pidfile"' EXIT INT TERM
wait "$ssh_pid"
