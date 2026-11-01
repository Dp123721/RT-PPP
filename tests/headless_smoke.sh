#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "$script_dir/.." && pwd)"
app_path="${1:-$repo_root/build/linux/bin/RT-PPP}"
config_path="${2:-$repo_root/tests/fixtures/headless_realtime.ini}"

if [[ ! -x "$app_path" ]]; then
    printf 'Executable not found or not executable: %s\n' "$app_path" >&2
    exit 2
fi
if [[ ! -r "$config_path" ]]; then
    printf 'Configuration file not readable: %s\n' "$config_path" >&2
    exit 2
fi
if ! command -v timeout >/dev/null 2>&1; then
    printf 'The headless smoke test requires the coreutils timeout command.\n' >&2
    exit 2
fi

printf 'Running without DISPLAY and WAYLAND_DISPLAY: %s\n' "$app_path"
timeout --signal=TERM --kill-after=5s 15s \
    env -u DISPLAY -u WAYLAND_DISPLAY \
    "$app_path" --headless --config "$config_path" --run-seconds 2
printf 'Linux headless startup smoke test passed.\n'
