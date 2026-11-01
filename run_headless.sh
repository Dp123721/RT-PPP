#!/usr/bin/env sh
set -eu

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
app_path=${RTPPP_APP:-"$script_dir/build/linux/bin/RT-PPP"}
config_path=${RTPPP_CONFIG:-"$script_dir/rtppp_settings.ini"}

if [ "$#" -gt 0 ]; then
    config_path=$1
    shift
fi

if [ ! -x "$app_path" ]; then
    printf 'Executable not found or not executable: %s\n' "$app_path" >&2
    exit 2
fi
if [ ! -r "$config_path" ]; then
    printf 'Configuration file not readable: %s\n' "$config_path" >&2
    exit 2
fi

QT_QPA_PLATFORM=${QT_QPA_PLATFORM:-offscreen}
export QT_QPA_PLATFORM
exec "$app_path" --headless --config "$config_path" "$@"
