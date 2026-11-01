#!/usr/bin/env bash
set -euo pipefail
runtime_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
venv_dir="$runtime_dir/.venv"
python_cmd="${PYTHON:-python3}"

if [[ ! -x "$venv_dir/bin/python" ]]; then
"$python_cmd" -m venv "$venv_dir"
fi
if [[ -d "$runtime_dir/wheelhouse" ]] && compgen -G "$runtime_dir/wheelhouse/*.whl" > /dev/null; then
    "$venv_dir/bin/python" -m pip install --no-index --find-links "$runtime_dir/wheelhouse" --requirement "$runtime_dir/requirements.txt"
else
    "$venv_dir/bin/python" -m pip install --disable-pip-version-check --requirement "$runtime_dir/requirements.txt"
fi
"$venv_dir/bin/python" -c 'import eccodes; print("RT-PPP GFS Python runtime ready:", eccodes.codes_get_api_version())'
