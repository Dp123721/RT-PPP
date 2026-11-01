#!/usr/bin/env sh
set -eu

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 OUTPUT_DIRECTORY" >&2
    exit 2
fi

src_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
out_dir=$1
mkdir -p "$out_dir"
gfortran -O2 -std=f2008 -ffree-line-length-none -static-libgfortran -static-libgcc \
    "$src_dir/gpt3_vmf3_worker.f90" "$src_dir/gpt3_5.f90" "$src_dir/vmf3_ht.f90" \
    -o "$out_dir/gpt3_vmf3_worker"
cp "$src_dir/gpt3_5.grd" "$out_dir/gpt3_5.grd"
printf '%s\n' "GPT3/VMF3 worker ready: $out_dir/gpt3_vmf3_worker"
