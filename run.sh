#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
build_dir="$project_dir/build/portable"

mkdir -p "$build_dir"
(cd "$build_dir" && cmake "$project_dir")
cmake --build "$build_dir" --target install

# 传入开发板 IP 时才部署，例如：./run.sh 192.168.1.158
if [[ $# -gt 0 ]]; then
    scp -r "$project_dir/install/" "root@$1:/root/"
fi
