#!/usr/bin/env sh
set -eu

repository_root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$repository_root"

started_at=$(date +%s)
cmake --workflow --preset verify
finished_at=$(date +%s)

echo "[verify] elapsed_s=$((finished_at - started_at)) report=$repository_root/build/verify/ctest.xml"
