#!/usr/bin/env bash
set -euo pipefail

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
report_path="${repository_root}/build/verify/ctest.xml"
started_at="$(date +%s)"

cd "${repository_root}"
cmake --workflow --preset verify

elapsed_seconds="$(( $(date +%s) - started_at ))"
printf '[verify] elapsed_s=%s report=%s\n' "${elapsed_seconds}" "${report_path}"
