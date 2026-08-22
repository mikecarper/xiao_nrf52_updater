#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "$0")/.." && pwd)"
test_bin="$(mktemp /tmp/xiao-updater-host-tests.XXXXXX)"
trap 'rm -f "$test_bin"' EXIT

extra_flags=()
if [[ "${SANITIZE:-0}" == "1" ]]; then
  extra_flags+=(
    -fsanitize=address,undefined
    -fno-omit-frame-pointer
  )
fi

"${CXX:-c++}" -std=c++11 -Wall -Wextra -Werror "${extra_flags[@]}" \
  -I"$repo_dir/src" \
  "$repo_dir/src/callback_log_event.cpp" \
  "$repo_dir/src/config_parse.cpp" \
  "$repo_dir/src/dfu_image_layout.cpp" \
  "$repo_dir/tests/host_tests.cpp" \
  -o "$test_bin"
"$test_bin"
