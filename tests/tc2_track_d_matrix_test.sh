#!/bin/sh
set -eu

repo_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
tmp_dir=$(mktemp -d "${TMPDIR:-/tmp}/tc2-track-d-matrix.XXXXXX")
trap 'rm -rf "$tmp_dir"' EXIT HUP INT TERM

host_cc=${HOSTCC:-clang}
"$host_cc" \
        -std=c11 -Wall -Wextra -Werror -pedantic -O2 \
        -I"$repo_dir" -DMODE_TC2 \
        "$repo_dir/tests/tc2_track_d_matrix_test.c" \
        "$repo_dir/tc2_prediction_model.c" \
        "$repo_dir/tc2_track_model.c" \
        "$repo_dir/tc2_motion_model.c" \
        "$repo_dir/tc2_route_monitor.c" \
        "$repo_dir/track_route.c" \
        "$repo_dir/track_data.c" \
        -o "$tmp_dir/tc2_track_d_matrix_test"

"$tmp_dir/tc2_track_d_matrix_test"
