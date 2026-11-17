#!/usr/bin/env bash
# "Long-exposure" still from a recorded MP4: per-pixel minimum over sampled
# frames, so the (dark) robot leaves a trail over the light floor.
#   experiments/trail_still.sh media/gazebo_nmpc_figure8.mp4 report/figures/gazebo_nmpc_figure8.png
set -euo pipefail
IN=$1 OUT=$2 TMP=$(mktemp -d)
ffmpeg -loglevel error -i "$IN" -vf "fps=3" "$TMP/f%04d.png"
convert "$TMP"/f*.png -evaluate-sequence min "$OUT"
rm -rf "$TMP"
echo "$OUT"
