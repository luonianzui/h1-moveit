#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
test ! -e "$root/run/preview_10x.mp4"
ffmpeg -hide_banner -loglevel warning -nostdin -n -i "$root/run/full_cycle.mp4" \
  -vf 'setpts=PTS/10,fps=12' -an -c:v libx264 -preset veryfast -crf 23 \
  -movflags +faststart "$root/run/preview_10x.mp4"
ffprobe -v error -count_frames -show_streams -show_format -of json "$root/run/preview_10x.mp4" \
  > "$root/preview_metadata.json"
ffmpeg -hide_banner -loglevel error -nostdin -i "$root/run/preview_10x.mp4" \
  -map 0:v:0 -f null - > "$root/preview_decode.log" 2>&1
