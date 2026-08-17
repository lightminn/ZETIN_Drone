#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
DECK_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd)
ASSET_DIR="$DECK_DIR/assets"
if [[ -n "${PYTHON_BIN:-}" ]]; then
  PYTHON_BIN="$PYTHON_BIN"
elif [[ -x /home/light/anaconda3/bin/python ]]; then
  PYTHON_BIN=/home/light/anaconda3/bin/python
else
  PYTHON_BIN="$SCRIPT_DIR/.venv/bin/python"
fi

if ! "$PYTHON_BIN" -c 'import manim' >/dev/null 2>&1; then
  echo "Manim을 불러오지 못했습니다: $PYTHON_BIN" >&2
  echo "README.md의 전용 렌더링 환경 설정을 먼저 실행하십시오." >&2
  exit 1
fi

MANIM_BIN=$(dirname -- "$PYTHON_BIN")/manim
if [[ -x "$MANIM_BIN" ]]; then
  MANIM_CMD=("$MANIM_BIN")
else
  MANIM_CMD=("$PYTHON_BIN" -m manim)
fi
WORK_DIR=$(mktemp -d)
trap 'find "$WORK_DIR" -mindepth 1 -delete; rmdir "$WORK_DIR"' EXIT

SCENES=(
  "AccelerometerAudience:accelerometer.mp4"
  "GyroAudience:gyro.mp4"
  "ComplementaryFilterAudience:complementary-filter.mp4"
  "GyroBiasAudience:gyro-bias.mp4"
  "ImuAxisSignsAudience:imu-axis-signs.mp4"
  "PiErrorAudience:pi-error-correction.mp4"
  "CascadeTimingAudience:cascade-loop-timing.mp4"
  "YawCorrectionAudience:yaw-correction.mp4"
  "LandingAmbiguityAudience:landing-ambiguity.mp4"
)

for entry in "${SCENES[@]}"; do
  scene=${entry%%:*}
  output=${entry#*:}
  media_dir="$WORK_DIR/$scene"
  "${MANIM_CMD[@]}" \
    --media_dir "$media_dir" \
    --resolution 1280,720 \
    --frame_rate 30 \
    --format mp4 \
    "$SCRIPT_DIR/audience_visualizations.py" \
    "$scene"

  rendered=$(find "$media_dir" -type f -name "$scene.mp4" -print -quit)
  if [[ -z "$rendered" ]]; then
    echo "rendered file not found for $scene" >&2
    exit 1
  fi

  ffmpeg -hide_banner -loglevel error -y \
    -i "$rendered" \
    -an -c:v libx264 -preset medium -crf 20 \
    -pix_fmt yuv420p -r 30 -movflags +faststart \
    "$ASSET_DIR/$output"
  echo "$scene -> assets/$output"
done
