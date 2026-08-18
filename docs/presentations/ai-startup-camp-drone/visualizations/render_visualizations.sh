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

VIDEO_SCENES=(
  "audience_visualizations.py:AccelerometerAudience:accelerometer.mp4"
  "audience_visualizations.py:GyroAudience:gyro.mp4"
  "audience_visualizations.py:ComplementaryFilterAudience:complementary-filter.mp4"
  "audience_visualizations.py:GyroBiasAudience:gyro-bias.mp4"
  "audience_visualizations.py:ImuAxisSignsAudience:imu-axis-signs.mp4"
  "audience_visualizations.py:PiErrorAudience:pi-error-correction.mp4"
  "audience_visualizations.py:CascadeTimingAudience:cascade-loop-timing.mp4"
  "audience_visualizations.py:YawCorrectionAudience:yaw-correction.mp4"
  "audience_visualizations.py:LandingAmbiguityAudience:landing-ambiguity.mp4"
)

STATIC_SCENES=(
  "static_diagram_visualizations.py:DroneClassificationStatic:drone-classification.png"
  "static_diagram_visualizations.py:QualificationWeightStatic:qualification-weight.png"
  "static_diagram_visualizations.py:AircraftUamStatic:aircraft-uam.png"
  "static_diagram_visualizations.py:MissionSpecsStatic:mission-specs.png"
  "static_diagram_visualizations.py:QuadcopterForceMotionStatic:quadcopter-force-motion.png"
  "static_diagram_visualizations.py:HelicopterQuadcopterTorqueStatic:helicopter-quadcopter-torque.png"
  "static_diagram_visualizations.py:SwarmSystemStatic:swarm-system.png"
  "static_diagram_visualizations.py:AttitudeCorrectionStatic:attitude-correction.png"
  "static_diagram_visualizations.py:SilClosedLoopStatic:sil-closed-loop.png"
  "static_diagram_visualizations.py:FailsafeTimelineStatic:failsafe-timeline.png"
  "static_diagram_visualizations.py:LandingObservabilityStatic:landing-observability.png"
  "static_diagram_visualizations.py:SharedStateRaceStatic:shared-state-race.png"
  "static_diagram_visualizations.py:TelemetryMotorBalanceStatic:telemetry-motor-balance.png"
)

for entry in "${VIDEO_SCENES[@]}"; do
  source=${entry%%:*}
  scene_and_output=${entry#*:}
  scene=${scene_and_output%%:*}
  output=${scene_and_output#*:}
  media_dir="$WORK_DIR/$scene"
  "${MANIM_CMD[@]}" \
    --media_dir "$media_dir" \
    --resolution 1280,720 \
    --frame_rate 30 \
    --format mp4 \
    "$SCRIPT_DIR/$source" \
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

for entry in "${STATIC_SCENES[@]}"; do
  source=${entry%%:*}
  scene_and_output=${entry#*:}
  scene=${scene_and_output%%:*}
  output=${scene_and_output#*:}
  media_dir="$WORK_DIR/$scene"
  "${MANIM_CMD[@]}" \
    --media_dir "$media_dir" \
    --resolution 1280,720 \
    --format png \
    --save_last_frame \
    "$SCRIPT_DIR/$source" \
    "$scene"

  rendered=$(find "$media_dir" -type f -name "$scene*.png" -print -quit)
  if [[ -z "$rendered" ]]; then
    echo "rendered frame not found for $scene" >&2
    exit 1
  fi

  cp -- "$rendered" "$ASSET_DIR/$output"
  echo "$scene -> assets/$output"
done
