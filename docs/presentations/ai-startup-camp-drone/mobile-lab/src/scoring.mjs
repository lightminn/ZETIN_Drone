function clamp(value, minimum, maximum) {
  return Math.min(maximum, Math.max(minimum, value));
}

function finiteOrZero(value) {
  return Number.isFinite(value) ? value : 0;
}

export function calculateStability({ roll = 0, pitch = 0, rollRate = 0, pitchRate = 0 } = {}) {
  const attitudePenalty = 2 * Math.hypot(finiteOrZero(roll), finiteOrZero(pitch));
  const ratePenalty = 4 * Math.hypot(finiteOrZero(rollRate), finiteOrZero(pitchRate));
  return clamp(100 - attitudePenalty - ratePenalty, 0, 100);
}

export function accumulateScore(metrics = {}, stability, dt) {
  const elapsed = Math.max(0, finiteOrZero(metrics.elapsed));
  const stabilityIntegral = Math.max(0, finiteOrZero(metrics.stabilityIntegral));
  const duration = Math.max(0, finiteOrZero(dt));
  const boundedStability = clamp(finiteOrZero(stability), 0, 100);

  return {
    stabilityIntegral: stabilityIntegral + boundedStability * duration,
    elapsed: elapsed + duration,
  };
}

export function calculateScore({ stabilityIntegral = 0, elapsed = 0 } = {}) {
  if (!(elapsed > 0)) return 0;
  return Math.round(clamp(stabilityIntegral / elapsed, 0, 100) * 10);
}
