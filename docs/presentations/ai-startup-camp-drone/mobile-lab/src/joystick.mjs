function roundZero(value) {
  return Object.is(value, -0) ? 0 : value;
}

export function joystickVector(clientX, clientY, rect) {
  const radius = Math.min(rect.width, rect.height) / 2;
  if (!(radius > 0)) return { x: 0, y: 0, magnitude: 0 };

  const x = clientX - (rect.left + rect.width / 2);
  const y = (rect.top + rect.height / 2) - clientY;
  const distance = Math.hypot(x, y);
  if (distance === 0) return { x: 0, y: 0, magnitude: 0 };

  const magnitude = Math.min(distance / radius, 1);
  return {
    x: roundZero((x / distance) * magnitude),
    y: roundZero((y / distance) * magnitude),
    magnitude,
  };
}
