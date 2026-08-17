"""Pure geometry used by the presentation visualizations."""

from __future__ import annotations

from math import cos, radians, sin


Vector2 = tuple[float, float]
AxisMapping = tuple[tuple[int, float], tuple[int, float], tuple[int, float]]


def gravity_components_2d(
    tilt_degrees: float, gravity_length: float
) -> tuple[Vector2, Vector2, Vector2]:
    """Project downward gravity onto the tilted body horizontal and vertical axes."""

    angle = radians(tilt_degrees)
    body_horizontal = (cos(angle), sin(angle))
    body_vertical = (-sin(angle), cos(angle))
    gravity = (0.0, -gravity_length)

    horizontal_scale = sum(a * b for a, b in zip(gravity, body_horizontal))
    vertical_scale = sum(a * b for a, b in zip(gravity, body_vertical))
    horizontal = tuple(horizontal_scale * axis for axis in body_horizontal)
    vertical = tuple(vertical_scale * axis for axis in body_vertical)
    return gravity, horizontal, vertical


def integrated_bias_angle_deg(bias_dps: float, elapsed_seconds: float) -> float:
    """Return the angle error created by integrating a constant gyro bias."""

    return bias_dps * elapsed_seconds


def body_axis_mapping(sensor_kind: str) -> AxisMapping:
    """Return the current firmware's sensor XYZ to body Roll/Pitch/Yaw mapping."""

    if sensor_kind == "gyro":
        return ((1, 1.0), (0, -1.0), (2, -1.0))
    if sensor_kind == "accel":
        return ((1, 1.0), (0, -1.0), (2, 1.0))
    raise ValueError(f"unsupported sensor kind: {sensor_kind}")


def transform_sensor_axes(
    sensor_xyz: tuple[float, float, float], sensor_kind: str
) -> tuple[float, float, float]:
    """Apply the same axis permutation and signs used by the flight firmware."""

    return tuple(
        sign * sensor_xyz[source_axis]
        for source_axis, sign in body_axis_mapping(sensor_kind)
    )


def wrap_degrees(angle_deg: float) -> float:
    """Wrap a heading to the firmware's [-180, 180) domain."""

    return (angle_deg + 180.0) % 360.0 - 180.0


def capture_heading_reference(yaw_estimate_deg: float, magnetic_heading_deg: float) -> float:
    """Capture the offset that keeps the current yaw instead of commanding north."""

    return wrap_degrees(yaw_estimate_deg - magnetic_heading_deg)


def referenced_heading_deg(magnetic_heading_deg: float, reference_offset_deg: float) -> float:
    """Convert magnetic heading into the yaw frame established at capture time."""

    return wrap_degrees(magnetic_heading_deg + reference_offset_deg)
