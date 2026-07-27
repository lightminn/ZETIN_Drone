"""Formatting helpers for failsafe telemetry consumers."""

import math


FAILSAFE_PHASE_NAMES = {
    0: "NONE",
    1: "DESCENDING",
    2: "CUT_LANDED",
    3: "CUT_TIMEOUT",
    4: "CUT_ABORT",
}


def phase_number(value):
    """Return an integer phase, or ``None`` for absent/malformed values."""

    if value is None:
        return None
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(numeric) or not numeric.is_integer():
        return None
    return int(numeric)


def format_failsafe_phase(value):
    """Return a human-readable enum label while retaining the wire value."""

    phase = phase_number(value)
    if phase is None:
        return "legacy/unknown"
    return f"{FAILSAFE_PHASE_NAMES.get(phase, 'UNKNOWN')} ({phase})"


def _format_trim(value):
    if value is None:
        return "unknown"
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return "unknown"
    if not math.isfinite(numeric):
        return "unknown"
    return f"{numeric:+.2f}°"


def format_monitor_status(sample):
    """Return status text and whether the monitor must use alert styling."""

    phase = phase_number(sample.get("Failsafe_Phase"))
    alert = phase is not None and phase != 0
    prefix = "AUTO-LAND ACTIVE — " if alert else ""
    text = (
        f"{prefix}Failsafe {format_failsafe_phase(phase)} | "
        f"Trim R {_format_trim(sample.get('Trim_Roll'))} / "
        f"P {_format_trim(sample.get('Trim_Pitch'))}"
    )
    return text, alert


def render_monitor_title(axis, system_summary, sample):
    """Render the monitor title and apply an alert box for nonzero phases."""

    failsafe_text, alert = format_monitor_status(sample)
    title = axis.set_title(f"{system_summary}\n{failsafe_text}")
    if alert:
        title.set_color("white")
        title.set_fontweight("bold")
        title.set_bbox(
            {
                "boxstyle": "round,pad=0.35",
                "facecolor": "crimson",
                "edgecolor": "darkred",
                "alpha": 0.95,
            }
        )
    return title
