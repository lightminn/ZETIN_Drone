"""Audience-first Manim visualizations used by the startup-camp deck.

The handoff ZIP used small black-background diagrams and a continuously
orbiting 3-D camera.  These scenes keep the original technical ideas but use
one fixed reading order throughout: condition at the top, one large causal
motion in the middle, and the takeaway at the bottom.
"""

from __future__ import annotations

import numpy as np
from manim import *

from geometry import (
    body_axis_mapping,
    capture_heading_reference,
    gravity_components_2d,
    integrated_bias_angle_deg,
    referenced_heading_deg,
)


config.pixel_width = 1280
config.pixel_height = 720
config.frame_rate = 30
config.frame_width = 16
config.frame_height = 9
config.background_color = "#06152F"
config.verbosity = "WARNING"

FONT = "Noto Sans CJK KR"
BG = "#06152F"
PANEL = "#0E2447"
PANEL_2 = "#142F55"
GRID = "#29415F"
WHITE = "#F7FAFF"
MUTED = "#A9B8CC"
BLUE = "#48A8FF"
CYAN = "#5FE3F3"
GREEN = "#55D68B"
YELLOW = "#FFD166"
ORANGE = "#FF9F43"
RED = "#FF6474"
TEXT_SCALE = 1.1


def text(label: str, size: int, color: str = WHITE, weight: str = "NORMAL") -> Text:
    return Text(
        label,
        font=FONT,
        font_size=size * TEXT_SCALE,
        color=color,
        weight=weight,
    )


def polyline(axes: Axes, xs, ys, color: str, width: float = 5) -> VMobject:
    points = [axes.c2p(float(x), float(y)) for x, y in zip(xs, ys)]
    return VMobject(color=color, stroke_width=width).set_points_smoothly(points)


def drone_icon(scale: float = 1.0, color: str = BLUE) -> VGroup:
    body = RoundedRectangle(
        width=1.8,
        height=0.56,
        corner_radius=0.15,
        fill_color=color,
        fill_opacity=1,
        stroke_color=WHITE,
        stroke_width=2,
    )
    arms = VGroup(
        Line(LEFT * 1.35, RIGHT * 1.35, color=MUTED, stroke_width=9),
        Line(UP * 0.78, DOWN * 0.78, color=MUTED, stroke_width=9),
    )
    motors = VGroup(
        *[
            Circle(radius=0.2, fill_color=PANEL_2, fill_opacity=1, stroke_color=CYAN, stroke_width=3)
            .move_to(point)
            for point in (LEFT * 1.35, RIGHT * 1.35, UP * 0.78, DOWN * 0.78)
        ]
    )
    nose = Triangle(fill_color=YELLOW, fill_opacity=1, stroke_width=0).scale(0.16)
    nose.rotate(-PI / 2).next_to(body, RIGHT, buff=-0.02)
    return VGroup(arms, motors, body, nose).scale(scale)


class ExplainerScene(Scene):
    def setup(self) -> None:
        self.camera.background_color = BG

    def heading(self, eyebrow: str, title: str) -> VGroup:
        small = text(eyebrow, 24, BLUE, "BOLD")
        large = text(title, 38, WHITE, "BOLD")
        group = VGroup(small, large).arrange(DOWN, aligned_edge=LEFT, buff=0.13)
        group.to_corner(UL, buff=0.55)
        rule = Line(LEFT * 7.45, RIGHT * 7.45, color=GRID, stroke_width=2)
        rule.next_to(group, DOWN, buff=0.23)
        self.play(FadeIn(small, shift=RIGHT * 0.18), FadeIn(large, shift=RIGHT * 0.18), Create(rule), run_time=0.65)
        return VGroup(group, rule)

    def panel(self, width: float, height: float, color: str = PANEL) -> RoundedRectangle:
        return RoundedRectangle(
            width=width,
            height=height,
            corner_radius=0.2,
            fill_color=color,
            fill_opacity=1,
            stroke_color=GRID,
            stroke_width=2,
        )

    def conclusion(self, message: str, color: str = BLUE) -> VGroup:
        box = RoundedRectangle(
            width=14.9,
            height=0.9,
            corner_radius=0.18,
            fill_color=PANEL_2,
            fill_opacity=1,
            stroke_color=color,
            stroke_width=3,
        ).to_edge(DOWN, buff=0.34)
        label = text(message, 29, WHITE, "BOLD").move_to(box)
        group = VGroup(box, label)
        self.play(FadeIn(group, shift=UP * 0.18), run_time=0.45)
        return group

    def hold_and_clear(self, seconds: float = 1.25) -> None:
        self.wait(seconds)
        self.play(FadeOut(Group(*self.mobjects)), run_time=0.45)


class AccelerometerAudience(ExplainerScene):
    """A fixed gravity arrow is projected onto the drone's tilted axes."""

    def construct(self) -> None:
        self.heading("가속도계", "중력은 그대로이고, 기체 축에서 읽는 값이 달라진다")

        frame = self.panel(9.4, 5.1).shift(LEFT * 2.1 + DOWN * 0.15)
        self.play(FadeIn(frame), run_time=0.35)

        body = VGroup(
            RoundedRectangle(width=4.4, height=0.55, corner_radius=0.12, fill_color=BLUE, fill_opacity=1, stroke_color=WHITE, stroke_width=2),
            Circle(radius=0.35, color=CYAN, stroke_width=5).shift(LEFT * 2.1),
            Circle(radius=0.35, color=CYAN, stroke_width=5).shift(RIGHT * 2.1),
        ).move_to(frame.get_center() + UP * 0.55)
        body_axis_x = Arrow(LEFT * 2.6, RIGHT * 2.6, buff=0, color=CYAN, stroke_width=5).move_to(body)
        body_axis_z = Arrow(DOWN * 1.55, UP * 1.55, buff=0, color=GREEN, stroke_width=5).move_to(body)
        rotating = VGroup(body, body_axis_x, body_axis_z)

        tilt_degrees = 28.0
        gravity_xy, horizontal_xy, vertical_xy = gravity_components_2d(
            tilt_degrees, 2.5
        )
        center = body.get_center()
        gravity = Arrow(
            center,
            center + np.array([*gravity_xy, 0]),
            buff=0,
            color=YELLOW,
            stroke_width=8,
        )
        gravity_label = text("중력 방향", 27, YELLOW, "BOLD").next_to(gravity, RIGHT, buff=0.25)
        gravity_label.shift(DOWN * 0.35)
        fixed_note = text("기체가 기울어도\n중력은 수직 아래", 25, WHITE, "BOLD")
        fixed_note.move_to(RIGHT * 5.3 + UP * 1.0)

        self.play(FadeIn(rotating), GrowArrow(gravity), FadeIn(gravity_label), FadeIn(fixed_note), run_time=0.75)
        self.play(Rotate(rotating, angle=tilt_degrees * DEGREES, about_point=center), run_time=1.35, rate_func=smooth)

        component_h = Arrow(center, center + np.array([*horizontal_xy, 0]), buff=0, color=ORANGE, stroke_width=7)
        component_v = Arrow(center, center + np.array([*vertical_xy, 0]), buff=0, color=GREEN, stroke_width=7)
        projection_guides = VGroup(
            DashedLine(component_h.get_end(), gravity.get_end(), color=MUTED, stroke_width=3),
            DashedLine(component_v.get_end(), gravity.get_end(), color=MUTED, stroke_width=3),
        )
        h_label = text("수평축 성분", 24, ORANGE, "BOLD")
        v_label = text("수직축 성분", 24, GREEN, "BOLD")
        component_legend = VGroup(h_label, v_label).arrange(
            DOWN, aligned_edge=LEFT, buff=0.28
        )
        component_legend.move_to(RIGHT * 5.25 + DOWN * 0.25)
        decompose = text("두 성분을 더하면\n원래 중력 벡터", 28, WHITE, "BOLD")
        decompose.move_to(RIGHT * 5.25 + DOWN * 1.35)
        self.play(
            GrowArrow(component_h),
            GrowArrow(component_v),
            Create(projection_guides),
            FadeIn(component_legend),
            FadeIn(decompose),
            run_time=0.9,
        )

        self.conclusion("두 성분의 비율을 보면 Roll·Pitch 기울기를 되찾을 수 있다")
        self.hold_and_clear()


class GyroAudience(ExplainerScene):
    """Angular-rate samples build one visible angle wedge by wedge."""

    def construct(self) -> None:
        self.heading("자이로", "순간 회전량을 계속 더하면 전체 회전각이 된다")

        dial = Circle(radius=2.2, color=GRID, stroke_width=5).shift(LEFT * 3.4 + DOWN * 0.15)
        center = dial.get_center()
        needle = Arrow(center, center + RIGHT * 1.75, buff=0, color=WHITE, stroke_width=8)
        body = drone_icon(0.62).move_to(center)
        self.play(Create(dial), FadeIn(body), GrowArrow(needle), run_time=0.7)

        equation_box = self.panel(6.2, 3.8).shift(RIGHT * 3.8 + DOWN * 0.05)
        rate = text("각속도 × 짧은 시간", 30, CYAN, "BOLD")
        equals = text("= 작은 회전량", 30, WHITE, "BOLD")
        formula = VGroup(rate, equals).arrange(DOWN, aligned_edge=LEFT, buff=0.3).move_to(equation_box.get_center() + UP * 0.8)
        sum_text = text("0°", 44, YELLOW, "BOLD").move_to(equation_box.get_center() + DOWN * 0.8)
        self.play(FadeIn(equation_box), FadeIn(formula), FadeIn(sum_text), run_time=0.65)

        colors = [CYAN, BLUE, GREEN, YELLOW, ORANGE]
        total = 0
        for color in colors:
            start = total * DEGREES
            total += 8
            arc = Arc(radius=2.2, start_angle=start, angle=8 * DEGREES, color=color, stroke_width=16).move_arc_center_to(center)
            new_needle = Arrow(center, center + rotate_vector(RIGHT * 1.75, total * DEGREES), buff=0, color=WHITE, stroke_width=8)
            new_sum = text(f"8°씩 더해 → {total}°", 38, YELLOW, "BOLD").move_to(sum_text)
            self.play(Create(arc), Transform(needle, new_needle), Transform(sum_text, new_sum), run_time=0.55, rate_func=linear)

        self.conclusion("자이로는 빠른 회전을 잘 잡지만, 작은 오차도 함께 누적된다", ORANGE)
        self.hold_and_clear()


class ComplementaryFilterAudience(ExplainerScene):
    """Three aligned traces make each sensor's role visible without a legend."""

    def construct(self) -> None:
        self.heading("상보필터", "빠른 자이로와 오래 유지되는 중력 기준을 함께 쓴다")

        xs = np.linspace(0, 6, 180)
        truth = np.where(xs < 1.4, 20 * xs / 1.4, np.where(xs < 4.2, 20, 20 * np.maximum(0, 1 - (xs - 4.2) / 1.1)))
        gyro = truth + 1.6 * xs
        accel = truth + 4.2 * np.sin(xs * 9)
        fused = truth + 0.65 * np.sin(xs * 5) * np.exp(-xs / 5)

        rows = [
            ("자이로", "빠르지만 서서히 표류", RED, gyro),
            ("가속도계", "장기 기준이지만 순간 진동", ORANGE, accel),
            ("상보필터", "빠르고 기준에서 벗어나지 않음", GREEN, fused),
        ]
        charts = VGroup()
        for index, (name, note, color, values) in enumerate(rows):
            panel = self.panel(14.2, 1.35, PANEL if index < 2 else PANEL_2)
            panel.move_to(DOWN * (index * 1.55 - 1.35) + DOWN * 0.15)
            axes = Axes(
                x_range=[0, 6, 1], y_range=[-5, 32, 10], x_length=8.0, y_length=0.9,
                axis_config={"include_tip": False, "stroke_color": GRID, "stroke_width": 2},
            ).move_to(panel.get_center() + RIGHT * 2.5)
            curve = polyline(axes, xs, values, color, 5 if index == 2 else 4)
            label = text(name, 27, color, "BOLD").move_to(panel.get_center() + LEFT * 5.25 + UP * 0.2)
            description = text(note, 20, MUTED).next_to(label, DOWN, aligned_edge=LEFT, buff=0.08)
            row = VGroup(panel, axes, curve, label, description)
            charts.add(row)
            self.play(FadeIn(panel), FadeIn(label), FadeIn(description), Create(curve), run_time=1.05)

        highlight = SurroundingRectangle(charts[2], color=GREEN, buff=0.08, stroke_width=4, corner_radius=0.2)
        self.play(Create(highlight), run_time=0.45)
        self.conclusion("짧은 움직임은 자이로, 긴 시간의 기준은 가속도계가 맡는다", GREEN)
        self.hold_and_clear()


class GyroBiasAudience(ExplainerScene):
    """A stationary airframe and a rising estimate are shown simultaneously."""

    def construct(self) -> None:
        self.heading("자이로 바이어스", "기체가 멈춰 있어도 센서값이 정확히 0은 아니다")

        left_panel = self.panel(5.1, 4.8).shift(LEFT * 4.7 + DOWN * 0.15)
        right_panel = self.panel(8.6, 4.8).shift(RIGHT * 2.4 + DOWN * 0.15)
        drone = drone_icon(0.9).move_to(left_panel.get_center() + UP * 0.45)
        stopped = text("실제 기체: 정지", 29, WHITE, "BOLD").move_to(left_panel.get_center() + DOWN * 1.15)
        bias_rate_dps = 0.1
        duration_seconds = 60.0
        bias = text("자이로 출력: +0.1°/s", 25, RED, "BOLD").next_to(stopped, DOWN, buff=0.2)
        self.play(FadeIn(left_panel), FadeIn(right_panel), FadeIn(drone), FadeIn(stopped), FadeIn(bias), run_time=0.75)

        axes = Axes(
            x_range=[0, 60, 10], y_range=[0, 7, 1], x_length=6.8, y_length=3.1,
            axis_config={"include_tip": False, "stroke_color": GRID, "stroke_width": 2},
        ).move_to(right_panel.get_center() + DOWN * 0.15)
        xs = np.linspace(0, duration_seconds, 120)
        estimate = np.array(
            [integrated_bias_angle_deg(bias_rate_dps, elapsed) for elapsed in xs]
        )
        curve = polyline(axes, xs, estimate, RED, 6)
        x_label = text("시간 (초)", 20, MUTED).next_to(axes.x_axis, DOWN, buff=0.12)
        y_label = text("누적 각도 오차", 20, MUTED).move_to(
            right_panel.get_center() + LEFT * 2.4 + UP * 1.75
        )
        end_label = text("60초 뒤 6°", 25, RED, "BOLD").next_to(
            axes.c2p(duration_seconds, estimate[-1]), LEFT, buff=0.25
        )
        self.play(Create(axes), FadeIn(x_label), FadeIn(y_label), Create(curve), run_time=2.2, rate_func=linear)
        self.play(FadeIn(end_label), run_time=0.35)

        self.conclusion("작은 고정 오차도 적분하면 시간에 비례한 각도 오차가 된다", RED)
        self.hold_and_clear()


class ImuAxisSignsAudience(ExplainerScene):
    """A wrong sensor-to-body sign makes the controller reinforce a tilt."""

    def construct(self) -> None:
        self.heading("센서축 → 기체축", "부호 하나가 반대면 제어가 기울기를 더 키운다")

        actual_panel = self.panel(4.1, 4.65).shift(LEFT * 5.25 + DOWN * 0.1)
        mapping_panel = self.panel(4.8, 4.65, PANEL_2).shift(DOWN * 0.1)
        result_panel = self.panel(4.1, 4.65).shift(RIGHT * 5.25 + DOWN * 0.1)

        actual_label = text("실제 기체", 24, MUTED, "BOLD").move_to(
            actual_panel.get_center() + UP * 1.65
        )
        actual = drone_icon(0.78).move_to(actual_panel.get_center() + UP * 0.25)
        actual.rotate(18 * DEGREES)
        actual_value = text("+Roll로 기울어짐", 28, WHITE, "BOLD").move_to(
            actual_panel.get_center() + DOWN * 1.45
        )

        sensor_names = ("X", "Y", "Z")
        body_names = ("Roll", "Pitch", "Yaw")
        mapping_title = text("현재 자이로 변환", 25, BLUE, "BOLD").move_to(
            mapping_panel.get_center() + UP * 1.7
        )
        mapping_rows = VGroup()
        for body_name, (source_axis, sign) in zip(
            body_names, body_axis_mapping("gyro")
        ):
            sign_text = "+" if sign > 0 else "−"
            mapping_rows.add(
                text(
                    f"{body_name} = {sign_text} 센서 {sensor_names[source_axis]}",
                    27,
                    GREEN if body_name == "Roll" else WHITE,
                    "BOLD",
                )
            )
        mapping_rows.arrange(DOWN, aligned_edge=LEFT, buff=0.42).move_to(
            mapping_panel.get_center() + DOWN * 0.15
        )

        result_name = text("잘못된 부호를 쓰면", 24, RED, "BOLD").move_to(
            result_panel.get_center() + UP * 1.65
        )
        wrong_signal = text("실제 +Roll\n→ 추정 −Roll", 27, RED, "BOLD").move_to(
            result_panel.get_center() + UP * 0.65
        )
        result_drone = drone_icon(0.7).move_to(result_panel.get_center() + DOWN * 0.55)
        result_drone.rotate(18 * DEGREES)
        result_note = text("반대로 보정", 25, MUTED, "BOLD").move_to(
            result_panel.get_center() + DOWN * 1.65
        )
        amplified_note = text("기울기 증가", 27, RED, "BOLD").move_to(result_note)

        arrow1 = Arrow(actual_panel.get_right(), mapping_panel.get_left(), buff=0.2, color=MUTED, stroke_width=5)
        arrow2 = Arrow(mapping_panel.get_right(), result_panel.get_left(), buff=0.2, color=MUTED, stroke_width=5)
        self.play(FadeIn(actual_panel), FadeIn(actual_label), FadeIn(actual_value), FadeIn(actual), run_time=0.65)
        self.play(GrowArrow(arrow1), FadeIn(mapping_panel), FadeIn(mapping_title), FadeIn(mapping_rows), run_time=0.85)
        self.play(
            GrowArrow(arrow2),
            FadeIn(result_panel),
            FadeIn(result_name),
            FadeIn(wrong_signal),
            FadeIn(result_drone),
            FadeIn(result_note),
            run_time=0.75,
        )
        self.play(
            result_drone.animate.rotate(18 * DEGREES),
            Transform(result_note, amplified_note),
            run_time=0.85,
        )

        self.conclusion("센서값을 기체축의 방향과 부호로 맞춘 뒤 제어기에 넣어야 한다", RED)
        self.hold_and_clear()


class PiErrorAudience(ExplainerScene):
    """P settles with residual error while PI returns to the zero line."""

    def construct(self) -> None:
        self.heading("P 제어와 I 제어", "계속 부는 바람에서는 P만으로 오차가 남는다")

        panel = self.panel(14.2, 5.05).shift(DOWN * 0.1)
        axes = Axes(
            x_range=[0, 8, 1], y_range=[-1, 13, 2], x_length=10.7, y_length=3.7,
            axis_config={"include_tip": False, "stroke_color": GRID, "stroke_width": 2},
        ).move_to(panel.get_center() + RIGHT * 0.25)
        zero = DashedLine(axes.c2p(0, 0), axes.c2p(8, 0), color=MUTED, stroke_width=3)
        xs = np.linspace(0, 8, 180)
        p = 8.5 * (1 - np.exp(-xs / 0.65))
        pi = p * np.exp(-xs / 2.7)
        p_curve = polyline(axes, xs, p, ORANGE, 6)
        pi_curve = polyline(axes, xs, pi, GREEN, 6)
        wind = VGroup(Arrow(LEFT * 0.8, RIGHT * 0.8, color=CYAN, stroke_width=7), text("일정한 바람", 23, CYAN, "BOLD")).arrange(DOWN, buff=0.08)
        wind.move_to(panel.get_center() + LEFT * 5.6 + UP * 1.25)
        self.play(FadeIn(panel), Create(axes), Create(zero), FadeIn(wind), run_time=0.75)
        p_label = text("P만: 오차가 남음", 25, ORANGE, "BOLD")
        pi_label = text("P + I: 오차가 0으로 복귀", 25, GREEN, "BOLD")
        curve_labels = VGroup(p_label, pi_label).arrange(RIGHT, buff=0.7)
        curve_labels.move_to(panel.get_center() + RIGHT * 0.8 + UP * 1.85)

        self.play(Create(p_curve), run_time=1.65, rate_func=linear)
        self.play(FadeIn(p_label), run_time=0.3)
        self.play(Create(pi_curve), run_time=1.65, rate_func=linear)
        self.play(FadeIn(pi_label), run_time=0.3)

        self.conclusion("I항은 오래 남은 오차를 기억하고 필요한 힘을 더 보탠다", GREEN)
        self.hold_and_clear()


class CascadeTimingAudience(ExplainerScene):
    """Inner-loop corrections repeat more densely than outer updates."""

    def construct(self) -> None:
        self.heading("캐스케이드 제어", "바깥 루프가 목표를 정하면 안쪽 루프가 더 촘촘하게 보정한다")

        outer_label = text("바깥 자세 루프", 28, RED, "BOLD").move_to(LEFT * 6.25 + UP * 1.2)
        inner_label = text("안쪽 각속도 루프", 28, BLUE, "BOLD").move_to(LEFT * 6.25 + DOWN * 1.2)
        self.play(FadeIn(outer_label), FadeIn(inner_label), run_time=0.45)

        groups = VGroup()
        for cycle in range(3):
            x = -2.5 + cycle * 4.15
            outer = RoundedRectangle(width=3.35, height=1.05, corner_radius=0.16, fill_color=RED, fill_opacity=0.18, stroke_color=RED, stroke_width=4)
            outer.move_to(np.array([x, 1.15, 0]))
            outer_text = text(f"목표 {cycle + 1}", 26, RED, "BOLD").move_to(outer)
            inners = VGroup()
            for index in range(4):
                box = RoundedRectangle(width=0.67, height=0.9, corner_radius=0.1, fill_color=BLUE, fill_opacity=0.22, stroke_color=BLUE, stroke_width=3)
                box.move_to(np.array([x - 1.2 + index * 0.8, -1.15, 0]))
                pulse = text("보정" if index < 3 else "…", 20, WHITE, "BOLD").move_to(box)
                inners.add(VGroup(box, pulse))
            connector = Arrow(outer.get_bottom(), inners.get_top(), buff=0.15, color=YELLOW, stroke_width=4)
            groups.add(VGroup(outer, outer_text, inners, connector))

        for group in groups:
            outer, outer_text, inners, connector = group
            self.play(FadeIn(outer), FadeIn(outer_text), GrowArrow(connector), run_time=0.4)
            for inner in inners:
                self.play(FadeIn(inner, scale=1.15), run_time=0.22)

        brace = Brace(groups[1][2], DOWN, color=CYAN)
        note = text("자세 목표 사이에서 여러 번 보정", 25, CYAN, "BOLD").next_to(brace, DOWN, buff=0.16)
        self.play(GrowFromCenter(brace), FadeIn(note), run_time=0.45)
        self.conclusion("빠른 안쪽 루프가 모터를 먼저 안정시키고, 바깥 루프가 자세를 이끈다")
        self.hold_and_clear()


class YawCorrectionAudience(ExplainerScene):
    """A drifting gyro heading is pulled back to a fixed magnetic reference."""

    def construct(self) -> None:
        self.heading("Yaw 기준", "자이로 추정값이 흐르면 잡아 둔 방향으로 천천히 되돌린다")

        compass = Circle(radius=2.35, color=GRID, stroke_width=6).shift(LEFT * 3.2 + DOWN * 0.05)
        center = compass.get_center()
        for direction, label in ((UP, "N"), (RIGHT, "E"), (DOWN, "S"), (LEFT, "W")):
            mark = Line(center + direction * 2.05, center + direction * 2.27, color=MUTED, stroke_width=4)
            letter = text(label, 22, MUTED, "BOLD").move_to(center + direction * 2.65)
            self.add(mark, letter)

        captured_yaw_deg = 25.0
        captured_magnetic_heading_deg = 62.0
        reference_offset_deg = capture_heading_reference(
            captured_yaw_deg, captured_magnetic_heading_deg
        )
        reference_heading_deg = referenced_heading_deg(
            captured_magnetic_heading_deg, reference_offset_deg
        )
        reference_vector = rotate_vector(
            UP * 1.85, -reference_heading_deg * DEGREES
        )
        estimate_vector = rotate_vector(
            UP * 1.75, -reference_heading_deg * DEGREES
        )
        reference = Arrow(center, center + reference_vector, buff=0, color=CYAN, stroke_width=9)
        estimate = Arrow(center, center + estimate_vector, buff=0, color=RED, stroke_width=8)
        true_label = text("잡아 둔 방향", 24, CYAN, "BOLD").move_to(center + LEFT * 2.0 + UP * 1.65)
        estimate_label = text("자이로 추정", 24, RED, "BOLD").move_to(center + RIGHT * 2.0 + UP * 1.5)
        self.play(Create(compass), GrowArrow(reference), GrowArrow(estimate), FadeIn(true_label), FadeIn(estimate_label), run_time=0.75)

        explanation = self.panel(6.2, 4.2).shift(RIGHT * 4.0 + DOWN * 0.05)
        step1 = text("1  작은 자이로 오차가 누적", 27, RED, "BOLD")
        step2 = text("2  지자기가 상대 기준 제공", 27, CYAN, "BOLD")
        step3 = text("3  오차를 조금씩 되돌림", 27, GREEN, "BOLD")
        steps = VGroup(step1, step2, step3).arrange(DOWN, aligned_edge=LEFT, buff=0.48).move_to(explanation)
        self.play(FadeIn(explanation), FadeIn(step1), run_time=0.5)

        drift_error_deg = 38.0
        drifted_heading_deg = reference_heading_deg + drift_error_deg
        drifted = Arrow(
            center,
            center + rotate_vector(UP * 1.75, -drifted_heading_deg * DEGREES),
            buff=0,
            color=RED,
            stroke_width=8,
        )
        self.play(Transform(estimate, drifted), run_time=1.3, rate_func=smooth)
        gap = Arc(
            radius=1.15,
            start_angle=(90.0 - drifted_heading_deg) * DEGREES,
            angle=drift_error_deg * DEGREES,
            color=YELLOW,
            stroke_width=7,
        ).move_arc_center_to(center)
        gap_label = text("누적 오차", 24, YELLOW, "BOLD").move_to(
            center + RIGHT * 1.6 + DOWN * 0.45
        )
        self.play(Create(gap), FadeIn(gap_label), FadeIn(step2), run_time=0.65)

        corrected_heading_deg = reference_heading_deg + 5.0
        corrected = Arrow(
            center,
            center + rotate_vector(UP * 1.75, -corrected_heading_deg * DEGREES),
            buff=0,
            color=GREEN,
            stroke_width=9,
        )
        self.play(Transform(estimate, corrected), FadeOut(gap), FadeOut(gap_label), FadeIn(step3), run_time=1.8, rate_func=smooth)
        self.conclusion("지자기는 북쪽으로 돌리지 않고, 처음 잡은 방향을 장기 기준으로 유지한다", GREEN)
        self.hold_and_clear()


class LandingAmbiguityAudience(ExplainerScene):
    """Grounded and constant-descent cases show the same 1g reading side by side."""

    def construct(self) -> None:
        self.heading("착지 판정의 한계", "가속도계 1g만으로는 정지와 등속 하강을 구분할 수 없다")

        left_panel = self.panel(6.8, 4.55).shift(LEFT * 3.65 + DOWN * 0.35)
        right_panel = self.panel(6.8, 4.55).shift(RIGHT * 3.65 + DOWN * 0.35)
        divider = Line(UP * 1.9, DOWN * 2.55, color=GRID, stroke_width=3)
        left_title = text("지면에 정지", 29, WHITE, "BOLD").move_to(left_panel.get_center() + UP * 1.8)
        right_title = text("등속으로 하강", 29, WHITE, "BOLD").move_to(right_panel.get_center() + UP * 1.8)

        ground_left = Line(LEFT * 2.5, RIGHT * 2.5, color=MUTED, stroke_width=8).move_to(left_panel.get_center() + DOWN * 1.25)
        left_drone = drone_icon(0.62).move_to(ground_left.get_center() + UP * 0.62)
        right_drone = drone_icon(0.62).move_to(right_panel.get_center() + UP * 0.75)
        left_accel = Arrow(left_drone.get_center(), left_drone.get_center() + UP * 1.25, buff=0, color=YELLOW, stroke_width=7)
        right_accel = Arrow(right_drone.get_center(), right_drone.get_center() + UP * 1.25, buff=0, color=YELLOW, stroke_width=7)
        left_value = text("가속도계: 1g", 27, YELLOW, "BOLD").move_to(left_panel.get_center() + DOWN * 1.95)
        right_value = text("가속도계: 1g", 27, YELLOW, "BOLD").move_to(right_panel.get_center() + DOWN * 1.95)

        self.play(FadeIn(left_panel), FadeIn(right_panel), Create(divider), FadeIn(left_title), FadeIn(right_title), run_time=0.65)
        self.play(Create(ground_left), FadeIn(left_drone), FadeIn(right_drone), GrowArrow(left_accel), GrowArrow(right_accel), FadeIn(left_value), FadeIn(right_value), run_time=0.8)

        down_arrow = Arrow(right_drone.get_center() + RIGHT * 2.0 + UP * 0.7, right_drone.get_center() + RIGHT * 2.0 + DOWN * 0.7, color=CYAN, stroke_width=7)
        velocity = text("속도는 아래로", 24, CYAN, "BOLD").next_to(down_arrow, RIGHT, buff=0.15)
        self.play(GrowArrow(down_arrow), FadeIn(velocity), right_drone.animate.shift(DOWN * 1.55), right_accel.animate.shift(DOWN * 1.55), run_time=1.8, rate_func=linear)

        same = text("움직임은 다르지만 센서값은 같다", 29, RED, "BOLD").move_to(UP * 2.45)
        self.play(FadeIn(same), run_time=0.45)
        self.conclusion("착지를 판단하려면 거리·광류처럼 움직임을 직접 보는 센서가 필요하다", RED)
        self.hold_and_clear()
