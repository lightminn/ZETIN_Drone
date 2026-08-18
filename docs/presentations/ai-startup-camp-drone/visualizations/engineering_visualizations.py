"""Audience-first animations for the deck's engineering evidence slides."""

from __future__ import annotations

from manim import *

from audience_visualizations import (
    BLUE,
    CYAN,
    GREEN,
    GRID,
    MUTED,
    PANEL,
    PANEL_2,
    RED,
    WHITE,
    YELLOW,
    ExplainerScene,
    text,
)


def evidence_badge(message: str, color: str = YELLOW) -> VGroup:
    """A compact boundary label that stays secondary to the main relationship."""

    label = text(message, 24, color, "BOLD")
    box = RoundedRectangle(
        width=label.width + 0.55,
        height=0.56,
        corner_radius=0.12,
        fill_color=PANEL_2,
        fill_opacity=1,
        stroke_color=color,
        stroke_width=2,
    )
    label.move_to(box)
    return VGroup(box, label)


def process_box(label: str, color: str = BLUE, width: float = 2.75) -> VGroup:
    box = RoundedRectangle(
        width=width,
        height=1.18,
        corner_radius=0.18,
        fill_color=PANEL,
        fill_opacity=1,
        stroke_color=color,
        stroke_width=3,
    )
    caption = text(label, 27, WHITE, "BOLD").move_to(box)
    return VGroup(box, caption)


def front_view_drone(scale: float = 1.0) -> VGroup:
    """Front-view airframe used for roll restoration and vertical motion."""

    arm = Line(LEFT * 1.8, RIGHT * 1.8, color=MUTED, stroke_width=11)
    motors = VGroup(
        *[
            Circle(
                radius=0.28,
                fill_color=PANEL_2,
                fill_opacity=1,
                stroke_color=CYAN,
                stroke_width=4,
            ).move_to(point)
            for point in (LEFT * 1.8, RIGHT * 1.8)
        ]
    )
    body = RoundedRectangle(
        width=1.25,
        height=0.55,
        corner_radius=0.14,
        fill_color=BLUE,
        fill_opacity=1,
        stroke_color=WHITE,
        stroke_width=2,
    )
    return VGroup(arm, motors, body).scale(scale)


def motor_layout() -> tuple[VGroup, dict[str, VGroup]]:
    """Correct X-frame layout: M1/M3 front, M4/M2 rear."""

    points = {
        "M1": LEFT * 1.45 + UP * 1.25,
        "M3": RIGHT * 1.45 + UP * 1.25,
        "M4": LEFT * 1.45 + DOWN * 1.25,
        "M2": RIGHT * 1.45 + DOWN * 1.25,
    }
    arms = VGroup(
        Line(points["M1"], points["M2"], color=MUTED, stroke_width=12),
        Line(points["M3"], points["M4"], color=MUTED, stroke_width=12),
    )
    motors: dict[str, VGroup] = {}
    for name, point in points.items():
        stroke = YELLOW if name == "M1" else BLUE if name == "M3" else GRID
        circle = Circle(
            radius=0.42,
            fill_color=PANEL_2,
            fill_opacity=1,
            stroke_color=stroke,
            stroke_width=5 if name in {"M1", "M3"} else 3,
        ).move_to(point)
        label = text(name, 25, WHITE, "BOLD").move_to(circle)
        motors[name] = VGroup(circle, label)
    body = Circle(
        radius=0.55,
        fill_color=BLUE,
        fill_opacity=1,
        stroke_color=WHITE,
        stroke_width=3,
    )
    nose = Triangle(fill_color=YELLOW, fill_opacity=1, stroke_width=0)
    nose.scale(0.14).move_to(UP * 0.36)
    group = VGroup(arms, *motors.values(), body, nose)
    return group, motors


class AttitudeCorrectionAudience(ExplainerScene):
    """A sensed roll error produces differential thrust and restoring torque."""

    def construct(self) -> None:
        self.heading("자세 복원", "기울기를 읽고 반대 토크를 만들어 수평으로 돌아온다")

        stage = self.panel(9.8, 4.75).shift(LEFT * 2.15 + DOWN * 0.12)
        feedback = self.panel(4.3, 4.75, PANEL_2).shift(RIGHT * 5.05 + DOWN * 0.12)
        self.play(FadeIn(stage), FadeIn(feedback), run_time=0.45)

        drone = front_view_drone(1.15).move_to(stage.get_center() + UP * 0.15)
        drone.rotate(16 * DEGREES)
        wind = Arrow(
            stage.get_left() + RIGHT * 0.4 + UP * 1.25,
            stage.get_left() + RIGHT * 2.05 + UP * 1.25,
            buff=0,
            color=YELLOW,
            stroke_width=7,
        )
        wind_label = text("외란", 26, YELLOW, "BOLD").next_to(wind, UP, buff=0.14)
        error = text("자세 오차", 28, WHITE, "BOLD").move_to(
            stage.get_center() + DOWN * 1.55
        )
        self.play(GrowArrow(wind), FadeIn(wind_label), FadeIn(drone), FadeIn(error), run_time=0.8)

        sensor = process_box("센서가 기울기 감지", CYAN, 3.45)
        command = process_box("오차 반대 출력 차이", GREEN, 3.45)
        loop = VGroup(sensor, command).arrange(DOWN, buff=0.65).move_to(feedback)
        link = Arrow(sensor.get_bottom(), command.get_top(), buff=0.14, color=CYAN, stroke_width=6)
        self.play(FadeIn(sensor, shift=DOWN * 0.15), GrowArrow(link), FadeIn(command), run_time=0.9)

        left_thrust = Arrow(
            drone.get_left() + DOWN * 0.25,
            drone.get_left() + UP * 1.25,
            buff=0,
            color=GREEN,
            stroke_width=8,
        )
        right_thrust = Arrow(
            drone.get_right() + DOWN * 0.18,
            drone.get_right() + UP * 0.65,
            buff=0,
            color=BLUE,
            stroke_width=6,
        )
        torque = CurvedArrow(
            stage.get_center() + RIGHT * 1.35 + UP * 1.25,
            stage.get_center() + RIGHT * 1.65 + DOWN * 0.75,
            angle=-1.35,
            color=GREEN,
            stroke_width=7,
        )
        torque_label = text("복원 토크", 26, GREEN, "BOLD").next_to(
            torque, RIGHT, buff=0.15
        )
        self.play(
            GrowArrow(left_thrust),
            GrowArrow(right_thrust),
            Create(torque),
            FadeIn(torque_label),
            run_time=0.85,
        )

        level_drone = front_view_drone(1.15).move_to(drone.get_center())
        level = Line(
            stage.get_center() + LEFT * 2.8 + DOWN * 1.15,
            stage.get_center() + RIGHT * 2.8 + DOWN * 1.15,
            color=GRID,
            stroke_width=3,
        )
        restored = text("수평 복원", 29, GREEN, "BOLD").move_to(error)
        self.play(
            Transform(drone, level_drone),
            FadeOut(VGroup(wind, wind_label, left_thrust, right_thrust, torque, torque_label)),
            Create(level),
            Transform(error, restored),
            run_time=1.0,
        )
        self.conclusion("센서 피드백과 모터 출력 차이가 닫힌 자세 복원 고리를 만든다", GREEN)
        self.hold_and_clear(1.15)


class SilClosedLoopAudience(ExplainerScene):
    """Host SIL closes the real flight code around a virtual plant."""

    def construct(self) -> None:
        self.heading("SIL 폐루프", "비행 코드를 가상 기체와 센서 안에서 반복 실행한다")
        badge = evidence_badge("HOST SIL · 실제 비행 증거 아님", YELLOW)
        badge.to_corner(UR, buff=0.62).shift(DOWN * 1.25)
        self.play(FadeIn(badge, shift=LEFT * 0.15), run_time=0.4)

        labels = ["가상 물리", "센서 합성", "실제 비행 코드", "모터 출력"]
        colors = [BLUE, CYAN, GREEN, YELLOW]
        blocks = VGroup(*[process_box(label, color, 2.75) for label, color in zip(labels, colors)])
        blocks.arrange(RIGHT, buff=0.65).move_to(DOWN * 0.1)
        arrows = VGroup(
            *[
                Arrow(
                    blocks[index].get_right(),
                    blocks[index + 1].get_left(),
                    buff=0.1,
                    color=BLUE,
                    stroke_width=6,
                )
                for index in range(3)
            ]
        )
        code_note = text("실제 스케치 포함", 24, GREEN, "BOLD")
        code_note.next_to(blocks[2], DOWN, buff=0.2)

        self.play(FadeIn(blocks[0], shift=RIGHT * 0.15), run_time=0.5)
        for index in range(3):
            additions = [GrowArrow(arrows[index]), FadeIn(blocks[index + 1], shift=RIGHT * 0.15)]
            if index == 1:
                additions.append(FadeIn(code_note))
            self.play(*additions, run_time=0.65)

        return_path = VGroup(
            Line(blocks[3].get_bottom(), blocks[3].get_bottom() + DOWN * 1.0, color=CYAN, stroke_width=5),
            Line(blocks[3].get_bottom() + DOWN * 1.0, blocks[0].get_bottom() + DOWN * 1.0, color=CYAN, stroke_width=5),
            Arrow(
                blocks[0].get_bottom() + DOWN * 1.0,
                blocks[0].get_bottom(),
                buff=0.08,
                color=CYAN,
                stroke_width=5,
            ),
        )
        return_label = text("출력이 다음 물리 상태를 갱신", 26, CYAN, "BOLD")
        return_label.next_to(return_path[1], DOWN, buff=0.16)
        self.play(Create(return_path), FadeIn(return_label), run_time=0.85)
        self.conclusion("코드의 폐루프 반응은 확인하지만 실물 센서·모터·비행은 확인하지 않는다", YELLOW)
        self.hold_and_clear(1.2)


class FailsafeTimelineAudience(ExplainerScene):
    """Failsafe separates controllable link loss from critical failure."""

    def construct(self) -> None:
        self.heading("Failsafe 분기", "RC 두절과 치명적 고장은 같은 경로로 처리하지 않는다")
        badge = evidence_badge("고정 시간·보편 출력값 없음", YELLOW)
        badge.to_corner(UR, buff=0.62).shift(DOWN * 1.25)
        self.play(FadeIn(badge), run_time=0.35)

        trigger = process_box("상태 판단", BLUE, 2.35).shift(LEFT * 5.75 + DOWN * 0.15)
        self.play(FadeIn(trigger), run_time=0.45)

        rc_loss = process_box("RC 신호 두절", YELLOW, 2.55).move_to(LEFT * 2.8 + UP * 0.85)
        descent = process_box("자세 유지\n제한 하강", BLUE, 2.55).move_to(UP * 0.85)
        limit = process_box("설정된 상한\n모터 정지", GREEN, 2.55).move_to(RIGHT * 2.8 + UP * 0.85)
        normal_path = VGroup(
            Arrow(trigger.get_right(), rc_loss.get_left(), buff=0.12, color=YELLOW, stroke_width=6),
            Arrow(rc_loss.get_right(), descent.get_left(), buff=0.12, color=BLUE, stroke_width=6),
            Arrow(descent.get_right(), limit.get_left(), buff=0.12, color=GREEN, stroke_width=6),
        )
        self.play(GrowArrow(normal_path[0]), FadeIn(rc_loss), run_time=0.55)
        self.play(GrowArrow(normal_path[1]), FadeIn(descent), run_time=0.7)
        self.play(GrowArrow(normal_path[2]), FadeIn(limit), run_time=0.7)

        critical = process_box("치명적 고장", RED, 2.55).move_to(LEFT * 1.4 + DOWN * 1.45)
        immediate = process_box("즉시 모터 정지", RED, 3.0).move_to(RIGHT * 2.0 + DOWN * 1.45)
        critical_path = VGroup(
            Arrow(trigger.get_right(), critical.get_left(), buff=0.12, color=RED, stroke_width=6),
            Arrow(critical.get_right(), immediate.get_left(), buff=0.12, color=RED, stroke_width=7),
        )
        self.play(GrowArrow(critical_path[0]), FadeIn(critical), run_time=0.6)
        self.play(GrowArrow(critical_path[1]), FadeIn(immediate), run_time=0.65)
        self.conclusion("자세 유지 가능 여부에 따라 제한 하강과 즉시 정지가 갈린다", YELLOW)
        self.hold_and_clear(1.35)


class LandingObservabilityAudience(ExplainerScene):
    """Rest and constant descent can produce indistinguishable IMU cues."""

    def construct(self) -> None:
        self.heading("착지 관측 한계", "IMU만으로 정지와 등속 하강을 확정적으로 가르기 어렵다")

        left = self.panel(6.55, 4.6).shift(LEFT * 3.55 + DOWN * 0.12)
        right = self.panel(6.55, 4.6).shift(RIGHT * 3.55 + DOWN * 0.12)
        self.play(FadeIn(left), FadeIn(right), run_time=0.45)

        grounded = front_view_drone(0.76).move_to(left.get_center() + UP * 0.45)
        descending = front_view_drone(0.76).move_to(right.get_center() + UP * 0.65)
        ground = Line(
            grounded.get_left() + LEFT * 0.5 + DOWN * 0.55,
            grounded.get_right() + RIGHT * 0.5 + DOWN * 0.55,
            color=MUTED,
            stroke_width=7,
        )
        down = Arrow(
            descending.get_bottom() + DOWN * 0.1,
            descending.get_bottom() + DOWN * 1.05,
            buff=0,
            color=BLUE,
            stroke_width=7,
        )
        left_label = text("지면 정지", 30, WHITE, "BOLD").move_to(left.get_center() + UP * 1.65)
        right_label = text("등속 하강", 30, WHITE, "BOLD").move_to(right.get_center() + UP * 1.65)
        self.play(
            FadeIn(VGroup(grounded, ground, left_label)),
            FadeIn(VGroup(descending, right_label)),
            GrowArrow(down),
            run_time=0.8,
        )

        def imu_trace(panel: Mobject) -> VGroup:
            baseline = Line(LEFT * 2.15, RIGHT * 2.15, color=GRID, stroke_width=3)
            trace = VMobject(color=GREEN, stroke_width=6).set_points_as_corners(
                [
                    LEFT * 2.05,
                    LEFT * 1.3 + UP * 0.04,
                    LEFT * 0.45 + DOWN * 0.03,
                    RIGHT * 0.35 + UP * 0.03,
                    RIGHT * 1.15 + DOWN * 0.02,
                    RIGHT * 2.05,
                ]
            )
            trace_group = VGroup(baseline, trace).move_to(panel.get_center() + DOWN * 1.15)
            label = text("IMU: 같은 중력 기준", 25, GREEN, "BOLD").next_to(
                trace_group, DOWN, buff=0.16
            )
            return VGroup(trace_group, label)

        left_trace = imu_trace(left)
        right_trace = imu_trace(right)
        self.play(Create(left_trace[0]), FadeIn(left_trace[1]), run_time=0.6)
        self.play(Create(right_trace[0]), FadeIn(right_trace[1]), run_time=0.6)

        uncertain = text("IMU 관측만으로 구분 어려움", 28, YELLOW, "BOLD")
        uncertain.move_to(DOWN * 2.65)
        self.play(FadeIn(uncertain), run_time=0.4)
        self.conclusion("거리 센서는 구분에 도움 · 현재 폐루프 착지 판정은 미검증", YELLOW)
        self.hold_and_clear(1.65)


class SharedStateRaceAudience(ExplainerScene):
    """An overlapping read-modify-write can overwrite a newer shared state."""

    def construct(self) -> None:
        self.heading("공유 상태 경쟁", "두 실행 흐름이 같은 상태를 읽고 쓰는 순서를 본다")
        badge = evidence_badge("가능한 race 시나리오 · 관측 사고 아님", YELLOW)
        badge.to_corner(UR, buff=0.62).shift(DOWN * 1.25)
        self.play(FadeIn(badge), run_time=0.4)

        communication = process_box("통신 코어", BLUE, 3.0).move_to(LEFT * 5.0 + UP * 0.55)
        control = process_box("제어 태스크", GREEN, 3.0).move_to(RIGHT * 5.0 + UP * 0.55)
        shared = process_box("공유 상태 A", YELLOW, 3.2).move_to(UP * 0.55)
        self.play(FadeIn(communication), FadeIn(control), FadeIn(shared), run_time=0.6)

        comm_copy = process_box("① A 읽기\n수정 시작", BLUE, 2.7).move_to(LEFT * 4.0 + DOWN * 1.35)
        control_copy = process_box("② A 읽기\n수정 시작", GREEN, 2.7).move_to(RIGHT * 4.0 + DOWN * 1.35)
        comm_read = Arrow(shared.get_left(), communication.get_right(), buff=0.15, color=BLUE, stroke_width=6)
        control_read = Arrow(shared.get_right(), control.get_left(), buff=0.15, color=GREEN, stroke_width=6)
        self.play(GrowArrow(comm_read), FadeIn(comm_copy), run_time=0.65)
        self.play(GrowArrow(control_read), FadeIn(control_copy), run_time=0.65)

        state_b = process_box("③ 제어가 B 쓰기", GREEN, 3.2).move_to(shared)
        control_write = Arrow(control.get_left(), shared.get_right(), buff=0.15, color=GREEN, stroke_width=7)
        self.play(GrowArrow(control_write), Transform(shared, state_b), run_time=0.7)

        state_c = process_box("④ 통신이 C 쓰기", RED, 3.2).move_to(shared)
        stale = text("옛 A 기반", 25, RED, "BOLD").next_to(comm_copy, DOWN, buff=0.16)
        comm_write = Arrow(communication.get_right(), shared.get_left(), buff=0.15, color=RED, stroke_width=7)
        self.play(FadeIn(stale), GrowArrow(comm_write), Transform(shared, state_c), run_time=0.75)
        overwrite = text("B 갱신이 덮일 수 있음", 28, RED, "BOLD").next_to(shared, DOWN, buff=0.22)
        self.play(FadeIn(overwrite), run_time=0.4)
        self.conclusion("읽기–수정–쓰기가 겹치면 최신 갱신을 잃을 수 있다", YELLOW)
        self.hold_and_clear(1.25)


class TelemetryMotorBalanceAudience(ExplainerScene):
    """Show only the measured aggregate direction, not invented row values."""

    def construct(self) -> None:
        self.heading("모터 출력 균형", "테더 구간의 집계 방향을 X형 모터 배치에서 읽는다")
        badge = evidence_badge("테더 구간 · 집계 방향", BLUE)
        badge.to_corner(UR, buff=0.62).shift(DOWN * 1.25)
        self.play(FadeIn(badge), run_time=0.4)

        airframe_panel = self.panel(6.5, 4.2).shift(LEFT * 4.1 + DOWN * 0.1)
        bars_panel = self.panel(7.2, 4.2, PANEL_2).shift(RIGHT * 3.45 + DOWN * 0.1)
        self.play(FadeIn(airframe_panel), FadeIn(bars_panel), run_time=0.45)

        airframe, motors = motor_layout()
        airframe.scale(0.9).move_to(airframe_panel.get_center() + DOWN * 0.05)
        layout_label = text("X형 모터 배치", 27, WHITE, "BOLD")
        layout_label.move_to(airframe_panel.get_center() + UP * 1.85)
        self.play(FadeIn(airframe), FadeIn(layout_label), run_time=0.7)

        baseline_y = bars_panel.get_bottom()[1] + 1.15
        m1_bar = Rectangle(
            width=1.25,
            height=1.5,
            fill_color=YELLOW,
            fill_opacity=0.85,
            stroke_color=YELLOW,
            stroke_width=3,
        ).move_to([2.65, baseline_y + 0.75, 0])
        m3_bar = Rectangle(
            width=1.25,
            height=2.05,
            fill_color=BLUE,
            fill_opacity=0.9,
            stroke_color=BLUE,
            stroke_width=3,
        ).move_to([4.75, baseline_y + 1.025, 0])
        baseline = Line(
            [1.65, baseline_y, 0],
            [5.75, baseline_y, 0],
            color=GRID,
            stroke_width=4,
        )
        m1_label = text("M1 평균", 27, YELLOW, "BOLD").next_to(m1_bar, DOWN, buff=0.18)
        m3_label = text("M3 평균", 27, BLUE, "BOLD").next_to(m3_bar, DOWN, buff=0.18)
        direction = text("M3 평균 > M1 평균", 31, WHITE, "BOLD")
        direction.move_to(bars_panel.get_center() + UP * 1.65)
        symbolic = text("막대는 크기 비례가 아닌 집계 방향", 24, MUTED, "BOLD")
        symbolic.move_to(bars_panel.get_center() + DOWN * 1.8)
        self.play(Create(baseline), GrowFromEdge(m1_bar, DOWN), FadeIn(m1_label), run_time=0.65)
        self.play(GrowFromEdge(m3_bar, DOWN), FadeIn(m3_label), FadeIn(direction), run_time=0.75)
        self.play(
            Indicate(motors["M1"], color=YELLOW),
            Indicate(motors["M3"], color=BLUE),
            FadeIn(symbolic),
            run_time=0.65,
        )

        causes = text("무게 배분 · 추력 · 테더 · 프레임 · 공력", 25, MUTED, "BOLD")
        causes.move_to(DOWN * 2.62)
        self.play(FadeIn(causes), run_time=0.4)
        self.conclusion("관측된 방향은 분명하지만 비대칭의 원인은 미확정이다", YELLOW)
        self.hold_and_clear(1.3)
