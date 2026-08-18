"""Audience-first animations for the deck's significance and flight-principle slides."""

from __future__ import annotations

from manim import *

from audience_visualizations import (
    BG,
    BLUE,
    CYAN,
    GREEN,
    GRID,
    MUTED,
    ORANGE,
    PANEL,
    PANEL_2,
    RED,
    WHITE,
    YELLOW,
    ExplainerScene,
    drone_icon,
    text,
)


def fixed_wing_icon(scale: float = 1.0, color: str = BLUE) -> VGroup:
    fuselage = RoundedRectangle(
        width=3.0,
        height=0.32,
        corner_radius=0.14,
        fill_color=color,
        fill_opacity=1,
        stroke_color=WHITE,
        stroke_width=2,
    )
    wing = Polygon(
        LEFT * 0.5 + UP * 0.08,
        LEFT * 0.15 + UP * 1.0,
        RIGHT * 0.45 + UP * 1.0,
        RIGHT * 0.75 + UP * 0.08,
        RIGHT * 0.45 + DOWN * 1.0,
        LEFT * 0.15 + DOWN * 1.0,
        fill_color=color,
        fill_opacity=1,
        stroke_color=WHITE,
        stroke_width=2,
    )
    tail = Polygon(
        LEFT * 1.25,
        LEFT * 1.65 + UP * 0.52,
        LEFT * 1.1 + UP * 0.52,
        LEFT * 0.7,
        LEFT * 1.1 + DOWN * 0.52,
        LEFT * 1.65 + DOWN * 0.52,
        fill_color=color,
        fill_opacity=1,
        stroke_color=WHITE,
        stroke_width=2,
    )
    nose = Triangle(fill_color=YELLOW, fill_opacity=1, stroke_width=0)
    nose.scale(0.14).rotate(-PI / 2).next_to(fuselage, RIGHT, buff=-0.02)
    return VGroup(wing, tail, fuselage, nose).scale(scale)


def helicopter_icon(scale: float = 1.0, color: str = BLUE) -> VGroup:
    body = Ellipse(
        width=2.4,
        height=0.9,
        fill_color=color,
        fill_opacity=1,
        stroke_color=WHITE,
        stroke_width=2,
    )
    tail_boom = Line(body.get_left(), LEFT * 2.35, color=MUTED, stroke_width=10)
    tail = VGroup(
        Line(LEFT * 2.35 + UP * 0.36, LEFT * 2.35 + DOWN * 0.36, color=CYAN, stroke_width=5),
        Circle(radius=0.14, color=WHITE, stroke_width=3).move_to(LEFT * 2.35),
    )
    mast = Line(UP * 0.35, UP * 0.78, color=MUTED, stroke_width=7)
    rotor = Line(LEFT * 1.9 + UP * 0.78, RIGHT * 1.9 + UP * 0.78, color=CYAN, stroke_width=7)
    return VGroup(tail_boom, tail, body, mast, rotor).scale(scale)


def x_quadcopter_icon(scale: float = 1.0, color: str = BLUE) -> VGroup:
    """Top-view quadcopter with four motors at diagonal X-frame endpoints."""

    diagonal = 1.25
    points = [
        LEFT * diagonal + UP * diagonal,
        RIGHT * diagonal + UP * diagonal,
        LEFT * diagonal + DOWN * diagonal,
        RIGHT * diagonal + DOWN * diagonal,
    ]
    arms = VGroup(
        Line(points[0], points[3], color=MUTED, stroke_width=10),
        Line(points[1], points[2], color=MUTED, stroke_width=10),
    )
    motors = VGroup(
        *[
            Circle(
                radius=0.28,
                fill_color=PANEL_2,
                fill_opacity=1,
                stroke_color=CYAN,
                stroke_width=4,
            ).move_to(point)
            for point in points
        ]
    )
    body = Circle(
        radius=0.55,
        fill_color=color,
        fill_opacity=1,
        stroke_color=WHITE,
        stroke_width=3,
    )
    nose = Triangle(fill_color=YELLOW, fill_opacity=1, stroke_width=0)
    nose.scale(0.14).move_to(UP * 0.38)
    return VGroup(arms, motors, body, nose).scale(scale)


def evtol_icon(scale: float = 1.0, color: str = BLUE) -> VGroup:
    wing = fixed_wing_icon(0.72, color)
    rotors = VGroup(
        *[
            Circle(radius=0.22, color=CYAN, stroke_width=4).move_to(point)
            for point in (
                LEFT * 1.25 + UP * 0.72,
                RIGHT * 1.25 + UP * 0.72,
                LEFT * 1.25 + DOWN * 0.72,
                RIGHT * 1.25 + DOWN * 0.72,
            )
        ]
    )
    return VGroup(wing, rotors).scale(scale)


def current_relationship(
    icon: Mobject, name: str, meaning: str, color: str = BLUE
) -> VGroup:
    icon.move_to(LEFT * 3.8 + DOWN * 0.1)
    name_label = text(name, 32, color, "BOLD")
    meaning_label = text(meaning, 30, WHITE, "BOLD")
    labels = VGroup(name_label, meaning_label).arrange(
        DOWN, aligned_edge=LEFT, buff=0.3
    )
    labels.move_to(RIGHT * 3.0 + DOWN * 0.1)
    connector = Arrow(
        icon.get_right() + RIGHT * 0.3,
        labels.get_left() + LEFT * 0.3,
        buff=0,
        color=color,
        stroke_width=6,
    )
    return VGroup(icon, connector, labels)


class DroneClassificationAudience(ExplainerScene):
    """Move through aircraft families, then focus on the quadcopter."""

    def construct(self) -> None:
        self.heading("드론 분류", "비행 방식이 달라지면 기체 구조도 달라진다")
        stage = self.panel(14.3, 4.75).shift(DOWN * 0.1)
        self.play(FadeIn(stage), run_time=0.35)

        relationships = [
            current_relationship(fixed_wing_icon(1.05), "고정익", "날개로 순항"),
            current_relationship(helicopter_icon(0.95), "단일로터", "큰 로터로 체공", GREEN),
            current_relationship(x_quadcopter_icon(0.95), "멀티로터", "여러 로터로 자세 제어", CYAN),
            current_relationship(evtol_icon(0.95), "수직이착륙기", "이착륙과 순항 결합", ORANGE),
        ]
        current = relationships[0]
        self.play(FadeIn(current, shift=RIGHT * 0.25), run_time=0.55)
        for relationship_index, next_relationship in enumerate(
            relationships[1:], start=1
        ):
            self.play(ReplacementTransform(current, next_relationship), run_time=0.65)
            current = next_relationship
            if relationship_index == 2:
                self.wait(1.0)

        quad = current_relationship(
            x_quadcopter_icon(1.02, CYAN), "멀티로터 안의 쿼드콥터", "X형 4로터 기체", CYAN
        )
        focus = SurroundingRectangle(
            quad, color=CYAN, buff=0.18, stroke_width=4, corner_radius=0.2
        )
        self.play(ReplacementTransform(current, quad), Create(focus), run_time=0.7)
        self.conclusion("이번 발표의 중심 기체는 멀티로터에 속하는 쿼드콥터이다", CYAN)
        self.hold_and_clear()


class QualificationWeightAudience(ExplainerScene):
    """Show the mixed legal weight criteria without turning them into design values."""

    def construct(self) -> None:
        self.heading("조종자 증명", "분류 전에 기체의 두 무게 기준을 실제로 재야 한다")

        axis = Arrow(LEFT * 6.5, RIGHT * 1.3, buff=0, color=GRID, stroke_width=6)
        axis.shift(UP * 0.8)
        axis_label = text("최대이륙중량 기준", 26, MUTED, "BOLD").next_to(
            axis, UP, buff=0.25
        )
        self.play(GrowArrow(axis), FadeIn(axis_label), run_time=0.55)

        nodes = VGroup()
        for x, label in zip((-5.2, -2.8, -0.4), ("4종", "3종", "2종")):
            dot = Circle(radius=0.23, fill_color=BLUE, fill_opacity=1, stroke_width=0)
            dot.move_to([x, 0.8, 0])
            node_label = text(label, 27, WHITE, "BOLD").next_to(dot, DOWN, buff=0.18)
            node = VGroup(dot, node_label)
            nodes.add(node)
            self.play(FadeIn(node, shift=RIGHT * 0.15), run_time=0.45)

        left_box = self.panel(5.8, 2.25, PANEL_2).move_to(LEFT * 3.25 + DOWN * 1.35)
        right_box = self.panel(5.8, 2.25, PANEL_2).move_to(RIGHT * 3.25 + DOWN * 1.35)
        left_text = VGroup(
            text("1종 기준 A", 26, YELLOW, "BOLD"),
            text("최대이륙중량 25kg 초과", 29, WHITE, "BOLD"),
        ).arrange(DOWN, buff=0.25).move_to(left_box)
        right_text = VGroup(
            text("1종 기준 B", 26, YELLOW, "BOLD"),
            text("연료 제외 자체중량 150kg 이하", 28, WHITE, "BOLD"),
        ).arrange(DOWN, buff=0.25).move_to(right_box)
        self.play(
            FadeIn(left_box, shift=DOWN * 0.2),
            FadeIn(left_text, shift=DOWN * 0.2),
            FadeIn(right_box, shift=DOWN * 0.2),
            FadeIn(right_text, shift=DOWN * 0.2),
            run_time=1.0,
        )
        and_badge = text("두 기준을 함께 확인", 27, CYAN, "BOLD")
        and_badge.move_to(DOWN * 2.85)
        self.play(FadeIn(and_badge), run_time=0.45)
        self.conclusion("법적 분류는 추정값이 아니라 신청 전 기체 계측에서 시작한다", YELLOW)
        self.hold_and_clear(1.4)


class AircraftUamAudience(ExplainerScene):
    """Compare aircraft strengths, then add the systems that make UAM."""

    def construct(self) -> None:
        self.heading("비행체와 UAM", "비행체 하나에 운항 체계가 연결되어야 UAM이 된다")
        baseline = Line(LEFT * 6.6, RIGHT * 6.6, color=GRID, stroke_width=4).shift(
            DOWN * 0.25
        )
        self.play(Create(baseline), run_time=0.4)

        relationships = [
            current_relationship(fixed_wing_icon(0.9), "고정익", "순항 효율"),
            current_relationship(helicopter_icon(0.82), "헬리콥터", "체공", GREEN),
            current_relationship(x_quadcopter_icon(0.82), "멀티콥터", "정밀 호버", CYAN),
            current_relationship(evtol_icon(0.85), "eVTOL", "수직이착륙 + 순항", ORANGE),
        ]
        current = relationships[0]
        self.play(FadeIn(current), run_time=0.5)
        for next_relationship in relationships[1:]:
            self.play(ReplacementTransform(current, next_relationship), run_time=0.65)
            current = next_relationship

        system_labels = VGroup(
            text("버티포트", 26, WHITE, "BOLD"),
            text("운항", 26, WHITE, "BOLD"),
            text("교통관리", 26, WHITE, "BOLD"),
        ).arrange(RIGHT, buff=0.8).move_to(DOWN * 2.35)
        system_arrows = VGroup(
            *[
                Arrow(
                    current.get_bottom(),
                    label.get_top(),
                    buff=0.18,
                    color=ORANGE,
                    stroke_width=4,
                )
                for label in system_labels
            ]
        )
        self.play(FadeIn(system_labels), Create(system_arrows), run_time=0.8)
        self.conclusion("eVTOL과 지상·운항·교통 체계의 결합이 UAM이다", ORANGE)
        self.hold_and_clear(1.4)


class MissionSpecsAudience(ExplainerScene):
    """Keep one airframe central while missions replace its required specification."""

    def construct(self) -> None:
        self.heading("임무별 요구 사양", "같은 기체라도 임무가 바뀌면 우선 사양이 달라진다")
        stage = self.panel(14.3, 4.75).shift(DOWN * 0.1)
        aircraft = x_quadcopter_icon(0.82).move_to(LEFT * 4.7 + DOWN * 0.05)
        self.play(FadeIn(stage), FadeIn(aircraft), run_time=0.55)

        pairs = [
            ("공연", "동기화", CYAN),
            ("물류", "탑재중량", YELLOW),
            ("안전 감시", "체공 · 통신", GREEN),
            ("국방 정찰", "보안 · 내환경성", ORANGE),
        ]
        mission, spec, color = pairs[0]
        current = VGroup(
            text(mission, 31, color, "BOLD"),
            Arrow(LEFT * 1.05, RIGHT * 1.05, buff=0, color=color, stroke_width=6),
            text(spec, 35, WHITE, "BOLD"),
        ).arrange(RIGHT, buff=0.45).move_to(RIGHT * 2.35 + DOWN * 0.05)
        self.play(FadeIn(current, shift=RIGHT * 0.2), run_time=0.55)
        for mission, spec, color in pairs[1:]:
            next_relation = VGroup(
                text(mission, 31, color, "BOLD"),
                Arrow(LEFT * 1.05, RIGHT * 1.05, buff=0, color=color, stroke_width=6),
                text(spec, 35, WHITE, "BOLD"),
            ).arrange(RIGHT, buff=0.45).move_to(RIGHT * 2.35 + DOWN * 0.05)
            self.play(ReplacementTransform(current, next_relation), run_time=0.65)
            current = next_relation

        scope_note = text("작전 절차가 아니라 요구 사양의 차이", 26, MUTED, "BOLD")
        scope_note.move_to(RIGHT * 2.4 + DOWN * 1.65)
        self.play(FadeIn(scope_note), run_time=0.4)
        self.conclusion("임무 정의가 기체와 시스템의 요구 사양을 결정한다", BLUE)
        self.hold_and_clear(1.45)


def force_state(
    label: str, thrust_length: float, thrust_color: str, tilt: float = 0.0
) -> VGroup:
    drone = x_quadcopter_icon(0.78).rotate(tilt)
    drone.move_to(LEFT * 2.2 + DOWN * 0.1)
    gravity = Arrow(
        drone.get_center(),
        drone.get_center() + DOWN * 1.75,
        buff=0.5,
        color=YELLOW,
        stroke_width=7,
    )
    thrust_direction = rotate_vector(UP, tilt)
    thrust = Arrow(
        drone.get_center(),
        drone.get_center() + thrust_direction * thrust_length,
        buff=0.5,
        color=thrust_color,
        stroke_width=9,
    )
    thrust_label = text("합추력", 27, thrust_color, "BOLD").next_to(
        thrust.get_end(), RIGHT, buff=0.2
    )
    gravity_label = text("중력", 27, YELLOW, "BOLD").next_to(
        gravity.get_end(), RIGHT, buff=0.2
    )
    state_label = text(label, 38, WHITE, "BOLD").move_to(RIGHT * 4.0)
    return VGroup(drone, gravity, thrust, thrust_label, gravity_label, state_label)


class QuadcopterForceMotionAudience(ExplainerScene):
    """Relate the resultant thrust vector to four translational motion states."""

    def construct(self) -> None:
        self.heading("힘과 운동", "합추력의 크기와 방향이 기체의 이동을 바꾼다")
        stage = self.panel(14.3, 4.75).shift(DOWN * 0.1)
        self.play(FadeIn(stage), run_time=0.35)

        states = [
            force_state("상승", 2.45, GREEN),
            force_state("호버링", 1.75, CYAN),
            force_state("하강", 1.15, ORANGE),
            force_state("수평 이동", 2.1, BLUE, -16 * DEGREES),
        ]
        current = states[0]
        self.play(FadeIn(current), run_time=0.6)
        for next_state in states[1:]:
            self.play(ReplacementTransform(current, next_state), run_time=0.75)
            current = next_state

        yaw_note = text("Yaw는 다음 장의 반작용 토크로 설명", 26, MUTED, "BOLD")
        yaw_note.move_to(RIGHT * 3.7 + DOWN * 1.35)
        self.play(FadeIn(yaw_note), run_time=0.35)
        self.conclusion("합추력을 중력과 맞추거나 기울이면 상승·호버·하강·이동이 정해진다", BLUE)
        self.hold_and_clear(1.3)


class HelicopterQuadcopterTorqueAudience(ExplainerScene):
    """Contrast tail-rotor cancellation with paired quadcopter reaction torques."""

    def construct(self) -> None:
        self.heading("반작용 토크", "헬기와 쿼드콥터는 서로 다른 방식으로 Yaw를 다룬다")
        left_panel = self.panel(6.8, 4.7).move_to(LEFT * 3.65 + DOWN * 0.1)
        right_panel = self.panel(6.8, 4.7, PANEL_2).move_to(RIGHT * 3.65 + DOWN * 0.1)
        self.play(FadeIn(left_panel), FadeIn(right_panel), run_time=0.45)

        helicopter = helicopter_icon(0.72).rotate(PI / 2).move_to(
            left_panel.get_center() + UP * 0.15
        )
        reaction = CurvedArrow(
            helicopter.get_center() + RIGHT * 1.05,
            helicopter.get_center() + UP * 1.05,
            angle=PI / 2,
            color=ORANGE,
            stroke_width=6,
        )
        tail_force = Arrow(
            helicopter.get_center() + DOWN * 1.7,
            helicopter.get_center() + DOWN * 1.7 + RIGHT * 1.25,
            buff=0,
            color=CYAN,
            stroke_width=7,
        )
        heli_labels = VGroup(
            text("헬리콥터", 28, WHITE, "BOLD"),
            text("꼬리로터 힘으로 상쇄", 26, CYAN, "BOLD"),
        ).arrange(DOWN, buff=0.18).move_to(left_panel.get_center() + DOWN * 1.55)
        self.play(
            FadeIn(helicopter), Create(reaction), GrowArrow(tail_force), FadeIn(heli_labels), run_time=1.0
        )

        quad = x_quadcopter_icon(0.68).move_to(right_panel.get_center() + UP * 0.15)
        rotor_positions = [motor.get_center() for motor in quad[1]]
        rotation_labels = VGroup(
            *[
                text(label, 24, color, "BOLD").move_to(position + direction * 0.48)
                for position, direction, label, color in zip(
                    rotor_positions,
                    (UL, UR, DL, DR),
                    ("CW", "CCW", "CCW", "CW"),
                    (ORANGE, CYAN, CYAN, ORANGE),
                )
            ]
        )
        yaw_arrow = CurvedArrow(
            quad.get_center() + RIGHT * 0.85,
            quad.get_center() + UP * 0.85,
            angle=PI / 2,
            color=YELLOW,
            stroke_width=6,
        )
        quad_labels = VGroup(
            text("쿼드콥터", 28, WHITE, "BOLD"),
            text("CW·CCW 토크 차이로 Yaw", 26, YELLOW, "BOLD"),
        ).arrange(DOWN, buff=0.18).move_to(right_panel.get_center() + DOWN * 1.55)
        self.play(
            FadeIn(quad), FadeIn(rotation_labels), Create(yaw_arrow), FadeIn(quad_labels), run_time=1.1
        )
        self.play(Indicate(tail_force, color=CYAN), Indicate(yaw_arrow, color=YELLOW), run_time=0.8)
        self.conclusion("헬기는 꼬리 힘으로 상쇄하고, 쿼드는 로터 쌍의 토크 차이로 회전한다", YELLOW)
        self.hold_and_clear(1.45)


class SwarmSystemAudience(ExplainerScene):
    """Expand one unverified vehicle into a swarm with explicit new system needs."""

    def construct(self) -> None:
        self.heading("군집 확장", "기체 수가 늘면 비행 제어 밖의 시스템 요구가 추가된다")
        single = x_quadcopter_icon(0.62).move_to(LEFT * 4.9 + DOWN * 0.15)
        single_label = text("검증이 필요한 단일 기체", 27, WHITE, "BOLD")
        single_label.next_to(single, DOWN, buff=0.3)
        self.play(FadeIn(single), FadeIn(single_label), run_time=0.55)

        positions = [
            LEFT * 1.8 + UP * 1.05,
            RIGHT * 1.8 + UP * 1.05,
            LEFT * 1.8 + DOWN * 1.05,
            RIGHT * 1.8 + DOWN * 1.05,
        ]
        fleet = VGroup(*[x_quadcopter_icon(0.38).move_to(p + RIGHT * 2.7) for p in positions])
        links = VGroup(
            Line(fleet[0], fleet[1], color=GRID, stroke_width=3),
            Line(fleet[0], fleet[2], color=GRID, stroke_width=3),
            Line(fleet[1], fleet[3], color=GRID, stroke_width=3),
            Line(fleet[2], fleet[3], color=GRID, stroke_width=3),
            Line(fleet[0], fleet[3], color=GRID, stroke_width=3),
        )
        expand_arrow = Arrow(LEFT * 2.8, LEFT * 0.8, color=BLUE, stroke_width=7)
        self.play(GrowArrow(expand_arrow), FadeIn(links), FadeIn(fleet), run_time=0.85)

        requirements = [
            ("공통 좌표", BLUE),
            ("통신", CYAN),
            ("상대 위치", GREEN),
            ("경로 · 충돌 회피", YELLOW),
            ("집단 안전", ORANGE),
        ]
        current = text(requirements[0][0], 31, requirements[0][1], "BOLD")
        current.move_to(DOWN * 2.25)
        self.play(FadeIn(current), run_time=0.4)
        for label, color in requirements[1:]:
            next_requirement = text(label, 31, color, "BOLD").move_to(current)
            self.play(ReplacementTransform(current, next_requirement), run_time=0.45)
            current = next_requirement

        status = text("후속 목표 · 미구현 · 미검증", 27, RED, "BOLD")
        status.move_to(RIGHT * 4.75 + UP * 2.15)
        status_box = SurroundingRectangle(status, color=RED, buff=0.15, stroke_width=3)
        self.play(FadeIn(status), Create(status_box), run_time=0.55)
        self.conclusion("군집은 단일 기체 검증 뒤에 좌표·통신·회피·안전을 더해야 하는 후속 목표이다", RED)
        self.hold_and_clear(1.25)
