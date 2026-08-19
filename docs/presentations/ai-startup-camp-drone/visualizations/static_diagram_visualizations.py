"""Simultaneous-comparison Manim diagrams for the startup-camp deck.

These scenes are intentionally static: every state required for comparison is
present in the same frame.  The surrounding HTML slide supplies the title, so
the image itself contains only labels, causal relationships, and one boundary
or takeaway line.
"""

from __future__ import annotations

import json
from decimal import Decimal, ROUND_HALF_UP
from math import cos
from pathlib import Path

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
from engineering_visualizations import front_view_drone, motor_layout
from significance_visualizations import (
    evtol_icon,
    fixed_wing_icon,
    helicopter_icon,
    x_quadcopter_icon,
)


def boundary_badge(message: str, color: str = YELLOW) -> VGroup:
    label = text(message, 24, color, "BOLD")
    box = RoundedRectangle(
        width=label.width + 0.58,
        height=0.62,
        corner_radius=0.12,
        fill_color=PANEL_2,
        fill_opacity=1,
        stroke_color=color,
        stroke_width=2,
    )
    label.move_to(box)
    return VGroup(box, label)


def takeaway(message: str, color: str = BLUE) -> VGroup:
    box = RoundedRectangle(
        width=15.1,
        height=0.76,
        corner_radius=0.14,
        fill_color=PANEL_2,
        fill_opacity=1,
        stroke_color=color,
        stroke_width=3,
    ).move_to(DOWN * 3.86)
    label = text(message, 25, WHITE, "BOLD").move_to(box)
    return VGroup(box, label)


def card(width: float, height: float, color: str = BLUE) -> RoundedRectangle:
    return RoundedRectangle(
        width=width,
        height=height,
        corner_radius=0.2,
        fill_color=PANEL,
        fill_opacity=1,
        stroke_color=color,
        stroke_width=3,
    )


def label_pill(message: str, color: str = BLUE, width: float | None = None) -> VGroup:
    label = text(message, 24, WHITE, "BOLD")
    resolved_width = max(width or 0.0, label.width + 0.55)
    pill = RoundedRectangle(
        width=resolved_width,
        height=0.58,
        corner_radius=0.15,
        fill_color=PANEL_2,
        fill_opacity=1,
        stroke_color=color,
        stroke_width=2,
    )
    label.move_to(pill)
    return VGroup(pill, label)


def process_node(message: str, color: str = BLUE, width: float = 2.5) -> VGroup:
    label = text(message, 25, WHITE, "BOLD")
    box = card(max(width, label.width + 0.6), 1.1, color)
    label.move_to(box)
    return VGroup(box, label)


def icon_with_caption(
    icon: Mobject,
    name: str,
    caption: str,
    position,
    color: str,
    *,
    highlighted: bool = False,
) -> VGroup:
    panel = card(3.55, 6.35, CYAN if highlighted else color).move_to(position)
    name_label = text(name, 29, color, "BOLD").move_to(
        panel.get_center() + UP * 2.4
    )
    icon.move_to(panel.get_center() + UP * 0.55)
    caption_label = text(caption, 24, WHITE, "BOLD").move_to(
        panel.get_center() + DOWN * 1.6
    )
    return VGroup(panel, name_label, icon, caption_label)


class StaticDiagramScene(ExplainerScene):
    """Base class whose scenes add one complete frame without timed reveals."""

    def setup(self) -> None:
        self.camera.background_color = BG


class DroneClassificationStatic(StaticDiagramScene):
    def construct(self) -> None:
        fixed = icon_with_caption(
            fixed_wing_icon(0.72),
            "고정익",
            "날개로 순항",
            LEFT * 5.75 + UP * 0.32,
            BLUE,
        )
        single = icon_with_caption(
            helicopter_icon(0.58),
            "단일로터",
            "큰 로터로 체공",
            LEFT * 1.92 + UP * 0.32,
            GREEN,
        )
        multi = icon_with_caption(
            x_quadcopter_icon(0.72, CYAN),
            "멀티로터",
            "여러 로터로 자세 제어",
            RIGHT * 1.92 + UP * 0.32,
            CYAN,
            highlighted=True,
        )
        quad = label_pill("쿼드콥터", CYAN, 2.55).move_to(
            multi[0].get_center() + DOWN * 2.35
        )
        vertical = icon_with_caption(
            evtol_icon(0.67),
            "수직이착륙기",
            "이착륙과 순항 결합",
            RIGHT * 5.75 + UP * 0.32,
            ORANGE,
        )
        note = takeaway("이번 기체 · 멀티로터 안의 X형 4로터 쿼드콥터", CYAN)
        self.add(fixed, single, multi, quad, vertical, note)


class QualificationWeightStatic(StaticDiagramScene):
    def construct(self) -> None:
        top_cards = VGroup()
        ranges = (
            ("4종", "250g 초과 · 2kg 이하", BLUE),
            ("3종", "2kg 초과 · 7kg 이하", CYAN),
            ("2종", "7kg 초과 · 25kg 이하", GREEN),
        )
        for x, (name, weight, color) in zip((-5.0, 0.0, 5.0), ranges):
            panel = card(4.55, 2.35, color).move_to([x, 2.02, 0])
            name_label = text(name, 31, color, "BOLD").move_to(
                panel.get_center() + UP * 0.48
            )
            weight_label = text(weight, 25, WHITE, "BOLD").move_to(
                panel.get_center() + DOWN * 0.48
            )
            top_cards.add(VGroup(panel, name_label, weight_label))

        first = card(14.55, 2.75, YELLOW).move_to(DOWN * 0.97)
        first_label = text("1종", 34, YELLOW, "BOLD").move_to(
            first.get_center() + LEFT * 5.85
        )
        criterion_a = label_pill("최대이륙중량 25kg 초과", YELLOW, 4.4)
        and_mark = text("그리고", 24, CYAN, "BOLD")
        criterion_b = label_pill(
            "연료 제외 자체중량 150kg 이하", YELLOW, 4.8
        )
        criteria = VGroup(criterion_a, and_mark, criterion_b).arrange(
            RIGHT, buff=0.28
        ).move_to(first.get_center() + RIGHT * 0.85 + UP * 0.22)
        measured = text("두 기준을 각각 계측한 뒤 함께 확인", 25, WHITE, "BOLD")
        measured.move_to(first.get_center() + RIGHT * 0.85 + DOWN * 0.78)
        note = takeaway("종별 판단의 출발점 · 완성 기체의 실제 무게 계측", YELLOW)
        self.add(top_cards, first, first_label, criteria, measured, note)


def aircraft_tradeoff(
    icon: Mobject,
    name: str,
    strength: str,
    tradeoff: str,
    position,
    color: str,
) -> VGroup:
    panel = card(3.55, 3.55, color).move_to(position)
    name_label = text(name, 28, color, "BOLD").move_to(panel.get_center() + UP * 1.22)
    icon.move_to(panel.get_center() + UP * 0.25)
    icon.scale(0.58)
    strength_label = text(strength, 24, WHITE, "BOLD").move_to(
        panel.get_center() + DOWN * 0.72
    )
    tradeoff_label = text(tradeoff, 24, MUTED, "BOLD").move_to(
        panel.get_center() + DOWN * 1.27
    )
    return VGroup(panel, name_label, icon, strength_label, tradeoff_label)


class AircraftUamStatic(StaticDiagramScene):
    def construct(self) -> None:
        aircraft = VGroup(
            aircraft_tradeoff(
                fixed_wing_icon(0.75),
                "고정익",
                "강점 · 순항 효율",
                "교환 · 호버 어려움",
                LEFT * 5.75 + UP * 1.75,
                BLUE,
            ),
            aircraft_tradeoff(
                helicopter_icon(0.64),
                "헬리콥터",
                "강점 · 체공·탑재",
                "교환 · 복잡한 구조",
                LEFT * 1.92 + UP * 1.75,
                GREEN,
            ),
            aircraft_tradeoff(
                x_quadcopter_icon(0.72),
                "멀티콥터",
                "강점 · 정밀 호버",
                "한계 · 체공·탑재",
                RIGHT * 1.92 + UP * 1.75,
                CYAN,
            ),
            aircraft_tradeoff(
                evtol_icon(0.7),
                "eVTOL",
                "강점 · 이착륙+순항",
                "교환 · 시스템 복잡도",
                RIGHT * 5.75 + UP * 1.75,
                ORANGE,
            ),
        )

        vehicle = label_pill("eVTOL", ORANGE, 2.0).move_to(LEFT * 5.55 + DOWN * 1.35)
        systems = VGroup(
            label_pill("버티포트", BLUE, 2.35),
            label_pill("운항", GREEN, 2.0),
            label_pill("교통관리", CYAN, 2.45),
        ).arrange(RIGHT, buff=0.78).move_to(RIGHT * 2.25 + DOWN * 1.35)
        links = VGroup(
            Arrow(vehicle.get_right(), systems[0].get_left(), buff=0.15, color=ORANGE, stroke_width=5),
            Line(systems[0].get_right(), systems[1].get_left(), color=GRID, stroke_width=4),
            Line(systems[1].get_right(), systems[2].get_left(), color=GRID, stroke_width=4),
        )
        network_label = text("UAM 이동 체계", 27, WHITE, "BOLD").move_to(DOWN * 2.35)
        note = takeaway("UAM · 기체와 지상·운항·교통 체계의 연결", ORANGE)
        self.add(aircraft, links, vehicle, systems, network_label, note)


def mission_tile(
    name: str,
    requirement: str,
    position,
    color: str,
    motif: Mobject,
) -> VGroup:
    panel = card(7.25, 3.05, color).move_to(position)
    name_label = text(name, 29, color, "BOLD").move_to(
        panel.get_center() + LEFT * 2.55 + UP * 1.0
    )
    motif.scale(0.76).move_to(panel.get_center() + LEFT * 2.45 + DOWN * 0.38)
    connector = Arrow(
        panel.get_center() + LEFT * 1.15,
        panel.get_center() + RIGHT * 0.25,
        buff=0,
        color=color,
        stroke_width=6,
    )
    requirement_label = text(requirement, 29, WHITE, "BOLD").move_to(
        panel.get_center() + RIGHT * 1.95
    )
    return VGroup(panel, connector, name_label, motif, requirement_label)


class MissionSpecsStatic(StaticDiagramScene):
    def construct(self) -> None:
        show_drones = VGroup(
            x_quadcopter_icon(0.29),
            x_quadcopter_icon(0.29).shift(RIGHT * 0.9),
            x_quadcopter_icon(0.29).shift(LEFT * 0.9),
        )
        show_links = VGroup(
            Line(show_drones[0], show_drones[1], color=CYAN, stroke_width=3),
            Line(show_drones[0], show_drones[2], color=CYAN, stroke_width=3),
        )
        show = VGroup(show_links, show_drones)

        cargo_drone = x_quadcopter_icon(0.37)
        cargo = RoundedRectangle(
            width=0.9,
            height=0.55,
            corner_radius=0.08,
            fill_color=YELLOW,
            fill_opacity=1,
            stroke_width=0,
        ).next_to(cargo_drone, DOWN, buff=0.28)
        sling = Line(cargo_drone.get_bottom(), cargo.get_top(), color=MUTED, stroke_width=3)
        logistics = VGroup(cargo_drone, sling, cargo)

        watch_drone = x_quadcopter_icon(0.38)
        scan = Arc(
            radius=1.0,
            start_angle=-2.7,
            angle=2.25,
            color=GREEN,
            stroke_width=5,
        ).next_to(watch_drone, DOWN, buff=0.1)
        watch = VGroup(watch_drone, scan)

        recon_drone = x_quadcopter_icon(0.38)
        shield = Polygon(
            UP * 0.48,
            RIGHT * 0.42 + UP * 0.18,
            RIGHT * 0.3 + DOWN * 0.35,
            DOWN * 0.58,
            LEFT * 0.3 + DOWN * 0.35,
            LEFT * 0.42 + UP * 0.18,
            fill_color=ORANGE,
            fill_opacity=0.85,
            stroke_color=WHITE,
            stroke_width=2,
        ).next_to(recon_drone, DOWN, buff=0.16)
        recon = VGroup(recon_drone, shield)

        tiles = VGroup(
            mission_tile("공연", "동기화", LEFT * 3.82 + UP * 1.74, CYAN, show),
            mission_tile("물류", "탑재중량", RIGHT * 3.82 + UP * 1.74, YELLOW, logistics),
            mission_tile("안전 감시", "체공 · 통신", LEFT * 3.82 + DOWN * 1.58, GREEN, watch),
            mission_tile("국방 정찰", "보안 · 내환경성", RIGHT * 3.82 + DOWN * 1.58, ORANGE, recon),
        )
        note = takeaway("임무 정의 → 우선 요구 사양", BLUE)
        self.add(tiles, note)


def force_tile(
    name: str,
    relation: str,
    position,
    color: str,
    thrust_length: float,
    *,
    tilt: float = 0.0,
) -> VGroup:
    panel = card(7.25, 3.05, color).move_to(position)
    name_label = text(name, 29, color, "BOLD").move_to(
        panel.get_center() + LEFT * 2.55 + UP * 1.0
    )
    drone = side_quadcopter_icon(0.48, color).rotate(tilt).move_to(
        panel.get_center() + LEFT * 1.85 + DOWN * 0.55
    )
    force_origin = drone.get_center()
    weight_vector = DOWN * 0.78
    gravity = Arrow(
        force_origin,
        force_origin + weight_vector,
        buff=0,
        color=YELLOW,
        stroke_width=6,
    )
    direction = rotate_vector(UP, tilt)
    thrust = Arrow(
        force_origin,
        force_origin + direction * thrust_length,
        buff=0,
        color=color,
        stroke_width=7,
    )
    thrust_label = text("합추력 T", 24, color, "BOLD").move_to(
        force_origin + LEFT * 0.92 + UP * 0.72
    )
    weight_label = text("무게 W", 24, YELLOW, "BOLD").move_to(
        force_origin + RIGHT * 1.03 + DOWN * 0.56
    )
    relation_label = text(relation, 28, WHITE, "BOLD").move_to(
        panel.get_center() + RIGHT * 1.85
    )
    component = VGroup()
    component_label = VGroup()
    if tilt:
        thrust_tip = thrust.get_end()
        vertical_tip = [force_origin[0], thrust_tip[1], 0]
        component_guide = DashedLine(
            force_origin,
            vertical_tip,
            color=MUTED,
            stroke_width=3,
        )
        component = Arrow(
            vertical_tip,
            thrust_tip,
            buff=0,
            color=CYAN,
            stroke_width=6,
        )
        component_label = text("앞으로 미는 힘", 24, CYAN, "BOLD").move_to(
            panel.get_center() + RIGHT * 1.82 + DOWN * 0.85
        )
        component = VGroup(component_guide, component)
    return VGroup(
        panel,
        name_label,
        drone,
        gravity,
        thrust,
        thrust_label,
        weight_label,
        relation_label,
        component,
        component_label,
    )


def side_quadcopter_icon(scale: float = 1.0, color: str = CYAN) -> VGroup:
    """Readable side profile used for pitch/translation force diagrams."""

    arm = Line(LEFT * 1.2, RIGHT * 1.2, color=MUTED, stroke_width=9)
    body = RoundedRectangle(
        width=0.92,
        height=0.38,
        corner_radius=0.12,
        fill_color=color,
        fill_opacity=1,
        stroke_color=WHITE,
        stroke_width=2,
    )
    rotor_left = Ellipse(
        width=1.0,
        height=0.18,
        color=CYAN,
        stroke_width=5,
    ).move_to(LEFT * 1.18 + UP * 0.18)
    rotor_right = rotor_left.copy().move_to(RIGHT * 1.18 + UP * 0.18)
    struts = VGroup(
        Line(LEFT * 1.18, LEFT * 1.18 + UP * 0.18, color=WHITE, stroke_width=4),
        Line(RIGHT * 1.18, RIGHT * 1.18 + UP * 0.18, color=WHITE, stroke_width=4),
    )
    legs = VGroup(
        Line(LEFT * 0.34 + DOWN * 0.18, LEFT * 0.52 + DOWN * 0.48, color=GRID, stroke_width=4),
        Line(RIGHT * 0.34 + DOWN * 0.18, RIGHT * 0.52 + DOWN * 0.48, color=GRID, stroke_width=4),
        Line(LEFT * 0.67 + DOWN * 0.48, RIGHT * 0.67 + DOWN * 0.48, color=GRID, stroke_width=4),
    )
    return VGroup(arm, struts, rotor_left, rotor_right, body, legs).scale(scale)


class QuadcopterForceMotionStatic(StaticDiagramScene):
    def construct(self) -> None:
        states = VGroup(
            force_tile(
                "상승",
                "합추력 > 무게",
                LEFT * 3.82 + UP * 1.74,
                GREEN,
                1.0,
            ),
            force_tile(
                "호버링",
                "합추력 = 무게",
                RIGHT * 3.82 + UP * 1.74,
                CYAN,
                0.78,
            ),
            force_tile(
                "하강",
                "합추력 < 무게",
                LEFT * 3.82 + DOWN * 1.58,
                ORANGE,
                0.56,
            ),
            force_tile(
                "수평 이동",
                "기울여 전진",
                RIGHT * 3.82 + DOWN * 1.58,
                BLUE,
                0.78 / cos(18 * DEGREES),
                tilt=-18 * DEGREES,
            ),
        )
        viewpoint = text("측면도", 24, MUTED, "BOLD").move_to(UP * 3.78)
        note = takeaway("기체를 기울이면 합추력도 기울어 전진 성분이 생김", BLUE)
        self.add(states, viewpoint, note)


def helicopter_top_view() -> tuple[VGroup, CurvedArrow, Arrow]:
    rotor_center = UP * 0.72
    rotor_disk = Circle(radius=1.42, color=CYAN, stroke_width=4).move_to(rotor_center)
    rotor_blades = VGroup(
        Line(LEFT * 1.25, RIGHT * 1.25, color=WHITE, stroke_width=9),
        Line(DOWN * 1.25, UP * 1.25, color=WHITE, stroke_width=9),
    ).rotate(22 * DEGREES).move_to(rotor_center)
    rotor_hub = Circle(
        radius=0.16,
        fill_color=YELLOW,
        fill_opacity=1,
        stroke_color=WHITE,
        stroke_width=2,
    ).move_to(rotor_center)
    body = Ellipse(
        width=1.05,
        height=1.75,
        fill_color=BLUE,
        fill_opacity=1,
        stroke_color=WHITE,
        stroke_width=2,
    ).move_to(rotor_center)
    tail_end = rotor_center + DOWN * 2.55
    boom = Line(body.get_bottom(), tail_end, color=MUTED, stroke_width=10)
    tail_rotor = Circle(radius=0.32, color=CYAN, stroke_width=5).move_to(tail_end)
    rotor_spin = CurvedArrow(
        rotor_center + LEFT * 1.0 + UP * 0.72,
        rotor_center + RIGHT * 1.0 + UP * 0.72,
        angle=-PI,
        color=BLUE,
        stroke_width=5,
    )
    reaction = CurvedArrow(
        rotor_center + RIGHT * 0.78,
        rotor_center + UP * 0.78,
        angle=PI / 2,
        color=ORANGE,
        stroke_width=7,
    )
    tail_force = Arrow(
        tail_end,
        tail_end + LEFT * 1.28,
        buff=0.05,
        color=CYAN,
        stroke_width=7,
    )
    return VGroup(
        rotor_disk,
        rotor_blades,
        boom,
        tail_rotor,
        body,
        rotor_hub,
        rotor_spin,
    ), reaction, tail_force


class HelicopterQuadcopterTorqueStatic(StaticDiagramScene):
    def construct(self) -> None:
        left_panel = card(7.25, 6.55, BLUE).move_to(LEFT * 3.82 + UP * 0.28)
        right_panel = card(7.25, 6.55, CYAN).move_to(RIGHT * 3.82 + UP * 0.28)

        helicopter, reaction, tail_force = helicopter_top_view()
        helicopter_diagram = VGroup(helicopter, reaction, tail_force)
        helicopter_diagram.scale(0.72).move_to(left_panel.get_center() + UP * 0.28)
        heli_name = text("헬리콥터", 30, BLUE, "BOLD").move_to(
            left_panel.get_center() + UP * 2.7
        )
        viewpoint = text("위에서 본 모습", 24, MUTED, "BOLD").move_to(
            left_panel.get_center() + UP * 2.15
        )
        rotor_label = text("메인로터 회전", 24, BLUE, "BOLD").move_to(
            left_panel.get_center() + LEFT * 1.96 + UP * 1.57
        )
        reaction_label = text("동체 반작용 토크", 24, ORANGE, "BOLD").move_to(
            left_panel.get_center() + RIGHT * 1.45 + UP * 0.15
        )
        tail_label = text("꼬리로터 힘", 24, CYAN, "BOLD").move_to(
            left_panel.get_center() + RIGHT * 1.47 + DOWN * 1.2
        )
        heli_cancel = text("토크 상쇄", 28, WHITE, "BOLD").move_to(
            left_panel.get_center() + DOWN * 2.38
        )

        quad = x_quadcopter_icon(0.93, CYAN).move_to(
            right_panel.get_center() + UP * 0.1
        )
        quad_name = text("쿼드콥터", 30, CYAN, "BOLD").move_to(
            right_panel.get_center() + UP * 2.7
        )
        rotation_legend = text(
            "CW · CCW = 로터 회전 방향", 24, YELLOW, "BOLD"
        ).move_to(right_panel.get_center() + UP * 2.1)
        offsets = (
            (-1.75, 1.15, "M1", "CW", ORANGE),
            (1.75, 1.15, "M3", "CCW", CYAN),
            (-1.75, -0.85, "M4", "CCW", CYAN),
            (1.75, -0.85, "M2", "CW", ORANGE),
        )
        rotor_labels = VGroup()
        for x, y, motor_name, rotation, color in offsets:
            motor_label = text(motor_name, 24, WHITE, "BOLD").move_to(
                right_panel.get_center() + RIGHT * x + UP * y
            )
            rotation_label = text(rotation, 24, color, "BOLD").next_to(
                motor_label, UP if y > 0 else DOWN, buff=0.1
            )
            rotor_labels.add(motor_label, rotation_label)
        cancel = text("평상시 토크 상쇄", 27, WHITE, "BOLD").move_to(
            right_panel.get_center() + DOWN * 1.75
        )
        yaw_pair = VGroup(
            text("M3·M4 출력 ↑", 24, CYAN, "BOLD"),
            text("M1·M2 출력 ↓", 24, ORANGE, "BOLD"),
        ).arrange(RIGHT, buff=0.42).move_to(
            right_panel.get_center() + DOWN * 2.25
        )
        yaw = text("기체 Yaw CW", 25, YELLOW, "BOLD").move_to(
            right_panel.get_center() + DOWN * 2.78
        )
        note = takeaway("헬기 · 꼬리 힘 / 쿼드 · CW·CCW 반작용 토크", YELLOW)
        self.add(
            left_panel,
            right_panel,
            helicopter_diagram,
            heli_name,
            viewpoint,
            rotor_label,
            reaction_label,
            tail_label,
            heli_cancel,
            quad,
            quad_name,
            rotation_legend,
            rotor_labels,
            cancel,
            yaw_pair,
            yaw,
            note,
        )


class SwarmSystemStatic(StaticDiagramScene):
    def construct(self) -> None:
        status = boundary_badge("후속 목표 · 미구현 · 미검증", YELLOW)
        status.move_to(RIGHT * 4.28 + UP * 3.75)
        left_panel = card(4.2, 6.2, BLUE).move_to(LEFT * 5.55 + UP * 0.16)
        right_panel = card(10.15, 6.2, CYAN).move_to(RIGHT * 2.3 + UP * 0.16)

        single_label = text("단일 기체", 30, BLUE, "BOLD").move_to(
            left_panel.get_center() + UP * 2.35
        )
        single = x_quadcopter_icon(0.72).move_to(left_panel.get_center() + UP * 0.55)
        single_boundary = text("자세·안전 검증 선행", 25, WHITE, "BOLD").move_to(
            left_panel.get_center() + DOWN * 1.75
        )
        expand = Arrow(
            left_panel.get_right() + RIGHT * 0.08,
            right_panel.get_left() + LEFT * 0.08,
            buff=0.05,
            color=YELLOW,
            stroke_width=7,
        )

        swarm_label = text("군집 체계", 30, CYAN, "BOLD").move_to(
            right_panel.get_center() + LEFT * 3.65 + UP * 2.35
        )
        fleet_positions = (
            right_panel.get_center() + LEFT * 2.2 + UP * 0.75,
            right_panel.get_center() + UP * 0.75,
            right_panel.get_center() + RIGHT * 2.2 + UP * 0.75,
            right_panel.get_center() + LEFT * 1.1 + DOWN * 0.75,
            right_panel.get_center() + RIGHT * 1.1 + DOWN * 0.75,
        )
        fleet = VGroup(*[x_quadcopter_icon(0.29).move_to(point) for point in fleet_positions])
        links = VGroup(
            *[
                Line(fleet[a], fleet[b], color=GRID, stroke_width=3)
                for a, b in ((0, 1), (1, 2), (0, 3), (1, 3), (1, 4), (2, 4), (3, 4))
            ]
        )
        requirement_top = VGroup(
            label_pill("공통 좌표", BLUE, 2.2),
            label_pill("통신", CYAN, 1.65),
            label_pill("상대 위치", GREEN, 2.15),
        ).arrange(RIGHT, buff=0.26)
        requirement_bottom = VGroup(
            label_pill("경로 · 충돌 회피", YELLOW, 3.15),
            label_pill("집단 안전", ORANGE, 2.1),
        ).arrange(RIGHT, buff=0.26)
        requirements = VGroup(requirement_top, requirement_bottom).arrange(
            DOWN, buff=0.18
        ).move_to(right_panel.get_center() + DOWN * 2.08)
        note = takeaway("단일 기체 검증 + 좌표·통신·회피·집단 안전", YELLOW)
        self.add(
            status,
            left_panel,
            right_panel,
            single_label,
            single,
            single_boundary,
            expand,
            swarm_label,
            links,
            fleet,
            requirements,
            note,
        )


class AttitudeCorrectionStatic(StaticDiagramScene):
    def construct(self) -> None:
        stage = card(15.1, 6.2, BLUE).move_to(UP * 0.2)
        xs = (-6.0, -3.0, 0.0, 3.0, 6.0)
        labels = (
            ("외란", YELLOW),
            ("센서 관측", CYAN),
            ("모터 출력 차이", GREEN),
            ("복원 토크", GREEN),
            ("수평 복원", BLUE),
        )
        stage_labels = VGroup(
            *[
                text(label, 25, color, "BOLD").move_to([x, 2.55, 0])
                for x, (label, color) in zip(xs, labels)
            ]
        )
        flow_arrows = VGroup(
            *[
                Arrow(
                    [xs[index] + 1.02, 0.55, 0],
                    [xs[index + 1] - 1.02, 0.55, 0],
                    buff=0,
                    color=GRID,
                    stroke_width=5,
                )
                for index in range(4)
            ]
        )

        tilted = front_view_drone(0.55).rotate(16 * DEGREES).move_to(
            [xs[0], 0.55, 0]
        )
        wind = Arrow(
            [xs[0] - 1.15, 1.55, 0],
            [xs[0] + 0.15, 1.55, 0],
            buff=0,
            color=YELLOW,
            stroke_width=7,
        )
        sensor = VGroup(
            Circle(
                radius=0.78,
                fill_color=PANEL_2,
                fill_opacity=1,
                stroke_color=CYAN,
                stroke_width=4,
            ),
            Line(LEFT * 0.48, RIGHT * 0.48, color=WHITE, stroke_width=4),
            Line(DOWN * 0.48, UP * 0.48, color=WHITE, stroke_width=4),
            Arc(radius=0.45, start_angle=0.1, angle=1.5, color=CYAN, stroke_width=5),
        ).move_to([xs[1], 0.55, 0])
        sensor_note = text("기울기 오차", 24, WHITE, "BOLD").move_to(
            [xs[1], -0.7, 0]
        )

        motor_base = Line(
            [xs[2] - 0.95, -0.05, 0],
            [xs[2] + 0.95, -0.05, 0],
            color=MUTED,
            stroke_width=9,
        )
        left_output = Arrow(
            [xs[2] - 0.75, -0.05, 0],
            [xs[2] - 0.75, 1.65, 0],
            buff=0.05,
            color=GREEN,
            stroke_width=8,
        )
        right_output = Arrow(
            [xs[2] + 0.75, -0.05, 0],
            [xs[2] + 0.75, 1.05, 0],
            buff=0.05,
            color=BLUE,
            stroke_width=6,
        )
        difference = text("좌·우 추력 차", 24, WHITE, "BOLD").move_to(
            [xs[2], -0.7, 0]
        )

        torque_drone = front_view_drone(0.52).move_to([xs[3], 0.5, 0])
        torque_arrow = CurvedArrow(
            [xs[3] + 0.9, 1.38, 0],
            [xs[3] + 1.0, -0.25, 0],
            angle=-1.35,
            color=GREEN,
            stroke_width=7,
        )
        opposite = text("오차 반대 방향", 24, WHITE, "BOLD").move_to(
            [xs[3], -0.72, 0]
        )

        level = front_view_drone(0.55).move_to([xs[4], 0.55, 0])
        horizon = Line(
            [xs[4] - 1.15, -0.25, 0],
            [xs[4] + 1.15, -0.25, 0],
            color=GRID,
            stroke_width=4,
        )
        feedback_path = VGroup(
            Line([xs[4], -1.15, 0], [xs[4], -1.75, 0], color=CYAN, stroke_width=4),
            Line([xs[4], -1.75, 0], [xs[1], -1.75, 0], color=CYAN, stroke_width=4),
            Arrow([xs[1], -1.75, 0], [xs[1], -0.75, 0], buff=0.05, color=CYAN, stroke_width=4),
        )
        feedback_label = text("반복 피드백", 24, CYAN, "BOLD").move_to(
            [1.5, -2.15, 0]
        )
        note = takeaway("관측 → 출력 차이 → 복원 토크의 닫힌 고리", GREEN)
        self.add(
            stage,
            stage_labels,
            flow_arrows,
            tilted,
            wind,
            sensor,
            sensor_note,
            motor_base,
            left_output,
            right_output,
            difference,
            torque_drone,
            torque_arrow,
            opposite,
            level,
            horizon,
            feedback_path,
            feedback_label,
            note,
        )


class SilClosedLoopStatic(StaticDiagramScene):
    def construct(self) -> None:
        stage = card(15.1, 5.95, BLUE).move_to(UP * 0.12)
        labels = ("가상 물리", "센서 합성", "실제 비행 코드", "모터 출력")
        colors = (BLUE, CYAN, GREEN, YELLOW)
        xs = (-5.75, -1.95, 1.95, 5.75)
        blocks = VGroup(
            *[
                process_node(label, color, 2.85).move_to([x, 0.75, 0])
                for label, color, x in zip(labels, colors, xs)
            ]
        )
        forward = VGroup(
            *[
                Arrow(
                    blocks[index].get_right(),
                    blocks[index + 1].get_left(),
                    buff=0.12,
                    color=BLUE,
                    stroke_width=6,
                )
                for index in range(3)
            ]
        )
        sketch = label_pill("실제 스케치 포함", GREEN, 2.85).next_to(
            blocks[2], DOWN, buff=0.28
        )
        return_path = VGroup(
            Line(blocks[3].get_bottom(), [xs[3], -1.6, 0], color=CYAN, stroke_width=5),
            Line([xs[3], -1.6, 0], [xs[0], -1.6, 0], color=CYAN, stroke_width=5),
            Arrow([xs[0], -1.6, 0], blocks[0].get_bottom(), buff=0.12, color=CYAN, stroke_width=5),
        )
        return_label = text("출력으로 다음 물리 상태 갱신", 25, CYAN, "BOLD")
        return_label.move_to(DOWN * 2.08)
        evidence = VGroup(
            label_pill("코드 폐루프 반응 확인", GREEN, 3.45),
            label_pill("실물 센서·모터·비행 미확인", YELLOW, 4.15),
        ).arrange(RIGHT, buff=0.62).move_to(DOWN * 2.7)
        note = takeaway("확인 범위 · host 안의 코드와 가상 기체 폐루프", YELLOW)
        self.add(stage, blocks, forward, sketch, return_path, return_label, evidence, note)


class FailsafeTimelineStatic(StaticDiagramScene):
    def construct(self) -> None:
        stage = card(15.1, 6.05, BLUE).move_to(UP * 0.08)
        trigger = process_node("상태 판단", BLUE, 2.35).move_to(LEFT * 6.0 + UP * 0.2)

        rc_loss = process_node("RC 신호 두절", YELLOW, 2.65)
        descent = process_node("자세 유지 · 제한 하강", BLUE, 3.25)
        limit = process_node("설정된 상한 · 모터 정지", GREEN, 3.55)
        VGroup(rc_loss, descent, limit).arrange(RIGHT, buff=0.4).move_to(
            RIGHT * 1.3 + UP * 1.42
        )
        upper_links = VGroup(
            Arrow(trigger.get_right(), rc_loss.get_left(), buff=0.04, color=YELLOW, stroke_width=6),
            Arrow(rc_loss.get_right(), descent.get_left(), buff=0.01, color=BLUE, stroke_width=6),
            Arrow(descent.get_right(), limit.get_left(), buff=0.01, color=GREEN, stroke_width=6),
        )
        upper_label = text("자세 유지 가능", 24, BLUE, "BOLD").move_to(
            LEFT * 2.9 + UP * 2.45
        )

        critical = process_node("치명적 고장", RED, 2.7).move_to(
            LEFT * 1.9 + DOWN * 1.35
        )
        immediate = process_node("즉시 모터 정지", RED, 3.0).move_to(
            RIGHT * 2.3 + DOWN * 1.35
        )
        lower_links = VGroup(
            Arrow(trigger.get_right(), critical.get_left(), buff=0.12, color=RED, stroke_width=6),
            Arrow(critical.get_right(), immediate.get_left(), buff=0.12, color=RED, stroke_width=7),
        )
        lower_label = text("자세 유지 불가 · 즉시 중단 조건", 24, RED, "BOLD").move_to(
            LEFT * 0.25 + DOWN * 2.35
        )
        note = takeaway("자세 유지 가능 여부에 따른 제한 하강 / 즉시 정지 분기", YELLOW)
        self.add(
            stage,
            trigger,
            upper_links,
            rc_loss,
            descent,
            limit,
            upper_label,
            lower_links,
            critical,
            immediate,
            lower_label,
            note,
        )


def imu_observation(center) -> VGroup:
    baseline = Line(LEFT * 1.85, RIGHT * 1.85, color=GRID, stroke_width=3)
    trace = VMobject(color=GREEN, stroke_width=6).set_points_as_corners(
        [
            LEFT * 1.72,
            LEFT * 1.05 + UP * 0.04,
            LEFT * 0.35 + DOWN * 0.03,
            RIGHT * 0.35 + UP * 0.03,
            RIGHT * 1.05 + DOWN * 0.02,
            RIGHT * 1.72,
        ]
    )
    label = text("IMU", 24, GREEN, "BOLD").next_to(baseline, LEFT, buff=0.18)
    return VGroup(baseline, trace, label).move_to(center)


class LandingObservabilityStatic(StaticDiagramScene):
    def construct(self) -> None:
        same = boundary_badge("IMU 관측 동일", GREEN).move_to(UP * 3.72)
        left_panel = card(7.25, 6.2, GREEN).move_to(LEFT * 3.82 + UP * 0.12)
        right_panel = card(7.25, 6.2, BLUE).move_to(RIGHT * 3.82 + UP * 0.12)
        left_label = text("지면 정지", 30, GREEN, "BOLD").move_to(
            left_panel.get_center() + UP * 2.52
        )
        right_label = text("등속 하강", 30, BLUE, "BOLD").move_to(
            right_panel.get_center() + UP * 2.52
        )
        grounded = front_view_drone(0.62).move_to(
            left_panel.get_center() + UP * 1.1
        )
        descending = front_view_drone(0.62).move_to(
            right_panel.get_center() + UP * 1.1
        )
        ground_left = Line(
            grounded.get_left() + LEFT * 0.3 + DOWN * 0.72,
            grounded.get_right() + RIGHT * 0.3 + DOWN * 0.72,
            color=MUTED,
            stroke_width=7,
        )
        ground_right = Line(
            descending.get_left() + LEFT * 0.3 + DOWN * 1.28,
            descending.get_right() + RIGHT * 0.3 + DOWN * 1.28,
            color=MUTED,
            stroke_width=7,
        )
        down = Arrow(
            descending.get_right() + RIGHT * 0.35 + UP * 0.45,
            descending.get_right() + RIGHT * 0.35 + DOWN * 0.62,
            buff=0,
            color=BLUE,
            stroke_width=7,
        )
        left_imu = imu_observation(left_panel.get_center() + DOWN * 0.62)
        right_imu = imu_observation(right_panel.get_center() + DOWN * 0.62)

        left_range = Arrow(
            grounded.get_center() + DOWN * 0.18,
            ground_left.get_center() + UP * 0.05,
            buff=0.18,
            color=YELLOW,
            stroke_width=5,
        )
        right_range = Arrow(
            descending.get_center() + DOWN * 0.18,
            ground_right.get_center() + UP * 0.05,
            buff=0.18,
            color=YELLOW,
            stroke_width=5,
        )
        left_range_label = text("거리 일정", 24, YELLOW, "BOLD").move_to(
            left_panel.get_center() + DOWN * 2.25
        )
        right_range_label = text("거리 감소", 24, YELLOW, "BOLD").move_to(
            right_panel.get_center() + DOWN * 2.25
        )
        distance = label_pill("거리 센서 추가 단서", YELLOW, 3.25).move_to(
            DOWN * 2.83
        )
        note = takeaway("폐루프 착지 판정 미검증", YELLOW)
        self.add(
            same,
            left_panel,
            right_panel,
            left_label,
            right_label,
            grounded,
            descending,
            ground_left,
            ground_right,
            down,
            left_imu,
            right_imu,
            left_range,
            right_range,
            left_range_label,
            right_range_label,
            distance,
            note,
        )


class LandingProbeEvidenceStatic(StaticDiagramScene):
    """Show why the probe response did not separate airborne and landed states."""

    def construct(self) -> None:
        left_panel = card(7.25, 5.25, BLUE).move_to(LEFT * 3.82 + UP * 0.45)
        right_panel = card(7.25, 5.25, CYAN).move_to(RIGHT * 3.82 + UP * 0.45)

        air_name = text("공중 프로브 2회", 29, BLUE, "BOLD").move_to(
            left_panel.get_center() + UP * 1.85
        )
        air_value = text("0.061g · 0.097g", 31, WHITE, "BOLD").move_to(
            left_panel.get_center() + UP * 0.8
        )
        air_bar = Line(
            left_panel.get_center() + LEFT * 2.0 + DOWN * 0.15,
            left_panel.get_center() + RIGHT * 0.25 + DOWN * 0.15,
            color=BLUE,
            stroke_width=14,
        )
        air_note = text("접지 전 기록", 24, MUTED, "BOLD").move_to(
            left_panel.get_center() + DOWN * 1.25
        )

        ground_name = text("접지 프로브 10회", 29, CYAN, "BOLD").move_to(
            right_panel.get_center() + UP * 1.85
        )
        ground_value = text("0.059~1.147g", 31, WHITE, "BOLD").move_to(
            right_panel.get_center() + UP * 0.8
        )
        ground_bar = Line(
            right_panel.get_center() + LEFT * 2.15 + DOWN * 0.15,
            right_panel.get_center() + RIGHT * 2.15 + DOWN * 0.15,
            color=CYAN,
            stroke_width=14,
        )
        ground_note = text("착지 확정 0회", 24, YELLOW, "BOLD").move_to(
            right_panel.get_center() + DOWN * 1.25
        )

        overlap = label_pill("값이 겹침", ORANGE, 2.35).move_to(UP * 0.3)
        premise = text("지면이면 무반응 전제 실패", 28, YELLOW, "BOLD").move_to(
            DOWN * 2.35
        )
        boundary = VGroup(
            label_pill("로깅 전용", BLUE, 2.25),
            label_pill("착지 판정 제외", YELLOW, 2.8),
        ).arrange(RIGHT, buff=0.5).move_to(DOWN * 2.95)
        note = takeaway("지면에서도 응답이 남아 하나의 임계값으로 분리할 수 없었음", YELLOW)
        self.add(
            left_panel,
            right_panel,
            air_name,
            air_value,
            air_bar,
            air_note,
            ground_name,
            ground_value,
            ground_bar,
            ground_note,
            overlap,
            premise,
            boundary,
            note,
        )


class SharedStateRaceStatic(StaticDiagramScene):
    def construct(self) -> None:
        stage = card(15.1, 6.05, BLUE).move_to(UP * 0.08)
        time_axis = Arrow(
            LEFT * 4.8 + UP * 2.45,
            RIGHT * 6.5 + UP * 2.45,
            buff=0,
            color=GRID,
            stroke_width=5,
        )
        time_label = text("실행 순서", 24, MUTED, "BOLD").move_to(
            LEFT * 6.2 + UP * 2.45
        )
        lane_divider = Line(
            LEFT * 5.15 + UP * 0.08,
            RIGHT * 6.7 + UP * 0.08,
            color=GRID,
            stroke_width=3,
        )
        communication = text("통신 코어", 27, BLUE, "BOLD").move_to(
            LEFT * 6.35 + UP * 1.25
        )
        control = text("제어 태스크", 27, GREEN, "BOLD").move_to(
            LEFT * 6.35 + DOWN * 1.05
        )

        comm_read = process_node("A 읽기", BLUE, 2.15).move_to(
            LEFT * 3.8 + UP * 1.25
        )
        comm_pending = DashedLine(
            comm_read.get_right(),
            [3.2, 1.25, 0],
            color=BLUE,
            stroke_width=4,
        )
        comm_write = process_node("옛 A 기반 C 쓰기", RED, 3.35).move_to(
            RIGHT * 4.95 + UP * 1.25
        )
        comm_finish = Arrow(
            [3.2, 1.25, 0],
            comm_write.get_left(),
            buff=0.08,
            color=RED,
            stroke_width=6,
        )

        control_read = process_node("A 읽기", GREEN, 2.15).move_to(
            LEFT * 1.8 + DOWN * 1.05
        )
        control_write = process_node("B 쓰기", GREEN, 2.15).move_to(
            RIGHT * 1.15 + DOWN * 1.05
        )
        control_flow = Arrow(
            control_read.get_right(),
            control_write.get_left(),
            buff=0.12,
            color=GREEN,
            stroke_width=6,
        )

        shared_a = label_pill("공유 상태 A", YELLOW, 2.55).move_to(
            LEFT * 4.4 + DOWN * 2.35
        )
        shared_b = label_pill("B 반영", GREEN, 2.05).move_to(
            RIGHT * 1.15 + DOWN * 2.35
        )
        shared_c = label_pill("C로 덮임", RED, 2.25).move_to(
            RIGHT * 4.95 + DOWN * 2.35
        )
        shared_track = VGroup(
            Arrow(shared_a.get_right(), shared_b.get_left(), buff=0.12, color=GREEN, stroke_width=5),
            Arrow(shared_b.get_right(), shared_c.get_left(), buff=0.12, color=RED, stroke_width=5),
        )
        loss = text("B 갱신 손실 가능", 25, RED, "BOLD").move_to(
            RIGHT * 3.05 + DOWN * 1.88
        )
        note = takeaway("겹친 읽기–수정–쓰기 → 최신 상태 손실 가능", YELLOW)
        self.add(
            stage,
            time_axis,
            time_label,
            lane_divider,
            communication,
            control,
            comm_read,
            comm_pending,
            comm_finish,
            comm_write,
            control_read,
            control_flow,
            control_write,
            shared_a,
            shared_track,
            shared_b,
            shared_c,
            loss,
            note,
        )


class TelemetryMotorBalanceStatic(StaticDiagramScene):
    def construct(self) -> None:
        left_panel = card(6.75, 6.15, BLUE).move_to(LEFT * 4.15 + UP * 0.12)
        right_panel = card(7.7, 6.15, CYAN).move_to(RIGHT * 3.55 + UP * 0.12)
        layout_label = text("X형 모터 배치", 29, BLUE, "BOLD").move_to(
            left_panel.get_center() + UP * 2.55
        )
        airframe, motors = motor_layout()
        airframe.move_to(left_panel.get_center() + UP * 0.05)
        tether = Line(
            motors["M3"].get_top(),
            motors["M3"].get_top() + RIGHT * 0.72 + UP * 1.0,
            color=YELLOW,
            stroke_width=6,
        )
        tether_label = text("M3 근처 테더", 24, YELLOW, "BOLD").move_to(
            left_panel.get_center() + RIGHT * 2.15 + UP * 0.53
        )
        front = text("전방", 24, YELLOW, "BOLD").move_to(
            left_panel.get_center() + UP * 2.0
        )

        direction = text("M3 평균 > M1 평균", 31, WHITE, "BOLD").move_to(
            right_panel.get_center() + UP * 2.45
        )
        baseline_y = right_panel.get_center()[1] - 0.95
        baseline = Line(
            [1.7, baseline_y, 0],
            [5.55, baseline_y, 0],
            color=GRID,
            stroke_width=4,
        )
        m1_bar = Rectangle(
            width=1.2,
            height=1.55,
            fill_color=YELLOW,
            fill_opacity=0.85,
            stroke_color=YELLOW,
            stroke_width=3,
        ).move_to([2.55, baseline_y + 0.775, 0])
        m3_bar = Rectangle(
            width=1.2,
            height=2.2,
            fill_color=BLUE,
            fill_opacity=0.9,
            stroke_color=BLUE,
            stroke_width=3,
        ).move_to([4.65, baseline_y + 1.1, 0])
        m1_label = text("M1 · 1334.2µs", 24, YELLOW, "BOLD").next_to(
            m1_bar, DOWN, buff=0.18
        ).shift(LEFT * 0.42)
        m3_label = text("M3 · 1360.0µs", 24, BLUE, "BOLD").next_to(
            m3_bar, DOWN, buff=0.18
        ).shift(RIGHT * 0.42)
        qualitative = text("차이 약 25.8µs", 24, MUTED, "BOLD")
        qualitative.move_to(right_panel.get_center() + UP * 1.65)
        tether_hypothesis = label_pill("M3 근처 테더", YELLOW, 2.55).move_to(
            right_panel.get_center() + LEFT * 2.4 + DOWN * 2.1
        )
        load_hypothesis = label_pill("줄 무게까지 함께 지지", BLUE, 3.85).move_to(
            right_panel.get_center() + RIGHT * 1.35 + DOWN * 2.1
        )
        inference_arrow = Arrow(
            tether_hypothesis.get_right(),
            load_hypothesis.get_left(),
            buff=0.1,
            color=YELLOW,
            stroke_width=5,
        )
        note = takeaway("관측과 현장 조건 해석 · M3가 테더 줄 무게까지 함께 지지", BLUE)
        self.add(
            left_panel,
            right_panel,
            layout_label,
            airframe,
            tether,
            tether_label,
            front,
            direction,
            baseline,
            m1_bar,
            m3_bar,
            m1_label,
            m3_label,
            qualitative,
            tether_hypothesis,
            inference_arrow,
            load_hypothesis,
            note,
        )


class ProductionEstimateStatic(StaticDiagramScene):
    """Compare one-unit and ten-unit planning ranges in one static frame."""

    DATA_PATH = Path(__file__).resolve().parents[1] / "production_estimate.json"

    @staticmethod
    def _range(values, suffix: str) -> str:
        return f"{values[0]:g}~{values[1]:g}{suffix}"

    @staticmethod
    def _money_range(values, scale: int, decimals: int = 1) -> str:
        quantum = Decimal("1") if decimals == 0 else Decimal("0.1")
        low = (Decimal(values[0]) / Decimal(scale)).quantize(
            quantum, rounding=ROUND_HALF_UP
        )
        high = (Decimal(values[1]) / Decimal(scale)).quantize(
            quantum, rounding=ROUND_HALF_UP
        )
        return f"약 {low:.{decimals}f}~{high:.{decimals}f}만원"

    @staticmethod
    def _single_money(values) -> str:
        low = (Decimal(values[0]) / Decimal(10000)).quantize(
            Decimal("0.1"), rounding=ROUND_HALF_UP
        )
        high = (Decimal(values[1]) / Decimal(10000)).quantize(
            Decimal("0.1"), rounding=ROUND_HALF_UP
        )
        if values[0] == values[1]:
            return f"약 {low:.1f}만원"
        return f"{low:.1f}~{high:.1f}만원"

    @staticmethod
    def _cost_item(
        name: str,
        value: str,
        color: str,
        evidence: str | None = None,
    ) -> VGroup:
        panel = RoundedRectangle(
            width=2.68,
            height=1.2,
            corner_radius=0.13,
            fill_color=PANEL_2,
            fill_opacity=1,
            stroke_color=color,
            stroke_width=2,
        )
        name_label = text(name, 24, color, "BOLD").move_to(
            panel.get_center() + UP * (0.36 if evidence else 0.23)
        )
        value_label = text(value, 24, WHITE, "BOLD").move_to(
            panel.get_center() + (ORIGIN if evidence else DOWN * 0.23)
        )
        evidence_label = VGroup()
        if evidence:
            evidence_label = text(evidence, 24, YELLOW, "BOLD").move_to(
                panel.get_center() + DOWN * 0.38
            )
        return VGroup(panel, name_label, value_label, evidence_label)

    def _time_panel(
        self,
        center,
        title: str,
        printer_label: str,
        hands_on_label: str,
        elapsed_label: str,
        drone_count: int,
    ) -> VGroup:
        panel = card(7.25, 3.35, BLUE if drone_count == 1 else CYAN).move_to(center)
        heading = text(title, 31, BLUE if drone_count == 1 else CYAN, "BOLD")
        heading.move_to(panel.get_center() + LEFT * 2.55 + UP * 1.12)
        icons = VGroup(
            drone_icon(0.27, CYAN).move_to(
                panel.get_center() + LEFT * 2.82 + UP * 0.35
            )
        )
        multiplier = VGroup()
        if drone_count > 1:
            multiplier = text(f"×{drone_count}", 27, WHITE, "BOLD").move_to(
                panel.get_center() + LEFT * 2.25 + UP * 0.35
            )

        printer_name = text("프린터 점유", 24, BLUE, "BOLD")
        printer_name.move_to(panel.get_center() + LEFT * 0.68 + UP * 0.72)
        printer_value = label_pill(printer_label, BLUE, 2.35)
        printer_value.move_to(panel.get_center() + RIGHT * 2.15 + UP * 0.72)
        hands_name = text("직접 작업", 24, GREEN, "BOLD")
        hands_name.move_to(panel.get_center() + LEFT * 0.78 + DOWN * 0.12)
        hands_value = label_pill(hands_on_label, GREEN, 2.35)
        hands_value.move_to(panel.get_center() + RIGHT * 2.15 + DOWN * 0.12)
        elapsed = text(elapsed_label, 24, WHITE, "BOLD")
        elapsed.move_to(panel.get_center() + RIGHT * 0.78 + DOWN * 1.03)
        return VGroup(
            panel,
            heading,
            icons,
            multiplier,
            printer_name,
            printer_value,
            hands_name,
            hands_value,
            elapsed,
        )

    def construct(self) -> None:
        data = json.loads(self.DATA_PATH.read_text(encoding="utf-8"))
        one_time = data["time"]["one_unit"]
        ten_time = data["time"]["ten_units"]
        cost = data["cost"]

        one_panel = self._time_panel(
            LEFT * 3.82 + UP * 1.55,
            "1대",
            self._range(one_time["printer_hours"], "시간"),
            self._range(one_time["hands_on_hours"], "시간"),
            "프린터 1대 · 약 2~3일",
            1,
        )
        ten_panel = self._time_panel(
            RIGHT * 3.82 + UP * 1.55,
            "10대",
            self._range(ten_time["printer_hours_one_printer"], "시간"),
            self._range(ten_time["hands_on_hours"], "시간"),
            "프린터 1대 · 약 10~20일",
            10,
        )

        cost_panel = card(15.1, 3.0, ORANGE).move_to(DOWN * 1.55)
        cost_heading = text("부품비", 28, ORANGE, "BOLD")
        cost_heading.move_to(cost_panel.get_center() + LEFT * 5.7 + UP * 1.05)
        one_cost = text(
            "1대 " + self._money_range(cost["one_unit_core_subtotal_krw"], 10000),
            25,
            WHITE,
            "BOLD",
        ).move_to(cost_panel.get_center() + LEFT * 1.7 + UP * 0.88)
        ten_cost = text(
            "10대 "
            + self._money_range(
                cost["ten_units_core_subtotal_krw"], 10000, decimals=0
            ),
            25,
            WHITE,
            "BOLD",
        ).move_to(cost_panel.get_center() + RIGHT * 2.55 + UP * 0.88)

        category_specs = (
            ("모터 4개", "motors_4", BLUE),
            ("ESC 4개", "escs_4", CYAN),
            ("MCU·센서 IC", "control_sensor_chips", GREEN),
            ("프레임 재료", "frame_material", ORANGE),
            ("배터리·프로펠러", "battery_propellers", YELLOW),
        )
        categories = VGroup(
            *[
                self._cost_item(
                    name,
                    self._single_money(cost["one_unit_categories_krw"][key]),
                    color,
                )
                for name, key, color in category_specs
            ]
        ).arrange(RIGHT, buff=0.14)
        categories.move_to(cost_panel.get_center() + RIGHT * 0.25 + UP * 0.02)
        optional = label_pill("3901-L0X 추가 · +4.8~4.9만원/대", CYAN, 5.15)
        optional.move_to(cost_panel.get_center() + RIGHT * 4.25 + DOWN * 1.12)
        self.add(
            one_panel,
            ten_panel,
            cost_panel,
            cost_heading,
            one_cost,
            ten_cost,
            categories,
            optional,
        )
