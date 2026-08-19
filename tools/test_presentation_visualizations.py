#!/usr/bin/env python3
"""Delivery checks for the audience-facing presentation visualizations."""

from __future__ import annotations

import json
import importlib.util
import re
import shutil
import subprocess
import sys
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
ASSET_DIR = REPO_ROOT / "docs" / "presentations" / "ai-startup-camp-drone" / "assets"
GEOMETRY_PATH = (
    REPO_ROOT
    / "docs"
    / "presentations"
    / "ai-startup-camp-drone"
    / "visualizations"
    / "geometry.py"
)
VISUALIZATION_SOURCE_PATH = GEOMETRY_PATH.with_name("audience_visualizations.py")
SIGNIFICANCE_SOURCE_PATH = GEOMETRY_PATH.with_name("significance_visualizations.py")
ENGINEERING_SOURCE_PATH = GEOMETRY_PATH.with_name("engineering_visualizations.py")
STATIC_SOURCE_PATH = GEOMETRY_PATH.with_name("static_diagram_visualizations.py")
MOTOR_CHART_SOURCE_PATH = GEOMETRY_PATH.with_name("render_chart_motor.py")
PRODUCTION_ESTIMATE_PATH = (
    REPO_ROOT
    / "docs"
    / "presentations"
    / "ai-startup-camp-drone"
    / "production_estimate.json"
)
PRODUCTION_ESTIMATE_DOC_PATH = PRODUCTION_ESTIMATE_PATH.with_name(
    "PRODUCTION_ESTIMATE.md"
)
VISUALIZATION_FILES = (
    "accelerometer.mp4",
    "gyro.mp4",
    "complementary-filter.mp4",
    "gyro-bias.mp4",
    "imu-axis-signs.mp4",
    "pi-error-correction.mp4",
    "cascade-loop-timing.mp4",
    "yaw-correction.mp4",
    "landing-ambiguity.mp4",
)
SIGNIFICANCE_SCENES = {
    "DroneClassificationAudience": "drone-classification.mp4",
    "QualificationWeightAudience": "qualification-weight.mp4",
    "AircraftUamAudience": "aircraft-uam.mp4",
    "MissionSpecsAudience": "mission-specs.mp4",
    "QuadcopterForceMotionAudience": "quadcopter-force-motion.mp4",
    "HelicopterQuadcopterTorqueAudience": "helicopter-quadcopter-torque.mp4",
    "SwarmSystemAudience": "swarm-system.mp4",
}
ENGINEERING_SCENES = {
    "AttitudeCorrectionAudience": "attitude-correction.mp4",
    "SilClosedLoopAudience": "sil-closed-loop.mp4",
    "FailsafeTimelineAudience": "failsafe-timeline.mp4",
    "LandingObservabilityAudience": "landing-observability.mp4",
    "SharedStateRaceAudience": "shared-state-race.mp4",
    "TelemetryMotorBalanceAudience": "telemetry-motor-balance.mp4",
}
STATIC_SCENES = {
    "DroneClassificationStatic": "drone-classification.png",
    "QualificationWeightStatic": "qualification-weight.png",
    "AircraftUamStatic": "aircraft-uam.png",
    "MissionSpecsStatic": "mission-specs.png",
    "QuadcopterForceMotionStatic": "quadcopter-force-motion.png",
    "HelicopterQuadcopterTorqueStatic": "helicopter-quadcopter-torque.png",
    "SwarmSystemStatic": "swarm-system.png",
    "AttitudeCorrectionStatic": "attitude-correction.png",
    "SilClosedLoopStatic": "sil-closed-loop.png",
    "FailsafeTimelineStatic": "failsafe-timeline.png",
    "LandingObservabilityStatic": "landing-observability.png",
    "LandingProbeEvidenceStatic": "landing-probe-evidence.png",
    "SharedStateRaceStatic": "shared-state-race.png",
    "TelemetryMotorBalanceStatic": "telemetry-motor-balance.png",
    "ProductionEstimateStatic": "production-estimate.png",
}
STATIC_REQUIRED_TEXT = {
    "DroneClassificationStatic": {
        "고정익",
        "단일로터",
        "멀티로터",
        "쿼드콥터",
        "수직이착륙기",
    },
    "QualificationWeightStatic": {
        "4종",
        "3종",
        "2종",
        "1종",
        "250g초과·2kg이하",
        "2kg초과·7kg이하",
        "7kg초과·25kg이하",
        "최대이륙중량25kg초과",
        "연료제외자체중량150kg이하",
    },
    "AircraftUamStatic": {
        "고정익",
        "헬리콥터",
        "멀티콥터",
        "eVTOL",
        "버티포트",
        "운항",
        "교통관리",
    },
    "MissionSpecsStatic": {
        "공연",
        "동기화",
        "물류",
        "탑재중량",
        "안전감시",
        "체공·통신",
        "국방정찰",
        "보안·내환경성",
    },
    "QuadcopterForceMotionStatic": {
        "측면도",
        "상승",
        "합추력>무게",
        "합추력T",
        "무게W",
        "호버링",
        "합추력=무게",
        "하강",
        "합추력<무게",
        "수평이동",
        "앞으로미는힘",
    },
    "HelicopterQuadcopterTorqueStatic": {
        "위에서본모습",
        "메인로터회전",
        "동체반작용토크",
        "꼬리로터힘",
        "CW",
        "CCW",
        "CW·CCW=로터회전방향",
        "평상시토크상쇄",
        "M3·M4출력↑",
        "M1·M2출력↓",
        "기체YawCW",
    },
    "SwarmSystemStatic": {
        "단일기체",
        "군집체계",
        "공통좌표",
        "통신",
        "상대위치",
        "경로·충돌회피",
        "집단안전",
        "후속목표·미구현·미검증",
    },
    "AttitudeCorrectionStatic": {
        "외란",
        "센서관측",
        "모터출력차이",
        "복원토크",
        "수평복원",
    },
    "SilClosedLoopStatic": {
        "가상물리",
        "센서합성",
        "실제비행코드",
        "모터출력",
        "실물센서·모터·비행미확인",
    },
    "FailsafeTimelineStatic": {
        "상태판단",
        "RC신호두절",
        "자세유지·제한하강",
        "설정된상한·모터정지",
        "치명적고장",
        "즉시모터정지",
    },
    "LandingObservabilityStatic": {
        "지면정지",
        "등속하강",
        "IMU관측동일",
        "거리센서추가단서",
        "폐루프착지판정미검증",
    },
    "LandingProbeEvidenceStatic": {
        "공중프로브2회",
        "0.061g·0.097g",
        "접지프로브10회",
        "0.059~1.147g",
        "값이겹침",
        "지면이면무반응전제실패",
        "로깅전용",
        "착지판정제외",
    },
    "SharedStateRaceStatic": {
        "통신코어",
        "제어태스크",
        "공유상태A",
        "A읽기",
        "B쓰기",
        "옛A기반C쓰기",
        "B갱신손실가능",
    },
    "TelemetryMotorBalanceStatic": {
        "X형모터배치",
        "M1",
        "M3",
        "M3평균>M1평균",
        "M1·1334.2µs",
        "M3·1360.0µs",
        "차이약25.8µs",
        "M3근처테더",
        "줄무게까지함께지지",
        "관측과현장조건해석·M3가테더줄무게까지함께지지",
    },
    "ProductionEstimateStatic": {
        "1대",
        "10대",
        "프린터점유",
        "직접작업",
        "6~10시간",
        "60~100시간",
        "부품비",
        "1대약22.2~23.3만원",
        "10대약222~233만원",
        "모터4개",
        "약8.0만원",
        "ESC4개",
        "약4.0만원",
        "MCU·센서IC",
        "약2.3만원",
        "프레임재료",
        "1.1~2.2만원",
        "배터리·프로펠러",
        "약6.8만원",
        "3901-L0X추가·+4.8~4.9만원/대",
    },
}
STATIC_FORBIDDEN_TEXT = {
    "드론분류",
    "조종자증명",
    "비행체와UAM",
    "임무별요구사양",
    "힘과운동",
    "반작용토크",
    "군집확장",
    "자세복원",
    "SIL폐루프",
    "Failsafe분기",
    "착지관측한계",
    "공유상태경쟁",
    "모터출력균형",
    "고정시간·보편출력값없음",
    "HOSTSIL·실제비행증거아님",
    "가능한race·관측사고아님",
    "예비산정·실측기록아님",
    "프로젝트추정",
    "배송·인건비·재출력·비행튜닝제외",
    "시간:PCB조립완료·첫출력성공·비용:FCPCB·전원·배선등별도",
    "추정",
    "원인확정아님",
}


def load_geometry_module():
    if not GEOMETRY_PATH.is_file():
        return None
    spec = importlib.util.spec_from_file_location(
        "presentation_visualization_geometry", GEOMETRY_PATH
    )
    if spec is None or spec.loader is None:
        return None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_visualization_module():
    if not VISUALIZATION_SOURCE_PATH.is_file():
        return None
    source_dir = str(VISUALIZATION_SOURCE_PATH.parent)
    if source_dir not in sys.path:
        sys.path.insert(0, source_dir)
    spec = importlib.util.spec_from_file_location(
        "presentation_audience_visualizations", VISUALIZATION_SOURCE_PATH
    )
    if spec is None or spec.loader is None:
        return None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_significance_module():
    if not SIGNIFICANCE_SOURCE_PATH.is_file():
        return None
    source_dir = str(SIGNIFICANCE_SOURCE_PATH.parent)
    if source_dir not in sys.path:
        sys.path.insert(0, source_dir)
    spec = importlib.util.spec_from_file_location(
        "presentation_significance_visualizations", SIGNIFICANCE_SOURCE_PATH
    )
    if spec is None or spec.loader is None:
        return None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_engineering_module():
    if not ENGINEERING_SOURCE_PATH.is_file():
        return None
    source_dir = str(ENGINEERING_SOURCE_PATH.parent)
    if source_dir not in sys.path:
        sys.path.insert(0, source_dir)
    spec = importlib.util.spec_from_file_location(
        "presentation_engineering_visualizations", ENGINEERING_SOURCE_PATH
    )
    if spec is None or spec.loader is None:
        return None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_static_module():
    if not STATIC_SOURCE_PATH.is_file():
        return None
    source_dir = str(STATIC_SOURCE_PATH.parent)
    if source_dir not in sys.path:
        sys.path.insert(0, source_dir)
    spec = importlib.util.spec_from_file_location(
        "presentation_static_diagram_visualizations", STATIC_SOURCE_PATH
    )
    if spec is None or spec.loader is None:
        return None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class PresentationVisualizationGeometryTests(unittest.TestCase):
    def test_video_text_is_rendered_ten_percent_larger(self) -> None:
        module = load_visualization_module()
        self.assertIsNotNone(module)

        rendered = module.text("가독성", 20)

        self.assertAlmostEqual(rendered.font_size, 22.0, places=6)

    def test_tilted_axis_components_reconstruct_downward_gravity(self) -> None:
        module = load_geometry_module()
        self.assertIsNotNone(module)

        gravity, horizontal, vertical = module.gravity_components_2d(28.0, 2.5)

        for actual, expected in zip(gravity, (0.0, -2.5)):
            self.assertAlmostEqual(actual, expected, places=4)
        for actual, expected in zip(horizontal, (-1.0363, -0.5510)):
            self.assertAlmostEqual(actual, expected, places=4)
        for actual, expected in zip(vertical, (1.0363, -1.9490)):
            self.assertAlmostEqual(actual, expected, places=4)
        self.assertAlmostEqual(horizontal[0] + vertical[0], gravity[0], places=4)
        self.assertAlmostEqual(horizontal[1] + vertical[1], gravity[1], places=4)

    def test_bias_example_matches_slide_value_after_sixty_seconds(self) -> None:
        module = load_geometry_module()
        self.assertIsNotNone(module)
        self.assertTrue(
            hasattr(module, "integrated_bias_angle_deg"),
            "the bias scene must use a testable integration model",
        )

        self.assertAlmostEqual(
            module.integrated_bias_angle_deg(0.1, 60.0), 6.0, places=6
        )

    def test_sensor_to_body_mapping_matches_firmware_signs(self) -> None:
        module = load_geometry_module()
        self.assertIsNotNone(module)
        self.assertTrue(
            hasattr(module, "transform_sensor_axes"),
            "the axis-sign scene must use the firmware sensor-to-body mapping",
        )

        sensor_sample = (1.0, 2.0, 3.0)
        self.assertEqual(
            module.transform_sensor_axes(sensor_sample, "gyro"),
            (2.0, -1.0, -3.0),
        )
        self.assertEqual(
            module.transform_sensor_axes(sensor_sample, "accel"),
            (2.0, -1.0, 3.0),
        )

    def test_magnetic_reference_preserves_heading_at_capture(self) -> None:
        module = load_geometry_module()
        self.assertIsNotNone(module)
        self.assertTrue(
            hasattr(module, "capture_heading_reference"),
            "the yaw scene must model a relative heading reference",
        )

        offset = module.capture_heading_reference(25.0, 62.0)
        self.assertAlmostEqual(offset, -37.0, places=6)
        self.assertAlmostEqual(
            module.referenced_heading_deg(62.0, offset), 25.0, places=6
        )
        self.assertAlmostEqual(
            module.referenced_heading_deg(82.0, offset), 45.0, places=6
        )


class PresentationProductionEstimateEvidenceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.assertTrue(
            PRODUCTION_ESTIMATE_PATH.is_file(),
            f"missing production estimate: {PRODUCTION_ESTIMATE_PATH}",
        )
        self.data = json.loads(PRODUCTION_ESTIMATE_PATH.read_text(encoding="utf-8"))

    def test_time_estimate_separates_machine_hands_on_and_elapsed_time(self) -> None:
        one = self.data["time"]["one_unit"]
        ten = self.data["time"]["ten_units"]

        self.assertEqual(one["printer_hours"], [24, 48])
        self.assertEqual(one["hands_on_hours"], [6, 10])
        self.assertEqual(one["elapsed_days_one_printer"], [2, 3])
        self.assertEqual(ten["printer_hours_one_printer"], [240, 480])
        self.assertEqual(ten["hands_on_hours"], [60, 100])
        self.assertEqual(ten["elapsed_days_one_printer"], [10, 20])
        self.assertEqual(
            ten["printer_hours_one_printer"],
            [value * 10 for value in one["printer_hours"]],
        )
        self.assertEqual(
            ten["hands_on_hours"],
            [value * 10 for value in one["hands_on_hours"]],
        )
        breakdown = one["hands_on_breakdown_hours"]
        self.assertEqual(
            [sum(bounds[index] for bounds in breakdown.values()) for index in (0, 1)],
            one["hands_on_hours"],
        )
        print_model = self.data["basis"]["print_model"]
        calculated_printer_hours = [
            round(
                mass_kg * 1000 / print_model["effective_output_grams_per_hour"]
            )
            for mass_kg in print_model["printed_mass_kg_per_unit"]
        ]
        self.assertEqual(calculated_printer_hours, one["printer_hours"])

    def test_cost_estimate_keeps_required_categories_and_unknowns_visible(self) -> None:
        cost = self.data["cost"]
        categories = cost["one_unit_categories_krw"]
        self.assertEqual(
            set(categories),
            {
                "motors_4",
                "escs_4",
                "control_sensor_chips",
                "frame_material",
                "battery_propellers",
            },
        )
        calculated = [
            sum(bounds[index] for bounds in categories.values()) for index in (0, 1)
        ]
        self.assertEqual(calculated, cost["one_unit_core_subtotal_krw"])
        self.assertEqual(
            cost["ten_units_core_subtotal_krw"],
            [value * 10 for value in cost["one_unit_core_subtotal_krw"]],
        )
        self.assertEqual(categories["motors_4"], [80000, 80000])
        self.assertEqual(categories["escs_4"], [40000, 40000])
        self.assertEqual(cost["one_unit_core_subtotal_krw"], [221900, 233100])
        self.assertEqual(cost["ten_units_core_subtotal_krw"], [2219000, 2331000])
        evidence = cost["category_evidence"]
        self.assertEqual(
            evidence["motors_4"], "user_confirmed_project_estimate"
        )
        self.assertEqual(
            evidence["escs_4"], "user_confirmed_project_estimate"
        )
        self.assertEqual(
            {
                evidence[key]
                for key in (
                    "control_sensor_chips",
                    "frame_material",
                    "battery_propellers",
                )
            },
            {"vendor_reference_price"},
        )
        self.assertEqual(cost["optional_3901_l0x_per_unit_krw"], [48000, 49000])
        self.assertFalse(cost["verified_bom"])
        self.assertEqual(
            cost["scope"], "project_estimates_and_reference_components"
        )
        for required_exclusion in (
            "custom_fc_pcb_power",
            "wiring_fasteners",
            "shipping_tax",
            "labor",
            "flight_tuning",
        ):
            self.assertIn(required_exclusion, cost["excluded_from_subtotal"])

    def test_estimate_declares_planning_assumptions_not_measured_history(self) -> None:
        basis = self.data["basis"]
        self.assertEqual(basis["evidence_level"], "planning_estimate")
        self.assertIn("parts_in_stock", basis["assumptions"])
        self.assertIn("one_printer", basis["assumptions"])
        self.assertIn("first_print_success", basis["assumptions"])
        self.assertIn("assembled_fc_pcb_available", basis["assumptions"])
        self.assertFalse(basis["includes_free_flight_validation"])

    def test_reference_products_are_not_claimed_as_confirmed_installed_parts(self) -> None:
        document = PRODUCTION_ESTIMATE_DOC_PATH.read_text(encoding="utf-8")
        self.assertIn("저장소에서 확인한 사양 범위", document)
        self.assertIn("구매 SKU", document)
        self.assertIn("설치 부품을 확정한 목록이 아니라", document)
        self.assertIn("예비 산정에 사용한 참고 품목", document)
        self.assertIn("사용자가 확인한 프로젝트 예상값", document)
        self.assertIn("현재 판매가 근거가 아니다", document)
        self.assertNotIn("근거는 [`index.html`](index.html)의 부품 구성 장표", document)


class PresentationVisualizationLayoutTests(unittest.TestCase):
    @staticmethod
    def _rendered_scene(scene_name: str):
        from manim import tempconfig

        module = load_visualization_module()
        if module is None:
            raise AssertionError("presentation visualization module is missing")
        with tempconfig(
            {
                "dry_run": True,
                "disable_caching": True,
                "verbosity": "ERROR",
                "frame_rate": 1,
                "progress_bar": "none",
            }
        ):
            scene = getattr(module, scene_name)()
            scene.hold_and_clear = lambda *args, **kwargs: None
            scene.render()
        return scene

    @staticmethod
    def _rendered_static_scene(scene_name: str):
        from manim import tempconfig

        module = load_static_module()
        if module is None:
            raise AssertionError("static diagram visualization module is missing")
        scene_class = getattr(module, scene_name, None)
        if scene_class is None:
            raise AssertionError(f"missing static scene: {scene_name}")
        with tempconfig(
            {
                "dry_run": True,
                "disable_caching": True,
                "verbosity": "ERROR",
                "frame_rate": 1,
                "progress_bar": "none",
            }
        ):
            scene = scene_class()
            scene.render()
        return scene

    @staticmethod
    def _mobjects(scene):
        def descendants(mobject):
            yield mobject
            for child in mobject.submobjects:
                yield from descendants(child)

        for root in scene.mobjects:
            yield from descendants(root)

    @classmethod
    def _text(cls, scene, normalized_text: str):
        for mobject in cls._mobjects(scene):
            if getattr(mobject, "text", None) == normalized_text:
                return mobject
        raise AssertionError(f"missing rendered text: {normalized_text}")

    @staticmethod
    def _bbox_overlaps(first, second, gap: float = 0.0) -> bool:
        return not (
            first.get_right()[0] + gap <= second.get_left()[0]
            or second.get_right()[0] + gap <= first.get_left()[0]
            or first.get_top()[1] + gap <= second.get_bottom()[1]
            or second.get_top()[1] + gap <= first.get_bottom()[1]
        )

    def _assert_contained(self, outer, inner, inset: float = 0.0) -> None:
        self.assertGreaterEqual(inner.get_left()[0], outer.get_left()[0] + inset)
        self.assertLessEqual(inner.get_right()[0], outer.get_right()[0] - inset)
        self.assertGreaterEqual(inner.get_bottom()[1], outer.get_bottom()[1] + inset)
        self.assertLessEqual(inner.get_top()[1], outer.get_top()[1] - inset)

    def test_accelerometer_component_labels_use_the_explanation_zone(self) -> None:
        scene = self._rendered_scene("AccelerometerAudience")

        horizontal = self._text(scene, "수평축성분")
        vertical = self._text(scene, "수직축성분")

        self.assertGreater(horizontal.get_left()[0], 2.8)
        self.assertGreater(vertical.get_left()[0], 2.8)

    def test_pi_curve_labels_stay_above_the_plot_lines(self) -> None:
        scene = self._rendered_scene("PiErrorAudience")

        p_label = self._text(scene, "P만:오차가남음")
        pi_label = self._text(scene, "P+I:오차가0으로복귀")

        self.assertGreater(p_label.get_bottom()[1], 1.3)
        self.assertGreater(pi_label.get_bottom()[1], 1.3)

    def test_complementary_filter_descriptions_clear_the_plot_area(self) -> None:
        scene = self._rendered_scene("ComplementaryFilterAudience")
        descriptions = [
            self._text(scene, "빠르지만서서히표류"),
            self._text(scene, "장기기준이지만순간진동"),
            self._text(scene, "빠르고기준에서벗어나지않음"),
        ]
        curves = sorted(
            (
                item
                for item in self._mobjects(scene)
                if type(item).__name__ == "VMobject"
                and len(item.get_all_points()) > 500
                and str(item.get_color()) in {"#FF6474", "#FF9F43", "#55D68B"}
            ),
            key=lambda item: item.get_center()[1],
            reverse=True,
        )

        self.assertEqual(len(curves), 3)
        for description, curve in zip(descriptions, curves):
            self.assertLess(
                description.get_right()[0] + 0.18,
                curve.get_left()[0],
            )

    def test_cascade_row_labels_clear_the_first_target_box(self) -> None:
        scene = self._rendered_scene("CascadeTimingAudience")

        outer = self._text(scene, "바깥자세루프")
        inner = self._text(scene, "안쪽각속도루프")
        cadence = self._text(scene, "자세목표사이에서여러번보정")

        self.assertLess(outer.get_right()[0], -4.4)
        self.assertLess(inner.get_right()[0], -4.4)
        self.assertGreater(cadence.get_bottom()[1], -3.2)

    def test_gyro_bias_axis_label_is_horizontal_above_the_plot(self) -> None:
        scene = self._rendered_scene("GyroBiasAudience")

        axis_label = self._text(scene, "누적각도오차")

        self.assertGreater(axis_label.width, axis_label.height)
        self.assertGreater(axis_label.get_bottom()[1], 1.25)

    def test_landing_comparison_label_sits_above_both_panels(self) -> None:
        scene = self._rendered_scene("LandingAmbiguityAudience")

        comparison = self._text(scene, "움직임은다르지만센서값은같다")

        self.assertGreater(comparison.get_bottom()[1], 2.05)

    def test_landing_velocity_label_stays_inside_right_panel_and_frame(self) -> None:
        scene = self._rendered_scene("LandingAmbiguityAudience")
        velocity = self._text(scene, "속도는아래로")
        right_panel = max(
            (
                item
                for item in self._mobjects(scene)
                if type(item).__name__ == "RoundedRectangle"
                and 6.7 < item.width < 6.9
                and 4.4 < item.height < 4.7
            ),
            key=lambda item: item.get_center()[0],
        )

        self._assert_contained(right_panel, velocity, inset=0.12)
        self.assertLessEqual(velocity.get_right()[0], 7.96)

    def test_static_aircraft_roles_separate_title_icon_and_tradeoff_rows(self) -> None:
        scene = self._rendered_static_scene("AircraftUamStatic")
        roles = scene.mobjects[0]

        self.assertEqual(len(roles), 4)
        for role in roles:
            panel, title, icon, strength, tradeoff = role
            for item in (title, icon, strength, tradeoff):
                self._assert_contained(panel, item, inset=0.08)
            for upper, lower in (
                (title, icon),
                (icon, strength),
                (strength, tradeoff),
            ):
                self.assertFalse(self._bbox_overlaps(upper, lower, gap=0.10))
                self.assertGreater(upper.get_bottom()[1], lower.get_top()[1])

    def test_static_mission_motifs_stay_inside_cards_and_clear_titles(self) -> None:
        scene = self._rendered_static_scene("MissionSpecsStatic")
        tiles = scene.mobjects[0]

        self.assertEqual(len(tiles), 4)
        for tile in tiles:
            panel, _connector, title, motif, requirement = tile
            for item in (title, motif, requirement):
                self._assert_contained(panel, item, inset=0.10)
            self.assertFalse(self._bbox_overlaps(title, motif, gap=0.14))

    def test_static_force_arrows_clear_titles_and_have_direct_labels(self) -> None:
        import math

        scene = self._rendered_static_scene("QuadcopterForceMotionStatic")
        tiles = scene.mobjects[0]

        self.assertEqual(len(tiles), 4)
        for tile in tiles:
            panel = tile[0]
            title = next(
                item
                for item in tile.submobjects
                if getattr(item, "text", None)
                in {"상승", "호버링", "하강", "수평이동"}
            )
            arrows = [
                item for item in tile.submobjects if type(item).__name__ == "Arrow"
            ]
            tile_text = {
                item.text: item
                for item in tile.submobjects
                if getattr(item, "text", None) is not None
            }
            self.assertIn("합추력T", tile_text)
            self.assertIn("무게W", tile_text)
            thrust_label = tile_text["합추력T"]
            weight_label = tile_text["무게W"]
            drone = next(
                item for item in tile.submobjects if type(item).__name__ == "VGroup"
            )
            self.assertGreaterEqual(len(arrows), 2)
            for label in (thrust_label, weight_label):
                self._assert_contained(panel, label, inset=0.08)
                self.assertFalse(self._bbox_overlaps(label, drone, gap=0.04))
            for arrow in arrows:
                self._assert_contained(panel, arrow, inset=0.08)
                self.assertLessEqual(arrow.get_top()[1] + 0.08, title.get_bottom()[1])
                for label in (thrust_label, weight_label):
                    self.assertFalse(self._bbox_overlaps(label, arrow, gap=0.04))
            self.assertFalse(
                self._bbox_overlaps(thrust_label, weight_label, gap=0.08)
            )
            self.assertLess(
                min(
                    math.dist(thrust_label.get_center()[:2], arrow.get_center()[:2])
                    for arrow in arrows
                    if str(arrow.get_color()) != "#FFD166"
                ),
                1.15,
            )
            weight_arrow = next(
                arrow for arrow in arrows if str(arrow.get_color()) == "#FFD166"
            )
            self.assertLess(
                math.dist(weight_label.get_center()[:2], weight_arrow.get_center()[:2]),
                1.5,
            )

        horizontal = tiles[3]
        component = next(
            item
            for item in horizontal.submobjects
            if getattr(item, "text", None) == "앞으로미는힘"
        )
        self.assertGreater(component.width, component.height)
        horizontal_text = {
            item.text: item
            for item in horizontal.submobjects
            if getattr(item, "text", None) is not None
        }
        horizontal_drone = next(
            item for item in horizontal.submobjects if type(item).__name__ == "VGroup"
        )
        self._assert_contained(horizontal[0], component, inset=0.08)
        self.assertFalse(self._bbox_overlaps(component, horizontal_drone, gap=0.04))
        self.assertFalse(
            self._bbox_overlaps(component, horizontal_text["무게W"], gap=0.08)
        )

        rotor_ellipses = [
            item
            for item in self._mobjects(scene)
            if type(item).__name__ == "Ellipse" and item.width > item.height * 2.0
        ]
        self.assertGreaterEqual(
            len(rotor_ellipses),
            8,
            "each side-view state should show the front and rear rotor discs",
        )

    def test_static_force_vectors_share_origin_and_encode_physical_relations(self) -> None:
        import math

        scene = self._rendered_static_scene("QuadcopterForceMotionStatic")
        arrows = [
            item for item in self._mobjects(scene) if type(item).__name__ == "Arrow"
        ]
        weight_arrows = [
            item for item in arrows if str(item.get_color()) == "#FFD166"
        ]
        self.assertEqual(len(weight_arrows), 4)

        state_colors = {
            "상승": "#55D68B",
            "호버링": "#5FE3F3",
            "하강": "#FF9F43",
            "수평이동": "#48A8FF",
        }
        vectors = {}
        for state, color in state_colors.items():
            self._text(scene, state)
            thrust = next(item for item in arrows if str(item.get_color()) == color)
            weight = min(
                weight_arrows,
                key=lambda item: math.dist(
                    item.get_center()[:2], thrust.get_center()[:2]
                ),
            )
            for axis in (0, 1):
                self.assertAlmostEqual(
                    thrust.get_start()[axis],
                    weight.get_start()[axis],
                    places=5,
                    msg=f"{state}: thrust and weight need a common origin",
                )
            thrust_vector = thrust.get_end() - thrust.get_start()
            weight_vector = weight.get_end() - weight.get_start()
            self.assertAlmostEqual(float(weight_vector[0]), 0.0, places=5)
            self.assertLess(float(weight_vector[1]), 0.0)
            vectors[state] = (
                thrust_vector,
                math.hypot(float(thrust_vector[0]), float(thrust_vector[1])),
                math.hypot(float(weight_vector[0]), float(weight_vector[1])),
            )

        self.assertGreater(vectors["상승"][1], vectors["상승"][2])
        self.assertAlmostEqual(vectors["호버링"][1], vectors["호버링"][2], places=5)
        self.assertGreater(vectors["하강"][1], 0.0)
        self.assertLess(vectors["하강"][1], vectors["하강"][2])
        horizontal_vector, _, horizontal_weight = vectors["수평이동"]
        self.assertGreater(float(horizontal_vector[0]), 0.0)
        self.assertAlmostEqual(
            float(horizontal_vector[1]), horizontal_weight, places=5
        )

    def test_static_rotor_mapping_and_helicopter_torques_have_correct_signs(self) -> None:
        import math

        scene = self._rendered_static_scene("HelicopterQuadcopterTorqueStatic")
        self._text(scene, "CW·CCW=로터회전방향")
        rendered_text = [
            item
            for item in self._mobjects(scene)
            if getattr(item, "text", None) is not None
        ]
        rotation_labels = [
            item for item in rendered_text if item.text in {"CW", "CCW"}
        ]
        expected_mapping = {
            "M1": (-1, 1, "CW"),
            "M3": (1, 1, "CCW"),
            "M4": (-1, -1, "CCW"),
            "M2": (1, -1, "CW"),
        }
        motor_labels = {
            name: self._text(scene, name) for name in expected_mapping
        }
        quad_center_x = (
            sum(label.get_center()[0] for label in motor_labels.values()) / 4
        )
        quad_center_y = (
            sum(label.get_center()[1] for label in motor_labels.values()) / 4
        )
        for name, (x_sign, y_sign, rotation) in expected_mapping.items():
            motor = motor_labels[name]
            self.assertGreater((motor.get_center()[0] - quad_center_x) * x_sign, 0)
            self.assertGreater((motor.get_center()[1] - quad_center_y) * y_sign, 0)
            nearest_rotation = min(
                rotation_labels,
                key=lambda item: math.dist(
                    item.get_center()[:2], motor.get_center()[:2]
                ),
            )
            self.assertEqual(nearest_rotation.text, rotation)

        rotor_disk = max(
            (
                item
                for item in self._mobjects(scene)
                if type(item).__name__ == "Circle" and item.get_center()[0] < 0
            ),
            key=lambda item: item.width,
        )
        reaction = next(
            item
            for item in self._mobjects(scene)
            if type(item).__name__ == "CurvedArrow"
            and str(item.get_color()) == "#FF9F43"
            and item.get_center()[0] < 0
        )
        tail_force = next(
            item
            for item in self._mobjects(scene)
            if type(item).__name__ == "Arrow"
            and str(item.get_color()) == "#5FE3F3"
            and item.get_center()[0] < 0
        )

        def moment_sign(arrow) -> float:
            radius = arrow.get_start() - rotor_disk.get_center()
            force = arrow.get_end() - arrow.get_start()
            return float(radius[0] * force[1] - radius[1] * force[0])

        reaction_moment = moment_sign(reaction)
        tail_force_moment = moment_sign(tail_force)
        self.assertNotEqual(reaction_moment, 0.0)
        self.assertNotEqual(tail_force_moment, 0.0)
        self.assertLess(reaction_moment * tail_force_moment, 0.0)

        main_rotor_blades = [
            item
            for item in self._mobjects(scene)
            if type(item).__name__ == "Line"
            and item.get_center()[0] < 0
            and item.get_length() > 1.4
        ]
        self.assertGreaterEqual(
            len(main_rotor_blades),
            2,
            "the helicopter main rotor needs visible crossing blades in top view",
        )

    def test_static_helicopter_arrows_have_adjacent_labels_and_yaw_example(self) -> None:
        import math

        scene = self._rendered_static_scene("HelicopterQuadcopterTorqueStatic")
        objects = list(self._mobjects(scene))
        labeled_arrows = (
            ("메인로터회전", "CurvedArrow", "#48A8FF"),
            ("동체반작용토크", "CurvedArrow", "#FF9F43"),
            ("꼬리로터힘", "Arrow", "#5FE3F3"),
        )
        for label_text, arrow_type, color in labeled_arrows:
            label = self._text(scene, label_text)
            arrow = next(
                item
                for item in objects
                if type(item).__name__ == arrow_type
                and str(item.get_color()) == color
                and item.get_center()[0] < 0
            )
            self.assertLess(
                math.dist(label.get_center()[:2], arrow.get_center()[:2]), 2.05
            )
            self.assertFalse(self._bbox_overlaps(label, arrow, gap=0.04))

        for required_example in (
            "M3·M4출력↑",
            "M1·M2출력↓",
            "기체YawCW",
            "평상시토크상쇄",
        ):
            self._text(scene, required_example)

    def test_static_removed_top_badges_keep_in_diagram_evidence(self) -> None:
        sil = self._rendered_static_scene("SilClosedLoopStatic")
        sil_text = {
            item.text
            for item in self._mobjects(sil)
            if getattr(item, "text", None) is not None
        }
        self.assertNotIn("HOSTSIL·실제비행증거아님", sil_text)
        self.assertIn("실물센서·모터·비행미확인", sil_text)

        race = self._rendered_static_scene("SharedStateRaceStatic")
        race_text = {
            item.text
            for item in self._mobjects(race)
            if getattr(item, "text", None) is not None
        }
        self.assertNotIn("가능한race·관측사고아님", race_text)
        for mechanism in ("A읽기", "B쓰기", "옛A기반C쓰기", "B갱신손실가능"):
            self.assertIn(mechanism, race_text)

    def test_static_landing_probe_shows_why_the_threshold_could_not_separate_states(self) -> None:
        scene = self._rendered_static_scene("LandingProbeEvidenceStatic")
        air_value = self._text(scene, "0.061g·0.097g")
        ground_value = self._text(scene, "0.059~1.147g")
        overlap = self._text(scene, "값이겹침")
        premise = self._text(scene, "지면이면무반응전제실패")
        self.assertLess(air_value.get_center()[0], ground_value.get_center()[0])
        self.assertGreater(air_value.get_center()[1], premise.get_center()[1])
        self.assertGreater(ground_value.get_center()[1], premise.get_center()[1])
        self.assertLess(abs(overlap.get_center()[0]), 1.2)
        for boundary in ("로깅전용", "착지판정제외"):
            self._text(scene, boundary)

    def test_static_tether_inference_is_explicit_and_bounded(self) -> None:
        import math

        scene = self._rendered_static_scene("TelemetryMotorBalanceStatic")
        rendered_text = {
            item.text
            for item in self._mobjects(scene)
            if getattr(item, "text", None) is not None
        }
        for required in (
            "M3근처테더",
            "줄무게까지함께지지",
            "관측과현장조건해석·M3가테더줄무게까지함께지지",
        ):
            self.assertIn(required, rendered_text)
        self.assertNotIn("추정", rendered_text)
        self.assertNotIn("원인확정아님", rendered_text)
        self.assertNotIn("질량배분·추력차이·테더·프레임·공력", rendered_text)

        m3 = self._text(scene, "M3")
        tether_lines = [
            item
            for item in self._mobjects(scene)
            if type(item).__name__ in {"Line", "DashedLine"}
            and str(item.get_color()) == "#FFD166"
        ]
        self.assertTrue(tether_lines)
        self.assertLess(
            min(
                math.dist(endpoint[:2], m3.get_center()[:2])
                for line in tether_lines
                for endpoint in (line.get_start(), line.get_end())
            ),
            0.85,
        )
        tether_label = self._text(scene, "M3근처테더")
        left_panel = next(
            item
            for item in scene.mobjects
            if type(item).__name__ == "RoundedRectangle"
            and item.get_center()[0] < 0
            and item.width > 6.0
        )
        self._assert_contained(left_panel, tether_label, inset=0.10)
        for obstacle_text in ("X형모터배치", "전방", "M1", "M2", "M3", "M4"):
            obstacle = self._text(scene, obstacle_text)
            self.assertFalse(
                self._bbox_overlaps(tether_label, obstacle, gap=0.08),
                f"tether label overlaps {obstacle_text}",
            )
        for tether_line in tether_lines:
            self.assertFalse(
                self._bbox_overlaps(tether_label, tether_line, gap=0.06)
            )

    def test_swarm_future_uses_caution_while_danger_stays_red(self) -> None:
        scene = self._rendered_static_scene("SwarmSystemStatic")
        boundary = self._text(scene, "후속목표·미구현·미검증")
        boundary_glyph_colors = {
            str(glyph.get_fill_color()) for glyph in boundary.submobjects
        }
        self.assertEqual(boundary_glyph_colors, {"#FFD166"})

        takeaway_box = next(
            item
            for item in self._mobjects(scene)
            if type(item).__name__ == "RoundedRectangle"
            and item.width > 14.0
            and item.get_center()[1] < -3.0
        )
        self.assertEqual(str(takeaway_box.get_stroke_color()), "#FFD166")

        danger_scene = self._rendered_static_scene("FailsafeTimelineStatic")
        red_state_boxes = [
            item
            for item in self._mobjects(danger_scene)
            if type(item).__name__ == "RoundedRectangle"
            and str(item.get_stroke_color()) == "#FF6474"
            and item.height > 0.9
        ]
        for dangerous_state in ("치명적고장", "즉시모터정지"):
            danger_label = self._text(danger_scene, dangerous_state)
            nearest_red_box = min(
                red_state_boxes,
                key=lambda item: sum(
                    (item.get_center()[axis] - danger_label.get_center()[axis]) ** 2
                    for axis in (0, 1)
                ),
            )
            self.assertAlmostEqual(
                nearest_red_box.get_center()[0], danger_label.get_center()[0], places=5
            )
            self.assertAlmostEqual(
                nearest_red_box.get_center()[1], danger_label.get_center()[1], places=5
            )

    def test_significance_scenes_render(self) -> None:
        from manim import tempconfig

        module = load_significance_module()
        self.assertIsNotNone(module, "significance visualization module is missing")
        with tempconfig(
            {
                "dry_run": True,
                "disable_caching": True,
                "verbosity": "ERROR",
                "frame_rate": 1,
                "progress_bar": "none",
            }
        ):
            for scene_name in SIGNIFICANCE_SCENES:
                with self.subTest(scene_name=scene_name):
                    scene_class = getattr(module, scene_name, None)
                    self.assertIsNotNone(scene_class)
                    scene = scene_class()
                    scene.hold_and_clear = lambda *args, **kwargs: None
                    scene.render()

    def test_engineering_scenes_render(self) -> None:
        from manim import tempconfig

        module = load_engineering_module()
        self.assertIsNotNone(module, "engineering visualization module is missing")
        with tempconfig(
            {
                "dry_run": True,
                "disable_caching": True,
                "verbosity": "ERROR",
                "frame_rate": 1,
                "progress_bar": "none",
            }
        ):
            for scene_name in ENGINEERING_SCENES:
                with self.subTest(scene_name=scene_name):
                    scene_class = getattr(module, scene_name, None)
                    self.assertIsNotNone(scene_class)
                    scene = scene_class()
                    scene.hold_and_clear = lambda *args, **kwargs: None
                    scene.render()

    def test_engineering_scene_text_omits_dates_and_commit_hashes(self) -> None:
        from manim import Text, tempconfig
        from unittest.mock import patch

        module = load_engineering_module()
        self.assertIsNotNone(module)
        forbidden = re.compile(
            r"(?<!\d)(?:19|20)\d{2}-\d{2}-\d{2}(?!\d)"
            r"|(?<![0-9a-fA-F])[0-9a-fA-F]{7,40}(?![0-9a-fA-F])"
        )
        with tempconfig(
            {
                "dry_run": True,
                "disable_caching": True,
                "verbosity": "ERROR",
                "frame_rate": 1,
                "progress_bar": "none",
            }
        ):
            for scene_name in ENGINEERING_SCENES:
                with self.subTest(scene_name=scene_name):
                    rendered_text = []
                    original_text_init = Text.__init__

                    def record_text_init(text_object, *args, **kwargs):
                        original_text_init(text_object, *args, **kwargs)
                        rendered_text.append(text_object.text)

                    scene = getattr(module, scene_name)()
                    scene.hold_and_clear = lambda *args, **kwargs: None
                    with patch.object(Text, "__init__", record_text_init):
                        scene.render()
                    self.assertTrue(rendered_text, f"no Text rendered by {scene_name}")
                    self.assertFalse(
                        any(forbidden.search(value) for value in rendered_text),
                        rendered_text,
                    )

    def test_static_scenes_render_all_comparison_content_without_animation(self) -> None:
        from manim import Text, tempconfig
        from unittest.mock import patch

        module = load_static_module()
        self.assertIsNotNone(module, "static diagram visualization module is missing")
        forbidden_identifier = re.compile(
            r"(?<!\d)(?:19|20)\d{2}-\d{2}-\d{2}(?!\d)"
            r"|(?<![0-9a-fA-F])[0-9a-fA-F]{7,40}(?![0-9a-fA-F])"
        )
        with tempconfig(
            {
                "dry_run": True,
                "disable_caching": True,
                "verbosity": "ERROR",
                "frame_rate": 1,
                "progress_bar": "none",
            }
        ):
            for scene_name, required_text in STATIC_REQUIRED_TEXT.items():
                with self.subTest(scene_name=scene_name):
                    scene_class = getattr(module, scene_name, None)
                    self.assertIsNotNone(scene_class)
                    scene = scene_class()
                    scene.play = lambda *args, **kwargs: self.fail(
                        f"{scene_name} must show every comparison state at once"
                    )
                    scene.wait = lambda *args, **kwargs: self.fail(
                        f"{scene_name} must not rely on a timed reveal"
                    )
                    created_text = []
                    original_text_init = Text.__init__

                    def record_text_init(text_object, *args, **kwargs):
                        original_text_init(text_object, *args, **kwargs)
                        created_text.append(text_object)

                    with patch.object(Text, "__init__", record_text_init):
                        scene.render()

                    rendered_text = {item.text for item in created_text}
                    self.assertTrue(
                        required_text.issubset(rendered_text),
                        f"missing {sorted(required_text - rendered_text)} from {scene_name}",
                    )
                    self.assertFalse(
                        STATIC_FORBIDDEN_TEXT & rendered_text,
                        f"forbidden slide-level text duplicated inside {scene_name}",
                    )
                    self.assertFalse(
                        any(forbidden_identifier.search(value) for value in rendered_text),
                        sorted(rendered_text),
                    )
                    self.assertTrue(created_text, f"no Text rendered by {scene_name}")
                    for text_object in created_text:
                        self.assertGreaterEqual(
                            float(text_object.font_size),
                            26.0,
                            f"undersized text in {scene_name}: {text_object.text}",
                        )
                    for mobject in scene.mobjects:
                        self.assertGreaterEqual(mobject.get_left()[0], -7.96)
                        self.assertLessEqual(mobject.get_right()[0], 7.96)
                        self.assertGreaterEqual(mobject.get_bottom()[1], -4.46)
                        self.assertLessEqual(mobject.get_top()[1], 4.46)

    def test_production_estimate_keeps_scale_and_costs_clear_without_visible_caveats(self) -> None:
        scene = self._rendered_static_scene("ProductionEstimateStatic")
        texts = [
            item
            for item in self._mobjects(scene)
            if getattr(item, "text", None) is not None
        ]

        multiplier = next(item for item in texts if item.text == "×10")
        printer_labels = sorted(
            (item for item in texts if item.text == "프린터점유"),
            key=lambda item: item.get_center()[0],
        )
        self.assertEqual(len(printer_labels), 2)
        self.assertLess(
            multiplier.get_right()[0] + 0.18,
            printer_labels[1].get_left()[0],
        )

        rendered_text = {item.text for item in texts}
        for removed in (
            "예비산정·실측기록아님",
            "프로젝트추정",
            "배송·인건비·재출력·비행튜닝제외",
            "시간:PCB조립완료·첫출력성공·비용:FCPCB·전원·배선등별도",
        ):
            self.assertNotIn(removed, rendered_text)

        paired_cost_text = {
            "모터4개": "약8.0만원",
            "ESC4개": "약4.0만원",
            "MCU·센서IC": "약2.3만원",
            "프레임재료": "1.1~2.2만원",
            "배터리·프로펠러": "약6.8만원",
        }
        category_cards = [
            item
            for item in self._mobjects(scene)
            if type(item).__name__ == "RoundedRectangle"
            and 2.6 < item.width < 2.8
            and 1.1 < item.height < 1.3
        ]
        self.assertEqual(len(category_cards), 5)
        for name_text, value_text in paired_cost_text.items():
            name = next(item for item in texts if item.text == name_text)
            value = next(item for item in texts if item.text == value_text)
            self.assertAlmostEqual(
                name.get_center()[0], value.get_center()[0], delta=0.08
            )
            self.assertGreater(name.get_center()[1], value.get_center()[1])
            self.assertLess(name.get_center()[1] - value.get_center()[1], 0.6)
            card = min(
                category_cards,
                key=lambda item: abs(item.get_center()[0] - name.get_center()[0]),
            )
            for label in (name, value):
                self.assertGreaterEqual(label.get_left()[0], card.get_left()[0] + 0.04)
                self.assertLessEqual(label.get_right()[0], card.get_right()[0] - 0.04)
                self.assertGreaterEqual(
                    label.get_bottom()[1], card.get_bottom()[1] + 0.04
                )
                self.assertLessEqual(label.get_top()[1], card.get_top()[1] - 0.04)


    def test_telemetry_evidence_badge_clears_content_panels(self) -> None:
        from manim import tempconfig

        module = load_engineering_module()
        self.assertIsNotNone(module)
        with tempconfig(
            {
                "dry_run": True,
                "disable_caching": True,
                "verbosity": "ERROR",
                "frame_rate": 1,
                "progress_bar": "none",
            }
        ):
            scene = module.TelemetryMotorBalanceAudience()
            scene.hold_and_clear = lambda *args, **kwargs: None
            scene.render()

        rounded_rectangles = [
            item
            for item in self._mobjects(scene)
            if type(item).__name__ == "RoundedRectangle"
        ]
        badge = next(
            item
            for item in rounded_rectangles
            if item.width < 8.0 and item.height < 0.7
        )
        panels = [
            item
            for item in rounded_rectangles
            if 6.0 < item.width < 8.0 and 4.0 < item.height < 5.0
        ]

        self.assertEqual(len(panels), 2)
        for panel in panels:
            self.assertGreater(badge.get_bottom()[1], panel.get_top()[1])

    def test_helicopter_tail_force_opposes_main_rotor_reaction_torque(self) -> None:
        from manim import tempconfig

        module = load_significance_module()
        self.assertIsNotNone(module)
        with tempconfig(
            {
                "dry_run": True,
                "disable_caching": True,
                "verbosity": "ERROR",
                "frame_rate": 1,
                "progress_bar": "none",
            }
        ):
            scene = module.HelicopterQuadcopterTorqueAudience()
            scene.hold_and_clear = lambda *args, **kwargs: None
            scene.render()

        objects = list(self._mobjects(scene))
        reaction = next(
            item
            for item in objects
            if type(item).__name__ == "CurvedArrow"
            and str(item.get_color()) == module.ORANGE
        )
        tail_force = next(
            item
            for item in objects
            if type(item).__name__ == "Arrow"
            and str(item.get_color()) == module.CYAN
            and item.get_center()[0] < 0
        )

        reaction_start = reaction.get_start()
        reaction_end = reaction.get_end()
        rightmost_endpoint = max(
            (reaction_start, reaction_end), key=lambda point: point[0]
        )
        rotor_center = (tail_force.get_start()[0], rightmost_endpoint[1])
        reaction_moment = (
            (reaction_start[0] - rotor_center[0])
            * (reaction_end[1] - rotor_center[1])
            - (reaction_start[1] - rotor_center[1])
            * (reaction_end[0] - rotor_center[0])
        )
        tail_arm = tail_force.get_start()[:2] - rotor_center
        tail_vector = tail_force.get_end()[:2] - tail_force.get_start()[:2]
        tail_moment = tail_arm[0] * tail_vector[1] - tail_arm[1] * tail_vector[0]

        self.assertNotEqual(reaction_moment, 0)
        self.assertNotEqual(tail_moment, 0)
        self.assertLess(reaction_moment * tail_moment, 0)

    def test_helicopter_tail_force_clears_explanation_labels(self) -> None:
        from manim import tempconfig

        module = load_significance_module()
        self.assertIsNotNone(module)
        with tempconfig(
            {
                "dry_run": True,
                "disable_caching": True,
                "verbosity": "ERROR",
                "frame_rate": 1,
                "progress_bar": "none",
            }
        ):
            scene = module.HelicopterQuadcopterTorqueAudience()
            scene.hold_and_clear = lambda *args, **kwargs: None
            scene.render()

        objects = list(self._mobjects(scene))
        tail_force = next(
            item
            for item in objects
            if type(item).__name__ == "Arrow"
            and str(item.get_color()) == module.CYAN
            and item.get_center()[0] < 0
        )
        diagram_obstacles = [tail_force] + [
            item
            for item in objects
            if type(item).__name__ in {"Line", "Ellipse"}
            and item.get_center()[0] < 0
        ]
        for label_text in ("헬리콥터", "꼬리로터힘으로상쇄"):
            label = self._text(scene, label_text)
            for obstacle in diagram_obstacles:
                horizontal_overlap = (
                    obstacle.get_left()[0] < label.get_right()[0]
                    and obstacle.get_right()[0] > label.get_left()[0]
                )
                vertical_overlap = (
                    obstacle.get_bottom()[1] < label.get_top()[1]
                    and obstacle.get_top()[1] > label.get_bottom()[1]
                )
                self.assertFalse(horizontal_overlap and vertical_overlap)

    def test_swarm_unimplemented_badge_precedes_multi_aircraft_network(self) -> None:
        from manim import tempconfig

        module = load_significance_module()
        self.assertIsNotNone(module)
        snapshots = []
        with tempconfig(
            {
                "dry_run": True,
                "disable_caching": True,
                "verbosity": "ERROR",
                "frame_rate": 1,
                "progress_bar": "none",
            }
        ):
            scene = module.SwarmSystemAudience()
            original_play = scene.play

            def record_scene_state(*animations, **kwargs):
                result = original_play(*animations, **kwargs)
                objects = list(self._mobjects(scene))
                motor_count = sum(
                    type(item).__name__ == "Circle"
                    and str(item.get_stroke_color()) == module.CYAN
                    for item in objects
                )
                rendered_text = {
                    item.text
                    for item in objects
                    if getattr(item, "text", None) is not None
                }
                snapshots.append((motor_count, rendered_text))
                return result

            scene.play = record_scene_state
            scene.hold_and_clear = lambda *args, **kwargs: None
            scene.render()

        multi_aircraft_snapshots = [
            rendered_text
            for motor_count, rendered_text in snapshots
            if motor_count >= 20
        ]
        self.assertTrue(multi_aircraft_snapshots)
        for rendered_text in multi_aircraft_snapshots:
            self.assertIn("후속목표·미구현·미검증", rendered_text)


class PresentationStaticDiagramDeliveryTests(unittest.TestCase):
    def test_motor_chart_uses_display_samples_without_conflicting_legend_averages(self) -> None:
        from PIL import Image

        source = MOTOR_CHART_SOURCE_PATH.read_text(encoding="utf-8")
        self.assertIn('label="M1 · 전-좌"', source)
        self.assertIn('label="M3 · 전-우"', source)
        self.assertNotIn("avg 1338", source)
        self.assertNotIn("avg 1365", source)
        self.assertIn('series["t"]', source)
        self.assertIn('series["m1"]', source)
        self.assertIn('series["m3"]', source)

        with Image.open(ASSET_DIR / "chart_motor.png") as chart:
            self.assertEqual(chart.size, (1280, 720))
            self.assertIn(chart.mode, {"RGB", "RGBA"})

    def test_static_python_diagrams_are_nonblank_hd_pngs(self) -> None:
        from PIL import Image, ImageChops

        for filename in STATIC_SCENES.values():
            with self.subTest(filename=filename):
                path = ASSET_DIR / filename
                self.assertTrue(path.is_file(), f"missing static diagram: {path}")
                with Image.open(path) as image:
                    self.assertEqual(image.size, (1280, 720))
                    self.assertIn(image.mode, {"RGB", "RGBA"})
                    rgb = image.convert("RGB")
                    background = Image.new("RGB", image.size, rgb.getpixel((0, 0)))
                    difference = ImageChops.difference(rgb, background)
                    self.assertIsNotNone(
                        difference.getbbox(), f"uniform static diagram: {path}"
                    )


@unittest.skipUnless(shutil.which("ffprobe"), "ffprobe is required")
class PresentationVisualizationDeliveryTests(unittest.TestCase):
    def test_explainer_videos_use_readable_browser_delivery_profile(self) -> None:
        for filename in VISUALIZATION_FILES:
            with self.subTest(filename=filename):
                completed = subprocess.run(
                    [
                        "ffprobe",
                        "-v",
                        "error",
                        "-select_streams",
                        "v:0",
                        "-show_entries",
                        "stream=codec_name,width,height,r_frame_rate,pix_fmt",
                        "-show_entries",
                        "format=duration",
                        "-of",
                        "json",
                        str(ASSET_DIR / filename),
                    ],
                    check=True,
                    capture_output=True,
                    text=True,
                )
                metadata = json.loads(completed.stdout)
                stream = metadata["streams"][0]
                duration = float(metadata["format"]["duration"])

                self.assertEqual(stream["codec_name"], "h264")
                self.assertEqual((stream["width"], stream["height"]), (1280, 720))
                self.assertEqual(stream["r_frame_rate"], "30/1")
                self.assertEqual(stream["pix_fmt"], "yuv420p")
                self.assertGreaterEqual(duration, 5.0)
                self.assertLessEqual(duration, 12.0)


if __name__ == "__main__":
    unittest.main()
