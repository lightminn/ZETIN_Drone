import sys
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

import telemetry_schema  # noqa: E402
from telemetry_schema import (  # noqa: E402
    CSV_FIELDS,
    TELEMETRY_FIELDS,
    parse_telemetry_packet,
    sample_to_csv_row,
)


def packet(field_count):
    values = [str(index + 0.5) for index in range(9)]
    values.extend(str(index) for index in range(9, field_count))
    return ",".join(values)


GAIN_NAMES = (
    "Kp_Angle_Roll",
    "Kp_Angle_Pitch",
    "Kp_Angle_Yaw",
    "Kp_Rate_Roll",
    "Kp_Rate_Pitch",
    "Kp_Rate_Yaw",
    "Ki_Rate_Roll",
    "Ki_Rate_Pitch",
    "Ki_Rate_Yaw",
    "Kd_Rate_Roll",
    "Kd_Rate_Pitch",
    "Kd_Rate_Yaw",
)


class TelemetryCompatibilityTest(unittest.TestCase):
    def test_10_field_packet_keeps_extended_values_unknown(self):
        sample = parse_telemetry_packet(packet(10))
        self.assertEqual(9, sample["Throttle"])
        self.assertIsNone(sample["Fault_RC"])
        self.assertEqual(len(CSV_FIELDS), len(sample_to_csv_row("00:00:00.000", sample)))

    def test_14_field_packet_populates_legacy_fault_and_rc_fields(self):
        sample = parse_telemetry_packet(packet(14))
        self.assertEqual(13, sample["RC_Dropped_Pkts"])
        self.assertIsNone(sample["Fault_IMU1"])

    def test_21_field_packet_leaves_armed_unknown(self):
        sample = parse_telemetry_packet(packet(21))
        self.assertEqual(20, sample["Calibration_OK"])
        self.assertIsNone(sample["Armed"])
        for name in (
            "Motor_M1",
            "Motor_M2",
            "Motor_M3",
            "Motor_M4",
            "PID_Loop_Hz",
            "TgtRate_Roll",
            "TgtRate_Pitch",
            "TgtRate_Yaw",
            "MagHeading",
            "Mag_X",
            "Mag_Y",
            "Mag_Z",
        ):
            self.assertIsNone(sample[name])
        self.assertEqual(35, len(TELEMETRY_FIELDS))
        self.assertEqual(36, len(CSV_FIELDS))

    def test_22_field_packet_populates_armed(self):
        sample = parse_telemetry_packet(packet(22))
        self.assertEqual(21, sample["Armed"])
        self.assertIsNone(sample["Motor_M1"])
        self.assertIsNone(sample["PID_Loop_Hz"])

    def test_30_field_packet_leaves_mag_heading_unknown(self):
        sample = parse_telemetry_packet(packet(30))
        self.assertEqual(29.0, sample["TgtRate_Yaw"])
        self.assertIsNone(sample["MagHeading"])

    def test_31_field_packet_appends_mag_heading(self):
        line = ",".join(
            (
                "1.25", "-2.50", "3.75",
                "4.50", "-5.25", "6.00",
                "0.100", "-0.200", "1.000",
                "1100", "0", "1", "123", "4", "0", "1", "0", "1", "0", "0", "1",
                "1",
                "1101", "1102", "1103", "1104", "998",
                "12.50", "-23.75", "0.00",
                "87.25",
            )
        )

        sample = parse_telemetry_packet(line)

        self.assertEqual(1, sample["Armed"])
        self.assertIs(type(sample["Armed"]), int)
        for name, expected in (
            ("Motor_M1", 1101),
            ("Motor_M2", 1102),
            ("Motor_M3", 1103),
            ("Motor_M4", 1104),
            ("PID_Loop_Hz", 998),
        ):
            self.assertEqual(expected, sample[name])
            self.assertIs(type(sample[name]), int)
        for name, expected in (
            ("TgtRate_Roll", 12.5),
            ("TgtRate_Pitch", -23.75),
            ("TgtRate_Yaw", 0.0),
            ("MagHeading", 87.25),
        ):
            self.assertEqual(expected, sample[name])
            self.assertIs(type(sample[name]), float)
        for name in ("Mag_X", "Mag_Y", "Mag_Z"):
            self.assertIsNone(sample[name])

    def test_34_field_packet_appends_compensated_mag_xyz(self):
        line = ",".join(
            (
                "1.25", "-2.50", "3.75",
                "4.50", "-5.25", "6.00",
                "0.100", "-0.200", "1.000",
                "1100", "0", "1", "123", "4", "0", "1", "0", "1", "0", "0", "1",
                "1",
                "1101", "1102", "1103", "1104", "998",
                "12.50", "-23.75", "0.00",
                "87.25",
                "21.50", "-7.25", "42.00",
            )
        )

        sample = parse_telemetry_packet(line)

        for name, expected in (
            ("MagHeading", 87.25),
            ("Mag_X", 21.5),
            ("Mag_Y", -7.25),
            ("Mag_Z", 42.0),
        ):
            self.assertEqual(expected, sample[name])
            self.assertIs(type(sample[name]), float)
        row = sample_to_csv_row("00:00:00.000", sample)
        self.assertEqual(
            ["87.25", "21.5", "-7.25", "42.0", ""],
            [str(value) for value in row[-5:]],
        )

    def test_35_field_packet_parses_yaw_hold(self):
        packet = ",".join(["1"] * 34 + ["1"])
        sample = telemetry_schema.parse_telemetry_packet(packet)
        self.assertEqual(sample["Yaw_Hold"], 1)

    def test_34_field_packet_leaves_yaw_hold_none(self):
        packet = ",".join(["1"] * 34)
        sample = telemetry_schema.parse_telemetry_packet(packet)
        self.assertIsNone(sample["Yaw_Hold"])

    def test_csv_has_36_columns(self):
        self.assertEqual(len(telemetry_schema.CSV_FIELDS), 36)

    def test_explicit_type_map_covers_new_field_types(self):
        for name in ("Armed", "Motor_M1", "Motor_M2", "Motor_M3", "Motor_M4", "PID_Loop_Hz"):
            self.assertIs(telemetry_schema.TELEMETRY_FIELD_TYPES[name], int)
        for name in (
            "TgtRate_Roll",
            "TgtRate_Pitch",
            "TgtRate_Yaw",
            "MagHeading",
            "Mag_X",
            "Mag_Y",
            "Mag_Z",
        ):
            self.assertIs(telemetry_schema.TELEMETRY_FIELD_TYPES[name], float)

    def test_short_packet_is_rejected(self):
        with self.assertRaises(ValueError):
            parse_telemetry_packet(packet(9))

    def test_empty_required_field_is_rejected(self):
        # 필수 필드가 빈 문자열이면 None이 수신 도구의 포맷/연산을 죽이므로
        # 파서 단계에서 거부해야 한다.
        with self.assertRaises(ValueError):
            parse_telemetry_packet("1.0,2.0,3.0,4,5,6,7,8,9,")
        with self.assertRaises(ValueError):
            parse_telemetry_packet(",2.0,3.0,4,5,6,7,8,9,10")

    def test_empty_extended_field_stays_unknown(self):
        sample = parse_telemetry_packet(packet(10) + ",,1")
        self.assertIsNone(sample["Fault_RC"])
        self.assertEqual(1, sample["Fault_Critical"])


class GainsPacketTest(unittest.TestCase):
    def test_gains_packet_detection(self):
        self.assertTrue(telemetry_schema.is_gains_packet("  GAINS,1.0  "))
        self.assertFalse(telemetry_schema.is_gains_packet("GAINS"))
        self.assertFalse(telemetry_schema.is_gains_packet(packet(10)))

    def test_gains_packet_parses_all_named_values_as_floats(self):
        line = "GAINS," + ",".join(str(index / 10) for index in range(1, 13))

        gains = telemetry_schema.parse_gains_packet(line)

        self.assertEqual(list(GAIN_NAMES), list(gains))
        self.assertEqual(0.1, gains["Kp_Angle_Roll"])
        self.assertEqual(1.2, gains["Kd_Rate_Yaw"])
        self.assertTrue(all(type(value) is float for value in gains.values()))

    def test_gains_packet_rejects_wrong_field_count(self):
        with self.assertRaises(ValueError):
            telemetry_schema.parse_gains_packet("GAINS,1.0,2.0")


if __name__ == "__main__":
    unittest.main()
