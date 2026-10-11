"""Source constraints for CD-only takeover; no hardware access."""
import unittest
from pathlib import Path

SOURCE = (Path(__file__).parent / "main.c").read_text(encoding="utf-8")


class CDFollowTests(unittest.TestCase):
    def test_explicit_entry_without_exit_or_pivot(self):
        start = SOURCE.index("static void begin_cd_follow(uint32_t now)\n{")
        end = SOURCE.index("static void start_from_garage", start)
        body = SOURCE[start:end]
        for required in ("reset_encoders();", "turns = 1U;",
                         "state = FOLLOW;", "auto_phase = 1U;",
                         "post_exit_tracking = 0U;", "cd_follow_only = 1U;"):
            self.assertIn(required, body)
        self.assertNotIn("route_begin(", body)
        self.assertNotIn("state = PIVOT;", body)
        self.assertIn("c_target_yaw_x100 = imu_yaw_x100;", body)

    def test_explicit_serial_command_and_idle_boot(self):
        self.assertIn("command == 'T' || command == 't'", SOURCE)
        self.assertIn("begin_cd_follow(now);", SOURCE)
        self.assertNotIn("auto_boot_decided", SOURCE)
        self.assertIn("BOOT_STOPPED", SOURCE)

    def test_d_does_not_use_old_origin_window_as_gate(self):
        self.assertIn("if (cd_follow_only) return progress >= 200L;", SOURCE)
        self.assertIn("if (cd_follow_only) return 0U;", SOURCE)
        self.assertIn("cd_follow_only = 0U;", SOURCE)

    def test_stop_before_d_rotation(self):
        start = SOURCE.index("if (turns + 1U == CORNER_TARGET_COUNT)")
        end = SOURCE.index("state = ENTER_CORNER;", start)
        body = SOURCE[start:end]
        self.assertIn("stop_motors();", body)
        self.assertIn('stop_run("D_REACHED_FROM_BC_GARAGE");', body)
        self.assertIn("return;", body)

    def test_protection_kept(self):
        for required in ("now - run_start_ms > 20000U",
                         'stop_run("CD_HEADING_LIMIT")',
                         'stop_run("GIT_LINE_LOST")',
                         'stop_run("ENCODER_STALL")'):
            self.assertIn(required, SOURCE)


if __name__ == "__main__":
    unittest.main(verbosity=2)
