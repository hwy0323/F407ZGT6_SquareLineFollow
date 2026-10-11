"""Offline source checks only; never open a serial port or move the car."""
import re
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
SOURCE = (HERE / "main.c").read_text(encoding="utf-8")
ROUTE = (HERE / "route_data.h").read_text(encoding="utf-8")


class MergedRouteTests(unittest.TestCase):
    def test_exact_route(self):
        steps = re.findall(r"\{'([FRBL])',\s*(\d+)U,", ROUTE)
        self.assertEqual(steps, [
            ("F", "100"), ("R", "50"), ("F", "180"), ("R", "60"),
            ("F", "200"), ("F", "190"), ("B", "37"), ("R", "68"),
        ])

    def test_final_step_arms_handoff_after_stop(self):
        start = SOURCE.index("static void route_after_step(const char *reason)\n{")
        end = SOURCE.index("static void route_start_step", start)
        body = SOURCE[start:end]
        self.assertIn("stop_motors();", body)
        self.assertIn("auto_phase = 2U;", body)
        self.assertIn("auto_pause_start_ms = HAL_GetTick();", body)
        self.assertIn("manual_commands_enabled = 0U;", body)
        self.assertNotIn("EXIT_ONLY_DONE", body)

    def test_handoff_no_old_c_pivot_and_no_retry(self):
        start = SOURCE.index("if (auto_phase == 2U &&")
        end = SOURCE.index("if (auto_phase == 3U", start)
        body = SOURCE[start:end]
        self.assertIn("auto_phase = 4U;", body)
        self.assertIn("begin_cd_follow(now);", body)
        self.assertNotIn("begin_post_exit_follow(now);", body)

    def test_cd_preflight_and_terminal_stop(self):
        for text in ("CD_near_center_required", "imu_missing",
                     "D_REACHED_FROM_BC_GARAGE", "CD_HEADING_LIMIT",
                     "ENCODER_STALL", "GIT_LINE_LOST"):
            self.assertIn(text, SOURCE)
        self.assertIn("turns = 1U;", SOURCE)
        self.assertIn("BOOT_STOPPED", SOURCE)
        self.assertNotIn("auto_boot_decided", SOURCE)


if __name__ == "__main__":
    unittest.main(verbosity=2)
