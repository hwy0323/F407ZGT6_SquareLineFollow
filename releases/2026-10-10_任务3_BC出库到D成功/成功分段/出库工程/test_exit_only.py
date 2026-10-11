"""Static source/route checks; no hardware access."""
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).parent
SOURCE = (ROOT / "main.c").read_text(encoding="utf-8")


class ExitOnlyTests(unittest.TestCase):
    def test_same_five_exit_steps(self):
        route = (ROOT / "route_data.h").read_text(encoding="utf-8")
        self.assertEqual(re.findall(r"\{'([FR])',\s*(\d+)U,", route),
                         [('F','100'),('R','50'),('F','180'),('R','60'),('F','200')])

    def test_no_auto_c_phase(self):
        self.assertNotIn("auto_phase = 2U;", SOURCE)
        self.assertNotIn("auto_boot_decided", SOURCE)
        self.assertIn("BOOT_STOPPED", SOURCE)

    def test_route_completion_stops_and_enables_manual(self):
        start = SOURCE.index("static void route_after_step(const char *reason)\n{")
        stop = SOURCE.index("static void route_start_step", start)
        body = SOURCE[start:stop]
        complete = body[body.index("    route_active = route_waiting = 0U;"):]
        self.assertIn("stop_motors();", complete)
        self.assertIn("state = STOPPED;", complete)
        self.assertIn("auto_phase = 4U;", complete)
        self.assertIn("manual_commands_enabled = 1U;", complete)
        self.assertIn("EVENT,EXIT_ONLY_DONE,STOPPED", complete)

    def test_manual_gated_before_exit_and_busy(self):
        body = SOURCE[SOURCE.index("static void pivot_read_dap"):]
        self.assertIn("if (!manual_commands_enabled)", body)
        self.assertIn("else if (pivot_active || route_active || state != STOPPED)", body)
        self.assertIn("manual_commands_enabled = 0U;", SOURCE)
        self.assertIn("auto_phase == 3U || pivot_active", SOURCE)


if __name__ == "__main__":
    unittest.main(verbosity=2)
