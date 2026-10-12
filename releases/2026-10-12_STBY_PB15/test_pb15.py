"""Offline pin migration checks; no hardware access."""
import hashlib
import json
import re
import unittest
from pathlib import Path

RELEASE = Path(__file__).resolve().parent
ROOT = RELEASE.parents[1]
VERSIONS = json.loads((RELEASE / "versions.json").read_text(encoding="utf-8"))
MACROS = '''
/* Common TB6612 standby: high = enabled, low = stopped. */
#define MOTOR_STBY_PORT GPIOB
#define MOTOR_STBY_PIN GPIO_PIN_15
'''
OLD_INIT = '''  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  pin.Pin = GPIO_PIN_0 | GPIO_PIN_13;
  pin.Mode = GPIO_MODE_OUTPUT_PP;
  pin.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &pin);'''
NEW_INIT = '''  HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, GPIO_PIN_RESET);
  pin.Pin = MOTOR_STBY_PIN;
  pin.Mode = GPIO_MODE_OUTPUT_PP;
  pin.Pull = GPIO_PULLDOWN;
  pin.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTOR_STBY_PORT, &pin);
  pin.Pull = GPIO_NOPULL;
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  pin.Pin = GPIO_PIN_13;
  pin.Mode = GPIO_MODE_OUTPUT_PP;
  pin.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &pin);'''


class PinMigrationTests(unittest.TestCase):
    def test_only_stby_changed_in_driving_sources(self):
        for item in VERSIONS:
            if item["id"] == "oled":
                continue
            with self.subTest(version=item["id"]):
                old = (ROOT / item["base"] / "main.c").read_text(encoding="utf-8")
                new = (RELEASE / item["id"] / "main.c").read_text(encoding="utf-8")
                self.assertEqual(old.count(OLD_INIT), 1)
                expected = old.replace('#include "main.h"\n', '#include "main.h"\n' + MACROS, 1)
                expected = expected.replace(OLD_INIT, NEW_INIT, 1)
                expected = expected.replace("HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0,",
                                            "HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN,")
                self.assertEqual(new, expected)
                # These define route/encoder behavior; they must not change.
                for file in (RELEASE / item["id"]).glob("*"):
                    if file.suffix in (".c", ".h") and file.name != "main.c":
                        self.assertEqual(file.read_bytes(), (ROOT / item["base"] / file.name).read_bytes())

    def test_init_and_stop_for_all_versions(self):
        for item in VERSIONS:
            with self.subTest(version=item["id"]):
                source = (RELEASE / item["id"] / "main.c").read_text(encoding="utf-8")
                self.assertIn("#define MOTOR_STBY_PORT GPIOB", source)
                self.assertIn("#define MOTOR_STBY_PIN GPIO_PIN_15", source)
                self.assertNotIn("HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0,", source)
                self.assertNotIn("GPIO_PIN_0 | GPIO_PIN_13", source)
                self.assertIn("HAL_GPIO_Init(MOTOR_STBY_PORT, &pin);", source)
                self.assertIn("pin.Pull = GPIO_PULLDOWN;", source)
                self.assertIn("__HAL_RCC_GPIOB_CLK_ENABLE();", source)
                reset = "HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, GPIO_PIN_RESET);"
                self.assertGreaterEqual(source.count(reset), 2)
                self.assertLess(source.index("__HAL_RCC_GPIOB_CLK_ENABLE();",
                                            source.index("static void " + ("stationary_gpio_init" if item["id"] == "oled" else "gpio_init"))),
                                source.index("HAL_GPIO_Init(MOTOR_STBY_PORT, &pin);"))
                # PB15 should only appear via the STBY macro, not a peripheral assignment.
                self.assertEqual(source.count("#define MOTOR_STBY_PIN GPIO_PIN_15"), 1)

    def test_menu_copy_matches_active_root(self):
        self.assertEqual((RELEASE / "oled/main.c").read_text(encoding="utf-8"),
                         (ROOT / "main.c").read_text(encoding="utf-8"))

    def test_all_11_firmware_hashes_and_elf_exist(self):
        hashes = {}
        for line in (RELEASE / "SHA256SUMS.txt").read_text(encoding="utf-8-sig").splitlines():
            digest, name = line.split(maxsplit=1)
            hashes[name] = digest
        self.assertEqual(len(hashes), 11)
        for item in VERSIONS:
            name = "固件/" + item["name"] + "_STBY_PB15.bin"
            self.assertIn(name, hashes)
            path = RELEASE / name
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest().upper(), hashes[name])
            self.assertTrue(path.with_suffix(".elf").is_file())
            self.assertIn("-DSTM32F407xx", (RELEASE / "build_pb15.mk").read_text())

    def test_docs_link_to_existing_sources_and_firmware(self):
        for doc in ("README.md", "接线.md"):
            text = (RELEASE / doc).read_text(encoding="utf-8")
            for target in re.findall(r"\]\(([^)]+)\)", text):
                self.assertTrue((RELEASE / target).exists(), target)


if __name__ == "__main__":
    unittest.main(verbosity=2)
