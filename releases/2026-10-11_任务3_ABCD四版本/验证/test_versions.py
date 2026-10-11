"""Offline release-package checks; these do not validate the car on track."""
import hashlib
import unittest
from pathlib import Path

RELEASE = Path(__file__).resolve().parents[1]


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


class Task3ReleaseTests(unittest.TestCase):
    def test_published_firmware_hashes_match_known_artifacts(self):
        known = {
            "A/固件/F407ZGT6_Task3_ToA.bin":
                "B20D6F68A7D4366E854820C0BB45216FBC7B5DD684F98051DA8CDE5A590F4FAA",
            "C/固件/F407ZGT6_Task3_ToC_历史现场版.bin":
                "08073FC13A970A2CADD3E07EE387543399D05A8857E446AFAA7AF8926FADC999",
            "D/固件/F407ZGT6_Task3_ToD.bin":
                "058B8EFD7464C175416D1DAF984460FC5B3E2CB22088560F8798FDB58A3E5227",
        }
        for relative, expected in known.items():
            with self.subTest(version=relative):
                self.assertEqual(sha256(RELEASE / relative), expected)
        self.assertTrue((RELEASE / "B/固件/F407ZGT6_Task3_ToB_UNTESTED.bin").is_file())
        manifest = {}
        for line in (RELEASE / "SHA256SUMS.txt").read_text(encoding="utf-8").splitlines():
            digest, relative = line.split(maxsplit=1)
            manifest[relative] = digest.upper()
        self.assertEqual(set(manifest), {
            "A/固件/F407ZGT6_Task3_ToA.bin",
            "B/固件/F407ZGT6_Task3_ToB_UNTESTED.bin",
            "C/固件/F407ZGT6_Task3_ToC_历史现场版.bin",
            "D/固件/F407ZGT6_Task3_ToD.bin",
        })
        for relative, digest in manifest.items():
            self.assertEqual(sha256(RELEASE / relative), digest)

    def test_all_four_have_source_firmware_and_scope_docs(self):
        for version in "ABCD":
            with self.subTest(version=version):
                self.assertTrue((RELEASE / f"{version}/工程/main.c").is_file())
                self.assertTrue(any((RELEASE / f"{version}/固件").glob("*.bin")))
                self.assertTrue((RELEASE / f"{version}/README.md").is_file())
        for version in "ACD":
            self.assertTrue(any((RELEASE / f"{version}/串口记录").glob("*.csv")))

    def test_B_has_three_post_C_turns_and_explicit_test_pending_marker(self):
        source = (RELEASE / "B/工程/main.c").read_text(encoding="utf-8")
        readme = (RELEASE / "B/README.md").read_text(encoding="utf-8")
        self.assertIn("#define CORNER_TARGET_COUNT 4U", source)
        self.assertIn('"B_REACHED_FROM_BC_GARAGE"', source)
        self.assertIn('"TASK3_SQUARE_C_MULTI_TO_B_UNTESTED"', source)
        self.assertIn("turns == 3U", source)
        self.assertIn("尚未烧录", readme)
        self.assertIn("5100±900", readme)

    def test_A_and_D_labels_are_distinct_from_B_test_pending(self):
        a = (RELEASE / "A/工程/main.c").read_text(encoding="utf-8")
        d = (RELEASE / "D/工程/main.c").read_text(encoding="utf-8")
        self.assertIn('"TASK3_SQUARE_C_MULTI_TO_A"', a)
        self.assertIn("TASK3_MERGED68_OFFSET_TO_D", d)
        self.assertIn("复测结束后确认", (RELEASE / "A/README.md").read_text(encoding="utf-8"))
        self.assertIn("现场完美", (RELEASE / "D/README.md").read_text(encoding="utf-8"))
        self.assertIn("LINE_LOST", (RELEASE / "C/README.md").read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main(verbosity=2)
