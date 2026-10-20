"""Regression coverage for portable checks; never executes repository tools."""
import os
from pathlib import Path
import shutil
import tempfile
import unittest

from check_docs import anchors, check as check_docs
from check_script_syntax import check as check_syntax


class DocsTests(unittest.TestCase):
    def run_check(self, source, targets=None):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            sources = {"README.md": source, **(targets or {})}
            for name, content in sources.items():
                path = root / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(content, encoding="utf-8")
            return check_docs(root, list(sources))

    def test_same_file_bad_anchor_is_rejected(self):
        count, errors = self.run_check("# Usage\n[go](#missing)")
        self.assertEqual(count, 1)
        self.assertEqual(len(errors), 1)
        self.assertIn("missing heading", errors[0])

    def test_unicode_duplicates_and_explicit_anchor(self):
        text = '# CAN ID 与范围\n# CAN ID 与范围\n<a id="custom"></a>\n'
        text += '[a](#can-id-与范围) [b](#can-id-与范围-1) [c](#custom)'
        self.assertEqual(self.run_check(text)[1], [])

    def test_url_encoded_path_fragment_title_and_parentheses(self):
        source = '[guide](docs/使用%20说明(v2).md#%E8%B0%83%E5%8F%82 "title")'
        self.assertEqual(self.run_check(source, {"docs/使用 说明(v2).md": "# 调参"})[1], [])

    def test_missing_files_case_and_escape_are_rejected(self):
        _, errors = self.run_check('[a](missing.md) [b](guide.md) [c](../outside.md)',
                                   {"Guide.md": "# Guide"})
        self.assertEqual(len(errors), 3)

    def test_reference_and_image_links_are_checked(self):
        _, errors = self.run_check('[x][guide]\n![plot](absent.png)\n[guide]: missing.md')
        self.assertEqual(len(errors), 2)

    def test_code_external_and_evidence_labels_are_ignored(self):
        count, errors = self.run_check('```md\n[x](absent.md)\n```\n'
                                       '`[x](absent.md)` [E01][E02] '
                                       '[web](https://example.invalid/absent)')
        self.assertEqual((count, errors), (0, []))

    def test_heading_inside_code_is_not_an_anchor(self):
        self.assertNotIn("fake", anchors("```\n# Fake\n```"))

    def test_ignored_local_file_is_not_a_portable_target(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "README.md").write_text('[x](private.md)', encoding="utf-8")
            (root / "private.md").write_text('private', encoding="utf-8")
            self.assertEqual(len(check_docs(root, ["README.md"])[1]), 1)


class SyntaxTests(unittest.TestCase):
    def test_python_errors_detected_without_execution(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "bad.py").write_text("def broken(:\n", encoding="utf-8")
            (root / "good.py").write_text("raise RuntimeError('must not execute')\n", encoding="utf-8")
            counts, errors = check_syntax(root, ["bad.py", "good.py"])
            self.assertEqual(counts["Python"], 2)
            self.assertEqual(len(errors), 1)
            self.assertIn("bad.py", errors[0])

    def test_bash_errors_detected_without_execution(self):
        bash = os.environ.get("MECH_TEST_BASH") or shutil.which("bash")
        if not bash:
            self.skipTest("Bash is unavailable on this host")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "bad.sh").write_text("if true; then\n", encoding="utf-8")
            (root / "good.sh").write_text("exit 77\n", encoding="utf-8")
            counts, errors = check_syntax(root, ["bad.sh", "good.sh"], bash)
            self.assertEqual(counts["Bash"], 2)
            self.assertEqual(len(errors), 1)
            self.assertIn("bad.sh", errors[0])


if __name__ == "__main__":
    unittest.main()
