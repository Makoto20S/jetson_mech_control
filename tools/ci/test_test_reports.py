"""Actual skips, partial reports and failures must remain visible in artifacts."""
from pathlib import Path
import tempfile
import unittest

from summarize_test_reports import summarize


class ReportTests(unittest.TestCase):
    def exercise(self, xml, packages=('example',)):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            if xml is not None:
                path = root / 'example/test_results/example/report.xml'
                path.parent.mkdir(parents=True)
                path.write_text(xml)
            return summarize(root, packages)

    def test_gtest_attribute_skip_and_xml_skip_are_counted_once(self):
        report = self.exercise('<testsuite><testcase name="attribute" result="skipped"/>'
                               '<testcase name="child"><skipped/></testcase>'
                               '<testcase name="both" result="skipped"><skipped/></testcase></testsuite>')
        self.assertEqual(report['testcases'], 3)
        self.assertEqual(len(report['skipped']), 3)
        self.assertTrue(report['all_package_reports_available'])

    def test_failures_and_errors_preserved(self):
        report = self.exercise('<testsuite><testcase name="fail"><failure/></testcase>'
                               '<testcase name="error"><error/></testcase></testsuite>')
        self.assertEqual(len(report['failures']), 2)

    def test_partial_reports_do_not_imply_completion(self):
        report = self.exercise('<testsuite><testcase name="pass"/></testsuite>', ('example', 'missing'))
        self.assertFalse(report['all_package_reports_available'])
        self.assertEqual(report['missing_packages'], ['missing'])

    def test_no_reports_do_not_imply_success(self):
        report = self.exercise(None)
        self.assertFalse(report['all_package_reports_available'])
        self.assertEqual(report['testcases'], 0)

    def test_malformed_report_recorded_without_losing_collection(self):
        report = self.exercise('<testsuite>')
        self.assertTrue(report['parse_errors'])
        self.assertFalse(report['all_package_reports_available'])

    def test_disabled_case_is_visible_as_not_run(self):
        report = self.exercise('<testsuite><testcase name="disabled" status="notrun" result="suppressed"/></testsuite>')
        self.assertEqual(len(report['not_run']), 1)
        self.assertEqual(report['skipped'], [])


if __name__ == '__main__':
    unittest.main()
