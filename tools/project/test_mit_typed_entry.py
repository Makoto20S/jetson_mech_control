import unittest,contextlib,io
from test_ref01_mit_ros import parse_args
class Entry(unittest.TestCase):
 def test_default_legacy(self):
  self.assertFalse(parse_args(['--output','unused']).typed)
 def test_explicit_typed(self):
  a=parse_args(['--output','unused','--typed','--duration','6'])
  self.assertTrue(a.typed);self.assertFalse(a.predictive);self.assertEqual(len(a.cases),7)
 def test_prediction_not_allowed(self):
  with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
   parse_args(['--output','unused','--typed','--predictive'])
 def test_duration(self):
  for value in ('nan','inf','2','31'):
   with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
    parse_args(['--output','unused','--duration',value])
if __name__=='__main__':unittest.main()
