"""Strict inert CLI and rejected admission tests; never opens real devices."""
import json,os,pathlib,subprocess,sys,tempfile,unittest
EXE=pathlib.Path(sys.argv.pop(1)).resolve() if len(sys.argv)>1 else None
class Cli(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(prefix='duplex-cli-');self.root=pathlib.Path(self.temp.name);self.out=self.root/'must-not-exist'
    def tearDown(self):self.temp.cleanup()
    def call(self,*args,code=0):
        r=subprocess.run([str(EXE),*map(str,args)],cwd=self.root,capture_output=True,text=True,timeout=5)
        self.assertEqual(r.returncode,code,(args,r.stdout,r.stderr));self.assertFalse(self.out.exists());return r
    def reject(self,*args):
        r=self.call(*args,code=2);self.assertIn('error:',r.stderr);self.assertEqual(r.stdout,'');return r
    def test_default_inert(self):
        p=json.loads(self.call().stdout);self.assertFalse(p['run_requested']);self.assertEqual(list(self.root.iterdir()),[])
    def test_dry_paths_inert(self):
        json.loads(self.call('--port-a',self.root/'a','--port-b',self.root/'b','--output',self.out).stdout)
        self.assertEqual(list(self.root.iterdir()),[])
    def test_duplicates_unknown_missing(self):
        self.assertIn('duplex',self.call('--help').stdout)
        for flag in ('--run','--help','--motors-disconnected'):self.reject(flag,flag)
        for flag,value in (('--port-a','a'),('--port-b','b'),('--command-hz','1'),('--feedback-hz','0'),('--feedback-order','forward'),('--feedback-phase-ms','0'),('--rx-gate-ms','0'),('--seconds','1'),('--nonce','1')):self.reject(flag,value,flag,value)
        for args in (('--unknown',),('foo',),('--port-a',),('--seconds','--help'),('--output',''),('--profile','mode6'),('--packing','joined'),('--count-only',)):self.reject(*args)
    def test_numeric_strict(self):
        for flag in ('--command-hz','--feedback-hz','--feedback-phase-ms','--rx-gate-ms','--seconds','--nonce','--log-mib','--drain-ms'):
            for value in ('','-1','+1','0x1','1.0','1e2',' 1','1 ','4294967296','999999999999999999999'):self.reject(flag,value)
    def test_ranges(self):
        for flag,values in (('--command-hz',('0','501')),('--feedback-hz',('51',)),('--seconds',('0','61')),('--log-mib',('0','65')),('--drain-ms',('0','999','5001'))):
            for value in values:self.reject(flag,value)
        for args in (('--command-hz','1','--feedback-hz','0','--seconds','1','--log-mib','1','--nonce','0'),('--command-hz','500','--feedback-hz','50','--seconds','60','--log-mib','64','--nonce','4294967295'),('--command-hz','10','--feedback-hz','10','--seconds','2')):json.loads(self.call(*args).stdout)
    def test_admission_without_open(self):
        self.assertIn('motors-disconnected',self.reject('--run').stderr)
        self.reject('--run','--motors-disconnected')
        self.reject('--run','--motors-disconnected','--port-a','a','--port-b','a','--output',self.out)
        self.reject('--run','--motors-disconnected','--port-a',self.root/'absent-a','--port-b',self.root/'absent-b','--output',self.out)
        self.assertEqual(list(self.root.iterdir()),[])
    def test_feedback_order_inert(self):
        self.assertEqual(json.loads(self.call().stdout)['feedback_order'],'forward')
        for order in ('forward','reverse'):
            plan=json.loads(self.call('--feedback-order',order,'--seconds','2','--command-hz','10','--feedback-hz','10').stdout)
            self.assertEqual(plan['feedback_order'],order)
            ids=[int.from_bytes(bytes.fromhex(x)[7:11],'little') for x in plan['b_first_writes_hex']]
            self.assertEqual(ids,[0x2968,0x2969] if order=='forward' else [0x2969,0x2968])
        for order in ('Forward','reversed','0','reverse ',''):self.reject('--feedback-order',order)
    def test_feedback_phase_inert(self):
        self.assertEqual(json.loads(self.call().stdout)['feedback_phase_ms'],0)
        for phase in (0,5):
            plan=json.loads(self.call('--feedback-phase-ms',phase,'--seconds','2','--command-hz','10','--feedback-hz','10').stdout)
            self.assertEqual(plan['feedback_phase_ms'],phase)
            self.assertEqual(plan['feedback_order'],'forward')
        for phase in (1,4,6,100):self.reject('--feedback-phase-ms',phase)
        self.reject('--feedback-phase-ms','5','--feedback-hz','0')
    def test_rx_gate_inert(self):
        self.assertEqual(json.loads(self.call().stdout)['rx_gate_ms'],0)
        base=['--command-hz','10','--feedback-hz','10','--seconds','2','--feedback-order','forward']
        for phase in (0,5):
            p=json.loads(self.call(*base,'--feedback-phase-ms',phase,'--rx-gate-ms','6').stdout)
            self.assertEqual(p['rx_gate_ms'],6)
            self.assertEqual(p['rx_gate_anchor'],'A_nominal_tick')
            self.assertEqual(p['rx_gate_scope'],'transmit_only_both_ports')
        for gate in (1,5,7,100):self.reject(*base,'--rx-gate-ms',gate)
        for flag,value in [('--command-hz','11'),('--feedback-hz','0'),('--feedback-hz','11'),('--seconds','1'),('--seconds','3'),('--feedback-order','reverse')]:
            args=base.copy();args[args.index(flag)+1]=value
            self.reject(*args,'--rx-gate-ms','6')
    @unittest.skipUnless(sys.platform.startswith('linux'),'Linux PTY admission')
    def test_pty_untouched(self):
        m1,s1=os.openpty();m2,s2=os.openpty()
        try:
            self.reject('--run','--motors-disconnected','--port-a',os.ttyname(s1),'--port-b',os.ttyname(s2),'--output',self.out)
            for master in (m1,m2):
                os.set_blocking(master,False)
                with self.assertRaises(BlockingIOError):os.read(master,1024)
        finally:
            for fd in (m1,s1,m2,s2):os.close(fd)
if __name__=='__main__':
    if EXE is None or not EXE.is_file():raise SystemExit('usage: test_cli.py CLI_EXECUTABLE')
    unittest.main()
