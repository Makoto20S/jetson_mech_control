"""In-memory adversarial checks of independently verified replay submissions."""
import copy
import unittest
import audit

def fixture():
    events=[]
    for i in range(2500):
        events.extend([[i*2000000+100000,0,0],[i*2000000+120000,0,1]])
        if i%10==0:
            events.extend([[i*2000000+1000000,1,0],[i*2000000+1020000,1,1]])
    r={'profile':'fixed_forward__paired','events':events,'duration_ns':5000000000,
       'max_lateness_ns':250000,'read_guard_ns':200000,'timestamp_basis':'host_schedule_not_CAN_wire_time'}
    plan={'replay':r,'seconds':5,'command_hz':500,'feedback_hz':50,'feedback_order':'forward',
          'feedback_phase_ms':0,'rx_gate_ms':0,'a_first_writes_hex':audit.COMMAND_PACKETS_HEX,
          'b_first_writes_hex':audit.FEEDBACK_PACKETS_HEX}
    summary={'epoch_ns':10000000000,'collection_complete':True}
    scans=[]
    for port in (0,1):
        writes=[]
        for t,p,l in events:
            if p!=port:continue
            ident=(0x2968 if p else 0x668)+l
            writes.append({'id':ident,'payload':(audit.FEEDBACK if p else audit.COMMANDS)[ident],
                           'flags':12,'result':0,'begin_ns':summary['epoch_ns']+t,'end_ns':summary['epoch_ns']+t+1000})
        scans.append({'_frames':{'TX':writes}})
        summary['b' if port else 'a']={'max_lateness_ns':0,'min_send_interval_ns':20000}
    return plan,summary,*scans,copy.deepcopy(r)

class ReplayAudit(unittest.TestCase):
    def verify(self,args):
        f=audit.Findings();audit.verify_replay(*args,f);return [e['category'] for e in f.errors]
    def test_mixed_route(self):
        args=fixture();plan,summary,a,b,_=args;plan['mixed_receive']=True
        a['_frames']['TX']=sorted(a['_frames']['TX']+b['_frames']['TX'],key=lambda f:f['begin_ns']);b['_frames']['TX']=[]
        summary['b']['min_send_interval_ns']=0
        self.assertEqual(self.verify(args),[])
        b['_frames']['TX'].append(a['_frames']['TX'].pop())
        self.assertIn('replay_tx_count',self.verify(args))
    def test_sequence_duplicate_and_missing_cancel_counts(self):
        def scan(frames,direction):return {'_frames':{'TX':frames if direction=='TX' else [],'RX':frames if direction=='RX' else []}}
        frames=[{'id':0x2969,'payload':audit.feedback_payload(0x2969,n)} for n in (0,1)]
        a=scan(frames,'TX');b=scan([frames[0],frames[0]],'RX')
        r=audit.sequence_report(a,b);self.assertFalse(r['content_match']);self.assertEqual(len(r['deviations']),2)
    def test_sequence_tags_all500(self):
        tags=set()
        for i in audit.FEEDBACK:
            for n in range(250):
                payload=audit.feedback_payload(i,n);self.assertEqual(audit.feedback_ordinal({'id':i,'payload':payload}),n);tags.add(payload[2:4])
        self.assertEqual(len(tags),500)
    def test_wrong_tag_lane(self):self.assertIsNone(audit.feedback_ordinal({'id':0x2968,'payload':audit.feedback_payload(0x2969,0)}))
    def test_sequence_payload_replay(self):
        args=fixture();plan,summary,a,b,_=args;plan['mixed_receive']=True;plan['feedback_sequence']=True
        for lane,i in enumerate(audit.FEEDBACK):
            for n,f in enumerate(x for x in b['_frames']['TX'] if x['id']==i):f['payload']=audit.feedback_payload(i,n)
        a['_frames']['TX']=sorted(a['_frames']['TX']+b['_frames']['TX'],key=lambda f:f['begin_ns']);b['_frames']['TX']=[];summary['b']['min_send_interval_ns']=0
        self.assertEqual(self.verify(args),[])
        next(x for x in a['_frames']['TX'] if x['id'] in audit.FEEDBACK)['payload']=audit.FEEDBACK[0x2968]
        self.assertIn('replay_actual_tx_differs',self.verify(args))
    def test_valid(self):self.assertEqual(self.verify(fixture()),[])
    def test_external_recipe_required(self):
        args=list(fixture());args[-1]=None
        self.assertIn('replay_external_recipe_mismatch',self.verify(args))
    def test_wrong_payload(self):
        args=fixture();args[2]['_frames']['TX'][0]['payload']=audit.COMMANDS[0x669]
        self.assertIn('replay_actual_tx_differs',self.verify(args))
    def test_late_even_if_report_claims_good(self):
        args=fixture();args[2]['_frames']['TX'][0]['begin_ns']+=250001
        self.assertIn('replay_submission_lateness',self.verify(args))
    def test_early(self):
        args=fixture();args[2]['_frames']['TX'][0]['begin_ns']-=1
        self.assertIn('replay_submission_lateness',self.verify(args))
    def test_missing_event(self):
        args=fixture();args[3]['_frames']['TX'].pop()
        self.assertIn('replay_tx_count',self.verify(args))
    def test_cross_port_order(self):
        args=fixture();args[3]['_frames']['TX'][0]['begin_ns']=args[2]['_frames']['TX'][0]['begin_ns']-1
        self.assertIn('replay_global_submission_order',self.verify(args))
    def test_summary_lateness_not_trusted(self):
        args=fixture();args[1]['a']['max_lateness_ns']=99
        self.assertIn('replay_timing_summary',self.verify(args))

if __name__=='__main__':unittest.main()
