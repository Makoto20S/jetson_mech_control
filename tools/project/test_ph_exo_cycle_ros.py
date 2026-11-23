"""Formal upper reference integration; only MuJoCo/private PTY, never devices."""
import argparse,bisect,json,math,os,signal,time
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from ref01_mit_fixture import MitFixture
from ref01_dynamic_mit import current_scale
from ref01_bench_hold import configuration

ROOT=Path(__file__).resolve().parents[2]

class CycleFixture(MitFixture):
    def __init__(self):
        scale,_=current_scale()
        super().__init__(feedback_period_s=.005,acknowledge_disable=True,erpm_scale=scale,pulse_nm=0.)
        self.cap=10.
    def configuration(self,folder,case,duration,namespace):
        cfg=json.loads((ROOT/'config/parallel_exo/servo_pair.ref01.json').read_text())
        positions,_=self.feedback_words()
        params,_=configuration(cfg,{p['mid']:p['path'] for p in self.ports},
            {104:positions[0]*.1,105:positions[1]*.1},folder,namespace,True,
            manual_stop=True,dynamic=True,limits=dict(torque_nm=10.,current_a=6.,travel_deg=90.))
        xml=params[namespace+'/controller_manager']['ros__parameters']['robot_description']
        tree=ET.fromstring(xml)
        for control in tree.findall('ros2_control'):
            mid=int(control.find("hardware/param[@name='drive_id']").text)
            control.set('name','mit_comm_'+str(mid))
            ET.SubElement(control.find('hardware'),'param',name='command_trace_path').text=str(folder/('motor'+str(mid)+'.jsonl'))
        xml=ET.tostring(tree,encoding='unicode')
        params[namespace+'/controller_manager']['ros__parameters']['robot_description']=xml
        return xml,params

def candidate():
    return dict(k=[20.,20.],b=[.3,.3],tau=[10.,10.],ref='cycle',
                target=False,q=[0.,math.pi/9],amp=[math.radians(3),math.radians(-2)],
                period=[6.,4.],move=3.)

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--cases',nargs='+',choices=['cycle','mit_cycle','mit_drop','mit_loss'],
                   default=['cycle','mit_cycle','mit_drop','mit_loss'])
    a=p.parse_args();out=a.output.resolve()
    if os.environ.get('EXO_SYNC_LOCK_HELD')!='1' or os.environ.get('ROS_DOMAIN_ID')!='183' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise RuntimeError('isolated domain and sync lock required')
    if not out.is_relative_to(ROOT/'.local/validation') or any(p.is_symlink() for p in (out,*out.parents)):raise ValueError('project output required')
    out.mkdir(parents=True,exist_ok=False)
    os.environ['ROS_LOG_DIR']=str(out/'ros_logs')
    import test_ph_exo_upper_ros as harness
    from ph_exo_interfaces.msg import ImpedanceRef,ImpedanceStatus
    from std_srvs.srv import Trigger
    from rosidl_runtime_py.convert import message_to_ordereddict
    harness.rclpy.init();results=[]
    try:
        for case in a.cases:
            def driver(g):
                adapter=g['upper'].ph_exo;node=g['node'];executor=g['executor'];fx=g['fixture']
                refs=[];statuses=[]
                # Tap the actual publish call without adding a second consumer:
                # the production route intentionally requires one controller.
                original=adapter.plan_pub
                class Tap:
                    def publish(self,m):
                        refs.append(message_to_ordereddict(m));original.publish(m)
                    def get_subscription_count(self):return original.get_subscription_count()
                adapter.plan_pub=Tap()
                node.create_subscription(ImpedanceStatus,g['ns']+'/ref01_impedance/status',lambda m:statuses.append(message_to_ordereddict(m)),1)
                def spin(predicate,timeout):
                    end=time.monotonic()+timeout
                    while time.monotonic()<end and not predicate():executor.spin_once(timeout_sec=.005)
                    assert predicate(),adapter.snapshot()
                def delay(seconds):
                    end=time.monotonic()+seconds;spin(lambda:time.monotonic()>=end,seconds+1)
                def call(name,success=True):
                    c=node.create_client(Trigger,g['ns']+'/exoskeleton/impedance/'+name)
                    assert c.wait_for_service(timeout_sec=3)
                    f=c.call_async(Trigger.Request());spin(f.done,4);r=f.result();node.destroy_client(c)
                    assert r.success==success,r.message
                try:
                    # Readiness of the separately spawned state node is part
                    # of fixture setup, not the production start timeout.
                    spin(lambda:node.count_publishers(g['ns']+'/ph_exo/state')==1,12)
                    delay(.2)
                    assert not refs and (not fx or not fx.tx),'configuration auto-started output'
                    call('start');spin(lambda:adapter.session.phase in ('running','stopped','unconfirmed'),8)
                    assert adapter.session.phase=='running',adapter.snapshot()
                    start=adapter.started_at
                    if case in ('cycle','mit_cycle'):
                        spin(lambda:time.monotonic()-start>=13.1 or adapter.session.phase!='running',15)
                        assert adapter.session.phase=='running',adapter.snapshot()
                        call('emergency_stop')
                    else:
                        spin(lambda:time.monotonic()-start>=3. or adapter.session.phase!='running',4)
                        assert adapter.session.phase=='running',adapter.snapshot()
                        cut=time.monotonic()
                        if case=='mit_drop':
                            # Simulated upper reference interruption; feedback continues.
                            with adapter.lock:adapter.renew=False
                            delay(.6)
                            faults=[s for s in statuses if s['observed_at_monotonic_s']>=cut and s['mode']==3]
                            assert faults and faults[0]['reason']=='plan_expired',faults
                            for mid in (104,105):
                                tx=[r['ns']/1e9 for r in fx.tx if r['drive']==mid]
                                assert tx and tx[-1]<=cut+.12,(mid,tx[-1]-cut)
                            call('stop')
                        else:
                            os.killpg(g['state'].pid,signal.SIGTERM);g['state'].wait(timeout=5)
                    spin(lambda:adapter.session.phase in ('stopped','unconfirmed'),8)
                    spin(lambda:not adapter.worker or not adapter.worker.is_alive(),3)
                    stopped=adapter.snapshot()
                    blocked=False
                    assert stopped['state']=='stopped' and stopped['stop_ok'],stopped
                    assert stopped['disable_ok']==([True,True] if fx else [False,False])
                    assert g['manager'].poll() is None
                    if fx:assert not fx.errors,fx.errors
                    call('reset',success=not blocked);before=len(fx.tx) if fx else 0;seq=adapter.session.sequence;delay(.2)
                    assert adapter.session.phase==('unconfirmed' if blocked else 'debug') and not adapter.renew
                    assert adapter.session.sequence==seq and (not fx or len(fx.tx)==before)
                    assert refs and all(r['k']==[20.,20.] and r['b']==[.3,.3] and r['tau']==[10.,10.] for r in refs)
                    assert all(b['seq']>a['seq'] and b['t']>a['t'] for a,b in zip(refs,refs[1:]))
                    data=dict(case=case,passed=True,stop=stopped,reference_count=len(refs),software_only=True,
                              ports=[p['path'] for p in fx.ports] if fx else [],manager_survived=True,reset_did_not_restart=True,reset_debug=not blocked,reset_blocked=blocked)
                    if case in ('cycle','mit_cycle'):
                        q=np.array([r['q'] for r in refs]);span=np.rad2deg(np.ptp(q,axis=0))
                        assert np.allclose(span,[6.,2.],atol=.005),span
                        data['reference_span_deg']=span.tolist()
                    if fx and fx.rows:
                        times=[r['t'] for r in refs];errors=[]
                        for row in fx.rows:
                            i=bisect.bisect_right(times,row['ns']/1e9)-1
                            if i>=0 and times[0]+1<=row['ns']/1e9<=times[-1]:errors.append(np.rad2deg(np.array([row['psi_rad'],row['beta_rad']])-refs[i]['q']))
                        data['rmse_deg']=np.sqrt(np.mean(np.array(errors)**2,axis=0)).tolist()
                        data['peak_nm']=[max(abs(r['torque_nm']) for r in fx.tx if r['drive']==mid) for mid in (104,105)]
                        if case=='mit_drop':data['last_tx_after_cut_ms']={str(mid):1000*(max(r['ns']/1e9 for r in fx.tx if r['drive']==mid)-cut) for mid in (104,105)}
                    if case=='mit_drop':
                        previous_id=stopped['id']
                        call('start');spin(lambda:adapter.session.phase in ('running','stopped','unconfirmed'),8)
                        assert adapter.session.phase=='running',adapter.snapshot()
                        delay(.3)
                        assert adapter.session.phase=='running',adapter.snapshot()
                        call('stop');spin(lambda:adapter.session.phase in ('stopped','unconfirmed'),8)
                        spin(lambda:not adapter.worker or not adapter.worker.is_alive(),3)
                        again=adapter.snapshot()
                        assert again['state']=='stopped' and again['stop_ok'] and all(again['disable_ok']),again
                        assert again['id']!=previous_id
                        call('reset');assert adapter.session.phase=='debug'
                        data['explicit_restart_passed']=True
                    (g['folder']/'cycle_result.json').write_text(json.dumps(data,indent=2))
                    return data
                finally:
                    (g['folder']/'references.json').write_text(json.dumps(refs))
                    (g['folder']/'statuses.json').write_text(json.dumps(statuses))
            r=harness.run_case(out,case,driver,reference_config=candidate(),
                               fixture_factory=CycleFixture if case.startswith('mit_') else None)
            results.append(r);print(json.dumps(r),flush=True)
    finally:
        harness.rclpy.shutdown()
        (out/'summary.json').write_text(json.dumps(dict(results=results,completed=len(results)==len(a.cases),hardware_started=False),indent=2))
if __name__=='__main__':main()
