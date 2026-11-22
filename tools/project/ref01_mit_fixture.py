"""Software-only MIT/CDC device emulation and quantized REF01 plant."""
import csv
import gc
import json
import math
import os
from pathlib import Path
import select
import struct
import sys
import threading
import time
import xml.etree.ElementTree as ET
import numpy as np
import benchmark_mit_pty as wire
import mit_communication as mit

ROOT=Path(__file__).resolve().parents[2]
ERPM_SCALE=2*math.pi/(60*21*8)  # Explicit synthetic AKE60-8 mapping, not new calibration.
LSB=30/4095
CASES=("psi","beta","loss","stale_state","plan_loss","plan_replay","controller_stop")
PLUGIN="mech_bringup/Ak30KinematicSystem"
JOINTS=("motor104_joint","motor105_joint")
FIELDS=("mit_drive_id","mit_rx_sequence","mit_rx_us","mit_valid","mit_position_deg",
        "mit_erpm","mit_iq_a","mit_temperature_c","mit_fault_code")


def can_id(body):
    """TX extended classic CAN body; mode-15 disable has DLC zero."""
    if len(body)<6 or body[4]!=0x0c or body[5]>8 or len(body)!=6+body[5]:
        raise ValueError("CAN payload size/flags")
    return int.from_bytes(body[:4],"little")


def feedback_words(alpha,velocity,erpm_scale=ERPM_SCALE):
    pos=np.rad2deg(np.asarray(alpha)[::-1]+math.pi/4)
    speed=np.asarray(velocity)[::-1]/erpm_scale
    return [int(round(x*10)) for x in pos], [int(round(x/10)) for x in speed]


def decode_command(payload,cap=.2):
    data=bytes(payload)
    zero=bytes.fromhex(mit.ZERO_WIRE)
    if len(data)!=8 or data[:6]!=zero[:6] or data[6]>>4!=zero[6]>>4:
        raise ValueError("MIT position/velocity/gains changed")
    code=int.from_bytes(data[-2:],"big")&4095
    torque=code*30/4095-15
    if abs(torque)>cap+LSB: raise ValueError("MIT torque exceeds simulation envelope")
    return torque,code


class Plant:
    def __init__(self,pulse_nm=.09):
        self.pulse_nm=pulse_nm
        sys.path.insert(0,str(ROOT/"mujoco/ref01"))
        from ref01_dynamics_api import REF01Dynamics
        self.model=REF01Dynamics()
        self.state=self.model.reset(np.deg2rad([0.,20.]))
        # Warm Python/MuJoCo allocations before starting feedback deadlines.
        # Reset exactly to the original pose and zero velocity after warmup.
        for _ in range(30):
            self.model.step(np.zeros(2),steps=10)
        self.state=self.model.reset(np.deg2rad([0.,20.]))
    def advance(self,effort,t,case):
        q=self.state["q"].copy();dq=self.state["dq"].copy()
        disturbance=np.zeros(2)
        if .4<=t<1.: disturbance[1 if case=="beta" else 0]=self.pulse_nm
        self.state=self.model.step(np.asarray(effort)[::-1],steps=10,external_torque=disturbance)
        return dict(time_s=t,psi_rad=float(q[0]),beta_rad=float(q[1]),
                    dpsi_rad_s=float(dq[0]),dbeta_rad_s=float(dq[1]),
                    tau104_nm=float(effort[0]),tau105_nm=float(effort[1]),
                    closure_m=float(self.state["max_closure_m"]),
                    warnings=int(self.state["warnings"]))


def recovery(rows,case,cap=.2):
    t=np.array([r["time_s"] for r in rows])
    err=np.rad2deg(np.array([[r["psi_rad"],r["beta_rad"]] for r in rows])-[0.,math.pi/9])
    speed=np.rad2deg(np.array([[r["dpsi_rad_s"],r["dbeta_rad_s"]] for r in rows]))
    good=(np.max(abs(err),axis=1)<=.5)&(np.max(abs(speed),axis=1)<=1.)
    settled=np.flatnonzero((t>=1)&np.logical_and.accumulate(good[::-1])[::-1])
    tail=t>=2.5
    axis=1 if case=="beta" else 0
    peak=float(err[:,axis].max())
    checks=dict(samples_cover_3s=len(rows)>=601,finite=bool(np.isfinite(err).all() and np.isfinite(speed).all()),
                bounded_hold=float(abs(err[t<.4]).max())<.5,positive_deflection=peak>.5,
                recovered_both_axes=bool(len(settled) and t[settled[0]]<=2.5 and good[tail].all()),
                closure=max(r["closure_m"] for r in rows)<1e-4,
                no_solver_warnings=all(r["warnings"]==0 for r in rows),
                quantized_effort_envelope=all(abs(r[k])<=cap+LSB for r in rows for k in ("tau104_nm","tau105_nm")))
    return dict(passed=all(checks.values()),checks=checks,peak_deg=peak,
                tail_error_deg=float(abs(err[tail]).max()),tail_speed_deg_s=float(abs(speed[tail]).max()),
                settling_after_release_s=float(t[settled[0]]-1) if len(settled) else None,
                bands=dict(error_deg=.5,speed_deg_s=1.,settle_before_model_s=2.5),
                note="Quantized MIT criterion fixed before execution; distinct from ideal 0.05deg test.")


class MitFixture:
    def __init__(self,predictive=False,feedback_period_s=.020,acknowledge_disable=False,disable_ack_ids=(104,105),erpm_scale=ERPM_SCALE,pulse_nm=None):
        if feedback_period_s not in (.005,.020):raise ValueError("unsupported fixture feedback period")
        if not math.isfinite(erpm_scale) or erpm_scale<=0:raise ValueError("invalid ERPM scale")
        if pulse_nm is not None and (not math.isfinite(pulse_nm) or pulse_nm<0):raise ValueError("invalid pulse")
        self.erpm_scale=erpm_scale
        self.pulse_nm=pulse_nm
        self.feedback_period_s=feedback_period_s
        self.acknowledge_disable=acknowledge_disable
        self.disable_requests=[]
        self.disabled=[False,False]
        self.disable_ack_ids=set(disable_ack_ids)
        self.predictive=predictive
        self.cap=5. if predictive else .2
        self.ports=[];self.done=threading.Event();self.thread=None;self.errors=[]
        self.rows=[];self.tx=[];self.rx=[];self.last=[0,0];self.effort=[0.,0.]
        self.model_time=0.;self.start_ns=0;self.cut_ns=0;self.source_sequence=[0,0]
        self.duration=0.;self.case="";self.folder=None
        self.gc_was_enabled=None;self.max_emulator_gap_ns=0
    def start(self,folder,case,duration,namespace):
        import pty,tty
        self.folder=folder;self.case=case;self.duration=duration;self.plant=Plant(self.pulse_nm if self.pulse_nm is not None else .3 if self.predictive else .09)
        # Finite, capped fixture run: keep automatic cyclic collection out of the
        # synthetic feedback producer; restore the caller policy on teardown.
        self.gc_was_enabled=gc.isenabled()
        gc.disable()
        for mid in (104,105):
            master,slave=pty.openpty();tty.setraw(slave)
            if self.acknowledge_disable:os.set_blocking(master,False)
            path=os.ttyname(slave)
            if not path.startswith("/dev/pts/"):raise RuntimeError("not a PTY")
            self.ports.append(dict(mid=mid,master=master,slave=slave,path=path,buffer=b"",out=b""))
        xml,params=self.configuration(folder,case,duration,namespace)
        self.thread=threading.Thread(target=self.run,name="private_mit_mujoco",daemon=True)
        self.thread.start()
        return xml,params

    def configuration(self,folder,case,duration,namespace):
        if case not in CASES: raise ValueError("unknown MIT scenario")
        if len(self.ports)!=2 or {p["mid"] for p in self.ports}!={104,105}:
            raise ValueError("two private endpoints required")
        if any(not p["path"].startswith("/dev/pts/") or not p["path"][9:].isascii() or not p["path"][9:].isdigit() for p in self.ports):
            raise ValueError("physical device forbidden")
        # Fixture sources are created here, never discovered from machine config.
        cfg=json.loads((ROOT/"config/parallel_exo/servo_pair.ref01.json").read_text())
        for bus in cfg["buses"]:
            mid=next(m["id"] for m in cfg["motors"] if m["logical_bus"]==bus["logical_bus"])
            bus["device_path"]=self.ports[mid-104]["path"]
        previous=mit.NS;mit.NS=namespace
        try: xml,params=mit.artifacts(cfg,folder)
        finally:mit.NS=previous
        robot=ET.fromstring(xml)
        for control in robot.findall("ros2_control"):
            hw=control.find("hardware")
            mid=int(hw.find("param[@name='drive_id']").text)
            index=mid-104
            control.set("name","mit_comm_"+str(mid))
            hw.find("plugin").text=PLUGIN
            values={"simulation_only":"true","kinematic_erpm_to_rad_s":str(self.erpm_scale),
                    "torque_max_abs_erpm":"2000","control_period_ns":"5000000",
                    "command_ttl_ns":"5500000","command_hard_ttl_ns":"6000000",
                    "feedback_telemetry_log":"false"}
            if self.predictive:
                position,_=self.feedback_words()
                values.update(simulation_only="false",bench_profile="ref01_supported_v1",bench_rehearsal="true",
                    bench_reference_deg=str(position[index]*.1),torque_max_abs_erpm="327680")
            if case=="stale_state":
                values.update(simulation_freeze_after_s="1.2",simulation_freeze_duration_s=".45")
            for key,value in values.items():
                item=hw.find("param[@name='"+key+"']")
                if item is None:item=ET.SubElement(hw,"param",name=key)
                item.text=value
            joint=control.find("joint");old=joint.get("name");joint.set("name",JOINTS[index])
            robot.find("joint[@name='"+old+"']").set("name",JOINTS[index])
            for name in ("sample_time_s","sequence")+(("accepted_effort","accepted_time_s","accepted_sequence") if self.predictive else ()):
                ET.SubElement(joint,"state_interface",name=name)
        xml=ET.tostring(robot,encoding="unicode")
        # Reuse the official manager/controller configuration contract.
        from test_ref01_manager import build_configuration,MODEL
        _,params=build_configuration((ROOT/MODEL).resolve(),folder.resolve(),"psi",duration,namespace)
        params[namespace+"/controller_manager"]["ros__parameters"]["robot_description"]=xml
        if self.predictive:
            for key,value in params.items():
                if key.endswith("/ref01_impedance"):
                    value["ros__parameters"].update(simulation_only=False,bench_profile="ref01_supported_v1",
                        predictive_impedance=True,receive_time_uncertainty_s=.007)
        actual={p.text for p in robot.findall("./ros2_control/hardware/param[@name='device_path']")}
        if actual!={p["path"] for p in self.ports}:raise ValueError("endpoint mismatch")
        return xml,params

    def hardware_checks(self,hardware,interfaces):
        components={c.name:c for c in hardware.component}
        wanted={"mit_comm_104","mit_comm_105"}
        return dict(hardware_active=set(components)==wanted and all(
            components[n].class_type==PLUGIN and components[n].state.label=="active" for n in wanted),
            state_interfaces_available={j+"/"+i for j in JOINTS for i in ("position","velocity","sample_time_s","sequence")}
                <= {v.name for v in interfaces.state_interfaces if v.is_available})

    def plan_data(self,sequence,now):
        gains=(10.,10.,1.,1.) if self.predictive else (2.4,2.,.18,.15)
        return [1.,float(sequence),now,1.,0.,math.pi/9.,*gains,self.cap,self.cap]

    def feedback_words(self):
        return feedback_words(self.plant.state["alpha"],self.plant.state["alpha_velocity"],self.erpm_scale)

    def policy(self):
        if 1.2<=self.model_time<1.65:
            if self.case=="plan_loss":return "pause"
            if self.case=="plan_replay":return "replay"
        if self.model_time>=1.2 and self.case=="controller_stop":return "stop"
        return "publish"

    def send(self,port,frame,ack=False):
        if not self.acknowledge_disable:
            os.write(port["master"],frame)
            return
        # Test peer must remain stoppable while the driver is INACTIVE.
        # Finish partial packets before adding ACK; don't queue old feedback.
        if ack or not port["out"]:port["out"]+=frame
        if len(port["out"])>4096:raise RuntimeError("test peer output queue")
        self.flush(port)

    def flush(self,port):
        if port.get("out"):
            try:
                sent=os.write(port["master"],port["out"])
                port["out"]=port["out"][sent:]
            except BlockingIOError:
                pass

    def run(self):
        try:
            next_tick=time.monotonic_ns();next_feedback=next_tick;tick=0;last_loop=next_tick
            count=round(self.duration/.005)+1
            while not self.done.is_set():
                ready,_,_=select.select([p["master"] for p in self.ports],[],[],.0005)
                for index,port in enumerate(self.ports):
                    self.flush(port)
                    if port["master"] in ready:port["buffer"]+=os.read(port["master"],4096)
                    while len(port["buffer"])>=7:
                        buf=port["buffer"];size=int.from_bytes(buf[2:4],"little")+7
                        if size>1024:raise ValueError("CDC frame size")
                        if len(buf)<size:break
                        packet=buf[:size];port["buffer"]=buf[size:];body=packet[7:]
                        if packet!=wire.packet(body):raise ValueError("CDC CRC/header")
                        if body==bytes(6):continue
                        ident=can_id(body);stamp=time.monotonic_ns()
                        if ident==0x800+port["mid"]:
                            if body[4:6]!=bytes.fromhex("0c08"):raise ValueError("CAN flags")
                            torque,code=decode_command(body[6:],self.cap)
                            self.disabled[index]=False
                            self.last[index]=stamp;self.effort[index]=torque
                            self.tx.append(dict(ns=stamp,drive=port["mid"],code=code,torque_nm=torque,hex=body[6:].hex()))
                        elif self.acknowledge_disable and ident==0xf00+port["mid"]:
                            if len(body)!=6:raise ValueError("disable DLC must be zero")
                            self.disable_requests.append(dict(ns=stamp,drive=port["mid"]))
                            if not any(self.disabled):
                                # Explicit supported-bench fixture: when disabled,
                                # the external support holds this pose at rest.
                                self.plant.state=self.plant.model.reset(self.plant.state["q"],[0.,0.])
                            self.disabled[index]=True
                            self.effort[index]=0.
                            positions,speeds=self.feedback_words()
                            payload=struct.pack(">hhhBB",positions[index],speeds[index],0,40,0x77)
                            if port["mid"] in self.disable_ack_ids:
                                self.send(port,wire.packet(struct.pack("<IBB",0x2900+port["mid"],4,8)+payload),ack=True)
                        else:raise ValueError("Unexpected CAN ID "+hex(ident))
                now=time.monotonic_ns()
                self.max_emulator_gap_ns=max(self.max_emulator_gap_ns,now-last_loop);last_loop=now
                if self.start_ns==0 and all(self.last):
                    self.start_ns=now;next_tick=now
                if self.start_ns and not any(self.disabled) and now>=next_tick and tick<count:
                    t=tick*.005;self.model_time=t
                    if t>=1.2 and self.cut_ns==0:self.cut_ns=now
                    for i in range(2):
                        if now-self.last[i]>=60000000:self.effort[i]=0.
                    row=self.plant.advance(self.effort,t,self.case)
                    row.update(ns=now,finish_ns=time.monotonic_ns(),last_tx104_ns=self.last[0],last_tx105_ns=self.last[1])
                    self.rows.append(row);tick+=1
                    next_tick+=5000000
                    if next_tick<now:next_tick=now+5000000 # record wall stretch; no catch-up burst
                # Physics may consume measurable wall time; never test a
                # feedback deadline against the pre-physics timestamp.
                now=time.monotonic_ns()
                if now>=next_feedback:
                    positions,speeds=self.feedback_words()
                    for i,port in enumerate(self.ports):
                        if self.case=="loss" and i==1 and 1.2<=self.model_time<1.65:continue
                        payload=struct.pack(">hhhBB",positions[i],speeds[i],0,40,0)
                        self.send(port,wire.packet(struct.pack("<IBB",0x2900+port["mid"],4,8)+payload))
                        self.source_sequence[i]+=1
                        self.rx.append(dict(ns=time.monotonic_ns(),drive=port["mid"],sequence=self.source_sequence[i],hex=payload.hex()))
                    next_feedback=now+round(self.feedback_period_s*1e9)
                if len(self.tx)+len(self.rx)>100000 or len(self.rows)>24001:raise RuntimeError("record cap")
                if tick>=count:
                    (self.folder/"complete.json").write_text('{"complete":true}')
                    # Keep feedback and output alive until supervisor deactivates;
                    # measured plant data are frozen at this boundary.
                    break
            # Continue packets after completion until manager is deactivated.
            while not self.done.wait(.020) and self.start_ns:
                for i,port in enumerate(self.ports):
                    positions,speeds=self.feedback_words()
                    self.send(port,wire.packet(struct.pack("<IBB",0x2900+port["mid"],4,8)+
                             struct.pack(">hhhBB",positions[i],speeds[i],0,40,0)))
        except Exception as exc:
            self.errors.append(repr(exc))
            if self.folder:(self.folder/"writer_error.txt").write_text(repr(exc))

    def close(self):
        self.done.set()
        if self.thread:self.thread.join(5)
        if self.thread and self.thread.is_alive():raise RuntimeError("emulator did not stop")
        for port in self.ports:
            os.close(port["master"]);os.close(port["slave"])
        self.ports=[]
        if self.gc_was_enabled:gc.enable()
        if self.folder:
            for name,rows in (("plant.csv",self.rows),("wire_tx.csv",self.tx),("wire_rx.csv",self.rx)):
                if rows:
                    with (self.folder/name).open("w",newline="") as f:
                        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
            (self.folder/"emulator.json").write_text(json.dumps(dict(errors=self.errors,start_ns=self.start_ns,
                 cut_ns=self.cut_ns,physics_period_s=.0005,model_tick_s=.005,feedback_period_s=self.feedback_period_s,
                 synthetic_drive_watchdog_s=.060,erpm_scale=self.erpm_scale,torque_lsb_nm=LSB,
                 disable_requests=self.disable_requests,external_support_on_disable=self.acknowledge_disable,fixture_gc_disabled=self.gc_was_enabled is not None,max_emulator_gap_ms=self.max_emulator_gap_ns*1e-6),indent=2))


    def summarize(self,returncode,evidence,statuses):
        from ref01_manager_validation import REQUIRED_MANAGER_CHECKS
        ec=evidence.get("checks",{})
        checks={k:ec.get(k) is True for k in REQUIRED_MANAGER_CHECKS-
                {"explicit_controller_deactivation","explicit_hardware_deactivation"}}
        checks["cleanup_confirmed"]=(ec.get("explicit_controller_deactivation") is True and
            ec.get("explicit_hardware_deactivation") is True) or evidence.get("fault_shutdown_verified") is True
        checks.update(emulator_clean=not self.errors,manager_clean_exit=returncode==0,
                      two_motors_observed={r["drive"] for r in self.tx}=={104,105},
                      followed_before_event=any(r.get("mode")==2 for r in statuses),
                      source_feedback_seen={r["drive"] for r in self.rx}=={104,105},
                      complete_samples=len(self.rows)==round(self.duration/.005)+1)
        normal=self.case in ("psi","beta")
        result=dict(case=self.case,checks=checks,errors=self.errors)
        tx_by={mid:[r for r in self.tx if r["drive"]==mid] for mid in (104,105)}
        cut=self.cut_ns
        if normal:
            checks["no_controller_fault"]=not any(r.get("mode")==3 for r in statuses)
            checks["commands_reach_end"]=all(v and v[-1]["ns"]>=self.rows[-1]["ns"]-15000000 for v in tx_by.values())
            result["recovery"]=recovery(self.rows,self.case,self.cap)
        else:
            # Fixed before execution: 100 ms plan TTL + manager + driver hard TTL.
            checks["fault_injected"]=cut>0
            checks["healthy_until_injection"]=cut>0 and all(any(cut-30000000<=r["ns"]<=cut for r in v) for v in tx_by.values())
            checks["both_stopped_within_120ms"]=cut>0 and all(v and v[-1]["ns"]<=cut+120000000 for v in tx_by.values())
            checks["feedback_returned"]=cut>0 and all(any(r["drive"]==mid and r["ns"]>=cut+500000000 for r in self.rx) for mid in (104,105))
            checks["no_restart_after_recovery"]=cut>0 and not any(r["ns"]>=cut+450000000 for r in self.tx)
            checks["synthetic_watchdog_zero"]=bool(self.rows) and all(
                abs(r[k])<1e-12 for r in self.rows if r["time_s"]>=1.8 for k in ("tau104_nm","tau105_nm"))
            result["last_tx_after_injection_ms"]={str(mid):(v[-1]["ns"]-cut)*1e-6 if v and cut else None for mid,v in tx_by.items()}
        if self.case=="stale_state":
            from ref01_stale_validation import load_stale_state
            result["fault_cause"]=load_stale_state(self.folder,statuses)
            checks["stale_state_cause_verified"]=result["fault_cause"]["passed"]
        result["transport_passed"]=all(checks.values())
        result["recovery_passed"]=result.get("recovery",{}).get("passed")
        result["passed"]=result["transport_passed"] and (result["recovery_passed"] if normal else True)
        result["status_modes"]=sorted(set(r.get("mode") for r in statuses))
        result["status_reasons"]=sorted(set(r.get("reason_name","") for r in statuses))
        result["counts"]=dict(tx=len(self.tx),feedback=len(self.rx),plant=len(self.rows))
        result["wire_period_ms"]={}
        for mid,values in tx_by.items():
            periods=np.diff([r["ns"] for r in values])*1e-6
            result["wire_period_ms"][str(mid)]=dict(p50=float(np.percentile(periods,50)),p99=float(np.percentile(periods,99)),max=float(periods.max())) if len(periods) else None
        result["boundary"]="Private PTY + actual MIT codec/runtime + C++ controller + MuJoCo. 60ms zero is an emulated drive watchdog, not hardware disable evidence. Wire intervals include emulator scheduling."
        return result
