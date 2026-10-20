#!/usr/bin/env python3
"""Read-only paired feedback operator; manual calibration never commands motors."""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import selectors
import shutil
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from collections import deque

FRESH_NS = 200_000_000
STABLE_NS = 500_000_000
IDS = (104, 105)

def default_config(device_path):
    return {'schema_version':1, 'device_path':device_path, 'calibrated':False,
            'identity':{'board':'Livelybot 2024051701','motors':'AK30 IDs104/105'},
            'motors':[{'id':i,'joint_name':f'motor{i}_joint', 'target_scale':180/math.pi,
                       'target_offset':0.0,'feedback_scale':math.pi/180,'feedback_offset':0.0,
                       'target_mapping_verified':False,'feedback_mapping_verified':False,
                       'mapping_evidence':'unverified','speed_erpm':100.0,'acceleration_raw':100.0,
                       'position_min_rad':None,'position_max_rad':None,
                       'position_max_error_rad':0.25} for i in IDS]}

def finite(value):
    return isinstance(value,(float,int)) and not isinstance(value,bool) and math.isfinite(value)

def configured_buses(config):
    """Normalize old single-port and explicit multi-port declarations without I/O."""
    if config.get('schema_version') == 1:
        if 'buses' in config or any('logical_bus' in m for m in config.get('motors', [])):
            raise ValueError('schema 1不能混用多总线配置')
        buses = [dict(logical_bus=1, device_path=config.get('device_path'))]
    elif config.get('schema_version') == 2:
        if 'device_path' in config:
            raise ValueError('schema 2必须仅通过buses声明设备路径')
        buses = config.get('buses')
        if not isinstance(buses, list) or not 1 <= len(buses) <= 2:
            raise ValueError('必须配置一或两条总线')
    else:
        raise ValueError('配置版本无效')
    ids, paths = set(), set()
    for bus in buses:
        ident, path = bus.get('logical_bus'), bus.get('device_path')
        if type(ident) is not int or not 1 <= ident <= 255:
            raise ValueError('logical_bus必须为1到255的整数')
        if not isinstance(path, str) or not path or path.strip() != path:
            raise ValueError('设备路径无效')
        canonical = str(Path(path).resolve())
        if ident in ids or canonical in paths:
            raise ValueError('总线编号或设备路径重复')
        ids.add(ident); paths.add(canonical)
    if config.get('schema_version') == 2:
        assignments = [m.get('logical_bus') for m in config.get('motors', [])]
        if any(type(i) is not int or i not in ids for i in assignments) or set(assignments) != ids:
            raise ValueError('每台电机必须属于已声明总线，每条总线必须有电机')
    return buses


def selected_buses(config, motors):
    return [dict(bus, motors=[m for m in motors if m.get('logical_bus', 1) == bus['logical_bus']])
            for bus in configured_buses(config)
            if any(m.get('logical_bus', 1) == bus['logical_bus'] for m in motors)]


def split_buses(config, port104, port105):
    """Copy saved mappings/limits; changing topology never recalibrates motors."""
    validate_config(config, True)
    result = copy.deepcopy(config)
    result.pop('device_path', None)
    result.update(schema_version=2, buses=[dict(logical_bus=1, device_path=port105),
                                          dict(logical_bus=2, device_path=port104)])
    for motor in result['motors']:
        motor['logical_bus'] = 1 if motor['id']==105 else 2
    validate_config(result, True)
    return result


def validate_config(config, observe=False):
    configured_buses(config)
    motors=config.get('motors',[])
    if len(motors)!=2 or {m.get('id') for m in motors}!=set(IDS): raise ValueError('必须配置104和105')
    if len({m.get('joint_name') for m in motors})!=2: raise ValueError('关节名称重复')
    for m in motors:
        if m.get('joint_name')!=f"motor{m['id']}_joint": raise ValueError('关节名称与部署控制器不匹配')
        for key in ('target_scale','target_offset','feedback_scale','feedback_offset','speed_erpm','acceleration_raw','position_max_error_rad'):
            if not finite(m.get(key)): raise ValueError(f'{key}无效')
        if m['target_scale']==0 or m['feedback_scale']==0: raise ValueError('映射比例不能为零')
        if m['speed_erpm']<=0 or m['acceleration_raw']<=0 or m['position_max_error_rad']<=0: raise ValueError('运行参数必须为正')
        if m.get('target_mapping_verified') is not True or m.get('feedback_mapping_verified') is not True or not m.get('mapping_evidence') or m['mapping_evidence']=='unverified':
            raise ValueError('映射证据未声明；手动标定不能证明目标映射')
        if not observe:
            lo,hi=m.get('position_min_rad'),m.get('position_max_rad')
            if not finite(lo) or not finite(hi) or lo>=hi: raise ValueError('尚无有效标定范围')
    # Existing valid limits can be checked in isolation, but motion generation must be calibrated.
    return config

def select_motors(config, motor_id=None, observe=False):
    # Always validate the complete persistent pair before deriving a run subset.
    validate_config(config, observe)
    if motor_id is not None and motor_id != 105:
        raise ValueError('单机试验仅支持电机105')
    return sorted((m for m in config['motors'] if motor_id is None or m['id']==motor_id),
                  key=lambda m: m['id'])

def generate_urdf(config, observe=False, motor_id=None):
    validate_config(config,observe)
    if not observe and config.get('calibrated') is not True: raise ValueError('尚未完成标定，不能生成运动配置')
    motors=select_motors(config,motor_id,observe)
    root=ET.Element('robot',name='servo_pair'); ET.SubElement(root,'link',name='base')
    for m in motors:
        link=f"link_{m['id']}"; ET.SubElement(root,'link',name=link)
        j=ET.SubElement(root,'joint',name=m['joint_name'],type='continuous')
        ET.SubElement(j,'parent',link='base'); ET.SubElement(j,'child',link=link)
        ET.SubElement(j,'axis',xyz='0 0 1')
    for bus in selected_buses(config, motors):
        control=ET.SubElement(root,'ros2_control',name='servo_pair' if config['schema_version']==1 else f"servo_bus_{bus['logical_bus']}",type='system')
        hw=ET.SubElement(control,'hardware'); ET.SubElement(hw,'plugin').text='mech_bringup/Ak30ServoSystem'
        params={'profile':'ak30_servo_extended','device_path':bus['device_path'],'logical_bus':bus['logical_bus'],
                'control_period_ns':2_000_000,'command_ttl_ns':3_000_000,
                'command_hard_ttl_ns':6_000_000,'feedback_ttl_ns':FRESH_NS if observe else 60_000_000}
        if config['schema_version']==2: params['trace_name']=f"bus-{bus['logical_bus']}"
        for k,v in params.items(): ET.SubElement(hw,'param',name=k).text=str(v)
        for m in bus['motors']:
            j=ET.SubElement(control,'joint',name=m['joint_name'])
            p={k:m[k] for k in ('target_scale','target_offset','target_mapping_verified','feedback_scale','feedback_offset','feedback_mapping_verified','speed_erpm','acceleration_raw','position_max_error_rad')}
            p.update(drive_id=m['id'],position_min_rad=-1e6 if observe else m['position_min_rad'],position_max_rad=1e6 if observe else m['position_max_rad'])
            for k,v in p.items(): ET.SubElement(j,'param',name=k).text=str(v).lower() if isinstance(v,bool) else str(v)
            ET.SubElement(j,'state_interface',name='position')
            ET.SubElement(j,'command_interface',name='position'); ET.SubElement(j,'command_interface',name='command_generation')
    ET.indent(root); return ET.tostring(root,encoding='unicode')+'\n'

def paired(frame, now, arrival):
    if frame.get('schema_version')!=1 or frame.get('motor_command_frames')!=0: raise ValueError('反馈格式或零命令保证无效')
    stamp=frame.get('monotonic_ns')
    if not isinstance(stamp,int) or not 0<=now-stamp<=FRESH_NS or not 0<=now-arrival<=FRESH_NS: raise ValueError('反馈流过期')
    motors=frame.get('motors',[])
    if len(motors)!=2 or {m.get('id') for m in motors}!=set(IDS): raise ValueError('缺少成对反馈')
    result={}
    for m in motors:
        rx=m.get('host_rx_ns')
        if m.get('availability')!='fresh' or not isinstance(rx,int) or not 0<=now-rx<=FRESH_NS: raise ValueError('电机反馈过期')
        if m.get('raw_status')!=0: raise ValueError('电机故障状态')
        if not finite(m.get('position_rad')) or not finite(m.get('electrical_speed_erpm')): raise ValueError('位置或速度未知')
        result[m['id']]=m
    return result

class Calibration:
    def __init__(self,config):
        validate_config(config,True); self.config=copy.deepcopy(config)
        self.extrema={i:[math.inf,-math.inf] for i in IDS}; self.history=deque()
        self.captures={}; self.last=None; self.frame=None; self.invalid=False; self.started=time.time_ns()
    def ingest(self,frame,now):
        try:
            if self.last is None:
                # Startup unknown is expected until both identities have arrived. Faults and
                # malformed envelopes are never treated as an acquisition delay.
                if frame.get('schema_version')!=1 or frame.get('motor_command_frames')!=0:
                    raise ValueError('无效反馈')
                for m in frame.get('motors',[]):
                    if m.get('raw_status',0)!=0: raise ValueError('故障')
                if len(frame.get('motors',[]))==2 and {m.get('id') for m in frame['motors']}==set(IDS):
                    if any(m.get('availability')!='fresh' for m in frame['motors']): return False
            motors=paired(frame,now,now)
            if self.last is not None and now-self.last>FRESH_NS: raise ValueError('扫描期间反馈中断')
            if self.frame:
                prev={m['id']:m for m in self.frame['motors']}
                for i in IDS:
                    if motors[i]['sequence']<prev[i]['sequence']: raise ValueError('反馈序列倒退')
            for i,m in motors.items():
                self.extrema[i][0]=min(self.extrema[i][0],m['position_rad']); self.extrema[i][1]=max(self.extrema[i][1],m['position_rad'])
            self.history.append((now,{i:(m['position_rad'],m['electrical_speed_erpm']) for i,m in motors.items()}))
            while len(self.history)>1 and now-self.history[1][0]>=STABLE_NS: self.history.popleft()
            self.last=now; self.frame=frame
            return True
        except (ValueError,KeyError,TypeError):
            self.invalid=True; raise ValueError('扫描反馈无效；本次标定已作废，请重新开始')
    def capture(self,label,now):
        if self.invalid or self.frame is None: raise ValueError('无有效扫描')
        motors=paired(self.frame,now,self.last)
        if now-self.history[0][0]<STABLE_NS: raise ValueError('请停稳至少0.5秒后再按回车')
        for i in IDS:
            positions=[h[1][i][0] for h in self.history]
            if max(positions)-min(positions)>math.radians(.2) or any(abs(h[1][i][1])>10 for h in self.history): raise ValueError('两台电机尚未停稳')
        self.captures[label]={str(i):motors[i]['position_rad'] for i in IDS}
    def finish(self,margin):
        if self.invalid or set(self.captures)!={'right','left'} or not finite(margin) or margin<0: raise ValueError('端点或余量无效')
        result=copy.deepcopy(self.config)
        for m in result['motors']:
            i=m['id']; lo,hi=self.extrema[i]
            right=self.captures['right'][str(i)]; left=self.captures['left'][str(i)]
            tolerance=abs(m['feedback_scale'])*.5
            if not finite(right) or not finite(left) or abs(right-left)<=2*tolerance:
                raise ValueError('左右端点相同或太近')
            if min(right,left)<lo or max(right,left)>hi:
                raise ValueError('端点不在本次扫描范围内')
            inset=abs(m['feedback_scale'])*margin
            if not finite(lo) or not finite(hi) or lo+inset>=hi-inset: raise ValueError('内缩后范围塌缩，请重新标定')
            m.update(position_min_rad=lo+inset,position_max_rad=hi-inset)
        result['calibrated']=True
        result['calibration']={'started_unix_ns':self.started,'completed_unix_ns':time.time_ns(),
              'margin_device_deg':margin,'paired_endpoints_rad':self.captures,
              'observed_extrema_rad':{str(i):v for i,v in self.extrema.items()},
              'method':'manual right then left; continuous paired fresh normal feedback; no motor commands'}
        return result

def atomic_save(path,data):
    path=Path(path); path.parent.mkdir(parents=True,exist_ok=True)
    temp=None
    try:
        with tempfile.NamedTemporaryFile(mode='w',dir=path.parent,prefix=path.name+'.',suffix='.tmp',delete=False) as stream:
            temp=Path(stream.name); json.dump(data,stream,ensure_ascii=False,indent=2,allow_nan=False); stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
        if path.exists():
            backup=path.with_name(path.name+'.'+str(time.time_ns())+'.bak')
            shutil.copy2(path,backup)
        os.replace(temp,path); temp=None
        fd=os.open(path.parent,os.O_DIRECTORY)
        try: os.fsync(fd)
        finally: os.close(fd)
    finally:
        if temp is not None: temp.unlink(missing_ok=True)

def format_monitor(frame,now,arrival,config=None):
    if frame is None: return '等待成对反馈：104 未知；105 未知'
    lines=[]; index={m.get('id'):m for m in frame.get('motors',[])}
    for i in IDS:
        m=index.get(i,{})
        rx=m.get('host_rx_ns'); stamp=frame.get('monotonic_ns')
        fresh=isinstance(rx,int) and isinstance(stamp,int) and 0<=now-rx<=FRESH_NS and 0<=now-stamp<=FRESH_NS and 0<=now-arrival<=FRESH_NS and m.get('availability')=='fresh'
        def show(k,scale=1): return f'{m[k]*scale:.3f}' if finite(m.get(k)) else '未知'
        device=m.get('feedback_position_deg')
        if not finite(device) and finite(m.get('position_rad')):
            mapping=next((x for x in config['motors'] if x['id']==i),None) if config else None
            device=(m['position_rad']-mapping['feedback_offset'])/mapping['feedback_scale'] if mapping else math.degrees(m['position_rad'])
        angle=f'{device:.3f}' if finite(device) else '未知'
        lines.append(f"电机{i} | 角度 {angle} deg | 电速度 {show('electrical_speed_erpm')} ERPM | Iq {show('current_iq_a')} A | 温度 {show('temperature_c')} ℃ | 原始状态 {m.get('raw_status','未知')} | {'新鲜' if fresh else '过期/未知'}")
    return '\n'.join(lines)

def decode_lines(buffer):
    parts=buffer.split(b'\n')
    return [json.loads(line) for line in parts[:-1] if line.strip()],parts[-1]

class TerminalDisplay:
    """Keep an ANSI status area below durable capture/outcome messages."""
    def __init__(self,stream):
        self.stream=stream; self.tty=stream.isatty(); self.active=False
    def clear(self):
        if self.tty and self.active:
            self.stream.write('\x1b8\x1b[J'); self.stream.flush(); self.active=False
    def update(self,status,prompt=''):
        text=status+('\n'+prompt if prompt else '')
        if self.tty:
            if not self.active:
                rows=len(text.splitlines())
                # Reserve rows before anchoring so redraws do not scroll the terminal.
                self.stream.write('\n'*rows+f'\x1b[{rows}A\r\x1b7')
                self.active=True
            self.stream.write('\x1b8\x1b[J'+text)
        else: self.stream.write(text+'\n')
        self.stream.flush()
    def message(self,text):
        self.clear(); self.stream.write(text+'\n'); self.stream.flush()

def run_live(args,config):
    # Reader uses the framework runtime and a read-only observation URDF, never custom serial.
    with tempfile.TemporaryDirectory(prefix='servo-observe-') as directory:
        urdf=Path(directory)/'observe.urdf'; urdf.write_text(generate_urdf(config,True))
        proc=subprocess.Popen([args.reader,'--urdf',str(urdf)],stdout=subprocess.PIPE,bufsize=0)
        selector=selectors.DefaultSelector(); selector.register(proc.stdout,selectors.EVENT_READ,'reader')
        calibrating=args.command=='calibrate'; session=Calibration(config) if calibrating else None
        display=TerminalDisplay(sys.stdout)
        if calibrating:
            selector.register(sys.stdin,selectors.EVENT_READ,'input')
            print('手动向右扫描，停稳后按回车记录成对右端点；随后向左扫描。不会发送电机命令。',flush=True)
        frame=None; arrival=0; start=time.monotonic(); last_display=0; stage='right'; buffer=b''
        try:
            while True:
                now=time.monotonic_ns()
                if args.command=='monitor' and args.duration is not None and time.monotonic()-start>=args.duration: return
                if calibrating and session.last is not None and now-session.last>FRESH_NS: raise ValueError('反馈中断，本次标定已作废')
                for key,_ in selector.select(.05):
                    if key.data=='reader':
                        chunk=os.read(proc.stdout.fileno(),65536)
                        if not chunk: raise ValueError('反馈进程已退出；未保存')
                        frames,buffer=decode_lines(buffer+chunk)
                        if len(buffer)>1_000_000: raise ValueError('反馈行过长')
                        for frame in frames:
                            arrival=time.monotonic_ns()
                            if calibrating: session.ingest(frame,arrival)
                    else:
                        if not sys.stdin.readline(): raise ValueError('输入已关闭；未保存')
                        try: session.capture(stage,time.monotonic_ns())
                        except ValueError as exc: display.message(str(exc)); continue
                        if stage=='right':
                            stage='left'; display.message('右端点已记录。手动向左扫描，停稳后按回车记录成对左端点。')
                        else:
                            result=session.finish(args.margin_deg)
                            display.message('左端点已记录。\n'+json.dumps(result,ensure_ascii=False,indent=2))
                            # Stop feedback before a blocking confirmation; no commands are issued.
                            proc.terminate(); proc.wait(timeout=3)
                            if input('确认以上范围并保存配置及报告？输入 SAVE：').strip()=='SAVE':
                                report=Path(args.config).with_name(Path(args.config).stem+'.report.'+str(time.time_ns())+'.json')
                                atomic_save(report,result)
                                atomic_save(args.config,result)
                                print(f'已保存：{args.config}\n报告：{report}')
                            else: print('已取消，原配置保留。')
                            return
                now=time.monotonic_ns()
                if now-last_display>=100_000_000:
                    prompt=('手动向右扫描，停稳0.5秒后按回车记录右端点' if stage=='right' else '手动向左扫描，停稳0.5秒后按回车记录左端点') if calibrating else '只读监控；按 Ctrl+C 退出'
                    display.update(format_monitor(frame,now,arrival,config),prompt); last_display=now
        finally:
            display.clear()
            selector.close()
            if proc.poll() is None:
                proc.terminate()
                try: proc.wait(timeout=3)
                except subprocess.TimeoutExpired: proc.kill(); proc.wait()

def main(argv=None):
    parser=argparse.ArgumentParser(description='双电机手动标定与只读反馈监控')
    sub=parser.add_subparsers(dest='command',required=True)
    for name in ('calibrate','monitor','check','generate-urdf'):
        p=sub.add_parser(name); p.add_argument('--config',required=True)
        if name in ('calibrate','monitor'): p.add_argument('--reader',required=True)
        if name=='check': p.add_argument('--reader')
        if name=='calibrate': p.add_argument('--margin-deg',type=float,default=10)
        if name=='monitor': p.add_argument('--duration',type=float)
        if name=='generate-urdf': p.add_argument('--output',required=True); p.add_argument('--observe',action='store_true')
    p=sub.add_parser('split-buses', help='离线复制配置为双总线；保留标定，不打开设备')
    for option in ('config','output','port104','port105'): p.add_argument('--'+option,required=True)
    args=parser.parse_args(argv)
    try:
        config=json.loads(Path(args.config).read_text())
        if args.command=='split-buses':
            result=split_buses(config,args.port104,args.port105)
            with Path(args.output).open('x',encoding='utf-8') as stream:
                json.dump(result,stream,ensure_ascii=False,indent=2,allow_nan=False);stream.write('\n')
            print('双总线候选配置已生成；原配置未修改，未打开设备。')
        elif args.command=='generate-urdf': Path(args.output).write_text(generate_urdf(config,args.observe))
        elif args.command=='check':
            validate_config(config,True)
            if config.get('calibrated'): generate_urdf(config)
            if args.reader:
                with tempfile.TemporaryDirectory(prefix='servo-check-') as directory:
                    urdf=Path(directory)/'observe.urdf'; urdf.write_text(generate_urdf(config,True))
                    subprocess.run([args.reader,'--urdf',str(urdf),'--check'],check=True)
            print('配置有效；'+('已标定' if config.get('calibrated') else '未标定，仅允许观察'))
        else:
            if args.command=='monitor' and args.duration is not None and (not finite(args.duration) or args.duration<=0): raise ValueError('时长必须为正')
            if args.command=='calibrate' and (not finite(args.margin_deg) or args.margin_deg<0): raise ValueError('余量必须非负')
            run_live(args,config)
        return 0
    except KeyboardInterrupt: print('\n已中断，未覆盖标定配置。',file=sys.stderr); return 130
    except (ValueError,KeyError,OSError,TypeError,subprocess.SubprocessError) as exc: print(f'失败：{exc}',file=sys.stderr); return 1

if __name__=='__main__': sys.exit(main())
