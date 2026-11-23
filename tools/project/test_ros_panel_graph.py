"""Private PTY or native MuJoCo software graph driven through desktop SSH."""
import argparse,json,os,queue,signal,sys,threading,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"tools/project"))
import test_ph_exo_upper_ros as upper_test
PREFIX="PH_EXO_GRAPH "
def emit(**data):print(PREFIX+json.dumps(data),flush=True)
def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--case",choices=["mit_reset","mit_no_ack"],default="mit_reset")
    parser.add_argument("--cycle",action="store_true")
    args=parser.parse_args()
    os.chdir(ROOT)
    base=ROOT/".local/validation"/("ph_exo_panel_"+time.strftime("%Y%m%d_%H%M%S"))
    base.mkdir(exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID="183",ROS_LOCALHOST_ONLY="1",
                      ROS_LOG_DIR=str(base/"ros_logs"),PYTHONDONTWRITEBYTECODE="1")
    upper_test.rclpy.init()
    def driver(g):
        incoming=queue.Queue();closed=threading.Event()
        def reader():
            try:
                for line in sys.stdin:incoming.put(json.loads(line))
            finally:closed.set()
        threading.Thread(target=reader,daemon=True).start()
        emit(type="ready",api=g["ns"]+"/exoskeleton/impedance",
             node=g["ns"]+"/exoskeleton_control",output=str(base))
        end=time.monotonic()+100
        while time.monotonic()<end and not closed.is_set():
            g["executor"].spin_once(timeout_sec=.01)
            try:command=incoming.get_nowait()
            except queue.Empty:continue
            if command["cmd"]=="fault":
                os.killpg(g["state"].pid,signal.SIGTERM);g["state"].wait(timeout=5)
            elif command["cmd"]=="finish":break
            emit(type="check",state=g["upper"].ph_exo.snapshot(),tx=(len(g["fixture"].tx) if g["fixture"] else 0),
                 manager_alive=g["manager"].poll() is None)
        state=g["upper"].ph_exo.snapshot()
        data=dict(state=state,tx=(len(g["fixture"].tx) if g["fixture"] else 0),errors=g["fixture"].errors if g["fixture"] else [],
                  manager_alive=g["manager"].poll() is None,software_only=True)
        (base/"graph_result.json").write_text(json.dumps(data,indent=2))
        emit(type="done",**data)
        return data
    try:
        if args.cycle:
            from test_ph_exo_cycle_ros import candidate
            upper_test.run_case(base,"cycle",driver,reference_config=candidate())
        else:upper_test.run_case(base,args.case,driver)
    finally:upper_test.rclpy.shutdown()
if __name__=="__main__":main()
