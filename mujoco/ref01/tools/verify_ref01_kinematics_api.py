"""Check public API against the frozen MATLAB solution set and input boundaries."""
from pathlib import Path
import sys, json, math, hashlib, subprocess
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from ref01_kinematics import inverse_kinematics, inverse_kinematics_many, KinematicsError

baseline_path=ROOT/'fixtures/ref01_kinematics_validation_20261103/matlab_baseline.json'
baseline=json.loads(baseline_path.read_text(encoding='utf-8-sig'))
max_error=0.
for pose,expected in zip(baseline['q_deg'],baseline['alpha_rad'],strict=True):
    deg=inverse_kinematics(*pose)
    rad=inverse_kinematics(*(math.radians(x) for x in pose),unit='rad')
    max_error=max(max_error,*(abs(x-y) for x,y in zip(rad,expected)),
                  *(abs(math.radians(x)-y) for x,y in zip(deg,expected)))
assert max_error<1e-12
invalid=[(46,20,'deg'),(0,-.01,'deg'),(0,45.01,'deg'),(-46,20,'deg'),
         (float('nan'),20,'deg'),(0,float('inf'),'deg'),('10',20,'deg'),(True,20,'deg'),
         (0,20,'degrees'),(0,1,'rad')]
for psi,beta,unit in invalid:
    try: inverse_kinematics(psi,beta,unit=unit)
    except KinematicsError: pass
    else: raise AssertionError((psi,beta,unit))
poses=[(-45,0),(0,20),(45,45)]
assert inverse_kinematics_many(poses)==[inverse_kinematics(*p) for p in poses]
assert inverse_kinematics_many([])==[]
try: inverse_kinematics_many([(0,20),(0,60)])
except KinematicsError as exc: assert 'pose[1]' in str(exc)
else: raise AssertionError('Invalid batch accepted')
result=subprocess.run([sys.executable,'-I','-S','-B',str(ROOT/'ref01_kinematics.py'),
                       '--psi','10','--beta','20'],capture_output=True,text=True,check=True)
example=json.loads(result.stdout)
assert (example['alpha1'],example['alpha2'])==inverse_kinematics(10,20)
assert 'mujoco' not in sys.modules and 'numpy' not in sys.modules
record=dict(status='PASS',matlab_cases=len(baseline['q_deg']),max_error_rad=max_error,
            invalid_cases_rejected=len(invalid),batch_tests_passed=True,standard_library_only=True,example=example,
            baseline_sha256=hashlib.sha256(baseline_path.read_bytes()).hexdigest(),
            api_sha256=hashlib.sha256((ROOT/'ref01_kinematics.py').read_bytes()).hexdigest())
out=ROOT/'output/ref01_kinematics_api';out.mkdir(exist_ok=True)
(out/'verification.json').write_text(json.dumps(record,indent=2),encoding='utf-8')
print(json.dumps(record,indent=2))
