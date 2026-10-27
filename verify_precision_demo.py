"""Run two real Gazebo laps, record dosing feedback, and check measured results."""
import csv
import argparse
from datetime import datetime,timezone
import json
from pathlib import Path
import subprocess
import sys
import time
import rclpy
from std_msgs.msg import String
from std_srvs.srv import Trigger

ROOT=Path(__file__).resolve().parent

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--laps',type=int,default=2)
    parser.add_argument('--stop-during-watering',action='store_true')
    args=parser.parse_args()
    directory=ROOT/'evidence'/datetime.now(timezone.utc).strftime('precision_%Y%m%dT%H%M%SZ')
    directory.mkdir()
    rclpy.init(); observer=rclpy.create_node('pai_precision_verifier')
    latest={}; samples=[]
    def on_can(m):
        state=json.loads(m.data); latest.update(state); samples.append(state)
    observer.create_subscription(String,'/pai_watering/can_state',on_can,10)
    stop=observer.create_client(Trigger,'/pai_patrol/stop'); future=None
    with (directory/'console.log').open('w') as output:
        process=subprocess.Popen([sys.executable,str(ROOT/'run_precision_demo.py'),'--reset-plants',
            '--laps',str(args.laps),'--log-dir',str(directory)],stdout=output,stderr=subprocess.STDOUT)
        started=time.monotonic()
        try:
            while process.poll() is None:
                rclpy.spin_once(observer,timeout_sec=.05)
                if args.stop_during_watering and latest.get('angle_deg',0)>28 and latest.get('flow_ml_s',0)>5 and future is None and stop.service_is_ready():
                    future=stop.call_async(Trigger.Request())
                if time.monotonic()-started>560: raise subprocess.TimeoutExpired(process.args,560)
        except subprocess.TimeoutExpired:
            process.terminate(); process.wait(timeout=12)
    logs=list(directory.glob('patrol_*.jsonl'))
    events=[json.loads(line) for line in logs[0].read_text().splitlines()] if logs else []
    decisions=[e for e in events if e['event']=='ai_decision']
    completed=[e for e in events if e['event']=='dose_completed']
    finished=next((e for e in reversed(events) if e['event']=='finished'),{})
    tracefiles=list(directory.glob('*_precision.csv'))
    rows=list(csv.DictReader(tracefiles[0].open())) if tracefiles else []
    active=[r for r in rows if r['phase'] in ('pour','upright')]
    maximum=max((float(r['angle_deg']) for r in active),default=0)
    tracking_difference=max((abs(float(r['angle_deg'])-float(r['command_deg'])) for r in active),default=0)
    final_state=json.loads((ROOT/'plant_scene/live_plants.json').read_text())
    expected=['water','skip','water','skip']+['skip']*4*(args.laps-1)
    passed=(process.returncode==0 and finished.get('result')=='completed'
            and [e['action'] for e in decisions]==expected and len(completed)==2
            and all(abs(e['error_ml'])<=5 for e in completed)
            and maximum>25 and tracking_difference>1
            and all(e['soil_after']>e['soil_before'] and e['leaf_after']<e['leaf_before'] for e in completed))
    if args.stop_during_watering:
        passed=(process.returncode==0 and finished.get('result')=='stopped_by_service'
                and future is not None and future.done() and future.result().success
                and maximum>28 and abs(latest.get('angle_deg',999))<1
                and latest.get('flow_ml_s',999)<.05)
    report=dict(passed=passed,exit_code=process.returncode,result=finished.get('result'),
                wall_seconds=round(time.monotonic()-started,2),
                dose_results=completed,actions=[e['action'] for e in decisions],
                measured_angle_samples=len(rows),peak_measured_angle_deg=maximum,
                max_command_measurement_difference_deg=tracking_difference,
                stop_during_watering=args.stop_during_watering,
                final_measured_angle_deg=latest.get('angle_deg'),final_virtual_flow_ml_s=latest.get('flow_ml_s'),
                final_plants=final_state['plants'],
                limitation='Virtual soil/leaf sensors and calibrated virtual flow; angle comes from an actual Gazebo joint.')
    (directory/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(directory=str(directory),**report),indent=2),flush=True)
    observer.destroy_node(); rclpy.shutdown()
    return 0 if passed else 1

if __name__=='__main__': raise SystemExit(main())
