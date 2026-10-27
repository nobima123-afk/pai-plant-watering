"""PowerShell/docker entry point for live simulated sensors and precision patrol."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parent

def main():
    p=argparse.ArgumentParser(); p.add_argument('--reset-plants',action='store_true')
    p.add_argument('--laps',type=int,default=1); p.add_argument('--log-dir')
    args=p.parse_args()
    children=[]; interrupted=[]
    def stop(*_):
        interrupted.append(True)
        if children and children[-1].poll() is None: children[-1].terminate()
    for sig in (signal.SIGINT,signal.SIGTERM): signal.signal(sig,stop)
    try:
        info=subprocess.check_output(['gz','model','-m','burger','-i'],text=True,timeout=10)
        if args.reset_plants or 'pai_can_tilt_joint' not in info:
            subprocess.run([sys.executable,str(ROOT/'install_precision_can.py')],check=True,
                           env=dict(os.environ,RMW_IMPLEMENTATION='rmw_cyclonedds_cpp'))
        subprocess.run([sys.executable,str(ROOT/'setup_plant_scene.py')],check=True,
                       env=dict(os.environ,RMW_IMPLEMENTATION='rmw_cyclonedds_cpp'))
        # Let the virtual sensors start before the patrol begins; state is persistent.
        command=[sys.executable,str(ROOT/'precision_plants.py')]
        if args.reset_plants: command.append('--reset-plants')
        sensors=subprocess.Popen(command); children.append(sensors)
        time.sleep(1.)
        if sensors.poll() is not None: raise RuntimeError('Plant sensors did not start')
        command=[sys.executable,str(ROOT/'patrol.py'),'--precision-watering','--laps',str(args.laps),'--timeout','600']
        if args.log_dir: command.extend(['--log-dir',args.log_dir])
        patrol=subprocess.Popen(command); children.append(patrol)
        while patrol.poll() is None:
            if sensors.poll() is not None:
                patrol.terminate(); raise RuntimeError('Plant sensors stopped during patrol')
            time.sleep(.1)
        return patrol.returncode
    finally:
        for child in reversed(children):
            if child.poll() is None:
                child.terminate()
                try: child.wait(timeout=6)
                except subprocess.TimeoutExpired: child.kill(); child.wait()

if __name__=='__main__': raise SystemExit(main())
