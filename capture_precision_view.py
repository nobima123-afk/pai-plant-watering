"""Capture the actual Linux Gazebo screen while the measured can is pouring."""
import json
import os
from pathlib import Path
import subprocess
import time
import rclpy
from std_msgs.msg import String
from attach_watering_can import close_view

def main():
    rclpy.init(); node=rclpy.create_node('pai_dosing_capture'); states=[]
    node.create_subscription(String,'/pai_watering/can_state',lambda m:states.append(json.loads(m.data)),10)
    until=time.monotonic()+90
    try:
        while time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.05)
            if states and states[-1]['angle_deg']>28 and states[-1]['flow_ml_s']>5:
                close_view()
                root=Path(__file__).resolve().parent/'evidence'
                target=root/'precision_can_pouring.png'
                env=dict(os.environ,DISPLAY=':0')
                subprocess.run(['import','-window','root',str(target)],check=True,env=env,timeout=8)
                (root/'precision_can_capture_state.json').write_text(json.dumps(states[-1],indent=2))
                print(target); return
        raise TimeoutError('No measured pouring angle during capture window')
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()
