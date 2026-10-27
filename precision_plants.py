"""Mutable, explicitly virtual soil/leaf sensors driven by simulated delivered volume."""
import argparse
import copy
import json
from pathlib import Path
import signal
import time
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from dose_model import values
from scene_painter import ScenePainter

ROOT=Path(__file__).resolve().parent

class PlantWorld:
    def __init__(self,rows):
        self.rows=copy.deepcopy(rows); self.sessions={}
        for row in self.rows: values(row)

    def apply(self,plant_id,session_id,cumulative_ml):
        if not isinstance(cumulative_ml,(int,float)) or not 0<=cumulative_ml<=200:
            raise ValueError('Invalid cumulative virtual dose')
        key=(plant_id,session_id)
        previous=self.sessions.get(key,0.)
        if cumulative_ml<previous: raise ValueError('Dose counter went backwards')
        row=next(r for r in self.rows if r['plant_id']==plant_id)
        delta=cumulative_ml-previous; self.sessions[key]=cumulative_ml
        row['soil_moisture_pct']=min(100.,row['soil_moisture_pct']+delta/4.)
        row['leaf_wilt_pct']=max(0.,row['leaf_wilt_pct']-delta/2.)
        row['total_received_ml']=row.get('total_received_ml',0.)+delta
        if delta: row['hours_since_water']=0.
        return row,delta

class PlantSensors(Node):
    def __init__(self,reset=False):
        super().__init__('pai_plant_sensors')
        self.path=ROOT/'plant_scene/live_plants.json'
        initial=json.loads((ROOT/'simulated_plants.json').read_text())
        saved=json.loads(self.path.read_text()) if self.path.exists() and not reset else initial
        self.world=PlantWorld(saved['plants'])
        self.painter=ScenePainter()
        self.pub=self.create_publisher(String,'/pai_plants/state',10)
        self.create_subscription(String,'/pai_watering/plant_delivery',self.delivery,10)
        self.create_timer(.2,self.publish)
        self.previous={}
        self.save()

    def save(self):
        self.path.write_text(json.dumps(dict(schema_version=1,data_source='simulated_sensor',plants=self.world.rows),indent=2)+'\n')

    def delivery(self,msg):
        try:
            m=json.loads(msg.data)
            row,delta=self.world.apply(m['plant_id'],m['session_id'],m['cumulative_ml'])
            if delta: self.save()
        except (ValueError,KeyError,StopIteration) as exc:
            self.get_logger().error(f'Virtual delivery rejected: {exc}')

    def publish(self):
        self.pub.publish(String(data=json.dumps(dict(data_source='simulated_sensor',wall_monotonic_s=time.monotonic(),plants=self.world.rows))))
        for row in self.world.rows:
            key=(round(row['soil_moisture_pct']),round(row['leaf_wilt_pct']))
            if self.previous.get(row['plant_id'])!=key:
                self.painter.plant(row); self.previous[row['plant_id']]=key

def main():
    p=argparse.ArgumentParser(); p.add_argument('--reset-plants',action='store_true'); args=p.parse_args()
    stop=[]
    for sig in (signal.SIGINT,signal.SIGTERM): signal.signal(sig,lambda *_:stop.append(True))
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO); node=PlantSensors(args.reset_plants)
    try:
        while not stop: rclpy.spin_once(node,timeout_sec=.1)
    finally:
        node.save(); node.painter.close(); node.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()
