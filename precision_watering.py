"""Closed-loop dosing with measured Gazebo joint angle and simulated flow feedback."""
import csv
import hashlib
import json
import math
from pathlib import Path
import time
import uuid
from std_msgs.msg import Float64, String
from dose_model import DoseModel, values
from scene_painter import ScenePainter

ROOT=Path(__file__).resolve().parent

class PrecisionWatering:
    def __init__(self,node):
        self.node=node; self.model=DoseModel(); self.phase='idle'; self.active=False
        self.plants=None; self.plant_time=0.; self.can=None; self.can_time=0.
        self.current=None; self.session_id=None; self.tare=0.; self.target=0.; self.peak=0.
        self.started=0.; self.phase_time=0.; self.last_draw=0.; self.upright_since=None
        self.last_progress=0.
        self.layout=json.loads((ROOT/'plant_scene/layout.json').read_text())
        self.painter=ScenePainter()
        self.tilt=node.create_publisher(Float64,'/pai_watering/tilt_cmd',10)
        self.delivery=node.create_publisher(String,'/pai_watering/plant_delivery',10)
        node.create_subscription(String,'/pai_plants/state',self.on_plants,10)
        node.create_subscription(String,'/pai_watering/can_state',self.on_can,10)
        self.trace_path=node.log_path.with_name(node.log_path.stem+'_precision.csv')
        self.trace=self.trace_path.open('x',newline='')
        self.writer=csv.writer(self.trace)
        self.writer.writerow(['elapsed_s','sim_time','phase','waypoint','command_deg','angle_deg',
                              'flow_ml_s','total_ml','session_ml','target_ml','tank_ml','robot_x','robot_y'])
        self.model_sha=hashlib.sha256((ROOT/'ai_model/dose_model.json').read_bytes()).hexdigest()
        self.stop_tilt()

    def on_plants(self,msg):
        try:
            state=json.loads(msg.data)
            if state['data_source']!='simulated_sensor': return
            for row in state['plants']: values(row)
            self.plants=state['plants']; self.plant_time=time.monotonic()
        except (ValueError,KeyError,TypeError): pass

    def on_can(self,msg):
        try:
            state=json.loads(msg.data)
            for key in ('sim_time','angle_deg','angle_rad','flow_ml_s','total_ml','tank_ml'):
                if not math.isfinite(state[key]): return
            self.can=state; self.can_time=time.monotonic()
            if self.active: self.report_delivery()
            pose=self.node.pose or (None,None,None)
            self.writer.writerow([time.monotonic()-self.node.started,state['sim_time'],self.phase,
                self.current['waypoint'] if self.current else '',state['command_angle_deg'],state['angle_deg'],
                state['flow_ml_s'],state['total_ml'],self.amount(),self.target,state['tank_ml'],pose[0],pose[1]])
            self.trace.flush()
        except (ValueError,KeyError,TypeError): pass

    def ready(self):
        return (self.plants is not None and self.can is not None
                and time.monotonic()-self.plant_time<1.2 and time.monotonic()-self.can_time<.8)

    def near_planter(self):
        if self.node.pose is None: return False
        x,y,_=self.node.pose
        return any(math.hypot(x-p['xy'][0],y-p['xy'][1])<.65 for p in self.layout['planters'])

    def stop_tilt(self):
        self.tilt.publish(Float64(data=0.))

    def amount(self):
        return max(0.,self.can['total_ml']-self.tare) if self.can and self.current else 0.

    def report_delivery(self):
        if self.current and self.session_id:
            self.delivery.publish(String(data=json.dumps(dict(plant_id=self.current['plant_id'],
                session_id=self.session_id,cumulative_ml=min(200.,self.amount())))))

    def plan(self,waypoint):
        if not self.ready(): raise ValueError('Live virtual soil/leaf or measured can sensors unavailable')
        row=next(r for r in self.plants if r['waypoint']==waypoint)
        prediction=self.model.predict(row)
        label=f'給水量 {prediction["target_ml"]:.1f} mL' if prediction['action']=='water' else '給水不要'
        return dict(data_source='simulated_sensor',actuator='gazebo_joint_virtual_flow',
                    plant_id=row['plant_id'],inputs={k:row[k] for k in self.model.m['features']},
                    model_sha256=self.model_sha,**prediction,
                    message=f'【模擬センサーAI】地点{waypoint}: 土壌水分{row["soil_moisture_pct"]:.1f}%、葉のしおれ{row["leaf_wilt_pct"]:.1f}% → {label}')

    def begin(self,waypoint,decision,route_goal):
        if self.can['tank_ml']<decision['target_ml']+5: raise ValueError('Virtual can needs refill/reinstallation')
        self.current=dict(next(r for r in self.plants if r['waypoint']==waypoint))
        self.target=decision['target_ml']; self.tare=self.can['total_ml']; self.peak=0.
        self.session_id=uuid.uuid4().hex; self.route_goal=route_goal
        self.plant_xy=self.layout['planters'][waypoint-1]['xy']
        dx,dy=[a-b for a,b in zip(self.plant_xy,route_goal)]; d=math.hypot(dx,dy)
        self.dock=(self.plant_xy[0]-.50*dx/d,self.plant_xy[1]-.50*dy/d)
        self.active=True; self.phase='approach'; self.started=self.phase_time=time.monotonic()
        self.node.event('dose_planned',waypoint=waypoint,target_ml=self.target,
                        soil_before=self.current['soil_moisture_pct'],leaf_before=self.current['leaf_wilt_pct'])

    def move_to(self,goal):
        x,y,yaw=self.node.pose
        distance=math.hypot(goal[0]-x,goal[1]-y)
        error=math.atan2(math.sin(math.atan2(goal[1]-y,goal[0]-x)-yaw),
                         math.cos(math.atan2(goal[1]-y,goal[0]-x)-yaw))
        if distance<.025:
            self.node.command(); return True
        self.node.command(0. if abs(error)>.12 else min(.055,.8*distance),max(-.5,min(.5,1.8*error)))
        return False

    def step(self):
        now=time.monotonic()
        if not self.ready():
            self.node.command(); self.stop_tilt()
            if self.active: raise ValueError('Precision sensor timeout; dosing stopped')
            return
        if not self.active:
            self.stop_tilt(); return
        if now-self.started>80: raise ValueError('Precision cycle timed out')
        self.peak=max(self.peak,self.can['angle_deg'])
        if self.phase=='approach':
            self.stop_tilt()
            if self.move_to(self.dock):
                self.phase='settle'; self.phase_time=now
        elif self.phase=='settle':
            self.node.command(); self.stop_tilt()
            if now-self.phase_time>.7:
                self.tare=self.can['total_ml']
                self.phase='pour'; self.phase_time=now
                self.node.event('dose_started',waypoint=self.current['waypoint'],target_ml=self.target,
                                measured_angle_deg=self.can['angle_deg'])
        elif self.phase=='pour':
            self.node.command()
            if now-self.last_progress>1.:
                self.last_progress=now
                self.node.event('dose_progress',measured_angle_deg=self.can['angle_deg'],
                    command_angle_deg=self.can['command_angle_deg'],flow_ml_s=self.can['flow_ml_s'],
                    delivered_ml=self.amount(),target_ml=self.target)
            if now-self.phase_time>22: raise ValueError('Dose regulation timed out')
            remaining=self.target-self.amount()
            if remaining<.8:
                self.phase='upright'; self.phase_time=now; self.stop_tilt()
            else:
                desired_flow=min(25.,remaining*1.8)
                gain=1.2*math.sqrt(max(1.,self.can['tank_ml'])/750.)
                degrees=min(50.,20.+desired_flow/gain)
                self.tilt.publish(Float64(data=math.radians(degrees)))
        elif self.phase=='upright':
            self.node.command(); self.stop_tilt()
            row=next(r for r in self.plants if r['plant_id']==self.current['plant_id'])
            acknowledged=row.get('total_received_ml',0)-self.current.get('total_received_ml',0)>=self.amount()-.05
            if abs(self.can['angle_deg'])<1. and self.can['flow_ml_s']<.05 and acknowledged:
                if abs(self.amount()-self.target)>5: raise ValueError('Volume error exceeds 5 virtual mL')
                self.node.event('dose_completed',waypoint=self.current['waypoint'],target_ml=self.target,
                    delivered_ml=round(self.amount(),3),error_ml=round(self.amount()-self.target,3),
                    peak_measured_angle_deg=round(self.peak,3),soil_before=self.current['soil_moisture_pct'],
                    soil_after=row['soil_moisture_pct'],leaf_before=self.current['leaf_wilt_pct'],leaf_after=row['leaf_wilt_pct'])
                self.phase='return'; self.phase_time=now
            elif now-self.phase_time>5: raise ValueError('Upright/plant acknowledgement timeout')
        elif self.phase=='return':
            self.stop_tilt()
            if self.move_to(self.route_goal):
                self.active=False; self.phase='idle'; self.current=None
                self.node.event('precision_cycle_completed')
        if now-self.last_draw>.12:
            self.last_draw=now
            show=self.phase in ('pour','upright') and self.can['flow_ml_s']>.1
            x,y,yaw=self.node.pose; a=self.can['angle_rad']
            lx=.015+.187*math.cos(a)+.146*math.sin(a)
            z=.30-.187*math.sin(a)+.146*math.cos(a)
            start=(x+math.cos(yaw)*lx,y+math.sin(yaw)*lx,z+.009)
            self.painter.stream(start,(*self.plant_xy,.22),now*2,show)

    def suspend(self):
        self.stop_tilt()

    def close(self):
        self.stop_tilt()
        self.painter.stream((0,0,0),(0,0,0),0,False)
        self.painter.close(); self.trace.close()
