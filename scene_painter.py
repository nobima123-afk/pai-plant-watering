"""Fast local Gazebo visual updates through the bundled native transport helper."""
import json
import math
from pathlib import Path
import select
import subprocess

ROOT=Path(__file__).resolve().parent

class ScenePainter:
    def __init__(self):
        self.process=subprocess.Popen([str(ROOT/'precision_plugin/build/pai_visual_bridge')],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1)
        readable,_,_=select.select([self.process.stdout],[],[],5)
        if not readable or self.process.stdout.readline().strip()!='READY':
            self.process.terminate(); raise RuntimeError('Gazebo visual helper not ready')

    def send(self,text):
        if self.process.poll() is not None: raise RuntimeError('Gazebo visual helper stopped')
        self.process.stdin.write(text.replace('\n',' ')+'\n'); self.process.stdin.flush()

    def plant(self,row):
        parent=f'pai_planter_{row["waypoint"]}::planter'
        dry=max(0.,min(1.,(70-row['soil_moisture_pct'])/70))
        soil=[a+(b-a)*dry for a,b in zip((.16,.095,.045),(.45,.30,.15))]
        def color(name,rgb,pose=''):
            r,g,b=rgb
            self.send(f'name:"{parent}::{name}" parent_name:"{parent}" visible:true {pose} material {{ ambient {{r:{r} g:{g} b:{b} a:1}} diffuse {{r:{r} g:{g} b:{b} a:1}} }}')
        color('soil',soil)
        color('number_plate',(.025,.36,.56) if row['soil_moisture_pct']>=55 else (.82,.38,.04))
        wilt=row['leaf_wilt_pct']/100
        rgb=[a+(b-a)*wilt for a,b in zip((.18,.60,.16),(.62,.47,.12))]
        pitch=-.35+.95*wilt
        for plant,sx in enumerate((-.16,0,.16)):
            for level in range(4):
                for side in range(2):
                    yaw=plant*.55+level*1.8+side*math.pi
                    q=(-math.sin(pitch/2)*math.sin(yaw/2),math.sin(pitch/2)*math.cos(yaw/2),
                       math.cos(pitch/2)*math.sin(yaw/2),math.cos(pitch/2)*math.cos(yaw/2))
                    pose=f'pose {{ position {{ x:{sx} y:0 z:{.27+.09*level} }} orientation {{ x:{q[0]} y:{q[1]} z:{q[2]} w:{q[3]} }} }}'
                    color(f'leaf_{plant}_{level}_{side}',rgb,pose)

    def stream(self,start,end,phase,visible):
        for i in range(8):
            if not visible:
                self.send(f'name:"pai_water_drop_{i}" parent_name:"default" visible:false')
                continue
            t=((i/8+phase)%1)
            x,y,z=[a+(b-a)*t for a,b in zip(start,end)]
            z+=.045*math.sin(math.pi*t)
            self.send(f'name:"pai_water_drop_{i}" parent_name:"default" parent_id:1 type:VISUAL visible:true cast_shadows:false pose {{position {{x:{x} y:{y} z:{z}}} orientation {{x:0 y:0 z:0 w:1}}}} geometry {{type:SPHERE sphere {{radius:.009}}}} material {{ambient {{r:.05 g:.65 b:1 a:1}} diffuse {{r:.05 g:.65 b:1 a:1}}}}')

    def close(self):
        if self.process.poll() is None:
            self.process.stdin.close()
            try: self.process.wait(timeout=3)
            except subprocess.TimeoutExpired: self.process.terminate()
