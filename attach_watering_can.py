#!/usr/bin/env python3
"""Attach a watering-can VISUAL to burger::base_link without changing physics/SDF."""
import argparse
import json
import math
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'watering_can'
PARENT = 'burger::base_link'


def add(a, b):
    return tuple(x+y for x, y in zip(a, b))


def mul(a, scale):
    return tuple(x*scale for x in a)


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def unit(a):
    length = math.sqrt(sum(x*x for x in a))
    return mul(a, 1/length)


class Mesh:
    def __init__(self):
        self.vertices, self.triangles = [], []

    def tube(self, a, b, r0, r1=None, sides=32, cap_start=True, cap_end=True):
        r1 = r0 if r1 is None else r1
        axis = unit(tuple(y-x for x, y in zip(a, b)))
        u = unit(cross(axis, (0, 0, 1) if abs(axis[2]) < .9 else (0, 1, 0)))
        v = cross(axis, u)
        start = len(self.vertices)
        for center, radius in ((a, r0), (b, r1)):
            for i in range(sides):
                theta = 2*math.pi*i/sides
                self.vertices.append(add(center, add(mul(u, radius*math.cos(theta)), mul(v, radius*math.sin(theta)))))
        for i in range(sides):
            j = (i+1) % sides
            self.triangles.extend([(start+i, start+j, start+sides+i),
                                   (start+j, start+sides+j, start+sides+i)])
        for enabled, offset, center, reverse in ((cap_start, 0, a, True), (cap_end, sides, b, False)):
            if enabled:
                middle = len(self.vertices)
                self.vertices.append(center)
                for i in range(sides):
                    tri = (middle, start+offset+i, start+offset+(i+1) % sides)
                    self.triangles.append(tuple(reversed(tri)) if reverse else tri)

    def handle(self):
        # Rear handle in the XZ plane, connected at both ends to the can body.
        points = [(-.025-.065*math.cos(t), 0, .303+.057*math.sin(t))
                  for t in [-math.pi/2 + i*math.pi/32 for i in range(33)]]
        for a, b in zip(points, points[1:]):
            self.tube(a, b, .007, sides=10)

    def box(self, center, size):
        x, y, z = center
        a, b, c = (s/2 for s in size)
        start = len(self.vertices)
        self.vertices.extend([(x+sx*a, y+sy*b, z+sz*c)
                              for sz in (-1, 1) for sy in (-1, 1) for sx in (-1, 1)])
        for face in ((0,2,3,1), (4,5,7,6), (0,1,5,4), (2,6,7,3), (0,4,6,2), (1,3,7,5)):
            self.triangles.extend([(start+face[0],start+face[1],start+face[2]),
                                   (start+face[0],start+face[2],start+face[3])])

    def save(self, path):
        # Duplicate back faces so a thin lip/handle remains visible from either side.
        faces = [f for t in self.triangles for f in (t, tuple(reversed(t)))]
        vertices = ' '.join(str(round(v, 8)) for p in self.vertices for v in p)
        indices = ' '.join(str(i) for face in faces for i in face)
        text = f'''<?xml version="1.0"?><COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1"><asset><created>2026-10-26T00:00:00Z</created><modified>2026-10-26T00:00:00Z</modified><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset><library_geometries><geometry id="part"><mesh><source id="positions"><float_array id="positions-array" count="{len(self.vertices)*3}">{vertices}</float_array><technique_common><accessor source="#positions-array" count="{len(self.vertices)}" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source><vertices id="vertices"><input semantic="POSITION" source="#positions"/></vertices><triangles count="{len(faces)}"><input semantic="VERTEX" source="#vertices" offset="0"/><p>{indices}</p></triangles></mesh></geometry></library_geometries><library_visual_scenes><visual_scene id="Scene"><node id="part"><instance_geometry url="#part"/></node></visual_scene></library_visual_scenes><scene><instance_visual_scene url="#Scene"/></scene></COLLADA>'''
        path.write_text(text, encoding='utf-8')


def make_parts():
    OUT.mkdir(exist_ok=True)
    parts = {}
    mount = Mesh()
    mount.box((0,0,.225), (.16,.14,.012))
    for x in (-.062,.062):
        for y in (-.055,.055):
            mount.box((x,y,.191), (.009,.009,.062))
    parts['mount'] = (mount, (.30,.34,.38))
    body = Mesh()
    body.tube((.015,0,.232), (.015,0,.348), .045, .050, cap_end=False)
    body.handle()
    body.tube((.055,0,.268), (.125,0,.314), .015, .011)
    body.tube((.125,0,.314), (.183,0,.363), .011, .009)
    parts['blue_can'] = (body, (.025,.40,.82))
    rim = Mesh()
    rim.tube((.015,0,.232),(.015,0,.239),.048)
    rim.tube((.015,0,.344),(.015,0,.353),.053,cap_start=False,cap_end=False)
    # Ring has an open centre; the dark inset is a visual water surface.
    parts['rim'] = (rim, (.08,.64,.96))
    dark = Mesh()
    dark.tube((.015,0,.338),(.015,0,.340),.047)
    parts['inset'] = (dark, (.008,.10,.17))
    head = Mesh()
    a, b = (.181,0,.361), (.202,0,.378)
    head.tube(a,b,.011,.025)
    axis = unit(tuple(y-x for x,y in zip(a,b)))
    cap = add(b,mul(axis,.004))
    head.tube(b,cap,.026)
    parts['shower_head'] = (head, (.80,.85,.88))
    holes = Mesh()
    u = (0,1,0)
    v = cross(axis,u)
    for angle in range(0,360,60):
        t = math.radians(angle)
        p = add(cap,add(mul(u,.016*math.cos(t)),mul(v,.016*math.sin(t))))
        holes.tube(add(p,mul(axis,.0004)),add(p,mul(axis,.001)),.0025,sides=10)
    holes.tube(add(cap,mul(axis,.0004)),add(cap,mul(axis,.001)),.0025,sides=10)
    parts['nozzle_holes'] = (holes, (.045,.07,.09))
    for name,(mesh,_) in parts.items():
        mesh.save(OUT/(name+'.dae'))
    return parts


def publish(topic, message):
    subprocess.run(['gz','topic','-p',topic,'-m',message],check=True,timeout=10)


def close_view():
    pose = [float(x) for x in subprocess.check_output(['gz','model','-m','burger','-p'],text=True,timeout=8).split()]
    x,y,z,_,_,yaw = pose
    c,s = math.cos(yaw),math.sin(yaw)
    target = (x+c*.035, y+s*.035, z+.20)
    eye = (x+c*.68-s*(-.78), y+s*.68+c*(-.78), z+.69)
    camera_yaw = math.atan2(target[1]-eye[1],target[0]-eye[0])
    pitch = math.atan2(eye[2]-target[2],math.hypot(target[0]-eye[0],target[1]-eye[1]))
    qx=-math.sin(pitch/2)*math.sin(camera_yaw/2)
    qy=math.sin(pitch/2)*math.cos(camera_yaw/2)
    qz=math.cos(pitch/2)*math.sin(camera_yaw/2)
    qw=math.cos(pitch/2)*math.cos(camera_yaw/2)
    publish('/gazebo/default/gui',f'camera {{ name:"gzclient_camera" pose {{ position {{ x:{eye[0]} y:{eye[1]} z:{eye[2]} }} orientation {{ x:{qx} y:{qy} z:{qz} w:{qw} }} }} view_controller:"orbit" }}')


def overview_view():
    layout=json.loads((ROOT/'plant_scene/layout.json').read_text())
    cx=sum(p[0] for p in layout['goals'])/4
    cy=sum(p[1] for p in layout['goals'])/4
    x,y,z=cx+3.6,cy-4.7,4.4
    yaw=math.atan2(cy-y,cx-x)
    pitch=math.atan2(z-.2,math.hypot(cx-x,cy-y))
    qx=-math.sin(pitch/2)*math.sin(yaw/2)
    qy=math.sin(pitch/2)*math.cos(yaw/2)
    qz=math.cos(pitch/2)*math.sin(yaw/2)
    qw=math.cos(pitch/2)*math.cos(yaw/2)
    publish('/gazebo/default/gui',f'camera {{ name:"gzclient_camera" pose {{ position {{ x:{x} y:{y} z:{z} }} orientation {{ x:{qx} y:{qy} z:{qz} w:{qw} }} }} view_controller:"orbit" }}')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--view',choices=['close','overview','keep'],default='close')
    args=parser.parse_args()
    info=subprocess.check_output(['gz','model','-m','burger','-i'],text=True,timeout=10)
    links=dict((name,int(number)) for number,name in re.findall(r'link \{\s+id: (\d+)\s+name: "([^"]+)"',info))
    if PARENT not in links:
        raise RuntimeError('The standard burger/base_link must be present')
    if 'burger::pai_can_link' in links:
        if args.view=='close': close_view()
        elif args.view=='overview':
            overview_view()
        print('Measured-joint watering can already installed; no duplicate visual was added.')
        return
    parts=make_parts()
    names=[]
    for name,(_,color) in parts.items():
        visual_name=PARENT+'::pai_can_'+name
        names.append(visual_name)
        r,g,b=color
        message=f'''name:"{visual_name}" parent_name:"{PARENT}" parent_id:{links[PARENT]}
type:VISUAL is_static:false visible:true cast_shadows:false
pose {{ position {{ x:0 y:0 z:0 }} orientation {{ x:0 y:0 z:0 w:1 }} }}
geometry {{ type:MESH mesh {{ filename:"{(OUT/(name+'.dae')).as_uri()}" scale {{ x:1 y:1 z:1 }} }} }}
material {{ ambient {{ r:{r} g:{g} b:{b} a:1 }} diffuse {{ r:{r} g:{g} b:{b} a:1 }} specular {{ r:.18 g:.18 b:.18 a:1 }} }}'''
        (OUT/(name+'.pbtxt')).write_text(message,encoding='utf-8')
        publish('/gazebo/default/visual',message)
    if args.view=='close':
        close_view()
    elif args.view=='overview':
        overview_view()
    manifest=dict(attachment='watering_can_visual',parent=PARENT,parent_id=links[PARENT],
                  visual_names=names,local_coordinates=True,physics='unchanged',
                  water_flow=False,notes='Client-side accessory; reapply after restarting gzclient.')
    (OUT/'attachment.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(manifest,indent=2))


if __name__=='__main__':
    main()
