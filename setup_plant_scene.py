#!/usr/bin/env python3
"""Add four self-contained 3D planters to the running Gazebo, via Docker/ROS."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

import rclpy
from gazebo_msgs.srv import SpawnEntity, DeleteEntity, GetModelList
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

ROOT = Path(__file__).resolve().parent
SCENE = ROOT / 'plant_scene'
MANAGED = [f'pai_planter_{i}' for i in range(1, 5)] + ['pai_patrol_markers']


def material(color):
    return f'<material><ambient>{color} 1</ambient><diffuse>{color} 1</diffuse><specular>0.06 0.06 0.06 1</specular></material>'


def visual(name, geometry, pose, color):
    return f'<visual name="{name}"><pose>{pose}</pose><geometry>{geometry}</geometry>{material(color)}<cast_shadows>false</cast_shadows></visual>'


def box(name, size, pose, color):
    return visual(name, f'<box><size>{size}</size></box>', pose, color)


def cylinder(name, radius, length, pose, color):
    return visual(name, f'<cylinder><radius>{radius}</radius><length>{length}</length></cylinder>', pose, color)


def leaf_mesh():
    # A closed, slightly cupped leaf, with front and back faces, in local XY.
    vertices = []
    for i in range(13):
        t = i / 12
        half = .065 * math.sin(math.pi*t)
        for sign in (-1, 0, 1):
            vertices.extend([.23*t, sign*half, .023*math.sin(math.pi*t)*(1-abs(sign))])
    triangles = []
    for i in range(12):
        for j in range(2):
            a, b = i*3+j, (i+1)*3+j
            for tri in ((a,b,a+1), (a+1,b,b+1)):
                triangles.extend(tri)
                triangles.extend(reversed(tri))
    points = ' '.join(map(str, vertices))
    indices = ' '.join(map(str, triangles))
    count = len(vertices)//3
    text = f'''<?xml version="1.0"?><COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
<asset><created>2026-10-26T00:00:00Z</created><modified>2026-10-26T00:00:00Z</modified><unit meter="1" name="meter"/><up_axis>Z_UP</up_axis></asset>
<library_geometries><geometry id="leaf"><mesh><source id="positions"><float_array id="positions-array" count="{len(vertices)}">{points}</float_array><technique_common><accessor source="#positions-array" count="{count}" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source><vertices id="vertices"><input semantic="POSITION" source="#positions"/></vertices><triangles count="{len(triangles)//3}"><input semantic="VERTEX" source="#vertices" offset="0"/><p>{indices}</p></triangles></mesh></geometry></library_geometries>
<library_visual_scenes><visual_scene id="Scene"><node id="leaf-node"><instance_geometry url="#leaf"/></node></visual_scene></library_visual_scenes><scene><instance_visual_scene url="#Scene"/></scene></COLLADA>'''
    (SCENE / 'leaf.dae').write_text(text, encoding='utf-8')


def planter(number, dry):
    parts = []
    terra = '.57 .23 .11'
    rim = '.78 .38 .20'
    parts.append(box('base', '.52 .32 .18', '0 0 .10 0 0 0', terra))
    parts.append(box('soil', '.46 .26 .035', '0 0 .20 0 0 0', '.45 .30 .15' if dry else '.16 .095 .045'))
    for name, size, pose in [('front', '.56 .035 .05', '0 -.16 .215 0 0 0'),
                              ('back', '.56 .035 .05', '0 .16 .215 0 0 0'),
                              ('left', '.035 .32 .05', '-.26 0 .215 0 0 0'),
                              ('right', '.035 .32 .05', '.26 0 .215 0 0 0')]:
        parts.append(box(name, size, pose, rim))
    for plant, sx in enumerate((-.16, 0, .16)):
        height = .40 + (.04 if plant == 1 else 0)
        parts.append(cylinder(f'stem_{plant}', .009, height,
                              f'{sx} 0 {0.21+height/2} 0 0 0', '.28 .37 .10'))
        for level in range(4):
            z = .27 + .09*level
            for side in range(2):
                yaw = plant*.55 + level*1.8 + side*math.pi
                pitch = .48 if dry else -.35
                color = ('.62 .47 .12' if level % 2 else '.40 .42 .10') if dry else ('.16 .48 .14' if level % 2 else '.24 .63 .20')
                parts.append(visual(f'leaf_{plant}_{level}_{side}',
                    f'<mesh><uri>{(SCENE / "leaf.dae").as_uri()}</uri><scale>.85 .85 .85</scale></mesh>',
                    f'{sx} 0 {z} 0 {pitch} {yaw}', color))
    # Numbered vertical plaque, built from geometry rather than a downloaded font.
    plaque = '.82 .38 .04' if dry else '.025 .36 .56'
    parts.append(box('number_plate', '.15 .025 .23', '0 -.188 .255 0 0 0', plaque))
    segments = {1:'bc', 2:'abdeg', 3:'abcdg', 4:'bcfg'}[number]
    positions = {'a':(0,.080,True), 'b':(.042,.041,False), 'c':(.042,-.041,False),
                 'd':(0,-.080,True), 'e':(-.042,-.041,False), 'f':(-.042,.041,False), 'g':(0,0,True)}
    for seg in segments:
        x, dz, horizontal = positions[seg]
        size = '.084 .006 .014' if horizontal else '.014 .006 .078'
        parts.append(box('digit_'+seg, size, f'{x} -.204 {0.255+dz} 0 0 0', '.98 .98 .90'))
    collision = '<collision name="planter_collision"><pose>0 0 .12 0 0 0</pose><geometry><box><size>.56 .36 .24</size></box></geometry></collision>'
    return f'<sdf version="1.6"><model name="pai_planter_{number}"><static>true</static><link name="planter">{collision}{"".join(parts)}</link></model></sdf>'


def markers(layout):
    parts = []
    points = [layout['origin'][:2], *layout['goals']]
    for i, ((ax, ay), (bx, by)) in enumerate(zip(points, points[1:])):
        yaw = math.atan2(by-ay, bx-ax)
        for j in range(8):
            t = (j+.5)/8
            parts.append(box(f'route_{i}_{j}', '.075 .025 .004',
                f'{ax+(bx-ax)*t} {ay+(by-ay)*t} .006 0 0 {yaw}', '.10 .55 .65'))
    for i, (x, y) in enumerate(layout['goals'], 1):
        parts.append(cylinder(f'stop_{i}', .12, .004, f'{x} {y} .009 0 0 0', '.96 .73 .10'))
    return f'<sdf version="1.6"><model name="pai_patrol_markers"><static>true</static><link name="markers">{"".join(parts)}</link></model></sdf>'


def call(node, service, kind, request):
    client = node.create_client(kind, service)
    try:
        if not client.wait_for_service(timeout_sec=8):
            raise RuntimeError(f'Service unavailable: {service}')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=15)
        if not future.done() or future.result() is None:
            raise RuntimeError(f'Service timed out: {service}')
        return future.result()
    finally:
        node.destroy_client(client)


def configure_view(layout):
    cx = sum(p[0] for p in layout['goals'])/4
    cy = sum(p[1] for p in layout['goals'])/4
    px, py, pz = cx+3.6, cy-4.7, 4.4
    yaw = math.atan2(cy-py, cx-px)
    pitch = math.atan2(pz-.2, math.hypot(cx-px, cy-py))
    qx = -math.sin(pitch/2)*math.sin(yaw/2)
    qy = math.sin(pitch/2)*math.cos(yaw/2)
    qz = math.cos(pitch/2)*math.sin(yaw/2)
    qw = math.cos(pitch/2)*math.cos(yaw/2)
    message = f'camera {{ name: "gzclient_camera" pose {{ position {{ x:{px} y:{py} z:{pz} }} orientation {{ x:{qx} y:{qy} z:{qz} w:{qw} }} }} view_controller:"orbit" }}'
    subprocess.run(['gz','topic','-p','/gazebo/default/gui','-m',message], check=True, timeout=8)
    # Remove only the client-only laser drawing; the sensor and /scan remain active.
    request = 'id:7821 request:"entity_delete" data:"burger::base_scan::hls_lfcd_lds_GUIONLY_laser_vis"'
    subprocess.run(['gz','topic','-p','/gazebo/default/request','-m',request], check=True, timeout=8)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--refresh', action='store_true', help='Replace only this script\'s five named models')
    args = parser.parse_args()
    SCENE.mkdir(exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('pai_plant_scene_setup')
    try:
        # A short discovery interval; refuse changes while a driver is running.
        until = time.monotonic()+1.2
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.1)
        if node.count_publishers('/cmd_vel'):
            raise RuntimeError('Stop all patrol/teleop processes before adding the scene')
        path = SCENE / 'layout.json'
        if path.exists():
            layout = json.loads(path.read_text())
        else:
            poses = []
            sub = node.create_subscription(Odometry, '/odom', lambda m: poses.append(m), qos_profile_sensor_data)
            until = time.monotonic()+8
            while not poses and time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.1)
            node.destroy_subscription(sub)
            if not poses:
                raise RuntimeError('No odometry; scene was not changed')
            p, q = poses[-1].pose.pose.position, poses[-1].pose.pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            c, s = math.cos(yaw), math.sin(yaw)
            def transform(x, y):
                return [p.x+c*x-s*y, p.y+s*x+c*y]
            layout = dict(schema_version=1, side=1.0, origin=[p.x,p.y,yaw],
                          goals=[transform(x,y) for x,y in [(1,0),(1,1),(0,1),(0,0)]],
                          planters=[dict(name=f'pai_planter_{i}', waypoint=i, plant_id=f'plant_{"ABCD"[i-1]}',
                                         xy=transform(x,y), dry=i in (1,3))
                                    for i,(x,y) in enumerate([(1.7,-.7),(1.7,1.7),(-.7,1.7),(-.7,-.7)],1)],
                          note='3D scenery only; AI still uses explicitly simulated observations.')
            path.write_text(json.dumps(layout,indent=2)+'\n', encoding='utf-8')
        leaf_mesh()
        response = call(node,'/get_model_list',GetModelList,GetModelList.Request())
        if not response.success:
            raise RuntimeError('Could not read current model list')
        existing = set(response.model_names)
        if args.refresh:
            for name in MANAGED:
                if name in existing:
                    response = call(node,'/delete_entity',DeleteEntity,DeleteEntity.Request(name=name))
                    if not response.success:
                        raise RuntimeError(response.status_message)
                    existing.remove(name)
        models = [(p['name'], planter(p['waypoint'],p['dry']), p['xy']) for p in layout['planters']]
        models.append(('pai_patrol_markers', markers(layout), [0,0]))
        for name, xml, xy in models:
            ET.fromstring(xml)
            (SCENE/(name+'.sdf')).write_text(xml, encoding='utf-8')
            if name not in existing:
                request = SpawnEntity.Request(name=name, xml=xml)
                request.initial_pose.position.x, request.initial_pose.position.y = map(float, xy)
                request.initial_pose.orientation.w = 1.
                response = call(node,'/spawn_entity',SpawnEntity,request)
                if not response.success:
                    raise RuntimeError(response.status_message)
            print(f'Visible model ready: {name}', flush=True)
        configure_view(layout)
        result = call(node,'/get_model_list',GetModelList,GetModelList.Request())
        if not set(MANAGED).issubset(result.model_names):
            raise RuntimeError('Scene verification failed')
        print(json.dumps(dict(scene_ready=True, models=MANAGED, layout=str(path)),indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
