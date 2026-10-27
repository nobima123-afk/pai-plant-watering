"""Clone the standard model, add a measured can joint, replace only the live burger."""
import json
import math
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET
import rclpy
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from attach_watering_can import Mesh, make_parts
from setup_plant_scene import call, material

ROOT=Path(__file__).resolve().parent
STANDARD=Path('/home/ubuntu/airobot_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf')


def build_model():
    parts=make_parts()
    directory=ROOT/'precision_plugin'
    directory.mkdir(exist_ok=True)
    rack=Mesh()
    rack.box((0,0,.292),(.16,.14,.012))
    for x in (-.062,.062):
        for y in (-.055,.055):
            rack.box((x,y,.225),(.009,.009,.13))
    rack.save(directory/'rack.dae')
    tree=ET.parse(STANDARD)
    model=tree.getroot().find('model')
    model.set('name','burger')
    base=model.find("./link[@name='base_link']")
    rack_visual=ET.fromstring(f'<visual name="pai_can_rack"><geometry><mesh><uri>{(directory/"rack.dae").as_uri()}</uri></mesh></geometry>{material(".28 .32 .35")}</visual>')
    base.append(rack_visual)
    link=ET.fromstring('''<link name="pai_can_link"><pose>.015 0 .30 0 0 0</pose><inertial><pose>0 0 .06 0 0 0</pose><mass>.025</mass><inertia><ixx>.00004</ixx><iyy>.00006</iyy><izz>.00004</izz></inertia></inertial></link>''')
    for name,(_,color) in parts.items():
        if name=='mount': continue
        color=' '.join(map(str,color))
        link.append(ET.fromstring(f'<visual name="pai_{name}"><pose>-.015 0 -.232 0 0 0</pose><geometry><mesh><uri>{(ROOT/"watering_can"/(name+".dae")).as_uri()}</uri></mesh></geometry>{material(color)}<cast_shadows>false</cast_shadows></visual>'))
    model.append(link)
    model.append(ET.fromstring('''<joint name="pai_can_tilt_joint" type="revolute"><parent>base_link</parent><child>pai_can_link</child><axis><xyz>0 1 0</xyz><limit><lower>0</lower><upper>.90</upper><effort>.045</effort><velocity>.95</velocity></limit><dynamics><damping>.001</damping></dynamics></axis></joint>'''))
    model.append(ET.fromstring(f'<plugin name="pai_can_control" filename="{directory/"build"/"libpai_can_control.so"}"><ros><namespace>/pai_watering</namespace></ros></plugin>'))
    path=directory/'burger_precision.sdf'
    ET.indent(tree,space='  ')
    tree.write(path,encoding='unicode',xml_declaration=True)
    return path


def main():
    rclpy.init(); node=rclpy.create_node('pai_can_installer')
    try:
        until=time.monotonic()+1.
        while time.monotonic()<until: rclpy.spin_once(node,timeout_sec=.1)
        if node.count_publishers('/cmd_vel'): raise RuntimeError('Stop patrol/teleop before installation')
        path=build_model()
        source=subprocess.check_output(['gz','model','-m','burger','-p'],text=True,timeout=8)
        values=[float(v) for v in source.split()]
        if len(values)!=6: raise RuntimeError('Could not save original robot pose')
        (ROOT/'precision_plugin'/'pose_before_install.json').write_text(json.dumps(values))
        x,y,z,roll,pitch,yaw=values
        request=SpawnEntity.Request(name='burger',xml=path.read_text())
        request.initial_pose.position.x=x; request.initial_pose.position.y=y
        request.initial_pose.position.z=max(.01,z)
        # Keep the planar pose, let the wheels settle naturally after respawn.
        request.initial_pose.orientation.z=math.sin(yaw/2); request.initial_pose.orientation.w=math.cos(yaw/2)
        deleted=call(node,'/delete_entity',DeleteEntity,DeleteEntity.Request(name='burger'))
        if not deleted.success: raise RuntimeError(deleted.status_message)
        result=call(node,'/spawn_entity',SpawnEntity,request)
        if not result.success:
            request.xml=STANDARD.read_text()
            restore=call(node,'/spawn_entity',SpawnEntity,request)
            raise RuntimeError(f'New model failed; original restored={restore.success}: {result.status_message}')
        print(json.dumps(dict(installed=True,model='burger',source_copy=str(path),original_unchanged=str(STANDARD))))
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__=='__main__': main()
