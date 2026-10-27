"""Record a real Gazebo run with captions derived from ROS measurements and JSONL events."""
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from std_msgs.msg import String
from attach_watering_can import close_view, overview_view

ROOT = Path(__file__).resolve().parent
FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'


def main():
    directory = ROOT / 'submission_capture' / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    directory.mkdir(parents=True)
    env = dict(os.environ, DISPLAY=':0')
    subprocess.run(['mate-screensaver-command', '--deactivate'], env=env, check=True)
    inhibitor = subprocess.Popen(['mate-screensaver-command', '--inhibit',
        '--application-name', 'PAI submission recording', '--reason', 'Record Gazebo demonstration'],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    textfile = directory / 'live.txt'
    def write_text(value):
        temp = directory / 'live.tmp'
        temp.write_text(value, encoding='utf-8')
        temp.replace(textfile)
    write_text('AI給水 巡回デモ\n\n初期化中\n2周を連続実行')
    geometry = subprocess.check_output(['xdpyinfo'], env=env, text=True)
    import re
    w, h = map(int, re.search(r'dimensions:\s+(\d+)x(\d+)', geometry).groups())
    if (w, h) != (1093, 737):
        raise RuntimeError(f'Recording layout changed: {w}x{h}; check the viewport before recording')
    filters = (f'crop=800:514:270:135,scale=932:598,pad=1280:720:0:0:color=0x17372D,'
        f'drawtext=fontfile={FONT}:textfile={textfile}:reload=1:expansion=none:'
        'x=952:y=22:fontsize=23:fontcolor=white:line_spacing=9,'
        f'drawtext=fontfile={FONT}:text=実行映像 2倍速 / 土と葉は模擬値、角度はGazebo関節から測定:'
        'expansion=none:x=25:y=614:fontsize=25:fontcolor=white,'
        f'drawtext=fontfile={FONT}:text=給水量は仮想mL。水滴は説明用表示。:'
        'expansion=none:x=25:y=663:fontsize=24:fontcolor=0xD4E3D9')
    rclpy.init()
    observer = rclpy.create_node('pai_submission_recorder')
    can = {}
    observer.create_subscription(String, '/pai_watering/can_state',
        lambda msg: can.update(json.loads(msg.data)), 10)
    trace = (directory / 'can_samples.jsonl').open('w')
    def sample(msg):
        row = dict(wall_monotonic_s=time.monotonic(), **json.loads(msg.data))
        trace.write(json.dumps(row)+'\n')
    observer.create_subscription(String, '/pai_watering/can_state', sample, 10)
    before = set((ROOT / 'evidence').glob('precision_*'))
    recording_log = (directory / 'ffmpeg.log').open('w')
    ff = subprocess.Popen(['ffmpeg','-hide_banner','-y','-f','x11grab','-draw_mouse','0',
        '-framerate','15','-video_size',f'{w-1}x{h-1}','-i',':0.0','-vf',filters,
        '-an','-c:v','libx264','-preset','ultrafast','-crf','19','-threads','2',
        '-pix_fmt','yuv420p',str(directory/'demo_realtime.mp4')],
        env=env, stdin=subprocess.PIPE, stdout=recording_log, stderr=subprocess.STDOUT, text=True)
    started = time.monotonic()
    output = (directory / 'verify_console.json').open('w')
    process = subprocess.Popen([sys.executable,str(ROOT/'verify_precision_demo.py'),'--laps','2'],
        stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
    logpath = None
    offset = 0
    waypoint = lap = 1
    phase = '巡回'
    decision = {}
    tare = 0.
    target = 0.
    events = []
    camera_jobs = []
    last_caption = 0.
    try:
        while process.poll() is None:
            rclpy.spin_once(observer,timeout_sec=.05)
            if ff.poll() is not None:
                raise RuntimeError('Screen recorder stopped; inspect ffmpeg.log')
            if time.monotonic()-started > 550:
                raise TimeoutError('Recording exceeded the bounded demonstration time')
            if logpath is None:
                added=set((ROOT/'evidence').glob('precision_*'))-before
                if added:
                    paths=list(next(iter(added)).glob('patrol_*.jsonl'))
                    if paths: logpath=paths[0]
            if logpath:
                with logpath.open() as stream:
                    stream.seek(offset)
                    for line in stream:
                        row=json.loads(line);events.append(row)
                        kind=row['event']
                        if kind=='waypoint_reached':
                            waypoint=row['waypoint'];lap=row['lap'];phase='植物の状態を確認'
                        elif kind=='ai_decision':
                            decision=row;target=row['target_ml']
                            phase='AI: 給水' if row['action']=='water' else 'AI: 給水見送り'
                        elif kind=='dose_started':
                            phase='角度を測定して給水';tare=can.get('total_ml',0.)
                            camera_jobs.append(subprocess.Popen([sys.executable,'-c',
                                'from attach_watering_can import close_view; close_view()'],cwd=ROOT))
                        elif kind=='dose_completed':
                            phase=f'給水完了 {row["delivered_ml"]:.1f} mL'
                            camera_jobs.append(subprocess.Popen([sys.executable,'-c',
                                'from attach_watering_can import overview_view; overview_view()'],cwd=ROOT))
                        elif kind=='precision_cycle_completed': phase='巡回経路へ復帰'
                        elif kind=='lap_completed': phase=f'{row["lap"]}周目を完了'
                        elif kind=='finished': phase='2周完了・停止'
                    offset=stream.tell()
            if time.monotonic()-last_caption>.2:
                last_caption=time.monotonic()
                inp=decision.get('inputs',{})
                soil=inp.get('soil_moisture_pct')
                leaf=inp.get('leaf_wilt_pct')
                soiltext='--' if soil is None else f'{soil:.1f}'
                leaftext='--' if leaf is None else f'{leaf:.1f}'
                amount=max(0.,can.get('total_ml',0.)-tare) if '給水' in phase and decision.get('action')=='water' else 0.
                write_text(f'AI給水 巡回デモ\n{lap}/2周目  地点{waypoint}\n\n{phase}\n\n'
                    f'模擬土壌水分 {soiltext}\n模擬しおれ度 {leaftext}\nAI目標 {target:.1f} mL\n\n'
                    f'測定角度 {can.get("angle_deg",0):.1f} 度\n指令角度 {can.get("command_angle_deg",0):.1f} 度\n'
                    f'仮想流量 {can.get("flow_ml_s",0):.1f}\nmL / Sim秒\n\n'
                    f'給水積算 {amount:.1f} mL\n（すべて仮想量）\n\n'
                    '1周目: 鉢1・3へ給水\n2周目: 状態を再判断')
        output.close()
        for job in camera_jobs:
            job.wait(timeout=12)
        result = json.loads((logpath.parent/'result.json').read_text()) if logpath else {}
        (directory/'run_result.json').write_text(json.dumps(result,indent=2)+'\n')
        (directory/'recording.json').write_text(json.dumps(dict(
            capture_start_monotonic=started,wall_seconds=time.monotonic()-started,
            verification_directory=str(logpath.parent) if logpath else None,
            source_resolution=[w,h],viewport=[270,135,800,514],
            events=events,verification=result,
            note='Screen capture in real time. Final movie will explicitly show 2x playback.'),indent=2)+'\n')
        if process.returncode or not result.get('passed'):
            raise RuntimeError('Recorded run did not pass verification')
        print(json.dumps({'recording_directory':str(directory),'passed':True,'run':result}),flush=True)
    finally:
        if process.poll() is None:
            subprocess.run(['ros2','service','call','/pai_patrol/stop','std_srvs/srv/Trigger','{}'],timeout=10)
            try: process.wait(timeout=12)
            except subprocess.TimeoutExpired: os.killpg(process.pid,signal.SIGTERM)
        if ff.poll() is None:
            ff.communicate('q',timeout=20)
        inhibitor.terminate();inhibitor.wait(timeout=5)
        trace.close();recording_log.close()
        observer.destroy_node();rclpy.shutdown()


if __name__=='__main__': main()
