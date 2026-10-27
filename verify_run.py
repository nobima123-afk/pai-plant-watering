"""Record an actual Gazebo run and check the result; run after sourcing ROS."""
import argparse
import csv
import json
import math
from pathlib import Path
import subprocess
import sys
import time
from datetime import datetime, timezone

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--stop-after', type=float, default=0)
    parser.add_argument('--watering-demo', action='store_true')
    parser.add_argument('--stop-during-watering', action='store_true',
                        help='Request stop 0.5 s after the first simulated watering decision')
    args = parser.parse_args()
    if args.stop_during_watering and not args.watering_demo:
        parser.error('--stop-during-watering requires --watering-demo')
    requested_stop = bool(args.stop_after or args.stop_during_watering)
    root = Path(__file__).resolve().parent
    directory = root / 'evidence' / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
    directory.mkdir(parents=True)
    rclpy.init()
    node = rclpy.create_node('pai_patrol_verifier')
    latest = {'odom': None, 'cmd': None}
    received_decisions = []
    node.create_subscription(Odometry, '/odom', lambda m: latest.update(odom=m), qos_profile_sensor_data)
    node.create_subscription(Twist, '/cmd_vel', lambda m: latest.update(cmd=m), 10)
    node.create_subscription(String, '/pai_watering/decision',
                             lambda m: received_decisions.append(json.loads(m.data)), 10)
    client = node.create_client(Trigger, '/pai_patrol/stop')
    started = time.monotonic()
    future = None
    with (directory / 'console.log').open('w') as console, (directory / 'trajectory.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['wall_elapsed_s', 'wall_monotonic_s', 'odom_stamp_s', 'x_m', 'y_m', 'actual_v_mps',
                         'actual_w_radps', 'command_v_mps', 'command_w_radps'])
        command = [sys.executable, str(root / 'patrol.py'), '--log-dir', str(directory)]
        if args.watering_demo:
            command.append('--watering-demo')
        process = subprocess.Popen(command, stdout=console, stderr=subprocess.STDOUT)
        exited_at = None
        last_record = 0
        try:
            while True:
                rclpy.spin_once(node, timeout_sec=0.05)
                now = time.monotonic()
                first_water = next((e for e in received_decisions if e.get('action') == 'water'), None)
                stop_due = ((args.stop_after and now - started >= args.stop_after)
                            or (args.stop_during_watering and first_water is not None
                                and now >= first_water['wall_monotonic_s'] + .5))
                if stop_due and future is None and client.service_is_ready():
                    future = client.call_async(Trigger.Request())
                if latest['odom'] is not None and latest['cmd'] is not None and now - last_record >= 0.2:
                    m, c = latest['odom'], latest['cmd']
                    p, v = m.pose.pose.position, m.twist.twist
                    writer.writerow([now-started, now, m.header.stamp.sec + m.header.stamp.nanosec/1e9,
                                     p.x, p.y, v.linear.x, v.angular.z, c.linear.x, c.angular.z])
                    stream.flush()
                    last_record = now
                if process.poll() is not None:
                    exited_at = exited_at or now
                    if now - exited_at >= 1.5:
                        break
                if now - started > 320:
                    raise TimeoutError('Verifier timed out')
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=10)
    logs = list(directory.glob('patrol_*.jsonl'))
    events = [json.loads(line) for line in logs[0].read_text().splitlines()] if logs else []
    reached = [e for e in events if e['event'] == 'waypoint_reached']
    result = next((e.get('result') for e in reversed(events) if e['event'] == 'finished'), None)
    m, c = latest['odom'], latest['cmd']
    stopped = (c is not None and c.linear.x == 0 and c.angular.z == 0 and m is not None
               and abs(m.twist.twist.linear.x) < 0.01 and abs(m.twist.twist.angular.z) < 0.02)
    expected = 'stopped_by_service' if requested_stop else 'completed'
    success = process.returncode == 0 and result == expected and stopped
    if not requested_stop:
        success = success and [e['waypoint'] for e in reached] == [1, 2, 3, 4]
    decisions = [e for e in events if e['event'] == 'ai_decision']
    actions = [e for e in events if e['event'] == 'watering_simulated']
    skipped = [e for e in events if e['event'] == 'watering_skipped']
    if args.watering_demo and not requested_stop:
        success = (success and [e['waypoint'] for e in decisions] == [1, 2, 3, 4]
                   and [e['action'] for e in decisions] == ['water', 'skip', 'water', 'skip']
                   and [e['waypoint'] for e in actions] == [1, 3]
                   and [e['waypoint'] for e in skipped] == [2, 4]
                   and [e.get('action') for e in received_decisions] == ['water', 'skip', 'water', 'skip']
                   and all(e['data_source'] == 'simulated' for e in decisions))
    # Verify that AI and simulated watering holds contain no moving commands.
    hold_intervals = []
    for e in decisions:
        start = next(r['wall_monotonic_s'] for r in reached
                     if r['waypoint'] == e['waypoint'] and r['lap'] == e['lap'])
        end = e['wall_monotonic_s']
        action = next((a for a in actions if a['waypoint'] == e['waypoint'] and a['lap'] == e['lap']), None)
        if action:
            end = action['wall_monotonic_s'] + action['duration_s']
        hold_intervals.append((start + .5, end - .1))
    moving_during_hold = []
    hold_samples = 0
    with (directory / 'trajectory.csv').open(newline='') as stream:
        for row in csv.DictReader(stream):
            stamp = float(row['wall_monotonic_s'])
            if any(start <= stamp <= end for start, end in hold_intervals):
                hold_samples += 1
                if (float(row['command_v_mps']) != 0 or float(row['command_w_radps']) != 0
                        or abs(float(row['actual_v_mps'])) > .01
                        or abs(float(row['actual_w_radps'])) > .02):
                    moving_during_hold.append(row)
    if args.watering_demo and not requested_stop:
        success = success and hold_samples >= 4 and not moving_during_hold
    finished_at = next((e['wall_monotonic_s'] for e in events if e['event'] == 'finished'), 0)
    stopped_during_watering = any(a['wall_monotonic_s'] <= finished_at
                                  <= a['wall_monotonic_s'] + a['duration_s'] for a in actions)
    if args.stop_during_watering:
        success = (success and stopped_during_watering and future is not None
                   and future.done() and future.result().success)
    report = dict(passed=bool(success), result=result, exit_code=process.returncode,
                  wall_seconds=round(time.monotonic()-started, 2),
                  watering_demo=args.watering_demo,
                  decision_topic_messages=len(received_decisions),
                  ai_decisions=[{k: e[k] for k in ('waypoint', 'plant_id', 'action', 'inputs', 'model_water_probability', 'data_source')} for e in decisions],
                  simulated_watering_count=len(actions),
                  stopped_during_simulated_watering=stopped_during_watering,
                  stationary_ai_hold_samples=hold_samples,
                  moving_during_ai_hold_count=len(moving_during_hold),
                  waypoints=[dict(waypoint=e['waypoint'], error_m=e['error_m'], pose=e['pose']) for e in reached],
                  stopped=bool(stopped), stop_service_success=(future.result().success if future and future.done() else None),
                  final_position=([m.pose.pose.position.x, m.pose.pose.position.y] if m else None),
                  final_linear_speed=(m.twist.twist.linear.x if m else None),
                  final_angular_speed=(m.twist.twist.angular.z if m else None))
    (directory / 'result.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(dict(directory=str(directory), **report), indent=2), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if success else 1


if __name__ == '__main__':
    raise SystemExit(main())
