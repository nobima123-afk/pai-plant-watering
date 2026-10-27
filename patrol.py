#!/usr/bin/env python3
"""One ROS 2 node: relative square patrol with odometry feedback and scan stop."""
import argparse
import json
import math
from pathlib import Path
import signal
import time
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from std_msgs.msg import String


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def front_distance(scan):
    """+inf means clear; missing/invalid front returns mean unknown (None)."""
    distances = []
    for i, value in enumerate(scan.ranges):
        if abs(wrap(scan.angle_min + i * scan.angle_increment)) > math.radians(30):
            continue
        if value == math.inf:
            distances.append(value)
        elif math.isfinite(value) and scan.range_min <= value <= scan.range_max:
            distances.append(value)
        elif math.isfinite(value) and 0 < value < scan.range_min:
            distances.append(0.0)  # too close to measure: stop
    return min(distances) if distances else None


class Patrol(Node):
    def __init__(self, args):
        super().__init__('pai_patrol')
        self.args = args
        self.inspector = None
        self.precision = None
        self.scene_origin = None
        self.pending_inspection = None
        self.hold_state = 'inspection_wait'
        if getattr(args, 'watering_demo', False):
            from watering_ai import SimulatedInspection
            self.inspector = SimulatedInspection(args.observations)
            layout_path = Path(__file__).resolve().parent / 'plant_scene' / 'layout.json'
            if layout_path.exists() and not getattr(args, 'free_route', False):
                layout = json.loads(layout_path.read_text(encoding='utf-8'))
                origin = layout.get('origin', [])
                if (layout.get('schema_version') != 1 or len(origin) != 3
                        or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in origin)
                        or not math.isclose(layout.get('side', -1), args.side, abs_tol=1e-6)):
                    raise ValueError('Plant scene route is incompatible; use --side 1 or --free-route')
                self.scene_origin = tuple(origin)
        self.decision_pub = (self.create_publisher(String, '/pai_watering/decision', 10)
                             if self.inspector else None)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_service(Trigger, '/pai_patrol/stop', self.on_stop)
        self.pose = None
        self.origin = None
        self.odom_time = self.scan_time = 0.0
        self.front = None
        self.blocked = False
        self.index = 0
        self.lap = 0
        self.hold_until = 0.0
        self.done = False
        self.result = None
        self.state = None
        self.started = time.monotonic()
        self.goals = []
        log_dir = Path(args.log_dir)
        log_dir.mkdir(parents=True, exist_ok=True)
        name = datetime.now(timezone.utc).strftime('patrol_%Y%m%dT%H%M%S_%fZ.jsonl')
        self.log_path = log_dir / name
        self.log_file = self.log_path.open('x', encoding='utf-8')
        if getattr(args, 'precision_watering', False):
            from precision_watering import PrecisionWatering
            self.precision = PrecisionWatering(self)
        self.event('started', side=args.side, laps=args.laps, log=str(self.log_path))
        if self.inspector:
            self.event('ai_enabled', data_source='simulated_sensor' if self.precision else 'simulated',
                       actuator='gazebo_joint_virtual_flow' if self.precision else 'log_only',
                       algorithm='relu_mlp_regression' if self.precision else 'logistic_regression', training_data_source='synthetic',
                       model_sha256=self.precision.model_sha if self.precision else self.inspector.model.sha256,
                       observations_sha256=self.inspector.observations_sha256)
        if self.scene_origin:
            self.event('scene_route', origin=self.scene_origin)
        # Wall timer deliberately stays active if Gazebo pauses or /clock stops.
        self.create_timer(0.1, self.tick)

    def event(self, kind, **fields):
        row = dict(event=kind, elapsed=round(time.monotonic() - self.started, 3),
                   wall_monotonic_s=time.monotonic(),
                   pose=self.pose, **fields)
        self.log_file.write(json.dumps(row, allow_nan=False) + '\n')
        self.log_file.flush()
        if getattr(self.args, 'verbose', False):
            self.get_logger().info(json.dumps(row, ensure_ascii=False))
        else:
            message = None
            if kind == 'started':
                message = f'巡回開始：1辺{self.args.side} m、{self.args.laps}周。記録: {self.log_path}'
            elif kind == 'ai_enabled':
                message = ('給水量AI有効：土と葉は模擬センサー、角度はGazebo関節から測定します。'
                           if self.precision else 'ローカルAI有効：植物データは模擬値、散水は待機と記録のみです。')
            elif kind == 'route':
                message = '開始位置を取得しました。巡回を始めます。'
            elif kind == 'scene_route':
                message = '表示したプランターの位置に合わせた固定ルートを使います。'
            elif kind == 'waypoint_reached':
                message = f'地点{fields["waypoint"]}に到着（{fields["lap"]}周目）。停止して確認します。'
            elif kind == 'ai_decision':
                message = fields['message']
            elif kind == 'watering_simulated':
                message = f'【模擬動作】散水を想定して{fields["duration_s"]}秒待機します。実際の散水はしません。'
            elif kind == 'watering_skipped':
                message = '【模擬動作】散水を見送り、次へ進みます。'
            elif kind == 'dose_started':
                message = f'【給水制御】目標{fields["target_ml"]:.1f} mL。実測角度を使ってじょうろを調整します。'
            elif kind == 'dose_completed':
                message = (f'【給水完了】目標{fields["target_ml"]:.1f} / 実績{fields["delivered_ml"]:.1f} mL、'
                           f'誤差{fields["error_ml"]:+.1f} mL、最大実測角度{fields["peak_measured_angle_deg"]:.1f}度。'
                           f'土壌水分 {fields["soil_before"]:.1f}→{fields["soil_after"]:.1f}%、'
                           f'葉のしおれ {fields["leaf_before"]:.1f}→{fields["leaf_after"]:.1f}%（模擬値）。')
            elif kind == 'dose_progress':
                message=(f'【角度・流量】実測{fields["measured_angle_deg"]:.1f}度 / 指令{fields["command_angle_deg"]:.1f}度、'
                         f'{fields["flow_ml_s"]:.1f} mL/秒、給水{fields["delivered_ml"]:.1f}/{fields["target_ml"]:.1f} mL（仮想流量）。')
            elif kind == 'lap_completed':
                message = f'{fields["lap"]}周目を完了しました。'
            elif kind == 'finished':
                message = '停止して終了しました。' + json.dumps({'result': fields['result']})
            elif kind == 'ai_error':
                message = 'AI入力に問題があるため停止します。' + fields['reason']
            elif kind == 'state' and fields['state'] in ('waiting_for_odometry', 'waiting_for_scan', 'obstacle_stop'):
                message = {'waiting_for_odometry': '位置情報を待っています。停止中です。',
                           'waiting_for_scan': '距離センサーを待っています。停止中です。',
                           'obstacle_stop': '前方に障害物があるため停止中です。'}[fields['state']]
            if message:
                self.get_logger().info(message)
        return row

    def set_state(self, state):
        if state != self.state:
            self.state = state
            self.event('state', state=state)

    def command(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(linear), float(angular)
        self.pub.publish(msg)

    def finish(self, result):
        if not self.done:
            self.command()
            if self.precision: self.precision.suspend()
            self.done = True
            self.result = result
            self.event('finished', result=result, laps_completed=self.lap)

    def on_stop(self, _request, response):
        self.finish('stopped_by_service')
        response.success = True
        response.message = 'Patrol stopped; zero velocity commanded.'
        return response

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if not all(math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w)):
            return
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)
        self.odom_time = time.monotonic()
        if self.origin is None:
            self.origin = self.scene_origin or self.pose
            ox, oy, initial_yaw = self.origin
            c, s = math.cos(initial_yaw), math.sin(initial_yaw)
            d = self.args.side
            self.goals = [(ox + c*x - s*y, oy + s*x + c*y)
                          for x, y in [(d, 0), (d, d), (0, d), (0, 0)]]
            self.event('route', origin=self.origin, goals=self.goals)

    def on_scan(self, msg):
        self.front = front_distance(msg)
        self.scan_time = time.monotonic()

    def tick(self):
        now = time.monotonic()
        if self.done:
            self.command()
            return
        if now - self.started > self.args.timeout:
            self.finish('timeout')
            return
        # Refuse simultaneous teleoperation or a second patrol instance.
        if self.count_publishers('/cmd_vel') > 1:
            self.finish('conflicting_cmd_vel_publisher')
            return
        if self.pose is None or now - self.odom_time > 1.0:
            self.command()
            if self.precision: self.precision.suspend()
            self.set_state('waiting_for_odometry')
            return
        if self.front is None or now - self.scan_time > 1.0:
            self.command()
            if self.precision: self.precision.suspend()
            self.set_state('waiting_for_scan')
            return
        # Hysteresis prevents chatter at the 0.35 m stop boundary.
        close_to_planter = self.precision is not None and (self.precision.active or self.precision.near_planter())
        stop_distance, resume_distance = (.14, .19) if close_to_planter else (.35, .45)
        self.blocked = self.front < (resume_distance if self.blocked else stop_distance)
        if self.blocked:
            self.command()
            if self.precision: self.precision.suspend()
            self.set_state('obstacle_stop')
            return
        if self.precision:
            if not self.precision.ready():
                self.command(); self.precision.suspend()
                if self.precision.active or now-self.started>12:
                    self.finish('precision_sensor_timeout')
                else:
                    self.set_state('waiting_for_precision_sensors')
                return
            try:
                was_active=self.precision.active
                self.precision.step()
                if was_active: return
            except (ValueError,RuntimeError) as exc:
                self.event('ai_error',reason=str(exc)); self.finish('precision_error'); return
        if now < self.hold_until:
            self.command()
            self.set_state(self.hold_state)
            return
        if self.pending_inspection is not None:
            self.command()
            self.inspect_waypoint(now)
            return
        if self.index == len(self.goals):
            self.lap += 1
            self.event('lap_completed', lap=self.lap)
            if self.lap >= self.args.laps:
                self.finish('completed')
                return
            self.index = 0
        x, y, yaw = self.pose
        gx, gy = self.goals[self.index]
        distance = math.hypot(gx - x, gy - y)
        if distance <= 0.07:
            self.command()
            self.event('waypoint_reached', lap=self.lap + 1,
                       waypoint=self.index + 1, error_m=round(distance, 4))
            if self.inspector:
                self.pending_inspection = self.index + 1
            self.index += 1
            self.hold_state = 'inspection_wait'
            self.hold_until = now + self.args.dwell
            return
        error = wrap(math.atan2(gy - y, gx - x) - yaw)
        angular = max(-0.6, min(0.6, 1.8 * error))
        if abs(error) > 0.15:
            self.set_state('turning')
            self.command(0.0, angular)
        else:
            self.set_state('driving')
            self.command(min(.055 if close_to_planter else .12, 0.8 * distance), angular)

    def inspect_waypoint(self, now):
        waypoint = self.pending_inspection
        self.pending_inspection = None
        try:
            decision = self.precision.plan(waypoint) if self.precision else self.inspector.inspect(waypoint)
        except (ValueError, KeyError, IndexError) as exc:
            self.event('ai_error', reason=str(exc), waypoint=waypoint)
            self.finish('ai_error')
            return
        row = self.event('ai_decision', lap=self.lap + 1, waypoint=waypoint, **decision)
        self.decision_pub.publish(String(data=json.dumps(row, ensure_ascii=False)))
        if self.precision:
            if decision['action']=='water':
                try: self.precision.begin(waypoint,decision,self.goals[waypoint-1])
                except ValueError as exc:
                    self.event('ai_error',reason=str(exc)); self.finish('precision_error')
            else:
                self.event('watering_skipped',waypoint=waypoint,action=decision['action'],actuator='gazebo_joint_virtual_flow')
            return
        if decision['action'] == 'water':
            self.event('watering_simulated', lap=self.lap + 1, waypoint=waypoint,
                       plant_id=decision['plant_id'], actuator='log_only',
                       duration_s=self.args.watering_seconds,
                       message='【模擬動作】散水を想定して待機します。実際の散水はしません。')
            self.hold_state = 'simulated_watering'
            self.hold_until = now + self.args.watering_seconds
        else:
            self.event('watering_skipped', lap=self.lap + 1, waypoint=waypoint,
                       plant_id=decision['plant_id'], action=decision['action'], actuator='log_only')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--side', type=float, default=1.0, help='Square side in metres (0.3..3)')
    parser.add_argument('--laps', type=int, default=1)
    parser.add_argument('--dwell', type=float, default=2.0, help='Seconds at each waypoint')
    parser.add_argument('--timeout', type=float, default=300.0, help='Wall seconds, including waits')
    parser.add_argument('--log-dir', default=str(Path(__file__).resolve().parent / 'logs'))
    parser.add_argument('--watering-demo', action='store_true', help='Offline AI with simulated plant inputs; no real watering')
    parser.add_argument('--precision-watering', action='store_true', help='Live virtual soil/leaf sensors with measured-joint volume regulation')
    parser.add_argument('--verbose', action='store_true', help='Print full JSON events in the console as well as the log file')
    parser.add_argument('--free-route', action='store_true', help='Ignore the fixed planter scene and use the current robot pose')
    parser.add_argument('--observations', default=str(Path(__file__).resolve().parent / 'simulated_plants.json'))
    parser.add_argument('--watering-seconds', type=float, default=3.0, help='Stationary simulated watering duration')
    args = parser.parse_args()
    if args.precision_watering: args.watering_demo=True
    if not (0.3 <= args.side <= 3 and 1 <= args.laps <= 20
            and math.isfinite(args.dwell) and args.dwell >= 0
            and math.isfinite(args.watering_seconds) and 0 <= args.watering_seconds <= 30
            and math.isfinite(args.timeout) and args.timeout > 0):
        parser.error('Invalid side, laps, dwell or timeout')
    interrupted = []
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda number, _frame: interrupted.append(number))
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = Patrol(args)
        while not node.done and not interrupted:
            rclpy.spin_once(node, timeout_sec=0.1)
        if interrupted:
            node.finish('interrupted')
        return 0 if node.result in ('completed', 'interrupted', 'stopped_by_service') else 1
    finally:
        if node is not None:
            if node.precision:
                until=time.monotonic()+3.
                while time.monotonic()<until:
                    node.command(); node.precision.suspend()
                    rclpy.spin_once(node,timeout_sec=.05)
                    if node.precision.can and abs(node.precision.can['angle_deg'])<.7:
                        break
            # Deliver repeated zero commands before tearing down the publisher.
            for _ in range(5):
                node.command()
                rclpy.spin_once(node, timeout_sec=0.05)
            node.log_file.close()
            if node.precision: node.precision.close()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
