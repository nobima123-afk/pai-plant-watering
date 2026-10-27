"""Isolated ROS-domain regression checks; no commands reach the Gazebo domain."""
import math
import os
import tempfile
import time
import unittest
import json
from types import SimpleNamespace
from argparse import Namespace

os.environ['ROS_DOMAIN_ID'] = '87'
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from patrol import Patrol, front_distance


class PatrolTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.node = Patrol(Namespace(side=1.0, laps=1, dwell=0.0,
                                     timeout=300.0, log_dir=self.temp.name))
        self.commands = []
        self.node.command = lambda v=0., w=0.: self.commands.append((v, w))
        self.node.on_odom(Odometry())
        self.node.front = math.inf
        self.node.scan_time = time.monotonic()

    def tearDown(self):
        self.node.log_file.close()
        self.node.destroy_node()
        self.temp.cleanup()

    def test_sensor_loss_and_obstacle_stop_resume(self):
        n = self.node
        n.tick()
        self.assertGreater(self.commands[-1][0], 0)
        n.front = 0.30
        n.tick()
        self.assertEqual(self.commands[-1], (0, 0))
        n.front = 0.40
        n.tick()
        self.assertEqual(self.commands[-1], (0, 0))
        n.front = 0.50
        n.tick()
        self.assertGreater(self.commands[-1][0], 0)
        n.scan_time -= 2
        n.tick()
        self.assertEqual(self.commands[-1], (0, 0))
        self.assertEqual(n.state, 'waiting_for_scan')
        n.scan_time = time.monotonic()
        n.odom_time -= 2
        n.tick()
        self.assertEqual(self.commands[-1], (0, 0))
        self.assertEqual(n.state, 'waiting_for_odometry')

    def test_arrivals_finish_and_stop(self):
        n = self.node
        for i, (x, y) in enumerate(n.goals):
            n.pose = (x, y, 0)
            n.tick()
            self.assertEqual(n.index, i + 1)
            self.assertEqual(self.commands[-1], (0, 0))
        n.tick()
        self.assertTrue(n.done)
        self.assertEqual(n.result, 'completed')

    def test_plant_scene_anchor_stays_fixed_across_robot_start_positions(self):
        n = self.node
        n.scene_origin = (2.0, 3.0, math.pi/2)
        for x, y in ((10., 10.), (-4., 8.)):
            n.origin = None
            msg = Odometry()
            msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
            msg.pose.pose.orientation.w = 1.
            n.on_odom(msg)
            self.assertEqual(n.origin, n.scene_origin)
            for actual, expected in zip(n.goals, [(2,4),(1,4),(1,3),(2,3)]):
                self.assertAlmostEqual(actual[0], expected[0])
                self.assertAlmostEqual(actual[1], expected[1])

    def test_conflicting_driver_aborts(self):
        self.node.count_publishers = lambda _: 2
        self.node.tick()
        self.assertEqual(self.node.result, 'conflicting_cmd_vel_publisher')
        self.assertEqual(self.commands[-1], (0, 0))

    def test_ai_inspection_keeps_robot_stopped_and_runs_once_per_waypoint(self):
        from watering_ai import SimulatedInspection
        n = self.node
        n.inspector = SimulatedInspection()
        messages = []
        n.decision_pub = SimpleNamespace(publish=lambda message: messages.append(message))
        n.args.watering_seconds = 3.0
        for i, (x, y) in enumerate(n.goals):
            n.pose = (x, y, 0)
            n.tick()  # arrive
            self.assertEqual(n.pending_inspection, i+1)
            n.tick()  # inspect after dwell
            self.assertIsNone(n.pending_inspection)
            self.assertEqual(len(messages), i+1)
            self.assertEqual(self.commands[-1], (0, 0))
            n.tick()  # must not duplicate the inspection
            self.assertEqual(len(messages), i+1)
            self.assertEqual(n.index, i+1)
            if i in (0, 2):
                self.assertGreater(n.hold_until, time.monotonic())
                self.assertEqual(self.commands[-1], (0, 0))
            n.hold_until = 0
        n.tick()
        self.assertEqual(n.result, 'completed')
        self.assertEqual([json.loads(m.data)['action'] for m in messages],
                         ['water', 'skip', 'water', 'skip'])

    def test_inspection_error_stops_instead_of_moving_on(self):
        n = self.node
        def invalid(_):
            raise ValueError('invalid observation')
        n.inspector = SimpleNamespace(inspect=invalid)
        n.pending_inspection = 1
        n.tick()
        self.assertEqual(n.result, 'ai_error')
        self.assertEqual(self.commands[-1], (0, 0))

    def test_scan_wrap_invalid_and_too_close(self):
        scan = LaserScan()
        scan.angle_increment = math.pi / 2
        scan.range_min, scan.range_max = 0.12, 3.5
        scan.ranges = [math.inf, 0.2, 0.2, 0.2, 0.28]
        self.assertAlmostEqual(front_distance(scan), 0.28, places=5)
        scan.ranges = [math.nan]
        self.assertIsNone(front_distance(scan))
        scan.ranges = [0.05]
        self.assertEqual(front_distance(scan), 0.)


if __name__ == '__main__':
    unittest.main()
