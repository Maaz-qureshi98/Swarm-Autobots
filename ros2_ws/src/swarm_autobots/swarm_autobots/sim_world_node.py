"""Simulation only: the embedded layer of N robots (tracked kinematics with slip and motor lag,
encoder + magnetometer dead reckoning, HC-SR04 / Sharp IR ray casting), using the same model
as the paper's experiments (sim_core.SwarmSim). Publishes what each robot's firmware would."""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from swarm_autobots_msgs.msg import Ranges
from swarm_autobots.sim_core import SwarmSim


def odom_msg(x, y, th, stamp, frame="map"):
    m = Odometry(); m.header.frame_id = frame; m.header.stamp = stamp
    m.pose.pose.position.x, m.pose.pose.position.y = float(x), float(y)
    m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
    return m


class SimWorld(Node):
    def __init__(self):
        super().__init__("sim_world")
        self.declare_parameter("num_robots", 4)
        self.declare_parameter("formation", "triangle")
        self.declare_parameter("seed", 0)
        self.declare_parameter("unmapped_obstacles", 0)
        N = self.get_parameter("num_robots").value
        self.sim = SwarmSim(1, N, form=self.get_parameter("formation").value, p_loss=0.0,
                            seed=self.get_parameter("seed").value,
                            n_unknown=self.get_parameter("unmapped_obstacles").value)
        self.N = N
        self.cmd = np.zeros((1, N, 2))
        self.odom = [self.create_publisher(Odometry, f"/robot_{i}/odom", 10) for i in range(N)]
        self.gt = [self.create_publisher(Odometry, f"/robot_{i}/ground_truth", 10) for i in range(N)]
        self.rng = [self.create_publisher(Ranges, f"/robot_{i}/ranges", 10) for i in range(N)]
        for i in range(N):
            self.create_subscription(Twist, f"/robot_{i}/cmd_vel", lambda m, i=i: self.on_cmd(i, m), 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, "/swarm/path", latched)
        self.publish_path()
        self.create_timer(self.sim.c.dt, self.tick)

    def publish_path(self):
        # stands in for planner_node: formation-aware A* on the scenario's mapped obstacles
        p = Path(); p.header.frame_id = "map"
        for (x, y) in self.sim.paths[0]:
            ps = PoseStamped(); ps.header.frame_id = "map"
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            p.poses.append(ps)
        self.path_pub.publish(p)

    def on_cmd(self, i, m):
        self.cmd[0, i] = (m.linear.x, m.angular.z)

    def tick(self):
        s = self.sim
        s.physics(self.cmd)
        r = s.sense()[0]
        stamp = self.get_clock().now().to_msg()
        for i in range(self.N):
            self.odom[i].publish(odom_msg(*s.est[0, i], stamp))
            self.gt[i].publish(odom_msg(s.pos[0, i, 0], s.pos[0, i, 1], s.th[0, i], stamp))
            m = Ranges(); m.front, m.left, m.right = [float(v) for v in r[i]]
            self.rng[i].publish(m)


def main():
    rclpy.init(); n = SimWorld()
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node(); rclpy.shutdown()
