"""Simulation only: emulates ESP-NOW broadcast between robots with a Gilbert-Elliott
channel per receiver (mean loss p, mean burst length L). On hardware the radios do this."""
import random
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from swarm_autobots_msgs.msg import LeaderState, FollowerStatus


class ChannelNode(Node):
    def __init__(self):
        super().__init__("espnow_channel")
        self.declare_parameter("num_robots", 4)
        self.declare_parameter("loss", 0.2)
        self.declare_parameter("burst", 4.0)
        self.declare_parameter("seed", 0)
        self.N = self.get_parameter("num_robots").value
        p, L = self.get_parameter("loss").value, self.get_parameter("burst").value
        self.q = 1.0 / L
        self.s = p * self.q / (1 - p) if p > 0 else 0.0
        self.rng = random.Random(self.get_parameter("seed").value)
        self.bad = [False] * self.N
        be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.rx_l = [self.create_publisher(LeaderState, f"/robot_{i}/espnow/rx/leader_state", be) for i in range(self.N)]
        self.rx_s = [self.create_publisher(FollowerStatus, f"/robot_{i}/espnow/rx/status", be) for i in range(self.N)]
        self.create_subscription(LeaderState, "/espnow/tx/leader_state", lambda m: self.forward(m, m.leader_id, self.rx_l), be)
        self.create_subscription(FollowerStatus, "/espnow/tx/status", lambda m: self.forward(m, m.robot_id, self.rx_s), be)
        self.create_timer(0.1, self.tick)                                 # channel state per 10 Hz slot

    def tick(self):
        for i in range(self.N):
            u = self.rng.random()
            self.bad[i] = (u > self.q) if self.bad[i] else (u < self.s)

    def forward(self, msg, sender, pubs):
        for i in range(self.N):
            if i != sender and not self.bad[i]:
                pubs[i].publish(msg)


def main():
    rclpy.init(); n = ChannelNode()
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node(); rclpy.shutdown()
