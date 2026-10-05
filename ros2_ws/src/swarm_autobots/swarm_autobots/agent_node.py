"""ROS 2 wrapper of one robot's swarm layer (identical node on every robot)."""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from swarm_autobots_msgs.msg import LeaderState, FollowerStatus, Ranges
from swarm_autobots.agent import SwarmAgent


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class AgentNode(Node):
    def __init__(self):
        super().__init__("swarm_agent")
        self.declare_parameter("robot_id", 0)
        self.declare_parameter("num_robots", 4)
        self.declare_parameter("formation", "triangle")
        rid = self.get_parameter("robot_id").value
        self.agent = SwarmAgent(rid, self.get_parameter("num_robots").value,
                                self.get_parameter("formation").value)
        self.pose, self.ranges = None, (2.0, 0.8, 0.8)
        ns = f"/robot_{rid}"
        be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)   # radio-like
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Odometry, ns + "/odom", self.on_odom, 10)
        self.create_subscription(Ranges, ns + "/ranges", self.on_ranges, 10)
        self.create_subscription(LeaderState, ns + "/espnow/rx/leader_state", self.on_leader, be)
        self.create_subscription(FollowerStatus, ns + "/espnow/rx/status", self.on_status, be)
        self.create_subscription(Path, "/swarm/path", self.on_path, latched)
        self.create_subscription(String, "/swarm/formation_cmd", self.on_formation, 10)
        self.cmd_pub = self.create_publisher(Twist, ns + "/cmd_vel", 10)
        self.tx_leader = self.create_publisher(LeaderState, "/espnow/tx/leader_state", be)
        self.tx_status = self.create_publisher(FollowerStatus, "/espnow/tx/status", be)
        self.k = 0
        self.create_timer(0.05, self.cycle)                                  # 20 Hz

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, m):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_ranges(self, m):
        self.ranges = (m.front, m.left, m.right)

    def on_leader(self, m):
        self.agent.on_leader_state(dict(leader_id=m.leader_id, seq=m.seq, formation_id=m.formation_id,
                                        x=m.x, y=m.y, theta=m.theta, v=m.v, w=m.w), self.now())

    def on_status(self, m):
        self.agent.on_status(m.robot_id, m.slot_error, self.now())

    def on_path(self, m):
        self.agent.set_path([(ps.pose.position.x, ps.pose.position.y) for ps in m.poses])

    def on_formation(self, m):
        self.agent.command_formation(m.data)

    def cycle(self):
        if self.pose is None:
            return
        v, w, out = self.agent.step(self.now(), self.pose, self.ranges)
        tw = Twist(); tw.linear.x = float(v); tw.angular.z = float(w)
        self.cmd_pub.publish(tw)
        self.k += 1
        if self.k % 2 == 0 and out is not None:                             # 10 Hz radio slot
            if isinstance(out, dict):
                m = LeaderState()
                m.leader_id, m.seq, m.formation_id = out["leader_id"], out["seq"], out["formation_id"]
                m.x, m.y, m.theta, m.v, m.w = [float(out[k]) for k in ("x", "y", "theta", "v", "w")]
                self.tx_leader.publish(m)
            else:
                s = FollowerStatus(); s.robot_id, s.leader_id, s.slot_error = out[0], out[1], float(out[2])
                self.tx_status.publish(s)


def main():
    rclpy.init(); n = AgentNode()
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node(); rclpy.shutdown()
