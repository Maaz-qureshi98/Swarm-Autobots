"""Formation-aware A* for a mapped arena (hardware use). Publishes the shared mission path."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from swarm_autobots.sim_core import astar_path
from swarm_autobots.agent import FORMATIONS


class Planner(Node):
    def __init__(self):
        super().__init__("formation_planner")
        self.declare_parameter("arena", [6.0, 6.0])
        self.declare_parameter("obstacles", [0.0])          # flat list x,y,r,x,y,r,...
        self.declare_parameter("start", [1.0, 3.0])
        self.declare_parameter("goal", [5.0, 3.0])
        self.declare_parameter("formation", "triangle")
        self.declare_parameter("robot_radius", 0.12)
        W, H = self.get_parameter("arena").value
        flat = self.get_parameter("obstacles").value
        obs = [tuple(flat[k:k + 3]) for k in range(0, len(flat) - 2, 3)]
        half_w = max(abs(o[1]) for o in FORMATIONS[self.get_parameter("formation").value])
        R = self.get_parameter("robot_radius").value
        start, goal = self.get_parameter("start").value, self.get_parameter("goal").value
        import numpy as np
        path = astar_path(W, H, obs, np.array(start), np.array(goal), infl=R + 0.10 + half_w)
        if path is None:
            self.get_logger().warn("formation corridor infeasible, falling back to robot-only inflation")
            path = astar_path(W, H, obs, np.array(start), np.array(goal), infl=R + 0.15)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Path, "/swarm/path", latched)
        msg = Path(); msg.header.frame_id = "map"
        for (x, y) in path:
            ps = PoseStamped(); ps.header.frame_id = "map"
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            msg.poses.append(ps)
        self.pub.publish(msg)
        self.get_logger().info(f"published path with {len(path)} points")


def main():
    rclpy.init(); n = Planner()
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node(); rclpy.shutdown()
