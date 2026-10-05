"""Hardware only: bridge between the ESP-NOW swarm and ROS 2 through a USB-connected ESP32
that listens to the broadcast channel. Robot-to-robot coordination stays on ESP-NOW; the
gateway mirrors all frames onto ROS topics (logging, RViz, rosbag) and forwards operator
commands (pattern switch) to the swarm. Serial framing: 0xAA, type, len, payload, xor.
type 1 = LeaderState (20 B), 2 = FollowerStatus (4 B), 3 = formation command (1 B)."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from swarm_autobots_msgs.msg import LeaderState, FollowerStatus
from swarm_autobots import frames
from swarm_autobots.agent import FORMATION_IDS


def frame(t, payload):
    x = t ^ len(payload)
    for b in payload:
        x ^= b
    return bytes([0xAA, t, len(payload)]) + payload + bytes([x])


class Gateway(Node):
    def __init__(self):
        super().__init__("espnow_gateway")
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 921600)
        import serial                                   # pyserial
        self.ser = serial.Serial(self.get_parameter("port").value, self.get_parameter("baud").value, timeout=0)
        self.buf = bytearray()
        self.pub_l = self.create_publisher(LeaderState, "/espnow/monitor/leader_state", 10)
        self.pub_s = self.create_publisher(FollowerStatus, "/espnow/monitor/status", 10)
        self.create_subscription(String, "/swarm/formation_cmd", self.on_formation, 10)
        self.create_timer(0.005, self.poll)

    def on_formation(self, m):
        self.ser.write(frame(3, bytes([FORMATION_IDS[m.data]])))

    def poll(self):
        self.buf += self.ser.read(256)
        while len(self.buf) >= 4:
            if self.buf[0] != 0xAA:
                self.buf.pop(0); continue
            t, n = self.buf[1], self.buf[2]
            if len(self.buf) < 4 + n:
                return
            payload, chk = bytes(self.buf[3:3 + n]), self.buf[3 + n]
            del self.buf[:4 + n]
            x = t ^ n
            for b in payload:
                x ^= b
            if x != chk:
                continue
            if t == 1 and n == frames.LEADER_LEN:
                d = frames.unpack_leader(payload); m = LeaderState()
                m.leader_id, m.seq, m.formation_id = d["leader_id"], d["seq"], d["formation_id"]
                m.x, m.y, m.theta, m.v, m.w = d["x"], d["y"], d["theta"], d["v"], d["w"]
                self.pub_l.publish(m)
            elif t == 2 and n == frames.STATUS_LEN:
                i, l, e = frames.unpack_status(payload); m = FollowerStatus()
                m.robot_id, m.leader_id, m.slot_error = i, l, e
                self.pub_s.publish(m)


def main():
    rclpy.init(); n = Gateway()
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node(); rclpy.shutdown()
