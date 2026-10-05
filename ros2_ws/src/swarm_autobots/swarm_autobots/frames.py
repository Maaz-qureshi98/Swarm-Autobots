"""Binary ESP-NOW frames used on the robots (same fields as the ROS messages).
LeaderState: 20 bytes  <B H B f f f h h  (id, seq, formation, x, y, theta, v [mm/s], w [mrad/s])
FollowerStatus: 4 bytes <B B H           (id, leader id, slot error [mm])"""
import struct

LEADER_FMT, STATUS_FMT = "<BHBfffhh", "<BBH"
LEADER_LEN, STATUS_LEN = struct.calcsize(LEADER_FMT), struct.calcsize(STATUS_FMT)
assert LEADER_LEN == 20 and STATUS_LEN == 4


def pack_leader(m):
    return struct.pack(LEADER_FMT, m["leader_id"], m["seq"] & 0xFFFF, m["formation_id"],
                       m["x"], m["y"], m["theta"],
                       max(-32768, min(32767, int(round(m["v"] * 1000)))),
                       max(-32768, min(32767, int(round(m["w"] * 1000)))))


def unpack_leader(b):
    i, seq, f, x, y, th, v, w = struct.unpack(LEADER_FMT, b)
    return dict(leader_id=i, seq=seq, formation_id=f, x=x, y=y, theta=th, v=v / 1000.0, w=w / 1000.0)


def pack_status(robot_id, leader_id, err):
    return struct.pack(STATUS_FMT, robot_id, leader_id, max(0, min(65535, int(round(err * 1000)))))


def unpack_status(b):
    i, l, e = struct.unpack(STATUS_FMT, b)
    return i, l, e / 1000.0
