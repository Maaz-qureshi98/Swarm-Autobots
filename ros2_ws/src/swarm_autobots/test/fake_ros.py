"""Minimal in-process stand-in for rclpy + message packages, used only to smoke-test the
node graph where ROS 2 is not installed. Timers run on a simulated clock."""
import sys, types, heapq, itertools

class _Bus:
    def __init__(self): self.subs = {}; self.t = 0.0; self.timers = []; self.cnt = itertools.count()
BUS = _Bus()

class _Msg:
    def __init__(self, **kw):
        for k, v in self._defaults().items(): setattr(self, k, v)
        for k, v in kw.items(): setattr(self, k, v)
    def _defaults(self): return {}

def _ns(**fields):
    return staticmethod(lambda: {k: (v() if callable(v) else v) for k, v in fields.items()})

class Vector3(_Msg): _defaults = _ns(x=0.0, y=0.0, z=0.0)
class Twist(_Msg): _defaults = _ns(linear=Vector3, angular=Vector3)
class Point(_Msg): _defaults = _ns(x=0.0, y=0.0, z=0.0)
class Quaternion(_Msg): _defaults = _ns(x=0.0, y=0.0, z=0.0, w=1.0)
class Pose(_Msg): _defaults = _ns(position=Point, orientation=Quaternion)
class Header(_Msg): _defaults = _ns(frame_id="", stamp=None)
class PoseStamped(_Msg): _defaults = _ns(header=Header, pose=Pose)
class PoseWithCov(_Msg): _defaults = _ns(pose=Pose)
class Odometry(_Msg): _defaults = _ns(header=Header, pose=PoseWithCov)
class Path(_Msg): _defaults = _ns(header=Header, poses=list)
class String(_Msg): _defaults = _ns(data="")
class LeaderState(_Msg): _defaults = _ns(leader_id=0, seq=0, formation_id=0, x=0.0, y=0.0, theta=0.0, v=0.0, w=0.0)
class FollowerStatus(_Msg): _defaults = _ns(robot_id=0, leader_id=0, slot_error=0.0)
class Ranges(_Msg): _defaults = _ns(front=0.0, left=0.0, right=0.0)

class _Param:
    def __init__(self, v): self.value = v
class _Time:
    def __init__(self, t): self.nanoseconds = int(t * 1e9)
    def to_msg(self): return self.nanoseconds
class _Clock:
    def now(self): return _Time(BUS.t)
class _Pub:
    def __init__(self, topic, qos): self.topic = topic; self.latched = getattr(qos, "durability", None) == "TL"; self.last = None
    def publish(self, m):
        self.last = m
        for cb in list(BUS.subs.get(self.topic, [])): cb(m)
class Node:
    PARAMS = {}
    def __init__(self, name): self._name = name; self._params = {}
    def declare_parameter(self, k, v): self._params[k] = Node.PARAMS.get(self._name_key(), {}).get(k, v)
    def _name_key(self): return getattr(self, "_override", self._name)
    def get_parameter(self, k): return _Param(self._params[k])
    def create_publisher(self, typ, topic, qos):
        p = _Pub(topic, qos); BUS.subs.setdefault(topic, []); _PUBS.setdefault(topic, []).append(p); return p
    def create_subscription(self, typ, topic, cb, qos):
        BUS.subs.setdefault(topic, []).append(cb)
        for p in _PUBS.get(topic, []):
            if p.latched and p.last is not None: cb(p.last)
    def create_timer(self, period, cb):
        heapq.heappush(BUS.timers, (BUS.t + period, next(BUS.cnt), period, cb))
    def get_clock(self): return _Clock()
    def get_logger(self):
        class L:
            def info(s, *a): pass
            def warn(s, *a): pass
        return L()
    def destroy_node(self): pass
_PUBS = {}

def run_until(t_end):
    while BUS.timers and BUS.timers[0][0] <= t_end:
        t, c, per, cb = heapq.heappop(BUS.timers)
        BUS.t = t; cb()
        heapq.heappush(BUS.timers, (t + per, next(BUS.cnt), per, cb))

class QoSProfile:
    def __init__(self, depth=10, reliability=None, durability=None): self.durability = durability
class _Enum:
    BEST_EFFORT = "BE"; RELIABLE = "R"; TRANSIENT_LOCAL = "TL"; VOLATILE = "V"

def install():
    def mod(name, **attrs):
        m = types.ModuleType(name); m.__dict__.update(attrs); sys.modules[name] = m; return m
    mod("rclpy", init=lambda *a, **k: None, shutdown=lambda *a, **k: None, spin=lambda n: None)
    mod("rclpy.node", Node=Node)
    mod("rclpy.qos", QoSProfile=QoSProfile, ReliabilityPolicy=_Enum, DurabilityPolicy=_Enum)
    mod("geometry_msgs"); mod("geometry_msgs.msg", Twist=Twist, PoseStamped=PoseStamped)
    mod("nav_msgs"); mod("nav_msgs.msg", Odometry=Odometry, Path=Path)
    mod("std_msgs"); mod("std_msgs.msg", String=String)
    mod("swarm_autobots_msgs"); mod("swarm_autobots_msgs.msg", LeaderState=LeaderState, FollowerStatus=FollowerStatus, Ranges=Ranges)
