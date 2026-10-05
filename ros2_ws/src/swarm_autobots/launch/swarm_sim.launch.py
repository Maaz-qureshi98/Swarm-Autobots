"""Simulated swarm: sim_world (embedded layer) + ESP-NOW channel + one agent node per robot.
ros2 launch swarm_autobots swarm_sim.launch.py num_robots:=4 formation:=Y loss:=0.2"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    N = int(LaunchConfiguration("num_robots").perform(context))
    form = LaunchConfiguration("formation").perform(context)
    loss = float(LaunchConfiguration("loss").perform(context))
    out = [Node(package="swarm_autobots", executable="sim_world", parameters=[{"num_robots": N, "formation": form}]),
           Node(package="swarm_autobots", executable="espnow_channel", parameters=[{"num_robots": N, "loss": loss}])]
    out += [Node(package="swarm_autobots", executable="swarm_agent", name=f"agent_{i}",
                 parameters=[{"robot_id": i, "num_robots": N, "formation": form}]) for i in range(N)]
    return out


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("num_robots", default_value="4"),
        DeclareLaunchArgument("formation", default_value="triangle"),
        DeclareLaunchArgument("loss", default_value="0.2"),
        OpaqueFunction(function=nodes)])
