from setuptools import setup
package_name = "swarm_autobots"
setup(
    name=package_name, version="0.1.0", packages=[package_name],
    data_files=[("share/ament_index/resource_index/packages", ["resource/" + package_name]),
                ("share/" + package_name, ["package.xml"]),
                ("share/" + package_name + "/launch", ["launch/swarm_sim.launch.py"])],
    install_requires=["setuptools", "numpy"], zip_safe=True,
    maintainer="Maaz Ahmad Qureshi", maintainer_email="m23qures@uwaterloo.ca",
    description="Swarm Autobots: decentralized formation swarming (ROS 2 middleware layer)",
    license="MIT",
    entry_points={"console_scripts": [
        "swarm_agent = swarm_autobots.agent_node:main",
        "espnow_channel = swarm_autobots.espnow_channel_node:main",
        "sim_world = swarm_autobots.sim_world_node:main",
        "formation_planner = swarm_autobots.planner_node:main",
        "espnow_gateway = swarm_autobots.espnow_gateway_node:main"]},
)
