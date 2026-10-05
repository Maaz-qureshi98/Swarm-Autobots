# Reproduce the results. Run from the repository root.
PY ?= python3

.PHONY: all experiments figs test

all: figs

experiments:            ## rerun every simulation (30-60 min on two cores)
	cd sim && $(PY) experiments.py e1 e1b e2 e3 e3x e4 e5 e6 e7a e7b && $(PY) sensitivity.py && $(PY) mismatch.py && $(PY) mismatch_compass.py && $(PY) e9_review.py && $(PY) e12_calib.py

figs:                   ## draw all figures and summary statistics from sim/results
	cd sim && $(PY) make_figs.py && $(PY) stats_tests.py && $(PY) make_robot_fig.py && $(PY) make_teaser.py && $(PY) make_cases_fig.py

test:                   ## agent and ROS graph tests (no ROS install needed)
	$(PY) ros2_ws/src/swarm_autobots/test/test_agent_closed_loop.py
	$(PY) ros2_ws/src/swarm_autobots/test/test_ros_graph.py
