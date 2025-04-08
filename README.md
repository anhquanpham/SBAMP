# SBAMP
Sampling Based Adaptive Motion Planning

### Steps to run:

```bash
git clone --recurse-submodules https://github.com/Shreyas0812/SBAMP.git
```

```bash
/SBAMP/src/f1tenth_gym_ros/f1tenth_gym$ pip install -e .
```

```
$ cd SBAMP
```

```bash
$ source /opt/ros/humble/setup.bash
$ colcon build
$ source install/local_setup.bash
$ ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
