colcon build --packages-select sbamp

source install/setup.bash

ros2 run sbamp sbamp_node.py
