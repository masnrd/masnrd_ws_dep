# `masnrd_msgs`

## Installation
```bash
colcon build --packages-select masnrd_msgs
```
- This will make the messages available to other ROS2 packages in the same workspace.

To confirm successful installation:
```bash
source install/setup.bash
ros2 interface show masnrd_msgs/msg/ReachedWaypoint
```

You should see:
```bash
# Message to indicate to pathfinder that drone has reached a waypoint.
# Values are in METRES.
uint64 timestamp # time since system start (microseconds)

float64 x
float64 y
```