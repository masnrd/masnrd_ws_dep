# Overall Drone System

## Installation
If the ROS2 distribution hasn't been initialised before, do so:
```bash
source /opt/ros/rolling/setup.bash # or your preferred ROS2 distribution
```

In the project root:
```bash
colcon build
```

## Usage
### Initialising the Environment
If the ROS2 distribution hasn't been initialised before, do so:
```bash
source /opt/ros/rolling/setup.bash # or your preferred ROS2 distribution
```

Initialise the environment with:
```bash
source install/local_setup.bash
```
- This provides access to the environment hooks for this workspace.
- This is an overlay on top of the underlying ROS2 environment (having sourced `setup.bash` for the ROS2 distribution earlier)

### SITL Testing
In a separate terminal, initialise the PX4 SITL stack with the simulator:
```bash
export PX4_HOME_LAT=1.40724
export PX4_HOME_LON=104.02896
make px4_sitl jmavsim
```
- The given latitude and longitude are needed for the hardcoded points on the pathfinder node. These coordinates are at BMTC, because the programmer of this piece of code thinks he's funny.

In another terminal, initialise the micro XRCE-DDS agent:
```bash
MicroXRCEAgent udp4 -p 8888
```

Having initialised the environment, run the central interface node and the pathfinder node on a third terminal:
```bash
ros2 run central_interface central_interface
```

