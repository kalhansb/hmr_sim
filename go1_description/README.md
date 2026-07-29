# robot_state_publisher_bringup

ROS 2 package to launch `robot_state_publisher` using a URDF file placed in this package `urdf/` directory.

## Structure

- `launch/publish_urdf.launch.py`: Launches `robot_state_publisher`
- `urdf/`: Put your `.urdf` file here
- `meshes/`: Put your `.dae` files (and other mesh assets) here

## Usage

Build from your workspace root:

```bash
colcon build --packages-select robot_state_publisher_bringup
source install/setup.bash
```

Launch using default file name (`robot.urdf`):

```bash
ros2 launch robot_state_publisher_bringup publish_urdf.launch.py
```

Launch with a specific file in `urdf/`:

```bash
ros2 launch robot_state_publisher_bringup publish_urdf.launch.py urdf_file:=my_robot.urdf
```
