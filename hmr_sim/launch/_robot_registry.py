"""Robot registry for hmr_sim robot_sim.launch.py.

Single source of truth mapping robot short-names to:
  - type: 'ugv' or 'uav' — used by batch scripts to pick nav mode
  - sdf_file: path relative to hmr_sim/models/
  - features: set of feature flags that drive bridge generation and
    downstream behaviour (e.g. uav_cmd_vel routes /cmd_vel under uav/)
  - default_use_imu: whether spawn_robot.launch.py namespaces the IMU

Naming convention:
  - atlas, bestla -> UGV (COSTAR_HUSKY variants, yellow/green)
  - rama, ravana  -> UAV (X4)

Bridge generation composes from FEATURE_BRIDGE_TEMPLATES: each feature
emits its list of (ros_topic, gz_topic, ros_type, gz_type, direction)
templates with '{ns}' replaced by the robot's name. The clock entry is
emitted once globally.
"""

ROBOTS = {
    'atlas': {
        'type': 'ugv',
        'sdf_file': 'COSTAR_HUSKY_SENSOR_CONFIG_REDUCED_YELLOW/model.sdf',
        'features': {'rgbd', 'segmentation', 'imu', 'odom_gt', 'ugv_cmd_vel'},
        'default_use_imu': True,
    },
    'bestla': {
        'type': 'ugv',
        'sdf_file': 'COSTAR_HUSKY_SENSOR_CONFIG_REDUCED_GREEN/model.sdf',
        'features': {'rgbd', 'segmentation', 'imu', 'odom_gt', 'ugv_cmd_vel'},
        'default_use_imu': True,
    },
    'husky': {
        'type': 'ugv',
        'sdf_file': 'COSTAR_HUSKY_SENSOR_CONFIG_1/model.sdf',
        'features': {'lidar', 'rgbd', 'imu', 'odom_gt', 'ugv_cmd_vel'},
        'default_use_imu': True,
    },
    'rama': {
        'type': 'uav',
        'sdf_file': 'X4_UAV_Config_RAISE/model.sdf',
        'features': {'rgbd', 'imu', 'odom_gt', 'pose_v', 'uav_cmd_vel'},
        'default_use_imu': True,
    },
    'ravana': {
        'type': 'uav',
        'sdf_file': 'X4_UAV_Config_RAISE/model.sdf',
        'features': {'rgbd', 'imu', 'odom_gt', 'pose_v', 'uav_cmd_vel'},
        'default_use_imu': True,
    },
    'firefly': {
        'type': 'uav',
        'sdf_file': 'X4_UAV_Config_2/model.sdf',
        'features': {'rgbd', 'imu', 'odom_gt', 'pose_v', 'uav_cmd_vel'},
        'default_use_imu': True,
    },
}

# Templates use '{ns}' as the robot-namespace placeholder.
FEATURE_BRIDGE_TEMPLATES = {
    'lidar': [
        ('{ns}/velodyne_points', '{ns}/laser_scan/points',
         'sensor_msgs/msg/PointCloud2', 'ignition.msgs.PointCloudPacked', 'GZ_TO_ROS'),
    ],
    'rgbd': [
        ('{ns}/rgbd_points', '{ns}/rgbd_camera/points',
         'sensor_msgs/msg/PointCloud2', 'ignition.msgs.PointCloudPacked', 'GZ_TO_ROS'),
        ('{ns}/rgbd_camera_info', '{ns}/rgbd_camera/camera_info',
         'sensor_msgs/msg/CameraInfo', 'ignition.msgs.CameraInfo', 'GZ_TO_ROS'),
        ('{ns}/rgbd_camera_image', '{ns}/rgbd_camera/image',
         'sensor_msgs/msg/Image', 'ignition.msgs.Image', 'GZ_TO_ROS'),
        ('{ns}/rgbd_camera_depth_image', '{ns}/rgbd_camera/depth_image',
         'sensor_msgs/msg/Image', 'ignition.msgs.Image', 'GZ_TO_ROS'),
    ],
    'segmentation': [
        ('{ns}/segmentation/labels', '{ns}/segmentation_camera/image/labels_map',
         'sensor_msgs/msg/Image', 'ignition.msgs.Image', 'GZ_TO_ROS'),
        ('{ns}/segmentation/colored', '{ns}/segmentation_camera/image/colored_map',
         'sensor_msgs/msg/Image', 'ignition.msgs.Image', 'GZ_TO_ROS'),
        ('{ns}/segmentation/camera_info', '{ns}/segmentation_camera/image/camera_info',
         'sensor_msgs/msg/CameraInfo', 'ignition.msgs.CameraInfo', 'GZ_TO_ROS'),
    ],
    'imu': [
        ('{ns}/imu/data', '{ns}/imu/data',
         'sensor_msgs/msg/Imu', 'ignition.msgs.IMU', 'GZ_TO_ROS'),
    ],
    'odom_gt': [
        ('{ns}/odom_ground_truth', '{ns}/odom_ground_truth',
         'nav_msgs/msg/Odometry', 'ignition.msgs.Odometry', 'GZ_TO_ROS'),
    ],
    'pose_v': [
        ('{ns}/pose', 'model/{ns}/pose',
         'geometry_msgs/msg/PoseArray', 'ignition.msgs.Pose_V', 'GZ_TO_ROS'),
    ],
    'ugv_cmd_vel': [
        ('{ns}/cmd_vel', '{ns}/cmd_vel',
         'geometry_msgs/msg/Twist', 'ignition.msgs.Twist', 'ROS_TO_GZ'),
    ],
    'uav_cmd_vel': [
        ('{ns}/cmd_vel', '{ns}/uav/cmd_vel',
         'geometry_msgs/msg/Twist', 'ignition.msgs.Twist', 'ROS_TO_GZ'),
    ],
}

CLOCK_BRIDGE_ENTRY = {
    'ros_topic_name': 'clock',
    'gz_topic_name': 'clock',
    'ros_type_name': 'rosgraph_msgs/msg/Clock',
    'gz_type_name': 'ignition.msgs.Clock',
    'direction': 'GZ_TO_ROS',
}


def get_robot(short_name):
    if short_name not in ROBOTS:
        known = ', '.join(sorted(ROBOTS.keys()))
        raise KeyError(f"Unknown robot '{short_name}'. Known robots: {known}")
    return ROBOTS[short_name]


def get_robot_type(short_name):
    return get_robot(short_name)['type']


def build_bridge_config(robot_names):
    entries = [dict(CLOCK_BRIDGE_ENTRY)]
    for name in robot_names:
        robot = get_robot(name)
        for feature in robot['features']:
            for ros_t, gz_t, ros_ty, gz_ty, direction in FEATURE_BRIDGE_TEMPLATES.get(feature, []):
                entries.append({
                    'ros_topic_name': ros_t.format(ns=name),
                    'gz_topic_name': gz_t.format(ns=name),
                    'ros_type_name': ros_ty,
                    'gz_type_name': gz_ty,
                    'direction': direction,
                })
    return entries
