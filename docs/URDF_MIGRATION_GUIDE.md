# URDF Migration Guide

This note records how robot description packages should be migrated into this
workspace without rewriting the robot model.

## Source Repository

WPB Home was migrated from:

```text
https://github.com/6-robot/wpb_home.git
```

Official source package:

```text
wpb_home_bringup/
```

The current upstream package contains:

```text
wpb_home_bringup/
  urdf/
    wpb_home.urdf
    wpb_home_mani.urdf
  meshes/
    wpb_home.dae
    wpb_home_mani.dae
    forearm.dae
    finger.dae
  rviz/
    imu_tf.rviz
    odom.rviz
    sensor.rviz
    urdf.rviz
  config/
    wpb_home.yaml
```

The upstream repository does not currently provide `wpbh_robot.urdf`,
`wpbh_robot.xacro`, or a Gazebo xacro for this robot.

## Files To Copy

For any robot migration, copy the official files first:

```text
official_package/urdf/
official_package/meshes/
official_package/rviz/
official_package/config/     # if it contains joint or sensor config
official_package/gazebo/     # if the official package has Gazebo xacro/plugins
```

Do not recreate meshes or simplify the visual model.

## Workspace Layout

Recommended layout:

```text
src/mr_description/
  urdf/<robot_name>/
    official .urdf and .xacro files
  urdf/generated/
    xacro-expanded debug URDF files
  meshes/<robot_name>/
    official mesh files
  rviz/<robot_name>/
    official rviz files
  config/<robot_name>/
    official joint/sensor config files
```

WPB Home currently uses:

```text
src/mr_description/
  urdf/wpb_home/
  urdf/generated/
  meshes/wpb_home/
  rviz/wpb_home/
  config/wpb_home/
```

## Package And Path Adaptation

Only change package names and paths required by this workspace.

Common replacements:

```text
package://<official_package>/meshes/... -> package://mr_description/meshes/<robot_name>/...
$(find <official_package>)/urdf/...     -> $(find mr_description)/urdf/<robot_name>/...
$(find <official_package>)/rviz/...     -> $(find mr_description)/rviz/<robot_name>/...
$(find <official_package>)/config/...   -> $(find mr_description)/config/<robot_name>/...
```

For WPB Home:

```text
package://wpb_home_bringup/meshes/... -> package://mr_description/meshes/wpb_home/...
```

## Sensor Links, Topics, And Frames

Keep official link and frame names stable unless a downstream stack explicitly
requires a remap.

WPB Home official URDF defines these important links:

```text
base_footprint
base_link
body_link
laser
kinect2_dock
kinect2_ir_optical_frame
kinect2_rgb_optical_frame
```

Official WPB Home launch files use these runtime topics/frames:

```text
/scan_raw       # rplidar_ros output before filtering
/scan           # filtered LaserScan output
laser           # rplidar frame_id
/odom           # published by wpb_home_core on hardware
imu/data_raw    # published by wpb_home_core when IMU is enabled
imu             # IMU message frame_id in upstream source
imu_base        # parent frame used by the upstream imu TF helper
```

Kinect topics are provided by the external `kinect2_bridge` package, not by the
URDF itself. Confirm the actual image topic names in the running system with
`rostopic list`; common Kinect2 topics differ from `/camera/rgb/image_raw`.

## Xacro Expansion

If an official `.xacro` exists, expand it into `urdf/generated/` for debugging:

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun xacro xacro $(rospack find mr_description)/urdf/<robot_name>/<model>.xacro \
  > src/mr_description/urdf/generated/<model>.urdf
```

WPB Home upstream currently provides plain `.urdf` files, but they can still be
passed through `xacro` for consistency:

```bash
rosrun xacro xacro $(rospack find mr_description)/urdf/wpb_home/wpb_home.urdf \
  > src/mr_description/urdf/generated/wpb_home.urdf
```

Validate expanded output:

```bash
check_urdf src/mr_description/urdf/generated/wpb_home.urdf
```

## Node And Topic Checks

Description-only check:

```bash
roslaunch mr_description wpb_home_description.launch model:=wpb_home use_rviz:=true
roslaunch mr_description wpb_home_description.launch model:=wpb_home_mani use_rviz:=true
```

Runtime topic checks:

```bash
rostopic list
rostopic echo -n 1 /scan
rostopic echo -n 1 /odom
rostopic echo -n 1 /imu/data_raw
rosrun tf view_frames
```

For official WPB Home hardware operation, the upstream bringup nodes are also
needed:

```text
wpb_home_core
wpb_home_lidar_filter
wpb_home_imu_tf
rplidar_ros/rplidarNode
kinect2_bridge
```

Those nodes depend on real hardware devices such as `/dev/ftdi`, `/dev/rplidar`,
and Kinect2 USB access. They are intentionally not faked in `mr_description`.

## Forbidden Operations

Do not:

```text
Rewrite official URDF into a local style.
Replace visual meshes with box/cylinder/sphere approximations.
Guess mesh names instead of copying official mesh files.
Delete official original URDF/xacro files after import.
Change sensor link names unless all downstream launch/config files are updated.
Make Gazebo plugins silently publish different topic or frame names.
Commit generated URDF over the official source file.
```

If a mesh path fails, fix the package path or copy the missing official mesh.
Do not replace the visual with a box.
