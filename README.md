# What's new with this fork

Disable the publishing of tf and use it with Turtlebot3

# ROS2 Wrapper for Intel&reg; RealSense&trade; Devices
These are packages for using Intel RealSense cameras (D400 and L500 series, SR300 camera and T265 Tracking Module) with ROS2 Eloquent.

LibRealSense supported version: v2.40.0 (see [realsense2_camera release notes](https://github.com/IntelRealSense/realsense-ros/releases))

## Installation Instructions

### Requirements:

- Ubuntu 18.04.5 LTS
- ROS2 Eloquent

### Install librealsense2 (Install the latest Intel&reg; RealSense&trade; SDK 2.0):

   - #### Install from [Debian Package](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages) - In that case treat yourself as a developer. Make sure you follow the instructions to also install librealsense2-dev and librealsense-dkms packages.

   - #### **OR** Build from sources by downloading the latest [Intel&reg; RealSense&trade; SDK 2.0](https://github.com/IntelRealSense/librealsense/releases/tag/v2.40.0) and follow the instructions under [Linux Installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)

### Create ROS2 Workspace:

```shell
cd ~
mkdir -p ros2_worskpace/src
echo "source ~/ros2_workspace/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Build custom Realsense packages:

```shell
cd ~/ros2_workspace/src
git clone -b eloquent https://github.com/MourtazaKASSAMALY/realsense-ros.git
cd ..
```

### Install dependencies

```bash
sudo apt-get install python-rosdep -y
sudo rosdep init
rosdep update --include-eol-distros
rosdep install -i --from-path src/realsense-ros/ --rosdistro eloquent -y
```

### Build

```bash
colcon build --symlink-install
source ~/.bashrc
```

## Using T265 with Turtlebot3 ##

To determine the USB port of your camera:
```bash
  rs-enumerate-devices
```
To run some experiments with the realsense-viewer included with the librealsense:
```bash
realsense-viewer
```
To launch the camera node:
```bash
ros2 launch realsense2_camera rs_launch.py device_type:=t265 enable_fisheye1:=false enable_fisheye2:=false
```
To remap odom topic from realsense node and generate corresponding tf. Setting the wheels odometry topic from the turtlebot3 is crucial as it is used to pace the publishing of remapped odometry and tf:
```bash
ros2 run realsense_remap remap_node --ros-args -p topic_odom_in:=/camera/odom/sample topic_odom_out:=/odom wheels_topic:=/wheels_odom frame_id_out:=odom child_frame_id_out:=realsense
```

Publish the static transform between the realsense frame and the footprint (rotation center) of the turtlebot3. In this case, the base_footprint is 20 cm behind and 1 cm to the right of the realsense camera center (the vertical shift is not important here as we only want to match the rotation axis):
```bash
ros2 run tf2_ros static_transform_publisher -0.20 0.01 0 0 0 0 realsense base_footprint
```

## Published Topics
The published topics differ according to the device and parameters.
After running the above command with D435i attached, the following list of topics will be available (This is a partial list. For full one type `ros2 topic list`):
- /camera/accel/imu_info
- /camera/color/camera_info
- /camera/color/image_raw
- /camera/depth/camera_info
- /camera/depth/color/points
- /camera/depth/image_rect_raw
- /camera/extrinsics/depth_to_color
- /camera/extrinsics/depth_to_infra1
- /camera/extrinsics/depth_to_infra2
- /camera/gyro/imu_info
- /camera/imu
- /camera/infra1/camera_info
- /camera/infra1/image_rect_raw
- /camera/infra2/camera_info
- /camera/infra2/image_rect_raw
- /camera/parameter_events
- /camera/rosout
- /parameter_events
- /rosout
- /tf_static

> Using an L515 device the list differs a little by adding a 4-bit confidence grade (pulished as a mono8 image):
>- /camera/confidence/camera_info
>- /camera/confidence/image_rect_raw
>
> It also replaces the 2 infrared topic sets with the single available one:
>- /camera/infra/camera_info
>- /camera/infra/image_raw

The "/camera" prefix is the namesapce specified in the given launch file.
When using D435 or D415, the gyro and accel topics wont be available. Likewise, other topics will be available when using T265 (see below).

## Available Parameters:
For the entire list of parameters type `ros2 param list`.

- **serial_no**: will attach to the device with the given serial number (*serial_no*) number. Default, attach to the first (in an inner list) RealSense device.
  - Note: serial number can also be defined with "_" prefix. For instance, serial number 831612073525 can be set in command line as `serial_no:=_831612073525`. That is a workaround until a better method will be found to ROS2's auto conversion of strings containing only digits into integers.
- **usb_port_id**: will attach to the device with the given USB port (*usb_port_id*). i.e 4-1, 4-2 etc. Default, ignore USB port when choosing a device.
- **device_type**: will attach to a device whose name includes the given *device_type* regular expression pattern. Default, ignore device type. For example, device_type:=d435 will match d435 and d435i. device_type=d435(?!i) will match d435 but not d435i.

- **rosbag_filename**: Will publish topics from rosbag file.
- **initial_reset**: On occasions the device was not closed properly and due to firmware issues needs to reset. If set to true, the device will reset prior to usage.
- **align_depth**: If set to true, will publish additional topics with the all the images aligned to the depth image.</br>
The topics are of the form: ```/camera/aligned_depth_to_color/image_raw``` etc.
- **filters**: any of the following options, separated by commas:</br>
 - ```colorizer```: will color the depth image. On the depth topic an RGB image will be published, instead of the 16bit depth values .
 - ```pointcloud```: will add a pointcloud topic `/camera/depth/color/points`. The texture of the pointcloud can be modified in rqt_reconfigure (see below) or using the parameters: `pointcloud_texture_stream` and `pointcloud_texture_index`. Run rqt_reconfigure to see available values for these parameters.</br>
 The depth FOV and the texture FOV are not similar. By default, pointcloud is limited to the section of depth containing the texture. You can have a full depth to pointcloud, coloring the regions beyond the texture with zeros, by setting `allow_no_texture_points` to true.

 - The following filters have detailed descriptions in : https://github.com/IntelRealSense/librealsense/blob/master/doc/post-processing-filters.md
   - ```disparity``` - convert depth to disparity before applying other filters and back.
   - ```spatial``` - filter the depth image spatially.
   - ```temporal``` - filter the depth image temporally.
   - ```hole_filling``` - apply hole-filling filter.
   - ```decimation``` - reduces depth scene complexity.
- **enable_sync**: gathers closest frames of different sensors, infra red, color and depth, to be sent with the same timetag. This happens automatically when such filters as pointcloud are enabled.
- ***<stream_type>*_width**, ***<stream_type>*_height**, ***<stream_type>*_fps**: <stream_type> can be any of *infra, color, fisheye, depth, gyro, accel, pose*. Sets the required format of the device. If the specified combination of parameters is not available by the device, the stream will not be published. Setting a value to 0, will choose the first format in the inner list. (i.e. consistent between runs but not defined). Note: for gyro accel and pose, only _fps option is meaningful.
- **enable_*<stream_name>***: Choose whether to enable a specified stream or not. Default is true. <stream_name> can be any of *infra1, infra2, color, depth, fisheye, fisheye1, fisheye2, gyro, accel, pose*.

- ***<stream_name>*_frame_id**: Choose whether to enable a specified stream or not. Default is true. <stream_name> can be any of *infra1, infra2, color, depth, fisheye, fisheye1, fisheye2, gyro, accel, pose*.


- ***<stream_name>*_frame_id**, ***<stream_name>*_optical_frame_id**, **aligned_depth_to_*<stream_name>*_frame_id**: Specify the different frame_id for the different frames. Especially important when using multiple cameras.

- **base_frame_id**: defines the frame_id all static transformations refers to.
- **odom_frame_id**: defines the origin coordinate system in ROS convention (X-Forward, Y-Left, Z-Up). pose topic defines the pose relative to that system.

- **unite_imu_method**: The D435i and T265 cameras have built in IMU components which produce 2 unrelated streams: *gyro* - which shows angular velocity and *accel* which shows linear acceleration. Each with it's own frequency. By default, 2 corresponding topics are available, each with only the relevant fields of the message sensor_msgs::Imu are filled out.
Setting *unite_imu_method* creates a new topic, *imu*, that replaces the default *gyro* and *accel* topics. The *imu* topic is published at the rate of the gyro. All the fields of the Imu message under the *imu* topic are filled out.
   - **linear_interpolation**: Every gyro message is attached by the an accel message interpolated to the gyro's timestamp.
   - **copy**: Every gyro message is attached by the last accel message.
- **clip_distance**: remove from the depth image all values above a given value (meters). Disable by giving negative value (default)
- **linear_accel_cov**, **angular_velocity_cov**: sets the variance given to the Imu readings. For the T265, these values are being modified by the inner confidence value.
- **hold_back_imu_for_frames**: Images processing takes time. Therefor there is a time gap between the moment the image arrives at the wrapper and the moment the image is published to the ROS environment. During this time, Imu messages keep on arriving and a situation is created where an image with earlier timestamp is published after Imu message with later timestamp. If that is a problem, setting *hold_back_imu_for_frames* to *true* will hold the Imu messages back while processing the images and then publish them all in a burst, thus keeping the order of publication as the order of arrival. Note that in either case, the timestamp in each message's header reflects the time of it's origin.
- **topic_odom_in**: For T265, add wheel odometry information through this topic. The code refers only to the *twist.linear* field in the message.
- **calib_odom_file**: For the T265 to include odometry input, it must be given a [configuration file](https://github.com/IntelRealSense/librealsense/blob/master/unit-tests/resources/calibration_odometry.json). Explanations can be found [here](https://github.com/IntelRealSense/librealsense/pull/3462). The calibration is done in ROS coordinates system.
- **publish_tf**: boolean, publish or not TF at all. Defaults to True.
- **tf_publish_rate**: double, positive values mean dynamic transform publication with specified rate, all other values mean static transform publication. Defaults to 0 
- **publish_odom_tf**: If True (default) publish TF from odom_frame to pose_frame.
- **infra_rgb**: When set to True (default: False), it configures the infrared camera to stream in RGB (color) mode, thus enabling the use of a RGB image in the same frame as the depth image, potentially avoiding frame transformation related errors. When this feature is required, you are additionally required to also enable `enable_infra:=true` for the infrared stream to be enabled.
  - **NOTE** The configuration required for `enable_infra` is independent of `enable_depth`
  - **NOTE** To enable the Infrared stream, you should enable `enable_infra:=true` NOT `enable_infra1:=true` nor `enable_infra2:=true`
  - **NOTE** This feature is only supported by Realsense sensors with RGB streams available from the `infra` cameras, which can be checked by observing the output of `rs-enumerate-devices`

## Other ways of using packages ##

### Start the camera node

To start the camera node in ROS:

```bash
  ros2 launch realsense2_camera rs_launch.py
```
or, with parameters specified in rs_launch.py, for example - pointcloud enabled:
```bash
ros2 launch realsense2_camera rs_launch.py enable_pointcloud:=true device_type:=d435
```
or, without using the supplement launch files:
```bash
ros2 run realsense2_camera realsense2_camera_node --ros-args -p filters:=colorizer
```

This will stream all camera sensors and publish on the appropriate ROS topics.

### Examples for using with T265

To start the camera node in ROS:

```bash
ros2 run realsense2_camera realsense2_camera_node --ros-args -p enable_pose:=true -p device_type:=t265 -p fisheye_width:=848 -p fisheye_height:=800
```
or, if you also have a d4xx connected, you can try out the launch file:
```bash
ros2 launch realsense2_camera rs_d400_and_t265_launch.py enable_fisheye12:=true enable_fisheye22:=true
```
- note: the parameters are called `enable_fisheye12` and `enable_fisheye22`. The node knows them as `enable_fisheye1` and `enable_fisheye2` but launch file runs 2 nodes and these parameters refer to the second one.

## Still on the pipelie:
* Support ROS2 life cycle.
* Enable and disable sensors and filters.

## License
Copyright 2018 Intel Corporation

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this project except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

**Other names and brands may be claimed as the property of others*
