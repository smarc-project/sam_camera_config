# Nvidia Jetson CSI camera launcher on SAM + car detection in simulator(in the end)
Using gstreamer and the Nvidia multimedia API to encode the video streams to compressed jpeg images with hardware accelerated (not GPU), and publish to ROS topics.

**Features**

* Control resolution and framerate
* Save compressed jpeg images
* Multi camera support

## Dependencies

* Gstreamer-1.0 and Nvidia multimedia API (installed by Jetpack)
* ROS Melodic
* `gscam`

## Installation 

### 0. Download `gscam` into `catkin_ws` if you don't have it
```
cd ~/catkin_ws/src
git clone https://github.com/ros-drivers/gscam.git
```

### 1. Download `sam_camera_config` into `catkin_ws`

### 2. Build and register `gscam` and `sam_camera_config` in ROS
```
cd ~/catkin_ws
catkin_make
```

## Usage
To publish the three cameras' compressed images to ROS using default settings:
```
roslaunch sam_camera_config multi_nv_jpeg.launch
```
The compressed jpeg images are published to the ROS topics, and if we have `rosbag record -a` running, the images are saved in the rosbags. 

To use only one camera:
```
roslaunch sam_camera_config nv_jpeg.launch
```
There is a few arguments could be changed here:

* `sensor_id`: The sensor id of the camera: 0,1,2
* `FPS`: the framerate: e.g. 30 (do not go beyond 60)
* `WIDTH`: the width, e.g.1280
* `HEIGHT`: the height, e.g. 720

The recommended resolutions are:

* `WIDTH`: 1280, `HEIGHT`: 720
* `WIDTH`: 1280, `HEIGHT`: 540
* `WIDTH`: 1920, `HEIGHT`: 1080

One example:
```
roslaunch sam_camera_config nv_jpeg.launch sensor_id:=1 HEIGHT:=360 WIDTH:=540 FPS:=15
```

## N.B.
The `multi_nv_jpeg.launch` simply calls three `nv_jpeg.launch` with three different `sensor_id` and all the other arguments as default in the launch file `nv_jpeg.launch`, so if you were to use different arguments ohter than the default like FPS,WIDTH,HEIGHT for all three cameras, go to file `nv_jpeg.launch` in `sam_camera_config/launch` and change the default settings.

According to the camera driver developer, the camera driver supports:  `3840x2160@60fps, 1920x1080@60fps, 1280x720@60fps and 1280x540@240fps`

# For the car detection in simulator
The `sam_detection.launch` is started with the bringup, just make sure that the `car_depth` parameter in this launch file is consistent with the depth of the car defined in the `asko_world/asko_env.scn`

By default it will take the image in down-ward looking camera, apply image enhancement, and detect yellow color in it, then use the size of car to calculate the relative 3d position in the camera frame, which is published to rosparam `/sam/detection/poi_down`. 
