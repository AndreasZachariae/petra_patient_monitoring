# PeTRA

## Patientenüberwachung

* Ist der Patient noch da?
* An welcher (2D-) Position befindet er sich?
	* Tracking mit Kamera und Pan-Tilt Einheit
* Wie weit ist der Patient entfernt?
	* Mit 3D-Kamera Pointcloud
	* Sensorische Kopplung bei Laufen bzw. mit Rollator / Rollstuhl
	* Beschleunigen / Bremsen der Platform
* Wie schnell läuft der Patient?
	* Zeitliche Veränderung der Pose abzüglich Roboterbewegung
* Gesundheitszustand des Patienten?
	* Sturzerkennung
	* gebeugter Gang / humpeln



## OpenPose 

### Installation:

`git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose`

`cd ~/openpose/build`

`~/cmake-3.18.4/bin/cmake-gui`

configure

generate

``make -j `nproc` ``

### Run:

`./build/examples/openpose/openpose.bin --scale_number 4 --scale_gap 0.25 --hand`

`./build/examples/openpose/openpose.bin --scale_number 4 --scale_gap 0.25 --face`

`./build/examples/openpose/openpose.bin --face --hand`



## ROS2 OpenPose Wrapper
https://github.com/firephinx/openpose_ros/tree/ros2

### Install:

`cd openpose/build`

`sudo make install`
(for unistall: sudo make uninstall)

add to ~/.bashrc
`LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib`

`cd workspace/src`

`git clone -b ros2 https://github.com/firephinx/openpose_ros.git`

### Run:
`ros2 launch openpose_ros openpose_ros.launch.py`

openpose_flags.cpp
- fps_max = 10
- scale_number = 4
- scale_gap = 0.25
- face = false
- hand = false
- display = 2



## FeatureExtractor
`ros2 run petra_patient_monitoring FeatureExtractor`



## ROS2 USB-Cam driver
https://github.com/klintan/ros2_usb_camera/tree/dashing-devel

`git clone https://github.com/klintan/ros2_usb_camera.git`

change usb_camera_driver.cpp line51 to `file:///home/andreas/petra_ws/src/ros2_usb_camera/config/camera.yaml`

### Run:
`ros2 run usb_camera_driver usb_camera_driver_node __ns:=/<your namespace> __params:=config.yaml`

Available parameters:

- frame_id -> transform frame_id of the camera, defaults to "camera"
- image_width -> image width (1280 default)
- image_height -> image height (720 default)
- fps -> video fps (10 default)
- camera_id -> id of camera to capture (0 - 100, 0 default)
- Calibration files
To use the camera info functionality you need to load a file from the camera_calibration (https://github.com/ros-perception/image_pipeline/tree/ros2/camera_calibration) library and put it in/name it file:///Users/<youruser>/.ros/camera_info/camera.yaml



## ROS2 image_view
http://wiki.ros.org/image_view

`sudo apt install ros-dashing-image-view`

### Run:
`ros2 run image_view image_view`

rosrun image_view image_view image:=<image topic> [image transport type]

## Alternative:
`rqt` 

Plugins -> Visualization -> Image view



## ROS2 Video Streamer
https://github.com/klintan/ros2_video_streamer

### Install:

`colcon build --symlink-install`

`source ./install/setup.bash`

Make sure to activate the workspace where `vision_opencv` is first.

`sudo apt install ros-dashing-vision-opencv`

pip install natsort

### Run:

`ros2 run camera_simulator camera_simulator --type video --path <my-video-path>`

`ros2 run camera_simulator camera_simulator --type video --path ~/openpose/examples/media/Sturzvideos/Sturz10_hoch_wave.mp4 --calibration_file ~/petra_ws/src/ros2_video_streamer/data/camera.yaml`

`ros2 launch camera_simulator camera_simulator.launch.py`



## ROS1 Video_stream_opencv
http://wiki.ros.org/video_stream_opencv

### Install:
sudo apt-get install ros-melodic-video-stream-opencv

### Run:

ros1_bridge

roslaunch video_stream_opencv camera.launch video_stream_provider:=/home/andreas/openpose/examples/media/Sturzvideos/Sturz10_hoch_wave.mp4



## ROS2 bag
https://github.com/ros2/rosbag2

### Install:
sudo apt install ros-dashing-rosbag2*

sudo apt install ros-dashing-ros2bag

### Run:

ros2 bag record /image /camera_info /openpose_ros/human_list -o fall10

ros2 bag play fall10