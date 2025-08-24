# point-lio-converter

## Intended use 

This small toolset allows to integrate SLAM solution provided by [point-lio](https://github.com/hku-mars/Point-LIO) with [HDMapping](https://github.com/MapsHD/HDMapping).
This repository contains ROS  workspace that :
  - submodule to tested revision of Point-LIO
  - a converter that listens to topics advertised from odometry node and save data in format compatible with HDMapping.

## Dependencies

```shell
sudo apt install -y nlohmann-json3-dev
```

## Building

Clone the repo
```shell
mkdir -p /test_ws/src
cd /test_ws/src
git clone https://github.com/marcinmatecki/Point-LIO-to-HDMapping.git --recursive
cd ..
catkin_make
```

## Usage - data SLAM:

Prepare recorded bag with estimated odometry:

In first terminal record bag:
```shell
rosbag record /cloud_registered /aft_mapped_to_init
```

and start odometry:
```shell 
cd /test_ws/
source ./install/setup.sh # adjust to used shell
For launch,check https://github.com/hku-mars/Point-LIO/
```

## Usage - conversion:

```shell
cd /test_ws/
source ./install/setup.sh # adjust to used shell
rosrun point-lio-to-hdmapping listener <recorded_bag> <output_dir>
```

## Example:

Download the dataset from [GitHub - ConSLAM](https://github.com/mac137/ConSLAM) or 
directly from this [Google Drive link](https://drive.google.com/drive/folders/1TNDcmwLG_P1kWPz3aawCm9ts85kUTvnU). 
Then, download **sequence2**.

## Record the bag file:

```shell
rosbag record /cloud_registered /Odometry -o {your_directory_for_the_recorded_bag}
```

## Point-LIO Launch:

```shell
cd /test_ws/
source ./devel/setup.sh # adjust to used shell
roslaunch point_lio mapping_velody16.launch
rosbag play {path_to_bag} /pp_points/synced2rgb:=/velodyne_points /imu/data:=/imu/data
```

## During the record (if you want to stop recording earlier) / after finishing the bag:

```shell
In the terminal where the ros record is, interrupt the recording by CTRL+C
Do it also in ros launch terminal by CTRL+C.
```

## Usage - Conversion (ROS bag to HDMapping, after recording stops):

```shell
cd /test_ws/
source ./install/setup.sh # adjust to used shell
rosrun point-lio-to-hdmapping listener <recorded_bag> <output_dir>
```
