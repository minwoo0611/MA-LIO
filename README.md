# MA-LIO
This repository is for MA-LIO(Multiple Asynchronous LiDAR Inertial Odometry using Point-wise Inter-LiDAR Uncertainty Propagation).
Our paper is currently under review, and the code will be released after review process.

## Time Table

### 2023.04.20 
We have initiated the process of opening our private dataset, which is employed for evaluation purposes in our research paper. The City dataset is currently being uploaded via Google Drive. Upon completion of the upload, a link will be provided for access.
### 2023.04.19 
We commenced the process of preparing the MA-LIO repository for open-sourcing by making it publicly accessible.

## 1. Prerequisites
### 1.1. livox_ros_driver
Follow [livox_ros_driver Installation](https://github.com/Livox-SDK/livox_ros_driver). To build the MA-LIO and file_player for City Datasets, this driver should be sourced before catkin_build or catkin_make.

## 2. Install

Use the following commands to download and compile the package.

```
cd ~/catkin_ws/src
git clone https://github.com/minwoo0611/MA-LIO.git
cd ..
catkin_build
```

## 3. Datasets
We utilize 3 datasets for the evaluation. The field of view of LiDARs in each dataset is exhibited as below.
<p align="center">
  <img width="712pix" src="figs/Datasets.jpg">
</p>

### 3.1. Hilti SLAM Dataset 2021
The Hilti SLAM Dataset was employed for the evaluation of MA-LIO in indoor and outdoor. This dataset comprises 200Hz IMU data, OS0-64, and Livox Horizon. To access and utilize the dataset, please follow this [link](https://hilti-challenge.com/dataset-2021.html)

## 3.2. UrbanNav Dataset
The UrbanNav Dataset was also employed for the evaluation. This dataset is composed of high-rise buildings and expansive urban environments, utilizing a 400Hz IMU and three spinning LiDAR sensors (HDL-32E, VLP 16, and LS-C16). Notably, the two 16-channel LiDAR sensors are inclined, providing an advantage in observing high-rise structures. The dataset can be found through the [link](https://github.com/IPNL-POLYU/UrbanNavDataset).

## 3.3. City Datasets
The City Datasets were utilized for the evaluation and ablation study of MA-LIO. This dataset was collected in urban environments using a car, exhibiting high velocity, dynamic objects, numerous rotations with U-turns, and tunnels. The dataset was acquired using three LiDAR sensors (Livox Avia, Livox Tele, and OS2-128) and a 100Hz IMU. The ground truth data was obtained using the SPAN CPT-7 system. The extrinsic transformation between the IMU and LiDAR sensors can be verified through the [Extrinsic.txt](https://github.com/minwoo0611/MA-LIO/blob/main/Extrinsic.txt) file.

In the datasets, we provide Livox Avia, Tele, and Ouster point files (.bin) containing full values (x, y, z, intensity, time, etc.) from their original point cloud data (PCD). The bin file names correspond to their timestamps. Additionally, the data_stamp.csv and ouster_stamp.csv files display the timestamps of our measurements, while the Groundtruth.txt file presents the ground truth trajectory with the format (timestamp, x, y, z, qx, qy, qz, qw). The xsens_imu.csv file contains IMU measurements.

By using the provided file player, users can transform the .bin files and .csv files into ROS messages. In the case of Ouster data, the files are converted to sensor_msgs::PointCloud2 format, while for Livox data, they are transformed into livox_ros_driver::CustomMsg format. Finally, the IMU data is presented as sensor_msgs::Imu.
