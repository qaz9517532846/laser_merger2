# laser_merger2
Muilti laser merger to pointcloud2 and laserScan for ROS2.

## Built with

- ROS Foxy under Ubuntu 20.04 LTS

## Required Package

- NVIDIA CUDA Package

- NVIDIA Cublas Package

------

## Getting Started

### Installation

``` bash
$ git clone https://github.com/qaz9517532846/laser_merger2.git
```

### ROS2 topic

------

| Topic                              | Description                                                       |
| ---                                | ---                                                               | 
| scan_0                             | Subscriber scan 1st.                                              |
| scan_1                             | Subscriber scan 2nd.                                              |
| scan_<num>                         | Subscriber num th.                                                |
| pointcloud                         | Merger pointcloud2 msg.                                           |
| scan                               | Merger laser scan msg.                                            ||

| Parameter                          | Description                                                       |
| ---                                | ---                                                               | 
| target_frame                       | target tf frame(Default: "base_link").                            |
| laser_num                          | Subscriber laser scan num.                                        |
| transform_tolerance                | TF transform tolerance.                                           |
| rate                               | Publish rate(Hz).                                                 |
| queue_size                         | Subscribe queue size.                                             |
| max_range                          | Merge laser scan max range.                                       |
| min_range                          | Merge laser scan min range.                                       |
| max_angle                          | Merge laser scan max angle.                                       |
| min_angle                          | Merge laser scan min angle.                                       |
| scan_time                          | Merge laser scan scan time.                                       |
| angle_increment                    | Merge laser scan angle increment.                                 |
| inf_epsilon                        | inf epsilon value.                                                |
| use_inf                            | use inf.                                                          ||

### Run

------

``` bash
$ ros2 run laser_merger2 laser_merger.launch.py
```

------

## Reference:

[1]. pointcloud_to_laserscan, https://github.com/ros-perception/pointcloud_to_laserscan

------

## License:

This repository is for your reference only. copying, patent applications, and academic journals are strictly prohibited.

Copyright © 2023 ZM Robotics Software Laboratory.
