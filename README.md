# laser_merger2
Muilti laser merger to pointcloud2 and laserScan for ROS2.

## Built with

- ROS Humble under Ubuntu 22.04 LTS

------

## Getting Started

### Installation

``` bash
$ git clone https://github.com/leroycorentin/laser_merger2.git
```

### ROS2 topic

------

| Topic                              | Description                                                       |
| ---                                | ---                                                               |
| pointcloud                         | Merger pointcloud2 msg.                                           |
| scan                               | Merger laser scan msg.                                            ||

| Parameter                          | Description                                                       |
| ---                                | ---                                                               | 
| target_frame                       | target tf frame(Default: "base_link").                            |
| scan_topics                        | List of topics on which to read the laser scans                   |
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
| use_inf                            | use inf.                                                          |
| output_pointcloud_topic            | Name of the output merged point cloud topic                       |
| output_scan_topic                  | Name of the output merged scan topic                              ||

### Run

------

``` bash
$ ros2 launch laser_merger2 laser_merger.launch.py
```
For example:
``` bash
$ ros2 launch laser_merger2 laser_merger.launch.py target_frame:=base scan_topics:="[/lidar, /lidar2]" output_pointcloud_topic:=/merged_pcl
```

### Result

------

1. Laser 1 Data

![image](https://github.com/qaz9517532846/laser_merger2/blob/main/image/front_laser.png)

2. Laser 2 Data

![image](https://github.com/qaz9517532846/laser_merger2/blob/main/image/rear_laser.png)


3. Merge Process Data Result

![image](https://github.com/qaz9517532846/laser_merger2/blob/main/image/merger_laser.png)

------

## Reference:

[1]. pointcloud_to_laserscan, https://github.com/ros-perception/pointcloud_to_laserscan

------

## License:

This repository is for your reference only. copying, patent applications, and academic journals are strictly prohibited.

Copyright © 2023 ZM Robotics Software Laboratory.
