# Robomaster_2024XJTLU_Vision [:Async: Code]
### _quner*2024_

## Features
- CPU+GPU Inference
- Higer Inference FPS
- Multi threads boosting included
- Linking with ROS2 messages (gary_msgs included written by [myaki-moe](https://github.com/myaki-moe) & [W-YXN](https://github.copm/W-YXN))
#

> 请注意，此版本代码为异步多线程版本
> 相对于同步线程版本，拥有更高帧率的同时会导致推理延迟增加。

#

## Build & Installation
> 如果你所在的地区访问Github速度慢或存在困难
> 可使用反向代理加速下载
> 反向代理: https://r.qunercloud.com
> 
```sh
git clone https://r.qunercloud.com/https://github.com/qunerCloud/Robomaster_2024_XJTLU_Vision.git
```
>
#### Environment requirements
- OpenVINO 2023.2  settled in /opt/intel
- ROS2 Humble (other version is OK)  settled in /opt/ros
Please Install the dependencies and build code then launch vision.
#
### Build
```sh
source /opt/ros/install/setup.bash
source ../gary_msgs/install/setup.bash
colcon build
```
### Run
```sh
source install/setup.bash
ros2 run quner_vision quner_vision
```
