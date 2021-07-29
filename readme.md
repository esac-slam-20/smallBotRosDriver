# smallBotRosDriver
这是一个用来运行在ros noetic的ros驱动库，主要服务于[smallBot](https://tea.lan.bigkeer.cn/SLAM2020/Project-SmallBot)
。
## 主要功能
1. baseDriver:接收[geometry_msgs/Twist](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html),并且根据[协议](https://tea.lan.bigkeer.cn/SLAM2020/Project-SmallBot-MCU)转化数据，并且通过串口发送到下位机。
2. odomDriver:根据[协议](https://tea.lan.bigkeer.cn/SLAM2020/Project-SmallBot-MCU)从下位机读取数据，然后转化到TF数据和[nav_msgs/Odometry](http://docs.ros.org/en/kinetic/api/nav_msgs/html/msg/Odometry.html)话题。
## 依赖库
1. 各种ros库
> 我写完才知道缺了啥，如果忘了添加，基本也是缺啥`apt install`一下啥
2. Eigen3
> 主要用于数学相关的计算
3. boost
> 主要是串口要用到
4. g++
> 要支持c++17及以上

## 安装方式
```
git clone xxx
git submodule update --init
cd ws
catkin_make
```
## 运行方式
```
rosrun smallBotRosDriver control_serial port baud_rate
```