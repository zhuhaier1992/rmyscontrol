# Readme
中文（详细）说明：https://www.jianshu.com/p/4b9c0f4658b1?v=1713596074817
### Intro
This repo provides simple control plan over robotic football, each team consists of several Yanshees and RoboMasters (RM). 

Positional feedback comes from motion capture system.
### Prerequisites
Ubuntu20.

Install ros2-galactic.

Install [robomaster-ros](https://github.com/jeguzzi/robomaster_ros).

Install [RVO2 v2.0.2](https://gamma.cs.unc.edu/RVO2/downloads/).

Use system python to install all required modules, conda could cause unexpected problem like "module not found". 

Install [nokov](https://cloud.tsinghua.edu.cn/d/222902bb277b4f09afaa/) by pip.

Install other python modules:

```
pip install interval sympy
```
Could need to install "empy, lark" by pip too.

For yanshee control, usually need to install nest-asyncio and google protobuf==3.20.0.

```
pip install nest-asyncio protobuf==3.20.0
```

For yanshee API, refer to [this website](https://yandev.ubtrobot.com/#/zh/api?api=YanAPI).

For robomaster API, check [this website](https://robomaster-dev.readthedocs.io/zh-cn/latest/). Better way is to check the [examples](https://github.com/dji-sdk/RoboMaster-SDK/tree/master/examples)
### Build
Pack this repo to a folder 'src', and put it in a workspace.

In workspace, run ```colcon build```.
### Usage
Start XINGYING (Nokov mocap software), check 'SDK' in settings and start running.

If XINGYING starts on another PC, then connect server to the switch of mocap by ethernet cable.(Important note: must be connected AFTER XINGYING is started.)

Run following command on server.
```
ros2 run motion_capture motion_capture
```
If it shows 'motion capture initialization finished!!' then run following commands in new terminals.
```python
ros2 run roscpp pub_pos # obastacle avoidance
ros2 launch rmcontrol ep_startup.launch.py # connect to RM EPs
ros2 launch rmcontrol s1_startup.launch.py # connect to RM S1s
ros2 launch yscontrol ys_control.launch.py # control Yanshees
ros2 run rmcontrol strategy # start control plans for RMs
ros2 launch rmcontrol ep_control.launch.py # control RMs
ros2 run rmcontrol draw_plot # optional. Show current and expected position of all robots and ball
ros2 run motion_capture inbetween #optional. If need a second server in local network to cooperate, then run this 'inbetween' node before ethernet connection to transmit positional info.
```
To stop RMs, stop strategy node and ep_startup.launch.py and s1_startup.launch.py.


### Hardware
./motion_capture: Nokov motion capture system

./rmcontrol: DJI Robomaster EP/S1

./yscontrol: Yanshee from UBTech (Always fall over. Recalibration works badly.)
### About
Provided by Haier Zhu from THU SIGS under instruction of Prof. Li, Xiang.

This work was supported in part by the National Natural Science Foundation of China under Grant U21A20517 and 52075290, and in part by the Science and Technology Innovation 2030-Key Project under Grant 2021ZD0201404.
