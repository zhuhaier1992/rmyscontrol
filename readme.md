# Readme
### Intro
This repo provides simple control plan over robotic football, each team consists of several Yanshees and RoboMasters (RM). 

Positional feedback comes from motion capture system.
### Build
Pack this repo to a folder 'src', and put it in a workspace.

In workspace, run ```colcon build```.
### Usage
Start XINGYING (Nokov mocap software), check 'SDK' in setting and start running.

If XINGYING starts on another PC, then connect server to the switch of mocap by ethernet cable.(Important note: must be connected AFTER XINGYING is started.)

Run following command on server.
```
ros2 run motion_capture motion_capture
```
If it shows 'motion capture initialization finished' then run following commands in new terminals.
```python
ros2 run rmcontrol rvo # obastacle avoidance
ros2 launch rmcontrol ep_startup.launch.py # connect to RM EPs
ros2 launch rmcontrol s1_startup.launch.py # connect to RM S1s
ros2 launch yscontrol ys_control.launch.py # control Yanshees
ros2 run rmcontrol strategy # start control plans for RMs
ros2 launch ep_control.launch.py # control RMs
ros2 run rmcontrol draw_plot # optional. Show current and expected position of all robots and ball
ros2 run motion_capture inbetween #optional. If need a second server in local network to cooperate, then run this 'inbetween' node before ethernet connection to transmit positional info.
```
To stop RMs, stop strategy node and ep_startup.launch.py and s1_startup.launch.py.


### Hardware
./motion_capture: Nokov motion capture system

./rmcontrol: DJI Robomaster EP

./yscontrol: Yanshee from UBTech 
### About
Provided by Haier Zhu from THU SIGS under instruction of Prof. Li, Xiang.

This work was supported in part by the National Natural Science Foundation of China under Grant U21A20517 and 52075290, and in part by the Science and Technology Innovation 2030-Key Project under Grant 2021ZD0201404.
