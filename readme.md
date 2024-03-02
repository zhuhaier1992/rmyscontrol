# Readme
### About
This repo provides simple control plan over robotic football, each team consists of several Yanshees and RoboMasters (RM). 

Positional feedback comes from motion capture system.
### Build
Pack this repo to a folder 'src', and put it in a workspace.

In workspace, run ```colcon build```.
### Usage
Start XINGYING (Nokov mocap software), check 'SDK' in setting and start running.

Connect server to the switch of mocap by ethernet cable.

Run following command on server.
```
ros2 run motion_capture motion_capture
```
If it shows 'motion capture initialization finished' then run following commands in new terminals.
```
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
