## Initialization
The **hoverboard_mvp** package is a ROS 2 package. It uses the Python3 package `pyquaterion`. To install this package run:
```bash
  $ pip3 install pyquaternion
```

Update the environment variables and after that, compile the project. Run on a terminal:
```bash
  $ source <your_ros2_workspace>/install/setup.bash
```

```bash
  $ colcon build --packages-select hoverboard_mvp
```
With the package compiled, initialize the simulation:
```bash
  $ ros2 launch hoverboard_mvp robot_simulation_launch.py
```
Initialize the controllers, running on another terminal:

```bash
  $ ros2 launch hoverboard_mvp controllers.launch.py 
```
## Using the Package

This package has the MVP implementation of a semi-autonomous hoverboard. The hoverboard has 2 modes: **teleop_mode** and **semi_autonomous mode**. On the **teleop_mode** you can control the vehicle using the *teleop_twist_keyboard* package to send velocities commands to it. On the **semi-autonomous mode** you can send the command to move the vehicle directly to a specific Pose. On this mode you also has the functionality to move back, in a semi-autonomous way, to a defined start position. Once initialized, the hoveboard starts on the **teleop_mode**.

As the hoverboard is initialized on the **teleop_mode**, to control the vehicle, open a terminal and run:

```bash
  $ ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/teleop/cmd_vel
``` 
The `--remap` argument is important in order to send the command through the correct topic. If the command was not sent through this topic, the vehicle will not move on the **teleop_mode**.

To switch between the modes, there is a *service* available with the name ***/set_semi_autonomous***. To enable the **semi-autonomous mode**, you should call the *service* sending the resquest.data as True. To send this command you can open another terminal and run:

```sh
  $ ros2 service call /set_semi_autonomous example_interfaces/srv/SetBool data: true 
```
Another possibility is using the *rqt* gui. On the terminal, run:

```sh
  $ rqt
```
Once the *rqt* window is opened, add the *Service Caller* plugin through, **Plugins -> Service -> Service Caller**. With the *Service Caller* plugin added, select the ***/set_semi_autonomous*** *service*, change the data area to True and press the Call button.

To change from **semi_autonomous mode** to **teleop_mode** the process is the same. Although, the request.data should be sent as False.

The hoverboard also has the functionality of record a path and drive back through this path to the initial position. First, it is necessary to record the path. To start the recording, there is another *service* called ***/start_tracking***. To enable the recording, call the *service* running on a terminal: 

```sh
  $ ros2 service call /start_tracking example_interfaces/srv/SetBool data: true 
```
As explained on the ***/set_semi_autonomous*** *service*, this service can, also, be called using *rqt*. To disable the path tracking, just call the  ***/start_tracking*** *service* passing the request.data as False. 

Now that you have your path recorded you MUST set the mode to **semi_autonomous mode** and call the ***/go_home*** *service* running this command on the terminal:

```sh
  $ ros2 action send_goal /go_home hoverboard_mvp/action/GoHome start: true  
```

The hoverboard should start moving back on the recorded path to the position where you started the ***/start_tracking***.
