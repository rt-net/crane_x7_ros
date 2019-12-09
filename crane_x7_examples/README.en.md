[English](README.en.md) | [日本語](README.md)

# crane_x7_examples

This package includes examples to control CRANE-X7 using `crane_x7_ros`.

## How to launch CRANE-X7 base packages

1. Connect a communication cable from CRANE-X7 to a PC.
1. Open terminal and launch `demo.launch` of `crane_x7_bringup` package.

This launch file has an argument to select CRANE-X7 or virtual CRANE-X7:

- fake_execution (default: true)


### Using virtual CRANE-X7


Launch virtual CRANE-X7 base packages with the following command:

```sh
roslaunch crane_x7_bringup demo.launch fake_execution:=true
```

### Using real CRANE-X7

Launch the base packages with the following command:

```sh
roslaunch crane_x7_bringup demo.launch fake_execution:=false
```

Default serial port name is `/dev/ttyUSB0`.
To change port name (e.g. `/dev/ttyUSB1`), launch the packages with arguments:

```sh
roslaunch crane_x7_bringup demo.launch fake_execution:=false port:=/dev/ttyUSB1
```

### Using Gazebo simulator

Launch the packages with the following command:

```sh
roslaunch crane_x7_gazebo crane_x7_with_table.launch
```

## Run Examples

Following examples will be executable after launch CRANE-X7 base packages.

### gripper_action_example.py

This is an example to open/close the gripper.

Run a node with the following command:

```sh
rosrun crane_x7_examples gripper_action_example.py
```

![gripper_action_example](https://github.com/rt-net/crane_x7_ros/blob/images/images/gazebo_gripper_example.gif)

---

### pose_groupstate_example.py

This is an example using `group_state` of SRDF.

CRANE-X7 changes its posture to `home` and `vertical` listed in SRDF file [crane_x7_moveit_config/config/crane_x7.srdf](../crane_x7_moveit_config/config/crane_x7.srdf).

Run a node with the following command:

```sh
rosrun crane_x7_examples pose_groupstate_example.py
```

![pose_groupstate_example](https://github.com/rt-net/crane_x7_ros/blob/images/images/gazebo_pose_groupstate.gif)

---

### joint_values_example.py

This is an example to change each joint values of arm one by one using `moveit_commander`.

Run a node with the following command:

```sh
rosrun crane_x7_examples joint_values_example.py
```

![joint_values_example](https://github.com/rt-net/crane_x7_ros/blob/images/images/gazebo_joint_values_example.gif)

---

### crane_x7_pick_and_place_demo.py

This is an example to grasp, pick up, carry and place an small object.

Run a node with the following command:

```sh
rosrun crane_x7_examples crane_x7_pick_and_place_demo.py
```

![bringup_rviz](https://github.com/rt-net/crane_x7_ros/blob/images/images/bringup_rviz.gif "bringup_rviz")

**Real environment setup**

Place the small object at a distance of 20 cm from CRANE-X7.

![bringup](https://github.com/rt-net/crane_x7_ros/blob/images/images/bringup.jpg "bringup")

This orange ball can be purchased at [this page](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1307&products_id=3701&language=en) in RT ROBOT SHOP.

Demo Video is [here **(YouTube Video)**](https://youtu.be/_8xBgpgMhk8).

---

### preset_pid_gain_example.py

This is an example to change PID gains of servo motors in bulk using `preset_reconfigure` of `crane_x7_control`.

Lists of PID gain preset values can be edited in [crane_x7_control/scripts/preset_reconfigure.py](../crane_x7_control/scripts/preset_reconfigure.py).

Launch nodes `preset_reconfigure.py` and `preset_pid_gain_example.py` with the following command:

```sh
roslaunch crane_x7_examples preset_pid_gain_example.launch
```

Demo Video is [here **(YouTube Video)**](https://youtu.be/0rBbgNDwm6Y).

---

### teaching_example.py

This is an example to generate an motion trajectory with direct teaching.

User can operate CRANE-X7 directly because the PID gains of servo motors will be small values.

Launch nodes with the following command:

```sh
roslaunch crane_x7_examples teaching_example.launch
```

Please see below for keyboard operation.

**Teaching Mode**

This is a mode at startup and the PID gains will be small values.

| Key | Function |
----|----
| s / S | **S**ave current posture |
| d / D | **D**elete all posture data |
| m / M | Transition to *Action **M**ode* |
| q / Q | **Q**uit application |


**Action Mode**

This is a mode transitioned from Teaching Mode and the PID gains will return to normal values.

| Key | Function |
----|----
| p / P | **P**layback a posture data |
| a / A | Playback **a**ll posture data consecutively |
| l / L | Toggle **l**oop playback (ON/OFF) |
| m / M | Transition to *Teaching **M**ode* |
| q / Q | **Q**uit application |


Demo Video is [here **(YouTube Video)**](https://youtu.be/--5_l1DpQ-0).

---

### joystick_example.py

This is an example to use joystick controller to change the hand position and posture, 
or to open and close of the gripper,
or to preset the PID gains 
or to generate a position trajectory with direct teaching.

Connect a joystick controller to a PC and check the device `/dev/input/js0` existence
then launch nodes with the following command:

#### for control CRANE-X7

```sh
roslaunch crane_x7_examples joystick_example.launch
```

#### for control virtual CRANE-X7

Please add an argument `sim` to avoid an error.

```sh
roslaunch crane_x7_examples joystick_example.launch sim:=true
```

#### Key configuration

This picture shows the default key configuration. The joystick controller is 
[Logicool Wireless Gamepad F710](https://support.logicool.co.jp/ja_jp/product/wireless-gamepad-f710).

![key_config](https://github.com/rt-net/crane_x7_ros/blob/images/images/joystick_example_key_config.png "key_config")

Key assignments can be edited with key numbers in [crane_x7_example/launch/joystick_example.launch](./launch/joystick_example.launch).

```xml
 <node name="joystick_example" pkg="crane_x7_examples" type="joystick_example.py" required="true" output="screen">
    <param name="button_shutdown_1" value="8" type="int" />
    <param name="button_shutdown_2" value="9" type="int" />

    <param name="button_name_enable" value="7" type="int" />
    <param name="button_name_home"  value="8" type="int" />

    <param name="button_preset_enable" value="7" type="int" />
    <param name="button_preset_no1" value="9" type="int" />
```

This picture shows the default key numbers.
![key_numbers](https://github.com/rt-net/crane_x7_ros/blob/images/images/joystick_example_key_numbers.png "key_numbers")

Please display `/joy` topic with the command `rostopic echo /joy` to check the default key numbers.

```sh
roslaunch crane_x7_examples joystick_example.launch sim:=true

# Enter the command in another terminal 
rostopic echo /joy

# Press buttons of a joystick controller
header: 
  seq: 1
  stamp: 
    secs: 1549359364
    nsecs: 214800952
  frame_id: ''
axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
buttons: [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
---
```

Demo Video is [here **(YouTube Video)**](https://youtu.be/IQci_vb3owM).

---

### obstacle_avoidance_example.py

This is an example to use `rosservice` to add dummy obstacles and to avoid the obstacles.

Launch nodes with the following command:

```sh
roslaunch crane_x7_examples obstacle_avoidance_example.launch
```

This example uses a [service file](./srv/ObstacleAvoidance.srv) 
including target posture of CRANE-X7 and size and posture of an obstacle.

These values can be edited in [`crane_x7_examples/scripts/obstacle_client.py`](./scripts/obstacle_client.py).

The default obstacle shape is rectangular.

```python
    # Define obstacle shape and pose
    obstacle_name = "box"
    obstacle_size = Vector3(0.28, 0.16, 0.14)
    obstacle_pose_stamped = PoseStamped()
    obstacle_pose_stamped.header.frame_id = "/base_link"
    obstacle_pose_stamped.pose.position.x = 0.35
    obstacle_pose_stamped.pose.position.z = obstacle_size.z/2.0
```

This example generates a dummy floor as an obstacle to move safely.

If this floor is unnecessary, please comment out some lines from [`crane_x7_examples/scripts/obstacle_avoidance_example.py`](./scripts/obstacle_avoidance_example.py).

```python
    # Generate dummy floor
    floor_name = "floor"
    floor_size = (2.0, 2.0, 0.01)
    floor_pose = PoseStamped()
    floor_pose.header.frame_id = "/base_link"
    floor_pose.pose.position.z = -floor_size[2]/2.0
    scene.add_box(floor_name, floor_pose, floor_size)
    rospy.sleep(SLEEP_TIME)
```

If MoveIt! did not generate trajectory to avoid an obstacle, 
CRANE-X7 will not move,
the example server will return value `result=False`,
then MoveIt! will calculate a trajectory to next target position.

![gazebo_obstacle_avoidance](https://github.com/rt-net/crane_x7_ros/blob/images/images/gazebo_obstacle_avoidance.gif)

---

### servo_info_example.py

This is an example to subscribe the servo motor status.

Run a node with the following command:

```sh
rosrun crane_x7_examples servo_info_example.py
```

This example subscribes topics of gripper joint `crane_x7_gripper_finger_a_joint`
and displays the servo motor current, position and temperature to a terminal.

```sh
 current [mA]: 0.0     dxl_position: 2634    temp [deg C]: 42.0  
 current [mA]: 2.69    dxl_position: 2634    temp [deg C]: 42.0  
 current [mA]: 0.0     dxl_position: 2634    temp [deg C]: 42.0  
 current [mA]: 0.0     dxl_position: 2634    temp [deg C]: 42.0  
 current [mA]: 2.69    dxl_position: 2634    temp [deg C]: 42.0
 ...
```

If the motor current exceed thresholds, the gripper will open/close.
This function enables user to open/close the gripper by hand.

Please refere [`crane_x7_control/README.md`](../crane_x7_control/README.md#ネームスペースとトピック) for details of the topics.

---

### pick_and_place_in_gazebo_example.py

This is an example to grasp, pick up, carry and place an small object
on **Gazebo** environments.

Launch nodes with the following command with arguments to control the gripper by EffortController.

```sh
roslaunch crane_x7_gazebo crane_x7_with_table.launch use_effort_gripper:=true
```

After Gazebo launch, run a node with the following command:

```sh
rosrun crane_x7_examples pick_and_place_in_gazebo_example.py
```

Demo Video is [here **(YouTube Video)**](https://youtu.be/YUSIregHHnM).

![gazebo_pick_and_place](https://github.com/rt-net/crane_x7_ros/blob/images/images/gazebo_pick_and_place.gif)

