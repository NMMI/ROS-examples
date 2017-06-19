![qbrobotics logo](http://www.qbrobotics.com/wp-content/themes/qbrobotics/img/logo.svg)

This repository contains just few examples and tutorials of how to properly set up several _qbrobotics®_ devices to work together on ROS related applications.

Please, refer to the following wikis respectively if you are dealing with a _qbhand_, a _qbmove_, or even both:
- [_qbhand_](http://wiki.ros.org/Robots/qbhand)
- [_qbmove_](http://wiki.ros.org/Robots/qbmove)

## Table of Contents
1. [Installation](#markdown-header-installation)
1. [Usage](#markdown-header-usage)
1. [ROS Packages Overview](#markdown-header-ros-packages-overview)
1. [Support, Bugs and Contribution](#markdown-header-support)
1. [Purchase](#markdown-header-purchase)

## Installation
>If you have not yet installed the ROS related packages for either your [_qbhand_](http://wiki.ros.org/Robots/qbhand)s or [_qbmove_](http://wiki.ros.org/Robots/qbmove)s (or even both if you need to use them together), please follow their wikis first.

To install also this packages, simply clone `git@bitbucket.org:qbrobotics/qbchain-ros.git` in your Catkin Workspace as you have done for the others. Note that there is no need to recompile since this repository contains only example configurations based on the other packages.

Actually the installation is not required. Nonetheless it is recommended to follow the proposed approach for your customizations to keep everything tidy and structured.

## Usage
>If it is the first time you are dealing with _qbrobotics®_ devices, or you have only worked with just one of them per time, please consider to spend five minutes on our examples before creating your own configuration. Otherwise you can skip to [Customization](#markdown-header-customization).

Each system configuration has a unique name. And that name is used as prefix for all the configuration files:
- `qb_chain_description/rviz/<unique_name>.rviz`: the [rviz](http://wiki.ros.org/rviz) configuration file.
- `qb_chain_description/urdf/<unique_name>.urdf.xacro`: the [urdf](http://wiki.ros.org/urdf)/[xacro](http://wiki.ros.org/xacro) model describing the physical chained system by connecting together simpler _qbhand_ or _qbmove_ modules (and possibly other parts).
- `qb_chain_control/config/<unique_name>_controllers.yaml`: the controllers related to all the _qbrobotics®_ devices of the chain.
- `qb_chain_control/config/<unique_name>_waypoints.yaml`: the waypoints that the chained system must follow in the Waypoint Control mode.
- `qb_chain_control/launch/<unique_name>_gui_control.launch`: the launch file to start the GUI Control mode for the given devices.
- `qb_chain_control/launch/<unique_name>_waypoint_control.launch`: the launch file to start the Waypoint Control mode for the given devices.

After you have looked at the proposed examples (at least the number and type of devices - and their IDs - must match your real setup), you can launch a given configuration by executing:
```
roslaunch qbchain <unique_name>_gui_control.launch
```

In such a case, several control nodes like the ones described in the single device wikis (cf. [_qbhand_](http://wiki.ros.org/Robots/qbhand) or [_qbmove_](http://wiki.ros.org/Robots/qbmove)) start, and let you control (e.g. through a GUI) all the connected devices together.

>If your real configuration does not resemble any of the given examples, read carefully the following instructions and create your own customization. Nonetheless it is recommended to test at least a subsystem chain with one of the proposed examples: if anything seems not to work as expected, feel free to ask for support on [our Bitbucket](https://bitbucket.org/account/user/qbrobotics/projects/ROS).

### Customization
1. Choose a name for your robot, e.g. `my_chain`.

1. The very first thing to set up is the chained system description, i.e. the [urdf](http://wiki.ros.org/urdf)/[xacro](http://wiki.ros.org/xacro) model exploiting single module descriptions, and possibly additional parts when required.

   Adding a _qbhand_ or a _qbmove_ is basically a matter of copy-pasting a couple of xml lines and set their values. This is possible thanks to the macros contained in `qb_hand.utils.xacro` and `qb_move.utils.xacro`, which need to be included.

   For example, if you have a robotic arm made of *qbmove*s, you can connect the device aimed to reproduce the longitudinal rotation of the wrist (the _qbmove_ called `wrist_roll`) to the one, i.e. to the shaft of the one, assigned to the elbow flexion (called `elbow_pitch`), by specifying only their relative position (in our example, 30 cm far along the x-axis with same orientation):
   ```xml
   <xacro:include filename="$(find qb_move_description)/urdf/qb_move.utils.xacro"/>

   <link name="root_link"/>
   ...
   <xacro:build_move_from_default_yaml namespace="wrist_roll" parent="elbow_pitch_shaft">
     <origin xyz="0.3 0 0" rpy="0 0 0"/>
   </xacro:build_move_from_default_yaml>
   ...
   ```

   And so on for all the other _qbrobotics®_ devices.

   >Note that to resemble your real configuration with more accuracy, additional meshes can be added to the model by specifying links fixed to their related _qbmove_. In such a case, do not forget to properly set their inertial properties.

   This file has to be saved in `qb_chain_description/urdf/my_chain.urdf.xacro`. You can skip the creation of a custom `qb_chain_description/rviz/my_chain.rviz`, but this will be set anyway when you save its configuration from within the GUI.

1. Fill the `qb_chain_control/config/my_chain_controllers.yaml` properly. This can be vague, but it is nothing more than a copy-paste of the default controller settings with the current device names specified in the description.

   Following the above example, this can be a valid configuration for the _qbmove_ called `wrist_roll`:
   ```
   wrist_roll_joint_state_controller:
     type: joint_state_controller/JointStateController
     publish_rate: 100
   
   # control the qbmove with shaft reference position and stiffness preset
   wrist_roll_position_and_preset_trajectory_controller:
     type: position_controllers/JointTrajectoryController
     joints:
       - wrist_roll_shaft_joint
       - wrist_roll_stiffness_preset_virtual_joint
     constraints:
       goal_time: 0.2
       stopped_velocity_tolerance: 0.1
       wrist_roll_shaft_joint:
         trajectory: 0.1
         goal: 0.05
       wrist_roll_stiffness_preset_virtual_joint:
         trajectory: 0.1
         goal: 0.05
     state_publish_rate: 100
     action_monitor_rate: 120
     stop_trajectory_duration: 0
   
   # control the qbmove with motor positions
   wrist_roll_motor_positions_trajectory_controller:
     type: position_controllers/JointTrajectoryController
     joints:
       - wrist_roll_motor_1_joint
       - wrist_roll_motor_2_joint
     constraints:
       goal_time: 0.2
       stopped_velocity_tolerance: 0.1
       wrist_roll_motor_1_joint:
         trajectory: 0.1
         goal: 0.05
       wrist_roll_motor_2_joint:
         trajectory: 0.1
         goal: 0.05
     state_publish_rate: 100
     action_monitor_rate: 120
     stop_trajectory_duration: 0
   ```
   
   For the other _qbmove_ you need to replace the prefix `wrist_roll` from all the occurrence with `elbow_pitch`, and that's it. The prefix must reflect the device name specified in the description model.
   
   >Note that to avoid name clashes, even joint names depends on the device name.
   
   Similar considerations can be made for a _qbhand_. Please, refer to the default controller in its control package, i.e. `qb_hand_contrtol`.

1. **[optional]** If required for simple applications or testing, a waypoint configuration file can be created in `qb_chain_control/config/my_chain_waypoints.yaml`. This file has the same structures of the ones of _qbhand_ and _qbmove_ but allows to specify several namespaces, each related to a single device.

   Once again, following the same example:
   ```
   # Waypoints describe the desired motion trajectory:
   #  - time [s]: can be either a single value or an interval for which all the specified joint_positions hold
   #  - joint_positions: a list of namespaces which contain the vector of joint position references of the relative device
   #                     (the measurement units depend on the device type, e.g. qbhand needs a single [0,1] value).
   #                     If a namespace is not specified for a given waypoint, it is assumed that it does not change the
   #                     device position, e.g. if the end-effector pose remains unchanged for duration of the grasp, the
   #                     user can avoid copy-pasting it for the involved waypoints. Nonetheless for critical applications
   #                     it is recommended to specify it to avoid interpolation approximation, e.g. following the above
   #                     example, the arm could start moving before the grasp has ended if the end-effector pose is not
   #                     specified for all the waypoints.
   
   waypoints:
     -
       time: [1.0]
       joint_positions:
         ...
         elbow_pitch: [0.0, 0.5]
         wrist_roll: [0.0, 0.5]
         ...
     -
       time: [2.0, 2.5]
       joint_positions:
         ...
         elbow_pitch: [1.0, 0.5]
         wrist_roll: [0.5, 0.5]
         ...
     -
       ...
   ```

1. Create GUI or Waypoint Control based launch files, respectively `qb_chain_control/launch/my_chain_gui_control.launch` and `qb_chain_control/launch/my_chain_waypoint_control.launch`.

   While the control mode selection is just a matter of ROS parameters, the launch files have to include the `*_bringup.launch` templates provided by `qb_device_description`, to properly load what needs. A simple example for the Waypoint Control is as follows:
   ```xml
   <launch>
     <include file="$(find qb_device_driver)/launch/communication_handler.launch"/>
   
     <!-- robot namspace -->
     <group ns="my_chain">

       ...

       <include file="$(find qb_device_bringup)/launch/component_bringup.launch">
         <arg name="control_action" value="position_and_preset"/>
         <arg name="control_duration" value="0.001"/>
         <arg name="controllers" value="wrist_roll_joint_state_controller
                                        wrist_roll_position_and_preset_trajectory_controller
                                        wrist_roll_motor_positions_trajectory_controller"/>
         <arg name="controllers_namespace" value="my_chain"/>
         <arg name="controllers_settings" value="qb_chain"/>
         <arg name="device_id" value="1"/>
         <arg name="device_control" value="qb_move"/>
         <arg name="device_description" value="qb_move"/>
         <arg name="device_namespace" value="wrist_roll"/>
         <arg name="device_urdf" value="qb_move"/>
         <arg name="use_controller_gui" value="false"/>
         <arg name="use_waypoints" value="true"/>
         <arg name="waypoint_namespace" value="my_chain"/>
         <arg name="waypoint_settings" value="qb_chain"/>
       </include>

       <include file="$(find qb_device_bringup)/launch/component_bringup.launch">
         <arg name="control_action" value="position_and_preset"/>
         <arg name="control_duration" value="0.001"/>
         <arg name="controllers" value="elbow_pitch_joint_state_controller
                                        elbow_pitch_position_and_preset_trajectory_controller
                                        elbow_pitch_motor_positions_trajectory_controller"/>
         <arg name="controllers_namespace" value="my_chain"/>
         <arg name="controllers_settings" value="qb_chain"/>
         <arg name="device_id" value="2"/>
         <arg name="device_control" value="qb_move"/>
         <arg name="device_description" value="qb_move"/>
         <arg name="device_namespace" value="elbow_pitch"/>
         <arg name="device_urdf" value="qb_move"/>
         <arg name="use_controller_gui" value="false"/>
         <arg name="use_waypoints" value="true"/>
         <arg name="waypoint_namespace" value="my_chain"/>
         <arg name="waypoint_settings" value="qb_chain"/>
       </include>
       
       ...

       <include file="$(find qb_device_bringup)/launch/description_bringup.launch">
         <arg name="device_urdf" value="my_chain"/>
         <arg name="device_description" value="qb_chain"/>
         <arg name="source_list" value="[wrist_roll/joint_states, elbow_pitch/joint_states, ...]"/>
         <arg name="use_rviz" value="true"/>
       </include>
     </group>
   </launch>
   ```

1. And finally you are ready to start you custom ROS nodes by executing:
   ```
   roslaunch qbchain my_chain_waypoint_control.launch
   ```

   >At this point you have the same configuration of a single _qbhand_ or _qbmove_ control node (and almost the same considerations are true), but you are controlling several _qbrobotics®_ devices together!

### Advanced 
>If you need something more sophisticated than a simple predefined waypoint guidance, you can extend the API similarly to the single device cases (cf. [_qbhand_](http://wiki.ros.org/Robots/qbhand) or [_qbmove_](http://wiki.ros.org/Robots/qbmove) for details).

## ROS Packages Overview
| |Packages|
|---:|---|
|[qb_chain](http://wiki.ros.org/qb_chain): |[qb_chain_control](http://wiki.ros.org/qb_chain_control), [qb_chain_description](http://wiki.ros.org/qb_chain_description)|

## Support, Bugs and Contribution
Since we are not only focused on this project it might happen that you encounter some trouble once in a while. Maybe we have just forget to think about your specific use case or we have not seen a terrible bug inside our code. In such a case, we are really sorry for the inconvenience and we will provide any support you need.

To help you in the best way we can, we are asking you to do the most suitable of the following steps:

1. It is the first time you are holding a _qbrobotics®_ device, or the first time you are using ROS, or even both: it is always a pleasure for us to solve your problems, but please consider first to read again the instructions above and the ROS tutorials. If you have ROS related questions the right place to ask is [ROS Answers](http://answers.ros.org/questions/).
1. You are a beginner user stuck on something you completely don't know how to solve or you are experiencing unexpected behaviour: feel free to contact us at [support+ros at qbrobotics.com](support+ros@qbrobotics.com), you will receive the specific support you need as fast as we can handle it.
1. You are quite an expert user, everything has always worked fine, but now you have founded something strange and you don't know how to fix it: we will be glad if you open an Issue in the package of interest on [our Bitbucket](https://bitbucket.org/account/user/qbrobotics/projects/ROS).
1. You are definitely an expert user, you have found a bug in our code and you have also correct it: it will be amazing if you open a Pull Request in the package of interest on [our Bitbucket](https://bitbucket.org/account/user/qbrobotics/projects/ROS); we will merge it as soon as possible.
1. You are comfortable with _qbrobotics®_ products but you are wondering whether is possible to add some additional software features: feel free to open respectively an Issue or a Pull Request in the package of interest on [our Bitbucket](https://bitbucket.org/account/user/qbrobotics/projects/ROS), according to whether it is just an idea or you have already provided your solution.

In any case, thank you for using [_qbrobotics®_](http://www.qbrobotics.com) solutions.

## Purchase
If you have just found out our company and you are interested in our products, come to [visit us](http://www.qbrobotics.com) and feel free to ask for a quote.