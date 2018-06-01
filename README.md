![qbrobotics logo](https://www.qbrobotics.com/wp-content/themes/qbrobotics/img/logo.svg)

This repository contains just few examples of how to properly set up several _qbrobotics®_ devices to work together on ROS related applications.

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

To install also this packages, simply clone `https://bitbucket.org/qbrobotics/qbchain-ros.git` in your Catkin Workspace as you have done for the others. Note that there is no need to recompile since this repository contains only example configurations based on the other packages.

Actually the installation is not required. Nonetheless it is recommended to follow the proposed approach for your customizations to keep everything tidy and structured.

## Usage
Each system configuration has a unique name. And that name is used as prefix for all the configuration files:
- `qb_chain_description/rviz/<unique_name>.rviz`: the [rviz](http://wiki.ros.org/rviz) configuration file.
- `qb_chain_description/urdf/<unique_name>.urdf.xacro`: the [urdf](http://wiki.ros.org/urdf)/[xacro](http://wiki.ros.org/xacro) model describing the physical chained system by connecting together simpler _qbhand_ or _qbmove_ modules (and possibly other parts).
- `qb_chain_control/launch/<unique_name>_control.launch`: the launch file to start the control Node for the given chained system. Be aware that this must strictly match the above URDF model of your actual system, e.g. device  types, joint names, ...
- `qb_chain_control/config/<unique_name>_waypoints.yaml [optional]`: the waypoints that the chained system must follow if `use_waypoints` is set.

After you have looked at the proposed examples you can launch a given configuration by executing:
```
roslaunch qb_chain_control <unique_name>_control.launch
```

Every launch file has a structure close to the ones explained in details in [_qbhand_](http://wiki.ros.org/Robots/qbhand) and [_qbmove_](http://wiki.ros.org/Robots/qbmove) packages. The only difference is that the argument `robot_hardware` (required by the [combined_robot_hw](http://wiki.ros.org/combined_robot_hw)) plays a crucial role when using several devices together, i.e. its names must match the ones of the loaded hardware interfaces to bring up everything correctly.

On the other hand, almost all the arguments used in the single-device launch files hold even for the chained system. Have a look at the following example:

![overview_launch](qb_chain_media/overview_launch.svg)

>Please remember that in a multi-device configuration, each _qbrobotics®_ device connected to your system **must have a unique ID**.

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

In any case, thank you for using [_qbrobotics®_](https://www.qbrobotics.com) solutions.

## Purchase
If you have just found out our company and you are interested in our products, come to [visit us](https://www.qbrobotics.com) and feel free to ask for a quote.