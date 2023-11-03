# What is this repository?
This Git repository demonstrates how to use **RPX-L132D1-ROS2**.   

**RPX-L132D1-ROS2** is a robot controller that is composed of a `Raspberry pi 4B` and a `HAT board` named **‘HAT-L132D1’**. 

This **HAT-L132D1** HAT board is designed by [TPM (Taiwan Pulse Motion)][tpm] company. It can calculate robot kinematics and interpolation, than send the signal to drivers through field bus called [‘Motionnet’][motionnet]. It also provides an API library called **’RpiMNet’** (libRpiMNet.so) in the style of C function.

The HAT board is connected to a Raspberry pi 4B module, which has Ubuntu and ROS2 installed. Together they are called **‘RPX-L132D1-ROS2’**.

Note: [ROS2][ros2] (Robot Operating System) is a set of open-source software libraries and tools for building robot applications.

![RPX-L132D1-ROS2](Image/RPX-L132D1-ROS2.png)

This Git repository demostrates how to use **RPX-L132D1-ROS2** controller by wrapping the RPiMNet API library into a ROS2 node, along with CAD modules for several types of robots, a simple GUI, and some sample client codes.

# Architecture
The overall robot controlling architecture can be divided into four parts:
1. **Robot Mechanics:**  
    The mechanical parts of robot. Current supported robots are:
    + [scara robot (4dof)][igus_4dof] by igus
    + [delta robot (3dof)][igus_3dof] by igus
    + [robotlink arm (5dof)][igus_5dof] by igus
    + [AR3 arm (6dof)][anninrobotics] by Annin Robotics

2. **Drivers:**  
   [TPM SVR-M1xx Driver series][svr-M1xx] are closed-loop step motors that use ['Motionnet'][motionnet] as field bus communication.
     
4. **Controller:**  
   The controller is RPX-L132D1-ROS2, as described above.  
   An API library is provided as dll file.

5. **User application (this repository):**  
   Wraps the RPiMNet API library into ROS2 nodes, and demostrate how to use it along with ROS2 built-in features such as Rviz or Moveit!.

![RPX-L132D1-ROS2](Image/Architecture%20of%20automation%20controller.png)

# Prerequisite
This guide assumes that the reader has experience with Linux and ROS2 project.  
The code is tested under following environment:
+ Ubuntu 22.04  
    -- on Windows VM Virtual Box  
    -- on Raspberry Pi 4B  
+ ROS2 Humble

By simply clone and build this repository, you can run in `simulation mode`.  
However, in order to run in `real mode` (i.e. control real-physical robot), you’ll need:
+ RPX-L132D1-ROS2 controller
+ Motionnet Drivers
+ Motors
+ Robot mechenics
  
If you are interested, please [contact TPM][contactTPM] or mail to: ros2TPM@tpm-pac.com

# Getting Started
You can find more instructions in the following documents:
- [Build, and Run Simulation](<Doc/[Getting Start] Build and Run simulation.md>)
- [Run on Real Robot](<Doc/[Demo] igus Delta Robot.pdf>)
- ~~[Use with MoveIt! library](Doc/)~~

# Folder overview
Here is a brief overview of each component within this repository. For more detiled information, navigate into individual folders. 

- **tpm_core_node**:  
  The ROS2 node that warps the RPiMNet API library. It provides ROS2-style Topics, Services, and Actions that can execute commands from users and monitor the statuses of drivers.
- **tpm_msgs**:  
  Contains the ROS2 message and service definitions. Used in `tpm_core_node`.
- **tpm_description**:  
  Contains CAD files and configuration files of several robot types. Used for Rviz 3D simulation, which is a build-in tool of ROS2.
- **tpm_sample_code**:  
  Provides C++ and python sample client codes. And most importantly, a sample-GUI called 'MyRosRobot' for users to call basic services from 'tpm_core_node', such as ServoOn/Off, Homing, Jogging, and monitor the driver position..
- **tpm_moveit**:  
  [Moveit!][moveit] is a famous open-source robot framework based on ROS.
  Here contains configuration files and sample codes that demostrate how to cooperate with Moveit!.
- **ExtraLib**:  
  Contains the header (.h) files and dll (.so) files of the RPiMNet library.
- **Doc**:  
  Contains instructions and manuals.
- **Image**:  
  Contains images for Readme.md files.



[tpm]: https://www.tpm-pac.com/
[contactTPM]: https://www.tpm-pac.com/contact-us/
[motionnet]: http://www.motionnet.jp/en/motionnet.html
[ros2]: https://docs.ros.org/en/humble/Tutorials.html
[igus_3dof]: https://www.igus.com.tw/product/20433?artNr=DLE-DR-0005
[igus_4dof]: https://www.igus.com.tw/product/20961?artNr=RL-SCR-0100
[igus_5dof]: https://www.igus.com.tw/product/20239?artNr=RL-DP-5
[anninrobotics]: https://www.anninrobotics.com/
[svr-M1xx]: https://www.tpm-pac.com/product-2/motionnet-3/nu-servo-drive-m/closed-loop-m-nu/
[moveit]: https://moveit.picknik.ai/main/index.html
