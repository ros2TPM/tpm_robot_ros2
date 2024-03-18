# tpm_core_node
This package describes a `ROS2 Node`, which is the core of this Git repository.  
* This node provides several ROS2-style interfaces (**Topics**, **Services**, and **Actions**) for user to control the robot;  
* This node also implements the communication to the `RPX-L132 hardware` by calling APIs in `libRPiMnet.so` library. 
* This README file will take a overview of this package.

### Table of Content  
* [Entry point](#entry-point)  
* [ROS2 Interface definition](#ros2-interface-definition)  
    * [Parameters](#parameters)
    * [Topics](#topics)
    * [Services](#services)
    * [Action](#actions)

# Entry point
The entry point is the `Main()` function of `\src\tpm_core_node.cpp` file.
```C++
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto myNode = std::make_shared<TpmCoreNode>();
  myNode->init();
  
  ROS_PRINT("==== Initialize complete ! ready to spin...");
  rclcpp::spin(myNode);
  rclcpp::shutdown();
  return 0;
}
```
The Main function creates a node instance of class `TpmCoreNode`, initialize it, and spin it up. This node will keep alive until user shut it down.

The `TpmCoreNode` class is also defined in the same file. Its public method `init()` does the following:  

1. Load ***Parameters*** from `\config\<robot name>.yaml` file, where `<robot name>` should be provided by user when launching this node.  

2. Create an instance of class `HwLib`, which is responsible of communicating with `RPX-L132 hardware`.  

3. By calling `hwLib.connect()` and `hwLib.init()`, it connects to the hardware, and initialize the system based on the number of motors detected.  

4. Create ***Topics***, ***Services***, and ***Actions***.

> 【Note】 
> the detailed implementation of ***Parameters***, ***Topics***, ***Services***, and ***Actions*** are defined in `\config\src\ros_interface\manager_xxx.cpp` files.

  
# Ros2 Interface definition

> 【Note】 Definition of units  
> - For length parameters, the unit is always **mm**.  
> - For rotational parameters, the unit is always **degree**.   
> - For all vel and acc, the unit is always **unit/sec** and **unit/sec^2**

## Parameters
The parameter values are defined in `\config\<robot name>.yaml` files.  

Modifing the parameters will change the behavior of in corresponding robot. Please be careful when doing this.

> 【Note】If a Parameter is not specified in file, it will be set to zero during initialization. 

There are three catagories of Parameters:
1. ***Robot Parameters***  
(These paratemers describe the robot as a whole. They do not belong to any single joint)
    - **robot_type** : the code indicates the robot configuration(such as 6 axis, SCARA, Delta, etc).  

    - **a / alpha / d / theta** : for serial robots, this is the DH table; For parallel robots, this is the place holder of mechanical dimensions.  

    - **max_xyz_jog_speed** : the maximum jog speed of robot end-pose in X/Y/Z translation

    - **max_abc_jog_speed** : the maximum jog speed of robot end-pose in A/B/C rotation.

2. ***Joint Parameters***   
(These parameters are arrays where each value corresponds to one joint.)
    - **theta_shift**: the difference between the zero-position of conventional usage and the zero-position defined by DH table.  

    - **pos/neg_limit** : the maximum/minimum value that each robot joint can reach.  

    - **pulse_per_unit** : the ratio between robot-unit and driver pulses.  
    Example: let value=100 for joint 1. When joint 1 moves one unit, the correspond driver will move 100 pulses.  

    - **max_jog_speed** : the maximum jog speed of each joint.

    - **home_dir** : the moving direction to search ORG during homing. Value is either 0 or 1.  

    - **home_offsets** : the expect position of that joint when reach the ORG sensor.

3. ***MotionNet parameters***:  
(These parameters define the behavior of MotionNet communication. They are only used in real-physical mode.)
    - **baudrate** : the baud rate of communication.  
    0 : 2.5 MHz  
    1 : 5  MHz  
    2 : 10 MHz  
    3 : 20 MHz (recommend)  

    - **out_mode** : The moving direction of motor.  
    If the direction is opposite, simply try another value.  
    0: falling-high  
    1: rising-high  

    - **ipt_mode** :  The type of input driver signal  
    0 : AB_x1  
    1 : AB_x2  
    2 : AB_x4  (recommend)  
    3 : CW_CCW

    - **alm/org_logic** :  The logic of Alarm/Org sensor.  
    If the sensor's On/Off behavior is opposite, simply try another value.  
    0 : low active  
    1 : high active  
    
    - **feedback_src** : the source of feedback position read from driver.  
    0 : external - read from encoder (recommanded)  
    1 : internal - the idea value that driver sents to motor 

    - **home_mode** : the mode of different homing strategy.  
    The recommended mode is 0, which simply search for ORG sensor then stop. For other modes, please check TPM-MotionNet driver. 




## Topics
> 【Note】
> * all Topics are updated every 500 ms.  
> * the messages are defined in `/tpm_msgs/msg/` folder.  


- **joint_states**: the status of each joint.  
This is a ROS2 built-in message type. see [doc](https://docs.ros.org/en/api/sensor_msgs/html/msg/JointState.html).  

  * string[] **name**  
  * float64[] **position**  
  * float64[] **velocity** (not implemented)  
  * float64[] **effort** (not implemented) 

- **robot_status**: the status of robot.   
This message is defined in `/tpm_msgs/msg/RobotStatus.msg`
  * AxisStatus[6] **axes** : the physical status of each joint. The definition of `AxisStatus` can be found in the same directory.  

  * float32[6] **end_pose** : x/y/z/a/b/c  

  * uint32 **buffer_depth** : the number of commands in buffer.


## Services
> 【Note】the services are defined in `/tpm_msgs/srv/` folder.  

- **axis_operation**:  
let the target axis do the specific function  
[Input]:  
    * \<int8\> **axis_id** : The target axis ID, start from 0.  
    Set to -1 means apply to all axes. 
    * \<int8\> **function** : A enum indicates which function to operate. Possible values are:  
      1: *SERVO_ON*  
      2: *SERVO_OFF*  
      3: *HOME*  --- start a complete homing process. It is a combination of several steps: SEARCH_ORG -> SET_AS_OFFSET -> MV_TO_ZERO.    

      4: *SET_AS_OFFSET* --- set current position as the `home_offsets` value defined in [Parameters](#parameters)  

      5: *SET_AS_ZERO* --- set current position as zero position  

      6: *MV_TO_ZERO* --- move to zero position  
      7: *CLEAR_ALM* --- clear driver alarm  
      8: *JOG_POS* --- move towards positive direction  
      9: *JOG_NEG* --- move towards negative direction  
      > 【Note】 Once *JOG* is called called, the robot will move until user calls `Stop` or reach `JOG_DIST` (maximum-jog-distance). See service **robot_operation** for more info.   
  
      >【Note】
      > To jog each end-pose, use service **jog_pose**  instead   

      10: *STOP* --- stop all movement    
      11: *SEARCH_ORG* --- start moving towards one direction (defined in `home_dir`) until ORG sensor is triggered.   

  [Output]: 
    * \<bool\> **ok** : the operation is success or not.  

- **robot_operation**:   
[Input]:  
 
    * \<int8\> **function** : A enum indicates witch function to operate. Possible values are:  
    1: *STOP*  --- stop all movement.       
    2: *FEEDRATE* --- set new feedrate.     
    3: *JOG_DIST*  --- set new maximum jogging distance.    

    * \<int8\> **arg1** : depends on which function you choose:  
      for *1.STOP*  --- `0` for smooth stop; `1` for rapid stop.     

      for *2.FEEDRATE* --- the new feerate. range [0~1]. The default value is `0.5`, which means 50% of `max_speed` defined in [Parameters](#parameters). 

      for *3.JOG_DIST*  --- the new maximum jog distance.  default value is `-1`, which means infinite.  

  [Output]: 
    * \<bool\> **ok** : the operation is success or not.  
    
- **rob_move**:  
  Move the robot to specified end position, which can be describe by `each joint` or by `end-effector`.  

  [Input]:  
  * \<uint8\> **function** : the moving type. Possible values are:  
    1. *PTP_AXIS* : the end position is described by `each joint`. The robot will directly move to the target joints, while the trajectory of end-effector is not guaranteed. 

    2. *LIN_POSE* : the end position is described by `end-effector`. the trajectory of end-effector will be a straight line.     

  * \<uint16\> **cmd_id** :Simply used to identify the commands. You can increase this number or simply set it to zero.  

  * \<MotionProfile\> **mp_data** : A data structure contains velocity, acceration and other settings. See `/tpm_msgs/msg/MotionProfile.msg` for more info.  

  * \<float32[]\> **axis** : An array indicates the target position of each joint. Only used if **function** is set to `PTP_AXIS`.  

  * \<float32[]\> **pose** : An array indicates the target position of end-effector in {X,Y,Z,A,B,C}. Only used if **function** is set to `LIN_POSE`.  

  * \<uint8\> **mask** : 

  [Output]:
  * \<bool\> **ok** : the operation is success or not.  

- **jog_pose**:     
  Jog the end-effector in {X,Y,Z,A,B,C} direction.  
  > 【Note】 Once called, the robot will move until user calls `Stop` or reach `JOG_DIST` (maximum-jog-distance). See service **robot_operation** for more info.   
  
  >【Note】
  > To jog each joint, use service `axis_operation` instead  

  [Input]:  
  * \<int8\> **pose_id** : the coordinate to be jogged. Possible values are {0,1,2,3,4,5}, where each value corresponds to {X,Y,Z,A,B,C} respectively.  
  Note: {A,B,C} means rotation around {X,Y,Z} axis respectively.  

  * \<int8\> **jog_dir** : the jogging direction. `1` for positive and `-1` for negative.  

  [Output]:
  * \<bool\> **ok** : the operation is success or not.


## Actions  
 
- **handle_ftj** : An action server of type `<FollowJointTrajectory>`, which is defined by the `MoveIt!` package. We implement the server side in order receive the client action invoked by `MoveIt` package.  

  In the **handle_ftj_accepted()** function, we use a PVT (position-velocity-time) function to move the robot in compliance with the data specified in `<FollowJointTrajectory>`.  

