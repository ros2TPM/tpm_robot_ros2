import struct
import array
import numpy as np

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters, GetParameters


from tpm_msgs.srv import AxisOperation
from tpm_msgs.srv import RobotOperation
from tpm_msgs.srv import JogPose
from tpm_msgs.msg import RobotStatus

from enum import Enum

# this is an Enum
class Robot_Parameter(Enum):
    move_speed  = 0
    jog_dist    = 1

class Axis_Parameter(Enum):
    home_offset = 0

#main class
class Lib(Node):
    def __init__(self):
        self.msg = RobotStatus()
        pass

    def initROS(self):
        self.uninitROS()

        rclpy.init()
        super().__init__('ros_robot_lib')

        # frequently called clients: declare as class member, instead of declare it every time when called.
        self.client_axisOp  = self.create_client(AxisOperation, '/tpm/axis_operation')
        self.client_robotOp = self.create_client(RobotOperation, '/tpm/robot_operation')
        self.client_robJog  = self.create_client(JogPose, '/rob/jog_pose')

        self.client_getParams = self.create_client(GetParameters, '/tpm_core_node/get_parameters')
        
        self.subscription = self.create_subscription(
            RobotStatus,
            "robot_status",
            self.robot_status_callback,
            1 # buffer size
        )
        self.subscription  # prevent unused variable warning

    def robot_status_callback(self, msg: RobotStatus):
        #print('robot_status_callback\n')
        #todo: 

        # method 1: keep a copy as class member
        self.msg = msg
        # method 2: invoke client UI to update
        # ???


        # self.get_logger().info('msg.axes.is_org: %d, %d, %d, %d' %
        #                         (msg.axes[0].is_org,
        #                         msg.axes[1].is_org,
        #                         msg.axes[2].is_org,
        #                         msg.axes[3].is_org))
        return
    
    def uninitROS(self):
        if hasattr(self, 'ros_robot_lib'):
            super().destroy_node()
        rclpy.try_shutdown()

    #region operation
    def axis_action(self, axisId, funcType):
        """
        Paramters:
            axisId (int): the id of axis, start from 0. Where -1 means all axis.
            funcType (an enum of 'Axis_Operation' or int) :indicates witch operation function should be called
            
        """
        print(f"axis_operation: Id={axisId}, func={funcType}")

        if not self.client_axisOp.wait_for_service(timeout_sec=1.0):
            print('"axis_operation" service not available.')
            return
        
        req = AxisOperation.Request()
        req.axis_id = axisId
        req.function = funcType

        future = self.client_axisOp.call_async(req)
        #future = self.client.call_async(req)
        #rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        #return future.result()    
        pass

    def rob_action(self, funcType, arg1):
        print(f"robot_operation: func={funcType}, arg1={arg1}")

        if not self.client_robotOp.wait_for_service(timeout_sec=1.0):
            print('"robot_operation" service not available.')
            return
        
        req = RobotOperation.Request()
        req.function = funcType
        req.arg1 = arg1

        future = self.client_robotOp.call_async(req)
        pass
    
    def jog_pose(self, poseId, dir):
        print(f"jog_pose: poseId={poseId}, dir={dir}")

        if not self.client_robJog.wait_for_service(timeout_sec=1.0):
            print('"robot_operation" service not available.')
            return
        
        req = JogPose.Request()
        req.pose_id = poseId
        req.jog_dir = dir

        future = self.client_robJog.call_async(req)
        pass

    def get_parameters(self, paramName):
        if not self.client_getParams.wait_for_service(timeout_sec=1.0):
            print('"get_parameters" service not available.')
            return

        req = GetParameters.Request()
        req.names = [paramName]

        res = self.client_getParams.call(req)
        return res.values[0]

    #endregion

    
