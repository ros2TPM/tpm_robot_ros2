import struct
import array
import numpy as np

import rclpy
from rclpy.node import Node

from tpm_core_msgs.srv import MailBox
from tpm_core_msgs.msg import RobotStatus
from tpm_core_msgs.msg import AxisStatus

from enum import Enum

# this is an Enum
class Robot_Parameter(Enum):
    move_speed  = 0
    jog_dist    = 1

class Axis_Operation(Enum):
    servo_on    = 0
    servo_off   = 1
    home        = 2
    set_pos_to_offset = 3
    set_pos_to_zero = 4
    mv_to_zero  = 5
    clear_alm   = 6
    jog_pos     = 7
    jog_neg     = 8
    stop        = 9
    search_org  = 10

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
        self.client_axisOp = self.create_client(MailBox, '/op/axis_operation')
        self.client_setAxisParam = self.create_client(MailBox, '/op/set_axis_param')
        self.client_setRobotParam = self.create_client(MailBox, '/op/set_robot_param')
        
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
        
        #send a request to service.
        sendBuff = struct.pack("=bb", funcType.value, axisId)#signed char.
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = self.client_axisOp.call_async(req)
        #future = self.client.call_async(req)
        #rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        #return future.result()    
        pass
    
    def set_robot_parameter(self, paramType, value):
        if not self.client_setRobotParam.wait_for_service(timeout_sec=1.0):
            print('"client_setRobotParam" service not available.')
            return
        
        sendBuff = struct.pack("=bd", paramType.value, value)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = self.client_setRobotParam.call(req)

    def set_axis_parameter(self, axisId, paramType, value):
        if not self.client_setAxisParam.wait_for_service(timeout_sec=1.0):
            print('"axis_setParam" service not available.')
            return
        
        sendBuff = struct.pack("=bb", paramType.value, axisId)

        if paramType == Axis_Parameter.home_offset:
            sendBuff += struct.pack("=d", value)

        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = self.client_setAxisParam.call(req)

    #endregion

    
