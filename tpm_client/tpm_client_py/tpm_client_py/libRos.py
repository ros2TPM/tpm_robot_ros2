import struct
import array
import numpy as np

import rclpy
from rclpy.node import Node

from tpm_core_msgs.srv import MailBox

class rosMnetLib(Node):
    def __init__(self):
        pass

    def initROS(self):
        self.uninitROS()

        rclpy.init()
        super().__init__('mnet_client')

        # frequently called clients: declare as class member, instead of declare it every time when called.
        self.cliGetRingSts      = self.create_client(MailBox, '/mnet/getRingStatus')
        self.cliGetRingErrCnt   = self.create_client(MailBox, '/mnet/getRingErrCnt')
        self.cliGetIO           = self.create_client(MailBox, '/mnet/getIO')
        self.cliGetIOBit        = self.create_client(MailBox, '/mnet/getIOBit')

        self.cliM1aGetIOSts     = self.create_client(MailBox, '/mnet/m1aGetIOStatus')
        self.cliM1aGetHwInfo    = self.create_client(MailBox, '/mnet/m1aGetHwInfo')
        self.cliM1aMotionDone   = self.create_client(MailBox, '/mnet/m1aMotionDone')
        self.cliM1aGetCurrSpd   = self.create_client(MailBox, '/mnet/m1aGetCurrSpd')
        self.cliM1aGetCommand   = self.create_client(MailBox, '/mnet/m1aGetCommand')
        self.cliM1aGetPosition  = self.create_client(MailBox, '/mnet/m1aGetPosition')
        self.cliM1aGetDio       = self.create_client(MailBox, '/mnet/m1aGetDio')

    def uninitROS(self):
        if hasattr(self, 'mnet_client'):
            super().destroy_node()
        rclpy.try_shutdown()

    def sendReqeust(self, future):
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        return future.result()

    def _mnet_open(self):
        self.initROS()

        client = self.create_client(MailBox, '/mnet/open')
        if not client.wait_for_service(timeout_sec=1.0):
            print('"mnet_open" service not available.')
            return -1
        
        future = client.call_async(MailBox.Request())
        res = self.sendReqeust(future)
        if res == None:
            return -1

        return res.result_code

    def _mnet_close(self):
        client = self.create_client(MailBox, '/mnet/close')
        if not client.wait_for_service(timeout_sec=1.0):
            print('"mnet_close" service not available.')
        
        future = client.call_async(MailBox.Request())
        res = self.sendReqeust(future)
        if res == None:
            return -1

        self.uninitROS()
        return res.result_code

    def _mnet_reset_ring(self, RingNo):
        client = self.create_client(MailBox, '/mnet/resetRing')
        if not client.wait_for_service(timeout_sec=1.0):
            print('"mnet_reset_ring" service not available.')

        sendBuff = struct.pack("=H", RingNo)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = client.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1

        return res.result_code

    def _mnet_start_ring(self, RingNo):
        client = self.create_client(MailBox, '/mnet/startRing')
        if not client.wait_for_service(timeout_sec=1.0):
            print('"mnet_start_ring" service not available.')
        
        sendBuff = struct.pack("=H", RingNo)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = client.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1

        return res.result_code

    def _mnet_stop_ring(self, RingNo):
        client = self.create_client(MailBox, '/mnet/stopRing')
        if not client.wait_for_service(timeout_sec=1.0):
            print('"mnet_stop_ring" service not available.')
        
        sendBuff = struct.pack("=H", RingNo)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = client.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1

        return res.result_code

    def _mnet_set_ring_config(self, RingNo, BaudRate):
        client = self.create_client(MailBox, '/mnet/setRingConfig')
        if not client.wait_for_service(timeout_sec=1.0):
            print('"mnet_set_ring_config" service not available.')
        
        sendBuff = struct.pack("=HB", RingNo, BaudRate)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = client.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1

        return res.result_code

    def _mnet_get_ring_count(self, Count):
        client = self.create_client(MailBox, '/mnet/getRingCount')
        if not client.wait_for_service(timeout_sec=1.0):
            print('"mnet_get_ring_config" service not available.')
        
        future = client.call_async(MailBox.Request())
        res = self.sendReqeust(future)
        if res == None:
            return -1

        recvBuff = res.buffer.tobytes()
        cnt = struct.unpack("=H", recvBuff[:struct.calcsize("=H")])[0]

        Count.clear()
        Count.append(cnt)
        return res.result_code

    def _mnet_get_ring_active_table(self, RingNo, DevTable):
        client = self.create_client(MailBox, '/mnet/getRingActiveTable')
        if not client.wait_for_service(timeout_sec=1.0):
            print('"mnet_get_ring_active_table" service not available.')

        sendBuff = struct.pack("=H", RingNo)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = client.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1

        recvBuff = res.buffer.tobytes()
        table0, table1 = struct.unpack("=2I", recvBuff[:struct.calcsize("=2I")])

        DevTable.clear()
        DevTable.append(table0)
        DevTable.append(table1)

        return res.result_code

    def _mnet_get_slave_type(self, RingNo, SlaveIP, SlaveType):
        client = self.create_client(MailBox, '/mnet/getSlaveType')
        if not client.wait_for_service(timeout_sec=1.0):
            print('"mnet_get_ring_active_table" service not available.')
        
        sendBuff = struct.pack("=2H", RingNo, SlaveIP)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = client.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1

        recvBuff = res.buffer.tobytes()
        slvType = struct.unpack("=B", recvBuff[:struct.calcsize("=B")])[0]

        SlaveType.clear()
        SlaveType.append(slvType)

        return res.result_code

    def _mnet_get_msg_slave_type(self, RingNo, SlaveIP, Type):
        print('_mnet_get_msg_slave_type(self, RingNo, SlaveIP, Type): is not implement.')
        return 0

    def _mnet_get_ring_status(self, RingNo, Status):
        sendBuff = struct.pack("=H", RingNo)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = self.cliGetRingSts.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1

        recvBuff = res.buffer.tobytes()
        status = struct.unpack("=H", recvBuff[:struct.calcsize("=H")])[0]

        Status.clear()
        Status.append(status)
        return res.result_code

    def _mnet_get_com_status(self, RingNo):
        client = self.create_client(MailBox, '/mnet/getComStatus')
        sendBuff = struct.pack("=HB", RingNo)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = client.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1
        
        return res.result_code

    def _mnet_get_ring_error_counter(self, RingNo, ErrCount):
        sendBuff = struct.pack("=H", RingNo)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = self.cliGetRingSts.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1

        recvBuff = res.buffer.tobytes()
        errCnt = struct.unpack("=H", recvBuff[:struct.calcsize("=H")])[0]

        ErrCount.clear()
        ErrCount.append(errCnt)
        return res.result_code

    def _mnet_reset_ring_error_counter(self, RingNo):
        client = self.create_client(MailBox, '/mnet/resetRingErrCnt')
        if not client.wait_for_service(timeout_sec=1.0):
            print('"mnet_reset_ring_error_counter" service not available.')
        
        sendBuff = struct.pack("=H", RingNo)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = client.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1

        return res.result_code

    def _mnet_get_communication_error_flag(self, RingNo, SlaveIP, IsError):
        print('_mnet_get_communication_error_flag(self, RingNo, SlaveIP, IsError): is not implement.')
        return 0

    def _mnet_reset_communication_error_flag(self, RingNo, SlaveIP):
        print('_mnet_reset_communication_error_flag(self, RingNo, SlaveIP): is not implement.')
        return 0

    def _mnet_get_communication_error_table(self, RingNo, ErrTable):
        print('_mnet_get_communication_error_table(self, RingNo, ErrTable): is not implement.')
        return 0

    def _mnet_reset_communication_error_table(self, RingNo):
        print('_mnet_reset_communication_error_table(self, RingNo): is not implement.')
        return 0

    def _mnet_set_io_error_check(self, RingNo, Enabled):
        print('_mnet_set_io_error_check(self, RingNo, Enabled): is not implement.')
        return 0

    def _mnet_io_input(self, RingNo, SlaveIP, PortNo):
        sendBuff = struct.pack("=2HB", RingNo, SlaveIP, PortNo)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = self.cliGetIO.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1
        
        return res.result_code

    def _mnet_io_output(self, RingNo, SlaveIP, PortNo, Val):
        client = self.create_client(MailBox, '/mnet/setIO')
        if not client.wait_for_service(timeout_sec=1.0):
            print('"mnet_io_output" service not available.')

        sendBuff = struct.pack("=2H2B", RingNo, SlaveIP, PortNo, Val)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = client.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1
        
        return res.result_code

    def _mnet_bit_io_input(self, RingNo, SlaveIP, PortNo, BitNo, OnOff):
        sendBuff = struct.pack("=2H2B", RingNo, SlaveIP, PortNo, BitNo)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = self.cliGetIOBit.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1

        recvBuff = res.buffer.tobytes()
        on_off = struct.unpack("=B", recvBuff[:struct.calcsize("=B")])[0]

        OnOff.clear()
        OnOff.append(on_off)
        return res.result_code

    def _mnet_bit_io_output(self, RingNo, SlaveIP, PortNo, BitNo, OnOff):
        client = self.create_client(MailBox, '/mnet/setIOBit')
        sendBuff = struct.pack("=2H3B", RingNo, SlaveIP, PortNo, BitNo, OnOff)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = client.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1
        
        return res.result_code

    def _mnet_bit_io_toggle(self, RingNo, SlaveIP, PortNo, BitNo):
        print('_mnet_bit_io_toggle(self, RingNo, SlaveIP, PortNo, BitNo): is not implement.')
        return 0

    def _mnet_a222_initial(self, RingNo, SlaveIP):
        print('_mnet_a222_initial(self, RingNo, SlaveIP): is not implement.')
        return 0

    def _mnet_a222_get_fpga_version(self, RingNo, SlaveIP, Ver):
        print('_mnet_a222_get_fpga_version(self, RingNo, SlaveIP, Ver): is not implement.')
        return 0

    def _mnet_a222_get_adc_version(self, RingNo, SlaveIP, Ver):
        print('_mnet_a222_get_adc_version(self, RingNo, SlaveIP, Ver): is not implement.')
        return 0

    def _mnet_a222_get_adc_error(self, RingNo, SlaveIP, Error):
        print('_mnet_a222_get_adc_error(self, RingNo, SlaveIP, Error): is not implement.')
        return 0

    def _mnet_a222_reset_adc(self, RingNo, SlaveIP):
        print('_mnet_a222_reset_adc(self, RingNo, SlaveIP): is not implement.')
        return 0

    def _mnet_a222_start_analog_input(self, RingNo, SlaveIP):
        print('_mnet_a222_start_analog_input(self, RingNo, SlaveIP): is not implement.')
        return 0

    def _mnet_a222_stop_analog_input(self, RingNo, SlaveIP):
        print('_mnet_a222_stop_analog_input(self, RingNo, SlaveIP): is not implement.')
        return 0

    def _mnet_a222_set_analog_input_enable(self, RingNo, SlaveIP, Enabled):
        print('_mnet_a222_set_analog_input_enable(self, RingNo, SlaveIP, Enabled): is not implement.')
        return 0

    def _mnet_a222_set_analog_input_type(self, RingNo, SlaveIP, Type):
        print('_mnet_a222_set_analog_input_type(self, RingNo, SlaveIP, Type): is not implement.')
        return 0

    def _mnet_a222_set_analog_input_gain(self, RingNo, SlaveIP, Gain):
        print('_mnet_a222_set_analog_input_gain(self, RingNo, SlaveIP, Gain): is not implement.')
        return 0

    def _mnet_a222_get_analog_input(self, RingNo, SlaveIP, ChannelNo, Data):
        print('_mnet_a222_get_analog_input(self, RingNo, SlaveIP, ChannelNo, Data): is not implement.')
        return 0

    def _mnet_a222_get_analog_input_ex(self, RingNo, SlaveIP, ChannelNo, Data, Val):
        print('_mnet_a222_get_analog_input_ex(self, RingNo, SlaveIP, ChannelNo, Data, Val): is not implement.')
        return 0

    def _mnet_a222_get_analog_input_all(self, RingNo, SlaveIP, Data):
        print('_mnet_a222_get_analog_input_all(self, RingNo, SlaveIP, Data): is not implement.')
        return 0

    def _mnet_a222_get_analog_input_all_ex(self, RingNo, SlaveIP, Data, Val):
        print('_mnet_a222_get_analog_input_all_ex(self, RingNo, SlaveIP, Data, Val): is not implement.')
        return 0

    def _mnet_a222_get_analog_input_value(self, RingNo, SlaveIP, ChannelNo, Val):
        print('_mnet_a222_get_analog_input_value(self, RingNo, SlaveIP, ChannelNo, Val): is not implement.')
        return 0

    def _mnet_a222_get_analog_input_value_all(self, RingNo, SlaveIP, Val):
        print('_mnet_a222_get_analog_input_value_all(self, RingNo, SlaveIP, Val): is not implement.')
        return 0

    def _mnet_a222_set_analog_output(self, RingNo, SlaveIP, ChannelNo, Volt):
        print('_mnet_a222_set_analog_output(self, RingNo, SlaveIP, ChannelNo, Volt): is not implement.')
        return 0

    def _mnet_a222_set_analog_output_all(self, RingNo, SlaveIP, Volt):
        print('_mnet_a222_set_analog_output_all(self, RingNo, SlaveIP, Volt): is not implement.')
        return 0

    def _mnet_a222_set_analog_output_value(self, RingNo, SlaveIP, ChannelNo, Val):
        print('_mnet_a222_set_analog_output_value(self, RingNo, SlaveIP, ChannelNo, Val): is not implement.')
        return 0

    def _mnet_a222_set_analog_output_value_all(self, RingNo, SlaveIP, Val):
        print('_mnet_a222_set_analog_output_value_all(self, RingNo, SlaveIP, Val): is not implement.')
        return 0

    def _mnet_a322_initial(self, RingNo, SlaveIP):
        print('_mnet_a322_initial(self, RingNo, SlaveIP): is not implement.')
        return 0

    def _mnet_a322_get_fpga_version(self, RingNo, SlaveIP, Ver):
        print('_mnet_a322_get_fpga_version(self, RingNo, SlaveIP, Ver): is not implement.')
        return 0

    def _mnet_a322_get_fw_version(self, RingNo, SlaveIP, Ver):
        print('_mnet_a322_get_fw_version(self, RingNo, SlaveIP, Ver): is not implement.')
        return 0

    def _mnet_a322_start_analog_input(self, RingNo, SlaveIP):
        print('_mnet_a322_start_analog_input(self, RingNo, SlaveIP): is not implement.')
        return 0

    def _mnet_a322_stop_analog_input(self, RingNo, SlaveIP):
        print('_mnet_a322_stop_analog_input(self, RingNo, SlaveIP): is not implement.')
        return 0

    def _mnet_a322_set_analog_input_enable(self, RingNo, SlaveIP, EnabledChannels):
        print('_mnet_a322_set_analog_input_enable(self, RingNo, SlaveIP, EnabledChannels): is not implement.')
        return 0

    def _mnet_a322_get_analog_input_enable(self, RingNo, SlaveIP, EnabledChannels):
        print('_mnet_a322_get_analog_input_enable(self, RingNo, SlaveIP, EnabledChannels): is not implement.')
        return 0

    def _mnet_a322_set_analog_input_range(self, RingNo, SlaveIP, ChannelNo, Range):
        print('_mnet_a322_set_analog_input_range(self, RingNo, SlaveIP, ChannelNo, Range): is not implement.')
        return 0

    def _mnet_a322_get_analog_input_range(self, RingNo, SlaveIP, ChannelNo, Range):
        print('_mnet_a322_get_analog_input_range(self, RingNo, SlaveIP, ChannelNo, Range): is not implement.')
        return 0

    def _mnet_a322_set_analog_input_average_cycles(self, RingNo, SlaveIP, Cycles):
        print('_mnet_a322_set_analog_input_average_cycles(self, RingNo, SlaveIP, Cycles): is not implement.')
        return 0

    def _mnet_a322_get_analog_input_average_cycles(self, RingNo, SlaveIP, Cycles):
        print('_mnet_a322_get_analog_input_average_cycles(self, RingNo, SlaveIP, Cycles): is not implement.')
        return 0

    def _mnet_a322_get_analog_input_value(self, RingNo, SlaveIP, ChannelNo, Val):
        print('_mnet_a322_get_analog_input_value(self, RingNo, SlaveIP, ChannelNo, Val): is not implement.')
        return 0

    def _mnet_a322_get_analog_input_value_all(self, RingNo, SlaveIP, Val):
        print('_mnet_a322_get_analog_input_value_all(self, RingNo, SlaveIP, Val): is not implement.')
        return 0

    def _mnet_a322_get_analog_input(self, RingNo, SlaveIP, ChannelNo, Volt):
        print('_mnet_a322_get_analog_input(self, RingNo, SlaveIP, ChannelNo, Volt): is not implement.')
        return 0

    def _mnet_a322_get_analog_input_ex(self, RingNo, SlaveIP, ChannelNo, Volt, Val):
        print('_mnet_a322_get_analog_input_ex(self, RingNo, SlaveIP, ChannelNo, Volt, Val): is not implement.')
        return 0

    def _mnet_a322_get_analog_input_all(self, RingNo, SlaveIP, Volt):
        print('_mnet_a322_get_analog_input_all(self, RingNo, SlaveIP, Volt): is not implement.')
        return 0

    def _mnet_a322_get_analog_input_all_ex(self, RingNo, SlaveIP, Volt, Val):
        print('_mnet_a322_get_analog_input_all_ex(self, RingNo, SlaveIP, Volt, Val): is not implement.')
        return 0

    def _mnet_a322_set_analog_output_range(self, RingNo, SlaveIP, ChannelNo, Range):
        print('_mnet_a322_set_analog_output_range(self, RingNo, SlaveIP, ChannelNo, Range): is not implement.')
        return 0

    def _mnet_a322_get_analog_output_range(self, RingNo, SlaveIP, ChannelNo, Range):
        print('_mnet_a322_get_analog_output_range(self, RingNo, SlaveIP, ChannelNo, Range): is not implement.')
        return 0

    def _mnet_a322_set_analog_output_value(self, RingNo, SlaveIP, ChannelNo, Val):
        print('_mnet_a322_set_analog_output_value(self, RingNo, SlaveIP, ChannelNo, Val): is not implement.')
        return 0

    def _mnet_a322_set_analog_output_value_all(self, RingNo, SlaveIP, Val):
        print('_mnet_a322_set_analog_output_value_all(self, RingNo, SlaveIP, Val): is not implement.')
        return 0

    def _mnet_a322_set_analog_output(self, RingNo, SlaveIP, ChannelNo, Volt):
        print('_mnet_a322_set_analog_output(self, RingNo, SlaveIP, ChannelNo, Volt): is not implement.')
        return 0

    def _mnet_a322_set_analog_output_ex(self, RingNo, SlaveIP, ChannelNo, Volt, Val):
        print('_mnet_a322_set_analog_output_ex(self, RingNo, SlaveIP, ChannelNo, Volt, Val): is not implement.')
        return 0

    def _mnet_a322_set_analog_output_all(self, RingNo, SlaveIP, Volt):
        print('_mnet_a322_set_analog_output_all(self, RingNo, SlaveIP, Volt): is not implement.')
        return 0

    def _mnet_a322_set_analog_output_all_ex(self, RingNo, SlaveIP, Volt, Val):
        print('_mnet_a322_set_analog_output_all_ex(self, RingNo, SlaveIP, Volt, Val): is not implement.')
        return 0

    def _mnet_a204_initial(self, RingNo, SlaveIP):
        print('_mnet_a204_initial(self, RingNo, SlaveIP): is not implement.')
        return 0

    def _mnet_a204_get_firmware_version(self, RingNo, SlaveIP, Ver):
        print('_mnet_a204_get_firmware_version(self, RingNo, SlaveIP, Ver): is not implement.')
        return 0

    def _mnet_a204_set_analog_output(self, RingNo, SlaveIP, ChannelNo, Volt):
        print('_mnet_a204_set_analog_output(self, RingNo, SlaveIP, ChannelNo, Volt): is not implement.')
        return 0

    def _mnet_a204_set_analog_output_ex(self, RingNo, SlaveIP, ChannelNo, Volt, Val):
        print('_mnet_a204_set_analog_output_ex(self, RingNo, SlaveIP, ChannelNo, Volt, Val): is not implement.')
        return 0

    def _mnet_a204_set_analog_output_all(self, RingNo, SlaveIP, Volt):
        print('_mnet_a204_set_analog_output_all(self, RingNo, SlaveIP, Volt): is not implement.')
        return 0

    def _mnet_a204_set_analog_output_all_ex(self, RingNo, SlaveIP, Volt, Val):
        print('_mnet_a204_set_analog_output_all_ex(self, RingNo, SlaveIP, Volt, Val): is not implement.')
        return 0

    def _mnet_a204_set_analog_output_value(self, RingNo, SlaveIP, ChannelNo, Val):
        print('_mnet_a204_set_analog_output_value(self, RingNo, SlaveIP, ChannelNo, Val): is not implement.')
        return 0

    def _mnet_a204_set_analog_output_value_all(self, RingNo, SlaveIP, Val):
        print('_mnet_a204_set_analog_output_value_all(self, RingNo, SlaveIP, Val): is not implement.')
        return 0

    def _mnet_ai8_initial(self, RingNo, SlaveIP):
        print('_mnet_ai8_initial(self, RingNo, SlaveIP): is not implement.')
        return 0

    def _mnet_ai8_get_hardware_info(self, RingNo, SlaveIP, DeviceID, VHDL_Version):
        print('_mnet_ai8_get_hardware_info(self, RingNo, SlaveIP, DeviceID, VHDL_Version): is not implement.')
        return 0

    def _mnet_ai8_set_cycle_time(self, RingNo, SlaveIP, SetValue):
        print('_mnet_ai8_set_cycle_time(self, RingNo, SlaveIP, SetValue): is not implement.')
        return 0

    def _mnet_ai8_enable_device(self, RingNo, SlaveIP, EnableDevice):
        print('_mnet_ai8_enable_device(self, RingNo, SlaveIP, EnableDevice): is not implement.')
        return 0

    def _mnet_ai8_enable_channel(self, RingNo, SlaveIP, ChannelNo, Enable):
        print('_mnet_ai8_enable_channel(self, RingNo, SlaveIP, ChannelNo, Enable): is not implement.')
        return 0

    def _mnet_ai8_set_channel_gain(self, RingNo, SlaveIP, ChannelNo, Gain):
        print('_mnet_ai8_set_channel_gain(self, RingNo, SlaveIP, ChannelNo, Gain): is not implement.')
        return 0

    def _mnet_ai8_get_value(self, RingNo, SlaveIP, ChannelNo, Value):
        print('_mnet_ai8_get_value(self, RingNo, SlaveIP, ChannelNo, Value): is not implement.')
        return 0

    def _mnet_ai8_get_value_all(self, RingNo, SlaveIP, Value):
        print('_mnet_ai8_get_value_all(self, RingNo, SlaveIP, Value): is not implement.')
        return 0

    def _mnet_ai8_get_voltage(self, RingNo, SlaveIP, ChannelNo, Voltage):
        print('_mnet_ai8_get_voltage(self, RingNo, SlaveIP, ChannelNo, Voltage): is not implement.')
        return 0

    def _mnet_ai8_get_voltage_all(self, RingNo, SlaveIP, Voltage):
        print('_mnet_ai8_get_voltage_all(self, RingNo, SlaveIP, Voltage): is not implement.')
        return 0

    def _mnet_ao4_initial(self, RingNo, SlaveIP):
        print('_mnet_ao4_initial(self, RingNo, SlaveIP): is not implement.')
        return 0

    def _mnet_ao4_get_hardware_info(self, RingNo, SlaveIP, DeviceID, VHDL_Version):
        print('_mnet_ao4_get_hardware_info(self, RingNo, SlaveIP, DeviceID, VHDL_Version): is not implement.')
        return 0

    def _mnet_ao4_reset_DAC(self, RingNo, SlaveIP):
        print('_mnet_ao4_reset_DAC(self, RingNo, SlaveIP): is not implement.')
        return 0

    def _mnet_ao4_clear_output_all(self, RingNo, SlaveIP):
        print('_mnet_ao4_clear_output_all(self, RingNo, SlaveIP): is not implement.')
        return 0

    def _mnet_ao4_set_output(self, RingNo, SlaveIP, ChannelNo, SetValue):
        print('_mnet_ao4_set_output(self, RingNo, SlaveIP, ChannelNo, SetValue): is not implement.')
        return 0

    def _mnet_ao4_set_voltage(self, RingNo, SlaveIP, ChannelNo, Voltage):
        print('_mnet_ao4_set_voltage(self, RingNo, SlaveIP, ChannelNo, Voltage): is not implement.')
        return 0

    def _mnet_ao4_set_voltage1(self, RingNo, SlaveIP, ChannelNo, Voltage, Value):
        print('_mnet_ao4_set_voltage1(self, RingNo, SlaveIP, ChannelNo, Voltage, Value): is not implement.')
        return 0

    def _mnet_ao4_set_output_all(self, RingNo, SlaveIP, SetValue1, SetValue2, SetValue3, SetValue4):
        print('_mnet_ao4_set_output_all(self, RingNo, SlaveIP, SetValue1, SetValue2, SetValue3, SetValue4): is not implement.')
        return 0

    def _mnet_ao4_set_voltage_all(self, RingNo, SlaveIP, Voltage1, Voltage2, Voltage3, Voltage4):
        print('_mnet_ao4_set_voltage_all(self, RingNo, SlaveIP, Voltage1, Voltage2, Voltage3, Voltage4): is not implement.')
        return 0

    def _mnet_ao4_set_voltage_all1(self, RingNo, SlaveIP, Voltage1, Voltage2, Voltage3, Voltage4, Value1, Value2, Value3, Value4):
        print('_mnet_ao4_set_voltage_all1(self, RingNo, SlaveIP, Voltage1, Voltage2, Voltage3, Voltage4, Value1, Value2, Value3, Value4): is not implement.')
        return 0

    def _mnet_ao4_set_coarse_gain(self, RingNo, SlaveIP, ChannelNo, SetValue):
        print('_mnet_ao4_set_coarse_gain(self, RingNo, SlaveIP, ChannelNo, SetValue): is not implement.')
        return 0

    def _mnet_c144_initial(self, RingNo, SlaveIP):
        print('_mnet_c144_initial(self, RingNo, SlaveIP): is not implement.')
        return 0

    def _mnet_c144_get_firmware_version(self, RingNo, SlaveIP, Ver):
        print('_mnet_c144_get_firmware_version(self, RingNo, SlaveIP, Ver): is not implement.')
        return 0

    def _mnet_c144_get_do(self, RingNo, SlaveIP, Val):
        print('_mnet_c144_get_do(self, RingNo, SlaveIP, Val): is not implement.')
        return 0

    def _mnet_c144_set_do(self, RingNo, SlaveIP, Val):
        print('_mnet_c144_set_do(self, RingNo, SlaveIP, Val): is not implement.')
        return 0

    def _mnet_c144_set_encoder_mode(self, RingNo, SlaveIP, ChannelNo, Mode):
        print('_mnet_c144_set_encoder_mode(self, RingNo, SlaveIP, ChannelNo, Mode): is not implement.')
        return 0

    def _mnet_c144_set_pls_iptmode(self, RingNo, SlaveIP, ChannelNo, PlsInMode, PlsInDir):
        print('_mnet_c144_set_pls_iptmode(self, RingNo, SlaveIP, ChannelNo, PlsInMode, PlsInDir): is not implement.')
        return 0

    def _mnet_c144_get_encoder(self, RingNo, SlaveIP, ChannelNo, EncPos):
        print('_mnet_c144_get_encoder(self, RingNo, SlaveIP, ChannelNo, EncPos): is not implement.')
        return 0

    def _mnet_c144_get_encoder_all(self, RingNo, SlaveIP, EncPos):
        print('_mnet_c144_get_encoder_all(self, RingNo, SlaveIP, EncPos): is not implement.')
        return 0

    def _mnet_c144_set_encoder(self, RingNo, SlaveIP, ChannelNo, EncPos):
        print('_mnet_c144_set_encoder(self, RingNo, SlaveIP, ChannelNo, EncPos): is not implement.')
        return 0

    def _mnet_c144_set_auto_compare_source(self, RingNo, SlaveIP, ChannelNo, SrcEncNo):
        print('_mnet_c144_set_auto_compare_source(self, RingNo, SlaveIP, ChannelNo, SrcEncNo): is not implement.')
        return 0

    def _mnet_c144_get_auto_compare_count(self, RingNo, SlaveIP, ChannelNo, Count):
        print('_mnet_c144_get_auto_compare_count(self, RingNo, SlaveIP, ChannelNo, Count): is not implement.')
        return 0

    def _mnet_c144_get_auto_compare_count_all(self, RingNo, SlaveIP, Count):
        print('_mnet_c144_get_auto_compare_count_all(self, RingNo, SlaveIP, Count): is not implement.')
        return 0

    def _mnet_c144_get_auto_compare_count_ex(self, RingNo, SlaveIP, ChannelNo, Count):
        print('_mnet_c144_get_auto_compare_count_ex(self, RingNo, SlaveIP, ChannelNo, Count): is not implement.')
        return 0

    def _mnet_c144_get_auto_compare_count_all_ex(self, RingNo, SlaveIP, Count):
        print('_mnet_c144_get_auto_compare_count_all_ex(self, RingNo, SlaveIP, Count): is not implement.')
        return 0

    def _mnet_c144_get_auto_compare_status(self, RingNo, SlaveIP, ChannelNo, OnOff):
        print('_mnet_c144_get_auto_compare_status(self, RingNo, SlaveIP, ChannelNo, OnOff): is not implement.')
        return 0

    def _mnet_c144_get_auto_compare_status_all(self, RingNo, SlaveIP, OnOff):
        print('_mnet_c144_get_auto_compare_status_all(self, RingNo, SlaveIP, OnOff): is not implement.')
        return 0

    def _mnet_c144_set_auto_compare_trigger(self, RingNo, SlaveIP, ChannelNo, Level, Width):
        print('_mnet_c144_set_auto_compare_trigger(self, RingNo, SlaveIP, ChannelNo, Level, Width): is not implement.')
        return 0

    def _mnet_c144_set_auto_compare_function(self, RingNo, SlaveIP, ChannelNo, Dir, StrPos, Interval, TrgCnt):
        print('_mnet_c144_set_auto_compare_function(self, RingNo, SlaveIP, ChannelNo, Dir, StrPos, Interval, TrgCnt): is not implement.')
        return 0

    def _mnet_c144_set_auto_compare_function_ex(self, RingNo, SlaveIP, ChannelNo, Dir, StrPos, Interval, TrgCnt):
        print('_mnet_c144_set_auto_compare_function_ex(self, RingNo, SlaveIP, ChannelNo, Dir, StrPos, Interval, TrgCnt): is not implement.')
        return 0

    def _mnet_c144_set_auto_compare_table(self, RingNo, SlaveIP, ChannelNo, Dir, Size, Table):
        print('_mnet_c144_set_auto_compare_table(self, RingNo, SlaveIP, ChannelNo, Dir, Size, Table): is not implement.')
        return 0

    def _mnet_c144_get_auto_compare_table_capacity(self, RingNo, SlaveIP, ChannelNo, Size):
        print('_mnet_c144_get_auto_compare_table_capacity(self, RingNo, SlaveIP, ChannelNo, Size): is not implement.')
        return 0

    def _mnet_c144_start_auto_compare(self, RingNo, SlaveIP, ChannelNo, OnOff):
        print('_mnet_c144_start_auto_compare(self, RingNo, SlaveIP, ChannelNo, OnOff): is not implement.')
        return 0

    def _mnet_c144_force_trigger_output(self, RingNo, SlaveIP, ChannelNo):
        print('_mnet_c144_force_trigger_output(self, RingNo, SlaveIP, ChannelNo): is not implement.')
        return 0

    def _mnet_p144_initial(self, RingNo, SlaveIP):
        print('_mnet_p144_initial(self, RingNo, SlaveIP): is not implement.')
        return 0

    def _mnet_p144_get_firmware_version(self, RingNo, SlaveIP, Ver):
        print('_mnet_p144_get_firmware_version(self, RingNo, SlaveIP, Ver): is not implement.')
        return 0

    def _mnet_p144_set_count(self, RingNo, SlaveIP, ChannelNo, Count):
        print('_mnet_p144_set_count(self, RingNo, SlaveIP, ChannelNo, Count): is not implement.')
        return 0

    def _mnet_p144_get_count(self, RingNo, SlaveIP, ChannelNo, Count):
        print('_mnet_p144_get_count(self, RingNo, SlaveIP, ChannelNo, Count): is not implement.')
        return 0

    def _mnet_p144_get_count_all(self, RingNo, SlaveIP, Count):
        print('_mnet_p144_get_count_all(self, RingNo, SlaveIP, Count): is not implement.')
        return 0

    def _mnet_p144_get_frequency(self, RingNo, SlaveIP, ChannelNo, Freq):
        print('_mnet_p144_get_frequency(self, RingNo, SlaveIP, ChannelNo, Freq): is not implement.')
        return 0

    def _mnet_p144_get_frequency_all(self, RingNo, SlaveIP, Freq):
        print('_mnet_p144_get_frequency_all(self, RingNo, SlaveIP, Freq): is not implement.')
        return 0

    def _mnet_p144_set_pulse_level(self, RingNo, SlaveIP, ChannelNo, Level):
        print('_mnet_p144_set_pulse_level(self, RingNo, SlaveIP, ChannelNo, Level): is not implement.')
        return 0

    def _mnet_p144_get_pulse_width(self, RingNo, SlaveIP, ChannelNo, Width):
        print('_mnet_p144_get_pulse_width(self, RingNo, SlaveIP, ChannelNo, Width): is not implement.')
        return 0

    def _mnet_p144_get_pulse_width_all(self, RingNo, SlaveIP, Width):
        print('_mnet_p144_get_pulse_width_all(self, RingNo, SlaveIP, Width): is not implement.')
        return 0

    def _mnet_p144_set_filter(self, RingNo, SlaveIP, ChannelNo, Filter):
        print('_mnet_p144_set_filter(self, RingNo, SlaveIP, ChannelNo, Filter): is not implement.')
        return 0

    def _mnet_p144_get_do(self, RingNo, SlaveIP, Val):
        print('_mnet_p144_get_do(self, RingNo, SlaveIP, Val): is not implement.')
        return 0

    def _mnet_p144_set_do_bit(self, RingNo, SlaveIP, BitNo, Mode, Val):
        print('_mnet_p144_set_do_bit(self, RingNo, SlaveIP, BitNo, Mode, Val): is not implement.')
        return 0

    def _mnet_m1a_initial(self, RingNo, SlaveIP):
        client = self.create_client(MailBox, '/mnet/m1aInit')

        sendBuff = struct.pack("=2H", RingNo, SlaveIP)
        byteArr = array.array('B', sendBuff)
        
        req = MailBox.Request()
        req.buffer = byteArr

        future = client.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1

        return res.result_code

    def _mnet_m1a_set_pls_outmode(self, RingNo, SlaveIP, pls_outmode):
        client = self.create_client(MailBox, '/mnet/m1aSetPlsOutmode')

        sendBuff = struct.pack("=3H", RingNo, SlaveIP, pls_outmode)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = client.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1

        return res.result_code

    def _mnet_m1a_set_pls_iptmode(self, RingNo, SlaveIP, pls_iptmode, pls_iptdir):
        client = self.create_client(MailBox, '/mnet/m1aSetPlsIptmode')

        sendBuff = struct.pack("=4H", RingNo, SlaveIP, pls_iptmode, pls_iptdir)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = client.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1

        return res.result_code

    def _mnet_m1a_set_feedback_src(self, RingNo, SlaveIP, FbkSrc):
        client = self.create_client(MailBox, '/mnet/m1aSetFeedbackSrc')

        sendBuff = struct.pack("=3H", RingNo, SlaveIP, FbkSrc)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = client.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1

        return res.result_code

    def _mnet_m1a_set_abs_reference(self, RingNo, SlaveIP, Ref):
        client = self.create_client(MailBox, '/mnet/m1aSetAbsRef')
        sendBuff = struct.pack("=3H", RingNo, SlaveIP, Ref)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = client.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1
        
        return res.result_code

    def _mnet_m1a_set_abs_ref_encoder_ignore_backlash(self, RingNo, SlaveIP, OnOff):
        print('_mnet_m1a_set_abs_ref_encoder_ignore_backlash(self, RingNo, SlaveIP, OnOff): is not implement.')
        return 0

    def _mnet_m1a_set_direction_change_delay(self, RingNo, SlaveIP, Enabled):
        print('_mnet_m1a_set_direction_change_delay(self, RingNo, SlaveIP, Enabled): is not implement.')
        return 0

    def _mnet_m1a_set_move_ratio(self, RingNo, SlaveIP, MoveRatio):
        print('_mnet_m1a_set_move_ratio(self, RingNo, SlaveIP, MoveRatio): is not implement.')
        return 0

    def _mnet_m1a_set_alm(self, RingNo, SlaveIP, alm_logic, alm_mode):
        client = self.create_client(MailBox, '/mnet/m1aSetAlm')

        sendBuff = struct.pack("=4H", RingNo, SlaveIP, alm_logic, alm_mode)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = client.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1

        return res.result_code

    def _mnet_m1a_set_inp(self, RingNo, SlaveIP, inp_enable, inp_logic):
        client = self.create_client(MailBox, '/mnet/m1aSetInp')

        sendBuff = struct.pack("=4H", RingNo, SlaveIP, inp_enable, inp_logic)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = client.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1

        return res.result_code

    def _mnet_m1a_set_el_mode(self, RingNo, SlaveIP, el_mode):
        print('_mnet_m1a_set_el_mode(self, RingNo, SlaveIP, el_mode): is not implement.')
        return 0

    def _mnet_m1a_set_sd(self, RingNo, SlaveIP, sd_enable, sd_logic, sd_latch, sd_mode):
        client = self.create_client(MailBox, '/mnet/m1aSetSd')

        sendBuff = struct.pack("=2H4h", RingNo, SlaveIP, sd_enable, sd_logic, sd_latch, sd_mode)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = client.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1

        return res.result_code

    def _mnet_m1a_set_erc(self, RingNo, SlaveIP, erc_logic, erc_on_time, erc_off_time):
        client = self.create_client(MailBox, '/mnet/m1aSetErc')

        sendBuff = struct.pack("=5H", RingNo, SlaveIP, erc_logic, erc_on_time, erc_off_time)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = client.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1

        return res.result_code

    def _mnet_m1a_set_erc_on(self, RingNo, SlaveIP, on_off):
        client = self.create_client(MailBox, '/mnet/m1aSetErcOn')
        sendBuff = struct.pack("=2Hh", RingNo, SlaveIP, on_off)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = client.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1
        
        return res.result_code

    def _mnet_m1a_set_auto_erc(self, RingNo, SlaveIP, Enable):
        print('_mnet_m1a_set_auto_erc(self, RingNo, SlaveIP, Enable): is not implement.')
        return 0

    def _mnet_m1a_set_svon(self, RingNo, SlaveIP, on_off):
        client = self.create_client(MailBox, '/mnet/m1aSetSvon')
        sendBuff = struct.pack("=3H", RingNo, SlaveIP, on_off)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = client.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1
        
        return res.result_code

    def _mnet_m1a_set_ralm(self, RingNo, SlaveIP, on_off):
        client = self.create_client(MailBox, '/mnet/m1aSetRalm')
        sendBuff = struct.pack("=3H", RingNo, SlaveIP, on_off)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = client.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1
        
        return res.result_code

    def _mnet_m1a_set_ltc_logic(self, RingNo, SlaveIP, ltc_logic):
        client = self.create_client(MailBox, '/mnet/m1aSetLtcLogic')

        sendBuff = struct.pack("=3H", RingNo, SlaveIP, ltc_logic)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = client.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1

        return res.result_code

    def _mnet_m1a_set_mechanical_input_filter(self, RingNo, SlaveIP, on_off):
        print('_mnet_m1a_set_mechanical_input_filter(self, RingNo, SlaveIP, on_off): is not implement.')
        return 0

    def _mnet_m1a_set_backlash(self, RingNo, SlaveIP, Value, Enable, CntSrc):
        print('_mnet_m1a_set_backlash(self, RingNo, SlaveIP, Value, Enable, CntSrc): is not implement.')
        return 0

    def _mnet_m1a_dio_output(self, RingNo, SlaveIP, DoNo, on_off):
        client = self.create_client(MailBox, '/mnet/m1aDioOutput')
        sendBuff = struct.pack("=4H", RingNo, SlaveIP, DoNo, on_off)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = client.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1
        
        return res.result_code

    def _mnet_m1a_dio_input(self, RingNo, SlaveIP, DiNo):
        print('_mnet_m1a_dio_input(self, RingNo, SlaveIP, DiNo): is not implement.')
        return 0

    def _mnet_m1a_get_dio(self, RingNo, SlaveIP, Val):
        client = self.create_client(MailBox, '/mnet/m1aGetDio')
        sendBuff = struct.pack("=2H", RingNo, SlaveIP)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = client.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1
        
        recvBuff = res.buffer.tobytes()
        value = struct.unpack("=B", recvBuff[:struct.calcsize("=B")])[0]

        Val.clear()
        Val.append(value)
        
        return res.result_code

    def _mnet_m1a_get_io_status(self, RingNo, SlaveIP, IO_status):
        sendBuff = struct.pack("=2H", RingNo, SlaveIP)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = self.cliM1aGetIOSts.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1

        recvBuff = res.buffer.tobytes()
        status = struct.unpack("=I", recvBuff[:struct.calcsize("=I")])[0]

        IO_status.clear()
        IO_status.append(status)
        return res.result_code

    def _mnet_m1a_load_motion_file(self, RingNo, SlaveIP, FilePath):
        print('_mnet_m1a_load_motion_file(self, RingNo, SlaveIP, FilePath): is not implement.')
        return 0

    def _mnet_m1a_get_command(self, RingNo, SlaveIP, Cmd):
        sendBuff = struct.pack("=2H", RingNo, SlaveIP)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = self.cliM1aGetCommand.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1

        recvBuff = res.buffer.tobytes()
        cmd = struct.unpack("=i", recvBuff[:struct.calcsize("=i")])[0]

        Cmd.clear()
        Cmd.append(cmd)

        return res.result_code

    def _mnet_m1a_set_command(self, RingNo, SlaveIP, Cmd):
        print('_mnet_m1a_set_command(self, RingNo, SlaveIP, Cmd): is not implement.')
        return 0

    def _mnet_m1a_reset_command(self, RingNo, SlaveIP):
        client = self.create_client(MailBox, '/mnet/m1aResetCommand')

        sendBuff = struct.pack("=2H", RingNo, SlaveIP)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = client.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1

        return res.result_code

    def _mnet_m1a_get_position(self, RingNo, SlaveIP, Pos):
        sendBuff = struct.pack("=2H", RingNo, SlaveIP)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = self.cliM1aGetPosition.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1

        recvBuff = res.buffer.tobytes()
        status = struct.unpack("=i", recvBuff[:struct.calcsize("=i")])[0]

        Pos.clear()
        Pos.append(status)

        return res.result_code

    def _mnet_m1a_set_position(self, RingNo, SlaveIP, Pos):
        print('_mnet_m1a_set_position(self, RingNo, SlaveIP, Pos): is not implement.')
        return 0

    def _mnet_m1a_reset_position(self, RingNo, SlaveIP):
        client = self.create_client(MailBox, '/mnet/m1aResetPotition')

        sendBuff = struct.pack("=2H", RingNo, SlaveIP)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = client.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1

        return res.result_code

    def _mnet_m1a_get_error_counter(self, RingNo, SlaveIP, ErrCnt):
        print('_mnet_m1a_get_error_counter(self, RingNo, SlaveIP, ErrCnt): is not implement.')
        return 0

    def _mnet_m1a_set_error_counter(self, RingNo, SlaveIP, ErrCnt):
        print('_mnet_m1a_set_error_counter(self, RingNo, SlaveIP, ErrCnt): is not implement.')
        return 0

    def _mnet_m1a_reset_error_counter(self, RingNo, SlaveIP):
        client = self.create_client(MailBox, '/mnet/m1aResetCounter')

        sendBuff = struct.pack("=2H", RingNo, SlaveIP)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = client.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1

        return res.result_code

    def _mnet_m1a_get_rest_command(self, RingNo, SlaveIP, Cmd):
        print('_mnet_m1a_get_rest_command(self, RingNo, SlaveIP, Cmd): is not implement.')
        return 0

    def _mnet_m1a_enable_command_ring_counter(self, RingNo, SlaveIP, RingCounter):
        print('_mnet_m1a_enable_command_ring_counter(self, RingNo, SlaveIP, RingCounter): is not implement.')
        return 0

    def _mnet_m1a_disable_command_ring_counter(self, RingNo, SlaveIP):
        print('_mnet_m1a_disable_command_ring_counter(self, RingNo, SlaveIP): is not implement.')
        return 0

    def _mnet_m1a_enable_position_ring_counter(self, RingNo, SlaveIP, RingCounter):
        print('_mnet_m1a_enable_position_ring_counter(self, RingNo, SlaveIP, RingCounter): is not implement.')
        return 0

    def _mnet_m1a_disable_position_ring_counter(self, RingNo, SlaveIP):
        print('_mnet_m1a_disable_position_ring_counter(self, RingNo, SlaveIP): is not implement.')
        return 0

    def _mnet_m1a_get_current_speed(self, RingNo, SlaveIP, Speed):
        sendBuff = struct.pack("=2H", RingNo, SlaveIP)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = self.cliM1aGetCurrSpd.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1

        recvBuff = res.buffer.tobytes()
        speed = struct.unpack("=I", recvBuff[:struct.calcsize("=I")])[0]

        Speed.clear()
        Speed.append(speed)

        return res.result_code

    def _mnet_m1a_get_command_without_backlash(self, RingNo, SlaveIP, Cmd):
        print('_mnet_m1a_get_command_without_backlash(self, RingNo, SlaveIP, Cmd): is not implement.')
        return 0

    def _mnet_m1a_get_position_without_backlash(self, RingNo, SlaveIP, Pos):
        print('_mnet_m1a_get_position_without_backlash(self, RingNo, SlaveIP, Pos): is not implement.')
        return 0

    def _mnet_m1a_set_tmove_speed(self, RingNo, SlaveIP, StrVel, MaxVel, Tacc, Tdec):
        client = self.create_client(MailBox, '/mnet/m1aSetTMoveSpd')
        sendBuff = struct.pack("=2H2I2f", RingNo, SlaveIP, StrVel, MaxVel, Tacc, Tdec)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = client.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1
        
        return res.result_code

    def _mnet_m1a_set_smove_speed(self, RingNo, SlaveIP, StrVel, MaxVel, Tacc, Tdec):
        client = self.create_client(MailBox, '/mnet/m1aSetSMoveSpd')
        sendBuff = struct.pack("=2H2I2f", RingNo, SlaveIP, StrVel, MaxVel, Tacc, Tdec)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = client.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1
        
        return res.result_code

    def _mnet_m1a_set_smove_speed_ex(self, RingNo, SlaveIP, StrVel, MaxVel, Tacc, Tdec, SVacc, SVdec):
        print('_mnet_m1a_set_smove_speed_ex(self, RingNo, SlaveIP, StrVel, MaxVel, Tacc, Tdec, SVacc, SVdec): is not implement.')
        return 0

    def _mnet_m1a_start_r_move(self, RingNo, SlaveIP, Dist):
        client = self.create_client(MailBox, '/mnet/m1aStartRMove')
        sendBuff = struct.pack("=2Hi", RingNo, SlaveIP, Dist)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = client.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1
        
        return res.result_code

    def _mnet_m1a_start_a_move(self, RingNo, SlaveIP, Pos):
        client = self.create_client(MailBox, '/mnet/m1aStartAMove')
        sendBuff = struct.pack("=2Hi", RingNo, SlaveIP, Pos)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = client.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1
        
        return res.result_code

    def _mnet_m1a_start_tr_move(self, RingNo, SlaveIP, Dist, StrVel, MaxVel, Tacc, Tdec):
        print('_mnet_m1a_start_tr_move(self, RingNo, SlaveIP, Dist, StrVel, MaxVel, Tacc, Tdec): is not implement.')
        return 0

    def _mnet_m1a_start_ta_move(self, RingNo, SlaveIP, Pos, StrVel, MaxVel, Tacc, Tdec):
        print('_mnet_m1a_start_ta_move(self, RingNo, SlaveIP, Pos, StrVel, MaxVel, Tacc, Tdec): is not implement.')
        return 0

    def _mnet_m1a_start_sr_move(self, RingNo, SlaveIP, Dist, StrVel, MaxVel, Tacc, Tdec):
        print('_mnet_m1a_start_sr_move(self, RingNo, SlaveIP, Dist, StrVel, MaxVel, Tacc, Tdec): is not implement.')
        return 0

    def _mnet_m1a_start_sa_move(self, RingNo, SlaveIP, Pos, StrVel, MaxVel, Tacc, Tdec):
        print('_mnet_m1a_start_sa_move(self, RingNo, SlaveIP, Pos, StrVel, MaxVel, Tacc, Tdec): is not implement.')
        return 0

    def _mnet_m1a_v_move(self, RingNo, SlaveIP, Dir):
        client = self.create_client(MailBox, '/mnet/m1aVMove')
        sendBuff = struct.pack("=2HB", RingNo, SlaveIP, Dir)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = client.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1
        
        return res.result_code

    def _mnet_m1a_tv_move(self, RingNo, SlaveIP, Dir, StrVel, MaxVel, Tacc):
        print('_mnet_m1a_tv_move(self, RingNo, SlaveIP, Dir, StrVel, MaxVel, Tacc): is not implement.')
        return 0

    def _mnet_m1a_sv_move(self, RingNo, SlaveIP, Dir, StrVel, MaxVel, Tacc):
        print('_mnet_m1a_sv_move(self, RingNo, SlaveIP, Dir, StrVel, MaxVel, Tacc): is not implement.')
        return 0

    def _mnet_m1a_sd_stop(self, RingNo, SlaveIP):
        client = self.create_client(MailBox, '/mnet/m1aSdStop')
        sendBuff = struct.pack("=2H", RingNo, SlaveIP)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = client.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1
        
        return res.result_code

    def _mnet_m1a_sd_stop_ex(self, RingNo, SlaveIP, Tdec):
        print('_mnet_m1a_sd_stop_ex(self, RingNo, SlaveIP, Tdec): is not implement.')
        return 0

    def _mnet_m1a_imd_stop(self, RingNo, SlaveIP):
        print('_mnet_m1a_imd_stop(self, RingNo, SlaveIP): is not implement.')
        return 0

    def _mnet_m1a_emg_stop(self, RingNo, SlaveIP):
        client = self.create_client(MailBox, '/mnet/m1aEmgStop')
        sendBuff = struct.pack("=2H", RingNo, SlaveIP)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = client.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1
        
        return res.result_code

    def _mnet_m1a_motion_done(self, RingNo, SlaveIP, MoSt):
        sendBuff = struct.pack("=2H", RingNo, SlaveIP)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = self.cliM1aMotionDone.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1

        recvBuff = res.buffer.tobytes()
        status = struct.unpack("=H", recvBuff[:struct.calcsize("=H")])[0]

        MoSt.clear()
        MoSt.append(status)

        return res.result_code

    def _mnet_m1a_triangle_correction(self, RingNo, SlaveIP, OnOff):
        print('_mnet_m1a_triangle_correction(self, RingNo, SlaveIP, OnOff): is not implement.')
        return 0

    def _mnet_m1a_set_home_config(self, RingNo, SlaveIP, home_mode, org_logic, ez_logic, ez_count, erc_out):
        client = self.create_client(MailBox, '/mnet/m1aSetHomeConfig')

        sendBuff = struct.pack("=7H", RingNo, SlaveIP, home_mode, org_logic, ez_logic, ez_count, erc_out)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = client.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1

        return res.result_code

    def _mnet_m1a_start_home_move(self, RingNo, SlaveIP, Dir):
        print('_mnet_m1a_start_home_move(self, RingNo, SlaveIP, Dir): is not implement.')
        return 0

    def _mnet_m1a_start_home_search(self, RingNo, SlaveIP, Dir, ORGOffset):
        print('_mnet_m1a_start_home_search(self, RingNo, SlaveIP, Dir, ORGOffset): is not implement.')
        return 0

    def _mnet_m1a_start_home_escape(self, RingNo, SlaveIP, Dir):
        print('_mnet_m1a_start_home_escape(self, RingNo, SlaveIP, Dir): is not implement.')
        return 0

    def _mnet_m1a_start_search_ez(self, RingNo, SlaveIP, Dir, EzCount):
        print('_mnet_m1a_start_search_ez(self, RingNo, SlaveIP, Dir, EzCount): is not implement.')
        return 0

    def _mnet_m1a_home_move(self, RingNo, SlaveIP, Dir, StrVel, MaxVel, Tacc):
        client = self.create_client(MailBox, '/mnet/m1aHomeMove')
        sendBuff = struct.pack("=2HB2If", RingNo, SlaveIP, Dir, StrVel, MaxVel, Tacc)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = client.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1
        
        return res.result_code

    def _mnet_m1a_home_search(self, RingNo, SlaveIP, Dir, StrVel, MaxVel, Tacc, ORGOffset):
        client = self.create_client(MailBox, '/mnet/m1aHomeSearch')
        sendBuff = struct.pack("=2HB2Ifi", RingNo, SlaveIP, Dir, StrVel, MaxVel, Tacc, ORGOffset)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = client.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1
        
        return res.result_code

    def _mnet_m1a_home_escape(self, RingNo, SlaveIP, Dir, StrVel, MaxVel, Tacc):
        print('_mnet_m1a_home_escape(self, RingNo, SlaveIP, Dir, StrVel, MaxVel, Tacc): is not implement.')
        return 0

    def _mnet_m1a_search_ez(self, RingNo, SlaveIP, Dir, EzCount, StrVel, MaxVel, Tacc):
        print('_mnet_m1a_search_ez(self, RingNo, SlaveIP, Dir, EzCount, StrVel, MaxVel, Tacc): is not implement.')
        return 0

    def _mnet_m1a_set_comparator_mode(self, RingNo, SlaveIP, CompNo, CmpSrc, CmpMethod, CmpAction):
        print('_mnet_m1a_set_comparator_mode(self, RingNo, SlaveIP, CompNo, CmpSrc, CmpMethod, CmpAction): is not implement.')
        return 0

    def _mnet_m1a_set_comparator_data(self, RingNo, SlaveIP, CompNo, Pos):
        print('_mnet_m1a_set_comparator_data(self, RingNo, SlaveIP, CompNo, Pos): is not implement.')
        return 0

    def _mnet_m1a_set_trigger_comparator(self, RingNo, SlaveIP, CmpSrc, CmpMethod):
        print('_mnet_m1a_set_trigger_comparator(self, RingNo, SlaveIP, CmpSrc, CmpMethod): is not implement.')
        return 0

    def _mnet_m1a_set_trigger_comparator_data(self, RingNo, SlaveIP, Data):
        print('_mnet_m1a_set_trigger_comparator_data(self, RingNo, SlaveIP, Data): is not implement.')
        return 0

    def _mnet_m1a_set_auto_trigger(self, RingNo, SlaveIP, CmpSrc, CmpMethod, Interval, on_off):
        print('_mnet_m1a_set_auto_trigger(self, RingNo, SlaveIP, CmpSrc, CmpMethod, Interval, on_off): is not implement.')
        return 0

    def _mnet_m1a_get_comparator_data(self, RingNo, SlaveIP, CompNo, Pos):
        print('_mnet_m1a_get_comparator_data(self, RingNo, SlaveIP, CompNo, Pos): is not implement.')
        return 0

    def _mnet_m1a_set_soft_limit(self, RingNo, SlaveIP, PLimit, MLimit):
        print('_mnet_m1a_set_soft_limit(self, RingNo, SlaveIP, PLimit, MLimit): is not implement.')
        return 0

    def _mnet_m1a_enable_soft_limit(self, RingNo, SlaveIP, Action):
        print('_mnet_m1a_enable_soft_limit(self, RingNo, SlaveIP, Action): is not implement.')
        return 0

    def _mnet_m1a_disable_soft_limit(self, RingNo, SlaveIP):
        print('_mnet_m1a_disable_soft_limit(self, RingNo, SlaveIP): is not implement.')
        return 0

    def _mnet_m1a_set_latch_signal(self, RingNo, SlaveIP, LtcSignal):
        print('_mnet_m1a_set_latch_signal(self, RingNo, SlaveIP, LtcSignal): is not implement.')
        return 0

    def _mnet_m1a_get_latch_data(self, RingNo, SlaveIP, LatchNo, Pos):
        print('_mnet_m1a_get_latch_data(self, RingNo, SlaveIP, LatchNo, Pos): is not implement.')
        return 0

    def _mnet_m1a_v_change(self, RingNo, SlaveIP, NewVel, Time):
        print('_mnet_m1a_v_change(self, RingNo, SlaveIP, NewVel, Time): is not implement.')
        return 0

    def _mnet_m1a_fix_speed_range(self, RingNo, SlaveIP, MaxVel):
        print('_mnet_m1a_fix_speed_range(self, RingNo, SlaveIP, MaxVel): is not implement.')
        return 0

    def _mnet_m1a_unfix_speed_range(self, RingNo, SlaveIP):
        print('_mnet_m1a_unfix_speed_range(self, RingNo, SlaveIP): is not implement.')
        return 0

    def _mnet_m1a_get_speed_range(self, RingNo, SlaveIP, MinVel, MaxVel):
        print('_mnet_m1a_get_speed_range(self, RingNo, SlaveIP, MinVel, MaxVel): is not implement.')
        return 0

    def _mnet_m1a_start_p_change(self, RingNo, SlaveIP, NewPos):
        print('_mnet_m1a_start_p_change(self, RingNo, SlaveIP, NewPos): is not implement.')
        return 0

    def _mnet_m1a_start_d_change(self, RingNo, SlaveIP, NewDist):
        print('_mnet_m1a_start_d_change(self, RingNo, SlaveIP, NewDist): is not implement.')
        return 0

    def _mnet_m1a_get_preregister_status(self, RingNo, SlaveIP, Full):
        print('_mnet_m1a_get_preregister_status(self, RingNo, SlaveIP, Full): is not implement.')
        return 0

    def _mnet_m1a_get_preregister_depth(self, RingNo, SlaveIP, Depth):
        print('_mnet_m1a_get_preregister_depth(self, RingNo, SlaveIP, Depth): is not implement.')
        return 0

    def _mnet_m1a_cancel_preregister(self, RingNo, SlaveIP):
        print('_mnet_m1a_cancel_preregister(self, RingNo, SlaveIP): is not implement.')
        return 0

    def _mnet_m1a_shift_preregister(self, RingNo, SlaveIP):
        print('_mnet_m1a_shift_preregister(self, RingNo, SlaveIP): is not implement.')
        return 0

    def _mnet_m1a_check_int(self, RingNo, SlaveIP, IntStatus):
        print('_mnet_m1a_check_int(self, RingNo, SlaveIP, IntStatus): is not implement.')
        return 0

    def _mnet_m1a_check_error(self, RingNo, SlaveIP, ErrorStatus):
        print('_mnet_m1a_check_error(self, RingNo, SlaveIP, ErrorStatus): is not implement.')
        return 0

    def _mnet_m1a_check_event(self, RingNo, SlaveIP, EventStatus):
        print('_mnet_m1a_check_event(self, RingNo, SlaveIP, EventStatus): is not implement.')
        return 0

    def _mnet_m1a_set_int_factor(self, RingNo, SlaveIP, int_factor):
        print('_mnet_m1a_set_int_factor(self, RingNo, SlaveIP, int_factor): is not implement.')
        return 0

    def _mnet_m1a_set_group(self, RingNo, GroupNo, AxisArray, AxisCount):
        print('_mnet_m1a_set_group(self, RingNo, GroupNo, AxisArray, AxisCount): is not implement.')
        return 0

    def _mnet_m1a_group_reset_command(self, RingNo, GroupNo):
        print('_mnet_m1a_group_reset_command(self, RingNo, GroupNo): is not implement.')
        return 0

    def _mnet_m1a_group_reset_position(self, RingNo, GroupNo):
        print('_mnet_m1a_group_reset_position(self, RingNo, GroupNo): is not implement.')
        return 0

    def _mnet_m1a_group_continuous_move(self, RingNo, GroupNo, Enable):
        print('_mnet_m1a_group_continuous_move(self, RingNo, GroupNo, Enable): is not implement.')
        return 0

    def _mnet_m1a_group_tr_lineN(self, RingNo, GroupNo, DistArray, StrVel, MaxVel, Tacc, Tdec):
        print('_mnet_m1a_group_tr_lineN(self, RingNo, GroupNo, DistArray, StrVel, MaxVel, Tacc, Tdec): is not implement.')
        return 0

    def _mnet_m1a_group_ta_lineN(self, RingNo, GroupNo, PosArray, StrVel, MaxVel, Tacc, Tdec):
        print('_mnet_m1a_group_ta_lineN(self, RingNo, GroupNo, PosArray, StrVel, MaxVel, Tacc, Tdec): is not implement.')
        return 0

    def _mnet_m1a_group_sr_lineN(self, RingNo, GroupNo, DistArray, StrVel, MaxVel, Tacc, Tdec):
        print('_mnet_m1a_group_sr_lineN(self, RingNo, GroupNo, DistArray, StrVel, MaxVel, Tacc, Tdec): is not implement.')
        return 0

    def _mnet_m1a_group_sa_lineN(self, RingNo, GroupNo, PosArray, StrVel, MaxVel, Tacc, Tdec):
        print('_mnet_m1a_group_sa_lineN(self, RingNo, GroupNo, PosArray, StrVel, MaxVel, Tacc, Tdec): is not implement.')
        return 0

    def _mnet_m1a_group_tr_lineN_continuous(self, RingNo, GroupNo, DistArray, StrVel, MaxVel, Tacc, Tdec):
        print('_mnet_m1a_group_tr_lineN_continuous(self, RingNo, GroupNo, DistArray, StrVel, MaxVel, Tacc, Tdec): is not implement.')
        return 0

    def _mnet_m1a_group_ta_lineN_continuous(self, RingNo, GroupNo, PosArray, StrVel, MaxVel, Tacc, Tdec):
        print('_mnet_m1a_group_ta_lineN_continuous(self, RingNo, GroupNo, PosArray, StrVel, MaxVel, Tacc, Tdec): is not implement.')
        return 0

    def _mnet_m1a_group_sr_lineN_continuous(self, RingNo, GroupNo, DistArray, StrVel, MaxVel, Tacc, Tdec):
        print('_mnet_m1a_group_sr_lineN_continuous(self, RingNo, GroupNo, DistArray, StrVel, MaxVel, Tacc, Tdec): is not implement.')
        return 0

    def _mnet_m1a_group_sa_lineN_continuous(self, RingNo, GroupNo, PosArray, StrVel, MaxVel, Tacc, Tdec):
        print('_mnet_m1a_group_sa_lineN_continuous(self, RingNo, GroupNo, PosArray, StrVel, MaxVel, Tacc, Tdec): is not implement.')
        return 0

    def _mnet_m1a_group_tr_arc(self, RingNo, GroupNo, AxIP, AyIP, OffsetCx, OffsetCy, OffsetEx, OffsetEy, Dir, StrVel, MaxVel, Tacc, Tdec):
        print('_mnet_m1a_group_tr_arc(self, RingNo, GroupNo, AxIP, AyIP, OffsetCx, OffsetCy, OffsetEx, OffsetEy, Dir, StrVel, MaxVel, Tacc, Tdec): is not implement.')
        return 0

    def _mnet_m1a_group_ta_arc(self, RingNo, GroupNo, AxIP, AyIP, Cx, Cy, Ex, Ey, Dir, StrVel, MaxVel, Tacc, Tdec):
        print('_mnet_m1a_group_ta_arc(self, RingNo, GroupNo, AxIP, AyIP, Cx, Cy, Ex, Ey, Dir, StrVel, MaxVel, Tacc, Tdec): is not implement.')
        return 0

    def _mnet_m1a_group_sr_arc(self, RingNo, GroupNo, AxIP, AyIP, OffsetCx, OffsetCy, OffsetEx, OffsetEy, Dir, StrVel, MaxVel, Tacc, Tdec):
        print('_mnet_m1a_group_sr_arc(self, RingNo, GroupNo, AxIP, AyIP, OffsetCx, OffsetCy, OffsetEx, OffsetEy, Dir, StrVel, MaxVel, Tacc, Tdec): is not implement.')
        return 0

    def _mnet_m1a_group_sa_arc(self, RingNo, GroupNo, AxIP, AyIP, Cx, Cy, Ex, Ey, Dir, StrVel, MaxVel, Tacc, Tdec):
        print('_mnet_m1a_group_sa_arc(self, RingNo, GroupNo, AxIP, AyIP, Cx, Cy, Ex, Ey, Dir, StrVel, MaxVel, Tacc, Tdec): is not implement.')
        return 0

    def _mnet_m1a_group_tr_arc_continuous(self, RingNo, GroupNo, AxIP, AyIP, OffsetCx, OffsetCy, OffsetEx, OffsetEy, Dir, StrVel, MaxVel, Tacc, Tdec):
        print('_mnet_m1a_group_tr_arc_continuous(self, RingNo, GroupNo, AxIP, AyIP, OffsetCx, OffsetCy, OffsetEx, OffsetEy, Dir, StrVel, MaxVel, Tacc, Tdec): is not implement.')
        return 0

    def _mnet_m1a_group_ta_arc_continuous(self, RingNo, GroupNo, AxIP, AyIP, Cx, Cy, Ex, Ey, Dir, StrVel, MaxVel, Tacc, Tdec):
        print('_mnet_m1a_group_ta_arc_continuous(self, RingNo, GroupNo, AxIP, AyIP, Cx, Cy, Ex, Ey, Dir, StrVel, MaxVel, Tacc, Tdec): is not implement.')
        return 0

    def _mnet_m1a_group_sr_arc_continuous(self, RingNo, GroupNo, AxIP, AyIP, OffsetCx, OffsetCy, OffsetEx, OffsetEy, Dir, StrVel, MaxVel, Tacc, Tdec):
        print('_mnet_m1a_group_sr_arc_continuous(self, RingNo, GroupNo, AxIP, AyIP, OffsetCx, OffsetCy, OffsetEx, OffsetEy, Dir, StrVel, MaxVel, Tacc, Tdec): is not implement.')
        return 0

    def _mnet_m1a_group_sa_arc_continuous(self, RingNo, GroupNo, AxIP, AyIP, Cx, Cy, Ex, Ey, Dir, StrVel, MaxVel, Tacc, Tdec):
        print('_mnet_m1a_group_sa_arc_continuous(self, RingNo, GroupNo, AxIP, AyIP, Cx, Cy, Ex, Ey, Dir, StrVel, MaxVel, Tacc, Tdec): is not implement.')
        return 0

    def _mnet_m1a_group_tr_arc3(self, RingNo, GroupNo, AxIP, AyIP, AzIP, OffsetCx, OffsetCy, OffsetEx, OffsetEy, DistZ, Dir, StrVel, MaxVel, Tacc, Tdec):
        print('_mnet_m1a_group_tr_arc3(self, RingNo, GroupNo, AxIP, AyIP, AzIP, OffsetCx, OffsetCy, OffsetEx, OffsetEy, DistZ, Dir, StrVel, MaxVel, Tacc, Tdec): is not implement.')
        return 0

    def _mnet_m1a_group_ta_arc3(self, RingNo, GroupNo, AxIP, AyIP, AzIP, Cx, Cy, Ex, Ey, PosZ, Dir, StrVel, MaxVel, Tacc, Tdec):
        print('_mnet_m1a_group_ta_arc3(self, RingNo, GroupNo, AxIP, AyIP, AzIP, Cx, Cy, Ex, Ey, PosZ, Dir, StrVel, MaxVel, Tacc, Tdec): is not implement.')
        return 0

    def _mnet_m1a_group_sr_arc3(self, RingNo, GroupNo, AxIP, AyIP, AzIP, OffsetCx, OffsetCy, OffsetEx, OffsetEy, DistZ, Dir, StrVel, MaxVel, Tacc, Tdec):
        print('_mnet_m1a_group_sr_arc3(self, RingNo, GroupNo, AxIP, AyIP, AzIP, OffsetCx, OffsetCy, OffsetEx, OffsetEy, DistZ, Dir, StrVel, MaxVel, Tacc, Tdec): is not implement.')
        return 0

    def _mnet_m1a_group_sa_arc3(self, RingNo, GroupNo, AxIP, AyIP, AzIP, Cx, Cy, Ex, Ey, PosZ, Dir, StrVel, MaxVel, Tacc, Tdec):
        print('_mnet_m1a_group_sa_arc3(self, RingNo, GroupNo, AxIP, AyIP, AzIP, Cx, Cy, Ex, Ey, PosZ, Dir, StrVel, MaxVel, Tacc, Tdec): is not implement.')
        return 0

    def _mnet_m1a_group_tr_arc3_continuous(self, RingNo, GroupNo, AxIP, AyIP, AzIP, OffsetCx, OffsetCy, OffsetEx, OffsetEy, DistZ, Dir, StrVel, MaxVel, Tacc, Tdec):
        print('_mnet_m1a_group_tr_arc3_continuous(self, RingNo, GroupNo, AxIP, AyIP, AzIP, OffsetCx, OffsetCy, OffsetEx, OffsetEy, DistZ, Dir, StrVel, MaxVel, Tacc, Tdec): is not implement.')
        return 0

    def _mnet_m1a_group_ta_arc3_continuous(self, RingNo, GroupNo, AxIP, AyIP, AzIP, Cx, Cy, Ex, Ey, PosZ, Dir, StrVel, MaxVel, Tacc, Tdec):
        print('_mnet_m1a_group_ta_arc3_continuous(self, RingNo, GroupNo, AxIP, AyIP, AzIP, Cx, Cy, Ex, Ey, PosZ, Dir, StrVel, MaxVel, Tacc, Tdec): is not implement.')
        return 0

    def _mnet_m1a_group_sr_arc3_continuous(self, RingNo, GroupNo, AxIP, AyIP, AzIP, OffsetCx, OffsetCy, OffsetEx, OffsetEy, DistZ, Dir, StrVel, MaxVel, Tacc, Tdec):
        print('_mnet_m1a_group_sr_arc3_continuous(self, RingNo, GroupNo, AxIP, AyIP, AzIP, OffsetCx, OffsetCy, OffsetEx, OffsetEy, DistZ, Dir, StrVel, MaxVel, Tacc, Tdec): is not implement.')
        return 0

    def _mnet_m1a_group_sa_arc3_continuous(self, RingNo, GroupNo, AxIP, AyIP, AzIP, Cx, Cy, Ex, Ey, PosZ, Dir, StrVel, MaxVel, Tacc, Tdec):
        print('_mnet_m1a_group_sa_arc3_continuous(self, RingNo, GroupNo, AxIP, AyIP, AzIP, Cx, Cy, Ex, Ey, PosZ, Dir, StrVel, MaxVel, Tacc, Tdec): is not implement.')
        return 0

    def _mnet_m1a_group_set_holding_start(self, RingNo, GroupNo, Enabled):
        print('_mnet_m1a_group_set_holding_start(self, RingNo, GroupNo, Enabled): is not implement.')
        return 0

    def _mnet_m1a_group_start(self, RingNo, GroupNo):
        print('_mnet_m1a_group_start(self, RingNo, GroupNo): is not implement.')
        return 0

    def _mnet_m1a_group_emg_stop(self, RingNo, GroupNo):
        print('_mnet_m1a_group_emg_stop(self, RingNo, GroupNo): is not implement.')
        return 0

    def _mnet_m1a_group_imd_stop(self, RingNo, GroupNo):
        print('_mnet_m1a_group_imd_stop(self, RingNo, GroupNo): is not implement.')
        return 0

    def _mnet_m1a_group_sd_stop(self, RingNo, GroupNo):
        print('_mnet_m1a_group_sd_stop(self, RingNo, GroupNo): is not implement.')
        return 0

    def _mnet_m1a_get_hardware_info(self, RingNo, SlaveIP, VendorID, DeviceID):
        sendBuff = struct.pack("=2H", RingNo, SlaveIP)
        byteArr = array.array('B', sendBuff)

        req = MailBox.Request()
        req.buffer = byteArr

        future = self.cliM1aGetHwInfo.call_async(req)
        res = self.sendReqeust(future)
        if res == None:
            return -1

        recvBuff = res.buffer.tobytes()
        vid, did = struct.unpack("=IH", recvBuff[:struct.calcsize("=IH")])

        VendorID.clear()
        VendorID.append(vid)
        DeviceID.clear()
        DeviceID.append(did)
        return res.result_code

    def _mnet_m1a_recovery_from_EEPROM(self, RingNo, SlaveIP):
        print('_mnet_m1a_recovery_from_EEPROM(self, RingNo, SlaveIP): is not implement.')
        return 0

    def _mnet_m1a_backup_to_EEPROM(self, RingNo, SlaveIP):
        print('_mnet_m1a_backup_to_EEPROM(self, RingNo, SlaveIP): is not implement.')
        return 0

