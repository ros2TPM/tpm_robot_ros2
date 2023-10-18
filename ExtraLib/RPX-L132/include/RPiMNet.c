#include "RPiMNet.h"

    I16 rpi_mnet_open(void) { return -4; }
    I16 rpi_mnet_close(void) { return -4; }
    I16 rpi_mnet_reset_ring(U16 RingNo) { return -4; }
    I16 rpi_mnet_start_ring(U16 RingNo) { return -4; }
    I16 rpi_mnet_stop_ring(U16 RingNo) { return -4; }
    I16 rpi_mnet_set_ring_config(U16 RingNo, U16 BaudRate) { return -4; }
    I16 rpi_mnet_get_ring_count(U16 *Count) { return -4; }
    I16 rpi_mnet_get_ring_active_table(U16 RingNo, U32 DevTable[2]) { return -4; }
    I16 rpi_mnet_get_slave_type(U16 RingNo, U16 SlaveIP, U8 *SlaveType) { return -4; }
    I16 rpi_mnet_get_msg_slave_type(U16 RingNo, U16 SlaveIP, U16 *Type) { return -4; }
    I16 rpi_mnet_get_ring_status(U16 RingNo, U16 *Status) { return -4; }
    I16 rpi_mnet_get_com_status(U16 RingNo) { return -4; }
    I16 rpi_mnet_get_ring_error_counter(U16 RingNo, U16 *ErrCount) { return -4; }
    I16 rpi_mnet_reset_ring_error_counter(U16 RingNo) { return -4; }
    I16 rpi_mnet_get_communication_error_flag(U16 RingNo, U16 SlaveIP, U8 *IsError) { return -4; }
    I16 rpi_mnet_reset_communication_error_flag(U16 RingNo, U16 SlaveIP) { return -4; }
    I16 rpi_mnet_get_communication_error_table(U16 RingNo, U32 ErrTable[2]) { return -4; }
    I16 rpi_mnet_reset_communication_error_table(U16 RingNo) { return -4; }
    I16 rpi_mnet_set_io_error_check(U16 RingNo, U8 Enabled) { return -4; }
    I16 rpi_mnet_io_input(U16 RingNo, U16 SlaveIP, U8 PortNo) { return -4; }
    I16 rpi_mnet_io_output(U16 RingNo, U16 SlaveIP, U8 PortNo, U8 Val) { return -4; }
    I16 rpi_mnet_bit_io_input(U16 RingNo, U16 SlaveIP, U8 PortNo, U8 BitNo, U8 *OnOff) { return -4; }
    I16 rpi_mnet_bit_io_output(U16 RingNo, U16 SlaveIP, U8 PortNo, U8 BitNo, U8 OnOff) { return -4; }
    I16 rpi_mnet_bit_io_toggle(U16 RingNo, U16 SlaveIP, U8 PortNo, U8 BitNo) { return -4; }
    I16 rpi_mnet_a222_initial(U16 RingNo, U16 SlaveIP) { return -4; }
    I16 rpi_mnet_a222_get_fpga_version(U16 RingNo, U16 SlaveIP, U16 *Ver) { return -4; }
    I16 rpi_mnet_a222_get_adc_version(U16 RingNo, U16 SlaveIP, U16 *Ver) { return -4; }
    I16 rpi_mnet_a222_get_adc_error(U16 RingNo, U16 SlaveIP, U8 *Error) { return -4; }
    I16 rpi_mnet_a222_reset_adc(U16 RingNo, U16 SlaveIP) { return -4; }
    I16 rpi_mnet_a222_start_analog_input(U16 RingNo, U16 SlaveIP) { return -4; }
    I16 rpi_mnet_a222_stop_analog_input(U16 RingNo, U16 SlaveIP) { return -4; }
    I16 rpi_mnet_a222_set_analog_input_enable(U16 RingNo, U16 SlaveIP, U8 Enabled) { return -4; }
    I16 rpi_mnet_a222_set_analog_input_type(U16 RingNo, U16 SlaveIP, U8 Type) { return -4; }
    I16 rpi_mnet_a222_set_analog_input_gain(U16 RingNo, U16 SlaveIP, U8 Gain) { return -4; }
    I16 rpi_mnet_a222_get_analog_input(U16 RingNo, U16 SlaveIP, U16 ChannelNo, F64 *Data) { return -4; }
    I16 rpi_mnet_a222_get_analog_input_ex(U16 RingNo, U16 SlaveIP, U16 ChannelNo, F64 *Data, U16 *Val) { return -4; }
    I16 rpi_mnet_a222_get_analog_input_all(U16 RingNo, U16 SlaveIP, F64 Data[8]) { return -4; }
    I16 rpi_mnet_a222_get_analog_input_all_ex(U16 RingNo, U16 SlaveIP, F64 Data[8], U16 Val[8]) { return -4; }
    I16 rpi_mnet_a222_get_analog_input_value(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U16 *Val) { return -4; }
    I16 rpi_mnet_a222_get_analog_input_value_all(U16 RingNo, U16 SlaveIP, U16 Val[8]) { return -4; }
    I16 rpi_mnet_a222_set_analog_output(U16 RingNo, U16 SlaveIP, U16 ChannelNo, F64 Volt) { return -4; }
    I16 rpi_mnet_a222_set_analog_output_all(U16 RingNo, U16 SlaveIP, const F64 Volt[4]) { return -4; }
    I16 rpi_mnet_a222_set_analog_output_value(U16 RingNo, U16 SlaveIP, U16 ChannelNo, I16 Val) { return -4; }
    I16 rpi_mnet_a222_set_analog_output_value_all(U16 RingNo, U16 SlaveIP, const I16 Val[4]) { return -4; }
    I16 rpi_mnet_a322_initial(U16 RingNo, U16 SlaveIP) { return -4; }
    I16 rpi_mnet_a322_get_fpga_version(U16 RingNo, U16 SlaveIP, U16 *Ver) { return -4; }
    I16 rpi_mnet_a322_get_fw_version(U16 RingNo, U16 SlaveIP, U32 *Ver) { return -4; }
    I16 rpi_mnet_a322_start_analog_input(U16 RingNo, U16 SlaveIP) { return -4; }
    I16 rpi_mnet_a322_stop_analog_input(U16 RingNo, U16 SlaveIP) { return -4; }
    I16 rpi_mnet_a322_set_analog_input_enable(U16 RingNo, U16 SlaveIP, U8 EnabledChannels) { return -4; }
    I16 rpi_mnet_a322_get_analog_input_enable(U16 RingNo, U16 SlaveIP, U8 *EnabledChannels) { return -4; }
    I16 rpi_mnet_a322_set_analog_input_range(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U8 Range) { return -4; }
    I16 rpi_mnet_a322_get_analog_input_range(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U8 *Range) { return -4; }
    I16 rpi_mnet_a322_set_analog_input_average_cycles(U16 RingNo, U16 SlaveIP, U16 Cycles) { return -4; }
    I16 rpi_mnet_a322_get_analog_input_average_cycles(U16 RingNo, U16 SlaveIP, U16 *Cycles) { return -4; }
    I16 rpi_mnet_a322_get_analog_input_value(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U16 *Val) { return -4; }
    I16 rpi_mnet_a322_get_analog_input_value_all(U16 RingNo, U16 SlaveIP, U16 Val[8]) { return -4; }
    I16 rpi_mnet_a322_get_analog_input(U16 RingNo, U16 SlaveIP, U16 ChannelNo, F64 *Volt) { return -4; }
    I16 rpi_mnet_a322_get_analog_input_ex(U16 RingNo, U16 SlaveIP, U16 ChannelNo, F64 *Volt, U16 *Val) { return -4; }
    I16 rpi_mnet_a322_get_analog_input_all(U16 RingNo, U16 SlaveIP, F64 Volt[8]) { return -4; }
    I16 rpi_mnet_a322_get_analog_input_all_ex(U16 RingNo, U16 SlaveIP, F64 Volt[8], U16 Val[8]) { return -4; }
    I16 rpi_mnet_a322_set_analog_output_range(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U8 Range) { return -4; }
    I16 rpi_mnet_a322_get_analog_output_range(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U8 *Range) { return -4; }
    I16 rpi_mnet_a322_set_analog_output_value(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U16 Val) { return -4; }
    I16 rpi_mnet_a322_set_analog_output_value_all(U16 RingNo, U16 SlaveIP, const U16 Val[4]) { return -4; }
    I16 rpi_mnet_a322_set_analog_output(U16 RingNo, U16 SlaveIP, U16 ChannelNo, F64 Volt) { return -4; }
    I16 rpi_mnet_a322_set_analog_output_ex(U16 RingNo, U16 SlaveIP, U16 ChannelNo, F64 Volt, U16 *Val) { return -4; }
    I16 rpi_mnet_a322_set_analog_output_all(U16 RingNo, U16 SlaveIP, const F64 Volt[4]) { return -4; }
    I16 rpi_mnet_a322_set_analog_output_all_ex(U16 RingNo, U16 SlaveIP, const F64 Volt[4], U16 Val[4]) { return -4; }
    I16 rpi_mnet_a204_initial(U16 RingNo, U16 SlaveIP) { return -4; }
    I16 rpi_mnet_a204_get_firmware_version(U16 RingNo, U16 SlaveIP, U16 *Ver) { return -4; }
    I16 rpi_mnet_a204_set_analog_output(U16 RingNo, U16 SlaveIP, U16 ChannelNo, F64 Volt) { return -4; }
    I16 rpi_mnet_a204_set_analog_output_ex(U16 RingNo, U16 SlaveIP, U16 ChannelNo, F64 Volt, I16 *Val) { return -4; }
    I16 rpi_mnet_a204_set_analog_output_all(U16 RingNo, U16 SlaveIP, const F64 Volt[4]) { return -4; }
    I16 rpi_mnet_a204_set_analog_output_all_ex(U16 RingNo, U16 SlaveIP, const F64 Volt[4], I16 Val[4]) { return -4; }
    I16 rpi_mnet_a204_set_analog_output_value(U16 RingNo, U16 SlaveIP, U16 ChannelNo, I16 Val) { return -4; }
    I16 rpi_mnet_a204_set_analog_output_value_all(U16 RingNo, U16 SlaveIP, const I16 Val[4]) { return -4; }
    I16 rpi_mnet_ai8_initial(U16 RingNo, U16 SlaveIP) { return -4; }
    I16 rpi_mnet_ai8_get_hardware_info(U16 RingNo, U16 SlaveIP, U8 *DeviceID, U8 *VHDL_Version) { return -4; }
    I16 rpi_mnet_ai8_set_cycle_time(U16 RingNo, U16 SlaveIP, U8 SetValue) { return -4; }
    I16 rpi_mnet_ai8_enable_device(U16 RingNo, U16 SlaveIP, U8 EnableDevice) { return -4; }
    I16 rpi_mnet_ai8_enable_channel(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U8 Enable) { return -4; }
    I16 rpi_mnet_ai8_set_channel_gain(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U8 Gain) { return -4; }
    I16 rpi_mnet_ai8_get_value(U16 RingNo, U16 SlaveIP, U16 ChannelNo, I16 *Value) { return -4; }
    I16 rpi_mnet_ai8_get_value_all(U16 RingNo, U16 SlaveIP, I16 Value[8]) { return -4; }
    I16 rpi_mnet_ai8_get_voltage(U16 RingNo, U16 SlaveIP, U16 ChannelNo, F64 *Voltage) { return -4; }
    I16 rpi_mnet_ai8_get_voltage_all(U16 RingNo, U16 SlaveIP, F64 Voltage[8]) { return -4; }
    I16 rpi_mnet_ao4_initial(U16 RingNo, U16 SlaveIP) { return -4; }
    I16 rpi_mnet_ao4_get_hardware_info(U16 RingNo, U16 SlaveIP, U8 *DeviceID, U8 *VHDL_Version) { return -4; }
    I16 rpi_mnet_ao4_reset_DAC(U16 RingNo, U16 SlaveIP) { return -4; }
    I16 rpi_mnet_ao4_clear_output_all(U16 RingNo, U16 SlaveIP) { return -4; }
    I16 rpi_mnet_ao4_set_output(U16 RingNo, U16 SlaveIP, U8 ChannelNo, I16 SetValue) { return -4; }
    I16 rpi_mnet_ao4_set_voltage(U16 RingNo, U16 SlaveIP, U8 ChannelNo, F64 Voltage) { return -4; }
    I16 rpi_mnet_ao4_set_voltage1(U16 RingNo, U16 SlaveIP, U8 ChannelNo, F64 Voltage, I16 *Value) { return -4; }
    I16 rpi_mnet_ao4_set_output_all(U16 RingNo, U16 SlaveIP, I16 SetValue1, I16 SetValue2, I16 SetValue3, I16 SetValue4) { return -4; }
    I16 rpi_mnet_ao4_set_voltage_all(U16 RingNo, U16 SlaveIP, F64 Voltage1, F64 Voltage2, F64 Voltage3, F64 Voltage4) { return -4; }
    I16 rpi_mnet_ao4_set_voltage_all1(U16 RingNo, U16 SlaveIP, F64 Voltage1, F64 Voltage2, F64 Voltage3, F64 Voltage4, I16 *Value1, I16 *Value2, I16 *Value3, I16 *Value4) { return -4; }
    I16 rpi_mnet_ao4_set_coarse_gain(U16 RingNo, U16 SlaveIP, U8 ChannelNo, I16 SetValue) { return -4; }
    I16 rpi_mnet_c144_initial(U16 RingNo, U16 SlaveIP) { return -4; }
    I16 rpi_mnet_c144_get_firmware_version(U16 RingNo, U16 SlaveIP, U16 *Ver) { return -4; }
    I16 rpi_mnet_c144_get_do(U16 RingNo, U16 SlaveIP, U8 *Val) { return -4; }
    I16 rpi_mnet_c144_set_do(U16 RingNo, U16 SlaveIP, U8 Val) { return -4; }
    I16 rpi_mnet_c144_set_encoder_mode(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U16 Mode) { return -4; }
    I16 rpi_mnet_c144_set_pls_iptmode(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U16 PlsInMode, U16 PlsInDir) { return -4; }
    I16 rpi_mnet_c144_get_encoder(U16 RingNo, U16 SlaveIP, U16 ChannelNo, I32 *EncPos) { return -4; }
    I16 rpi_mnet_c144_get_encoder_all(U16 RingNo, U16 SlaveIP, I32 EncPos[4]) { return -4; }
    I16 rpi_mnet_c144_set_encoder(U16 RingNo, U16 SlaveIP, U16 ChannelNo, I32 EncPos) { return -4; }
    I16 rpi_mnet_c144_set_auto_compare_source(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U16 SrcEncNo) { return -4; }
    I16 rpi_mnet_c144_get_auto_compare_count(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U16 *Count) { return -4; }
    I16 rpi_mnet_c144_get_auto_compare_count_all(U16 RingNo, U16 SlaveIP, U16 Count[4]) { return -4; }
    I16 rpi_mnet_c144_get_auto_compare_count_ex(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U32 *Count) { return -4; }
    I16 rpi_mnet_c144_get_auto_compare_count_all_ex(U16 RingNo, U16 SlaveIP, U32 Count[4]) { return -4; }
    I16 rpi_mnet_c144_get_auto_compare_status(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U16 *OnOff) { return -4; }
    I16 rpi_mnet_c144_get_auto_compare_status_all(U16 RingNo, U16 SlaveIP, U16 OnOff[4]) { return -4; }
    I16 rpi_mnet_c144_set_auto_compare_trigger(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U16 Level, U16 Width) { return -4; }
    I16 rpi_mnet_c144_set_auto_compare_function(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U8 Dir, I32 StrPos, I32 Interval, U16 TrgCnt) { return -4; }
    I16 rpi_mnet_c144_set_auto_compare_function_ex(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U8 Dir, I32 StrPos, I32 Interval, U32 TrgCnt) { return -4; }
    I16 rpi_mnet_c144_set_auto_compare_table(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U8 Dir, U16 Size, const I32 *Table) { return -4; }
    I16 rpi_mnet_c144_get_auto_compare_table_capacity(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U16 *Size) { return -4; }
    I16 rpi_mnet_c144_start_auto_compare(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U16 OnOff) { return -4; }
    I16 rpi_mnet_c144_force_trigger_output(U16 RingNo, U16 SlaveIP, U16 ChannelNo) { return -4; }
    I16 rpi_mnet_p144_initial(U16 RingNo, U16 SlaveIP) { return -4; }
    I16 rpi_mnet_p144_get_firmware_version(U16 RingNo, U16 SlaveIP, U16 *Ver) { return -4; }
    I16 rpi_mnet_p144_set_count(U16 RingNo, U16 SlaveIP, U16 ChannelNo, I32 Count) { return -4; }
    I16 rpi_mnet_p144_get_count(U16 RingNo, U16 SlaveIP, U16 ChannelNo, I32 *Count) { return -4; }
    I16 rpi_mnet_p144_get_count_all(U16 RingNo, U16 SlaveIP, I32 Count[4]) { return -4; }
    I16 rpi_mnet_p144_get_frequency(U16 RingNo, U16 SlaveIP, U16 ChannelNo, F64 *Freq) { return -4; }
    I16 rpi_mnet_p144_get_frequency_all(U16 RingNo, U16 SlaveIP, F64 Freq[4]) { return -4; }
    I16 rpi_mnet_p144_set_pulse_level(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U8 Level) { return -4; }
    I16 rpi_mnet_p144_get_pulse_width(U16 RingNo, U16 SlaveIP, U16 ChannelNo, F64 *Width) { return -4; }
    I16 rpi_mnet_p144_get_pulse_width_all(U16 RingNo, U16 SlaveIP, F64 Width[4]) { return -4; }
    I16 rpi_mnet_p144_set_filter(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U16 Filter) { return -4; }
    I16 rpi_mnet_p144_get_do(U16 RingNo, U16 SlaveIP, U8 *Val) { return -4; }
    I16 rpi_mnet_p144_set_do_bit(U16 RingNo, U16 SlaveIP, U16 BitNo, U8 Mode, I32 Val) { return -4; }
    I16 rpi_mnet_m1a_initial(U16 RingNo, U16 SlaveIP) { return -4; }
    I16 rpi_mnet_m1a_set_pls_outmode(U16 RingNo, U16 SlaveIP, U16 pls_outmode) { return -4; }
    I16 rpi_mnet_m1a_set_pls_iptmode(U16 RingNo, U16 SlaveIP, U16 pls_iptmode, U16 pls_iptdir) { return -4; }
    I16 rpi_mnet_m1a_set_feedback_src(U16 RingNo, U16 SlaveIP, U16 FbkSrc) { return -4; }
    I16 rpi_mnet_m1a_set_abs_reference(U16 RingNo, U16 SlaveIP, U16 Ref) { return -4; }
    I16 rpi_mnet_m1a_set_abs_ref_encoder_ignore_backlash(U16 RingNo, U16 SlaveIP, U8 OnOff) { return -4; }
    I16 rpi_mnet_m1a_set_direction_change_delay(U16 RingNo, U16 SlaveIP, U8 Enabled) { return -4; }
    I16 rpi_mnet_m1a_set_move_ratio(U16 RingNo, U16 SlaveIP, F32 MoveRatio) { return -4; }
    I16 rpi_mnet_m1a_set_alm(U16 RingNo, U16 SlaveIP, U16 alm_logic, U16 alm_mode) { return -4; }
    I16 rpi_mnet_m1a_set_inp(U16 RingNo, U16 SlaveIP, U16 inp_enable, U16 inp_logic) { return -4; }
    I16 rpi_mnet_m1a_set_el_mode(U16 RingNo, U16 SlaveIP, U16 el_mode) { return -4; }
    I16 rpi_mnet_m1a_set_sd(U16 RingNo, U16 SlaveIP, I16 sd_enable, I16 sd_logic, I16 sd_latch, I16 sd_mode) { return -4; }
    I16 rpi_mnet_m1a_set_erc(U16 RingNo, U16 SlaveIP, U16 erc_logic, U16 erc_on_time, U16 erc_off_time) { return -4; }
    I16 rpi_mnet_m1a_set_erc_on(U16 RingNo, U16 SlaveIP, I16 on_off) { return -4; }
    I16 rpi_mnet_m1a_set_auto_erc(U16 RingNo, U16 SlaveIP, I16 Enable) { return -4; }
    I16 rpi_mnet_m1a_set_svon(U16 RingNo, U16 SlaveIP, U16 on_off) { return -4; }
    I16 rpi_mnet_m1a_set_ralm(U16 RingNo, U16 SlaveIP, U16 on_off) { return -4; }
    I16 rpi_mnet_m1a_set_ltc_logic(U16 RingNo, U16 SlaveIP, U16 ltc_logic) { return -4; }
    I16 rpi_mnet_m1a_set_mechanical_input_filter(U16 RingNo, U16 SlaveIP, U16 on_off) { return -4; }
    I16 rpi_mnet_m1a_set_backlash(U16 RingNo, U16 SlaveIP, U16 Value, U16 Enable, U16 CntSrc) { return -4; }
    I16 rpi_mnet_m1a_dio_output(U16 RingNo, U16 SlaveIP, U16 DoNo, U16 on_off) { return -4; }
    I16 rpi_mnet_m1a_dio_input(U16 RingNo, U16 SlaveIP, U16 DiNo) { return -4; }
    I16 rpi_mnet_m1a_get_dio(U16 RingNo, U16 SlaveIP, U8 *Val) { return -4; }
    I16 rpi_mnet_m1a_get_io_status(U16 RingNo, U16 SlaveIP, U32 *IO_status) { return -4; }
    I16 rpi_mnet_m1a_load_motion_file(U16 RingNo, U16 SlaveIP, const char *FilePath) { return -4; }
    I16 rpi_mnet_m1a_get_command(U16 RingNo, U16 SlaveIP, I32 *Cmd) { return -4; }
    I16 rpi_mnet_m1a_set_command(U16 RingNo, U16 SlaveIP, I32 Cmd) { return -4; }
    I16 rpi_mnet_m1a_reset_command(U16 RingNo, U16 SlaveIP) { return -4; }
    I16 rpi_mnet_m1a_get_position(U16 RingNo, U16 SlaveIP, I32 *Pos) { return -4; }
    I16 rpi_mnet_m1a_set_position(U16 RingNo, U16 SlaveIP, I32 Pos) { return -4; }
    I16 rpi_mnet_m1a_reset_position(U16 RingNo, U16 SlaveIP) { return -4; }
    I16 rpi_mnet_m1a_get_error_counter(U16 RingNo, U16 SlaveIP, I32 *ErrCnt) { return -4; }
    I16 rpi_mnet_m1a_set_error_counter(U16 RingNo, U16 SlaveIP, I32 ErrCnt) { return -4; }
    I16 rpi_mnet_m1a_reset_error_counter(U16 RingNo, U16 SlaveIP) { return -4; }
    I16 rpi_mnet_m1a_get_rest_command(U16 RingNo, U16 SlaveIP, I32 *Cmd) { return -4; }
    I16 rpi_mnet_m1a_enable_command_ring_counter(U16 RingNo, U16 SlaveIP, U32 RingCounter) { return -4; }
    I16 rpi_mnet_m1a_disable_command_ring_counter(U16 RingNo, U16 SlaveIP) { return -4; }
    I16 rpi_mnet_m1a_enable_position_ring_counter(U16 RingNo, U16 SlaveIP, U32 RingCounter) { return -4; }
    I16 rpi_mnet_m1a_disable_position_ring_counter(U16 RingNo, U16 SlaveIP) { return -4; }
    I16 rpi_mnet_m1a_get_current_speed(U16 RingNo, U16 SlaveIP, U32 *Speed) { return -4; }
    I16 rpi_mnet_m1a_get_command_without_backlash(U16 RingNo, U16 SlaveIP, I32 *Cmd) { return -4; }
    I16 rpi_mnet_m1a_get_position_without_backlash(U16 RingNo, U16 SlaveIP, I32 *Pos) { return -4; }
    I16 rpi_mnet_m1a_set_tmove_speed(U16 RingNo, U16 SlaveIP, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec) { return -4; }
    I16 rpi_mnet_m1a_set_smove_speed(U16 RingNo, U16 SlaveIP, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec) { return -4; }
    I16 rpi_mnet_m1a_set_smove_speed_ex(U16 RingNo, U16 SlaveIP, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec, U32 SVacc, U32 SVdec) { return -4; }
    I16 rpi_mnet_m1a_start_r_move(U16 RingNo, U16 SlaveIP, I32 Dist) { return -4; }
    I16 rpi_mnet_m1a_start_a_move(U16 RingNo, U16 SlaveIP, I32 Pos) { return -4; }
    I16 rpi_mnet_m1a_start_tr_move(U16 RingNo, U16 SlaveIP, I32 Dist, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec) { return -4; }
    I16 rpi_mnet_m1a_start_ta_move(U16 RingNo, U16 SlaveIP, I32 Pos, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec) { return -4; }
    I16 rpi_mnet_m1a_start_sr_move(U16 RingNo, U16 SlaveIP, I32 Dist, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec) { return -4; }
    I16 rpi_mnet_m1a_start_sa_move(U16 RingNo, U16 SlaveIP, I32 Pos, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec) { return -4; }
    I16 rpi_mnet_m1a_v_move(U16 RingNo, U16 SlaveIP, U8 Dir) { return -4; }
    I16 rpi_mnet_m1a_tv_move(U16 RingNo, U16 SlaveIP, U8 Dir, U32 StrVel, U32 MaxVel, F32 Tacc) { return -4; }
    I16 rpi_mnet_m1a_sv_move(U16 RingNo, U16 SlaveIP, U8 Dir, U32 StrVel, U32 MaxVel, F32 Tacc) { return -4; }
    I16 rpi_mnet_m1a_sd_stop(U16 RingNo, U16 SlaveIP) { return -4; }
    I16 rpi_mnet_m1a_sd_stop_ex(U16 RingNo, U16 SlaveIP, F32 Tdec) { return -4; }
    I16 rpi_mnet_m1a_imd_stop(U16 RingNo, U16 SlaveIP) { return -4; }
    I16 rpi_mnet_m1a_emg_stop(U16 RingNo, U16 SlaveIP) { return -4; }
    I16 rpi_mnet_m1a_motion_done(U16 RingNo, U16 SlaveIP, U16 *MoSt) { return -4; }
    I16 rpi_mnet_m1a_triangle_correction(U16 RingNo, U16 SlaveIP, U16 OnOff) { return -4; }
    I16 rpi_mnet_m1a_set_home_config(U16 RingNo, U16 SlaveIP, U16 home_mode, U16 org_logic, U16 ez_logic, U16 ez_count, U16 erc_out) { return -4; }
    I16 rpi_mnet_m1a_start_home_move(U16 RingNo, U16 SlaveIP, U8 Dir) { return -4; }
    I16 rpi_mnet_m1a_start_home_search(U16 RingNo, U16 SlaveIP, U8 Dir, I32 ORGOffset) { return -4; }
    I16 rpi_mnet_m1a_start_home_escape(U16 RingNo, U16 SlaveIP, U8 Dir) { return -4; }
    I16 rpi_mnet_m1a_start_search_ez(U16 RingNo, U16 SlaveIP, U8 Dir, U16 EzCount) { return -4; }
    I16 rpi_mnet_m1a_home_move(U16 RingNo, U16 SlaveIP, U8 Dir, U32 StrVel, U32 MaxVel, F32 Tacc) { return -4; }
    I16 rpi_mnet_m1a_home_search(U16 RingNo, U16 SlaveIP, U8 Dir, U32 StrVel, U32 MaxVel, F32 Tacc, I32 ORGOffset) { return -4; }
    I16 rpi_mnet_m1a_home_escape(U16 RingNo, U16 SlaveIP, U8 Dir, U32 StrVel, U32 MaxVel, F32 Tacc) { return -4; }
    I16 rpi_mnet_m1a_search_ez(U16 RingNo, U16 SlaveIP, U8 Dir, U16 EzCount, U32 StrVel, U32 MaxVel, F32 Tacc) { return -4; }
    I16 rpi_mnet_m1a_set_comparator_mode(U16 RingNo, U16 SlaveIP, I16 CompNo, I16 CmpSrc, I16 CmpMethod, I16 CmpAction) { return -4; }
    I16 rpi_mnet_m1a_set_comparator_data(U16 RingNo, U16 SlaveIP, I16 CompNo, I32 Pos) { return -4; }
    I16 rpi_mnet_m1a_set_trigger_comparator(U16 RingNo, U16 SlaveIP, U16 CmpSrc, U16 CmpMethod) { return -4; }
    I16 rpi_mnet_m1a_set_trigger_comparator_data(U16 RingNo, U16 SlaveIP, I32 Data) { return -4; }
    I16 rpi_mnet_m1a_set_auto_trigger(U16 RingNo, U16 SlaveIP, U16 CmpSrc, U16 CmpMethod, U16 Interval, U16 on_off) { return -4; }
    I16 rpi_mnet_m1a_get_comparator_data(U16 RingNo, U16 SlaveIP, I16 CompNo, I32 *Pos) { return -4; }
    I16 rpi_mnet_m1a_set_soft_limit(U16 RingNo, U16 SlaveIP, I32 PLimit, I32 MLimit) { return -4; }
    I16 rpi_mnet_m1a_enable_soft_limit(U16 RingNo, U16 SlaveIP, U8 Action) { return -4; }
    I16 rpi_mnet_m1a_disable_soft_limit(U16 RingNo, U16 SlaveIP) { return -4; }
    I16 rpi_mnet_m1a_set_latch_signal(U16 RingNo, U16 SlaveIP, U8 LtcSignal) { return -4; }
    I16 rpi_mnet_m1a_get_latch_data(U16 RingNo, U16 SlaveIP, I16 LatchNo, I32 *Pos) { return -4; }
    I16 rpi_mnet_m1a_v_change(U16 RingNo, U16 SlaveIP, U32 NewVel, F32 Time) { return -4; }
    I16 rpi_mnet_m1a_fix_speed_range(U16 RingNo, U16 SlaveIP, U32 MaxVel) { return -4; }
    I16 rpi_mnet_m1a_unfix_speed_range(U16 RingNo, U16 SlaveIP) { return -4; }
    I16 rpi_mnet_m1a_get_speed_range(U16 RingNo, U16 SlaveIP, U32 *MinVel, U32 *MaxVel) { return -4; }
    I16 rpi_mnet_m1a_start_p_change(U16 RingNo, U16 SlaveIP, I32 NewPos) { return -4; }
    I16 rpi_mnet_m1a_start_d_change(U16 RingNo, U16 SlaveIP, I32 NewDist) { return -4; }
    I16 rpi_mnet_m1a_get_preregister_status(U16 RingNo, U16 SlaveIP, U8* Full) { return -4; }
    I16 rpi_mnet_m1a_get_preregister_depth(U16 RingNo, U16 SlaveIP, U8* Depth) { return -4; }
    I16 rpi_mnet_m1a_cancel_preregister(U16 RingNo, U16 SlaveIP) { return -4; }
    I16 rpi_mnet_m1a_shift_preregister(U16 RingNo, U16 SlaveIP) { return -4; }
    I16 rpi_mnet_m1a_check_int(U16 RingNo, U16 SlaveIP, U8* IntStatus) { return -4; }
    I16 rpi_mnet_m1a_check_error(U16 RingNo, U16 SlaveIP, U32* ErrorStatus) { return -4; }
    I16 rpi_mnet_m1a_check_event(U16 RingNo, U16 SlaveIP, U32* EventStatus) { return -4; }
    I16 rpi_mnet_m1a_set_int_factor(U16 RingNo, U16 SlaveIP, U32 int_factor) { return -4; }
    I16 rpi_mnet_m1a_set_group(U16 RingNo, U16 GroupNo, const U16* AxisArray, U16 AxisCount) { return -4; }
    I16 rpi_mnet_m1a_group_reset_command(U16 RingNo, U16 GroupNo) { return -4; }
    I16 rpi_mnet_m1a_group_reset_position(U16 RingNo, U16 GroupNo) { return -4; }
    I16 rpi_mnet_m1a_group_continuous_move(U16 RingNo, U16 GroupNo, U16 Enable) { return -4; }
    I16 rpi_mnet_m1a_group_tr_lineN(U16 RingNo, U16 GroupNo, const I32* DistArray, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec) { return -4; }
    I16 rpi_mnet_m1a_group_ta_lineN(U16 RingNo, U16 GroupNo, const I32* PosArray, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec) { return -4; }
    I16 rpi_mnet_m1a_group_sr_lineN(U16 RingNo, U16 GroupNo, const I32* DistArray, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec) { return -4; }
    I16 rpi_mnet_m1a_group_sa_lineN(U16 RingNo, U16 GroupNo, const I32* PosArray, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec) { return -4; }
    I16 rpi_mnet_m1a_group_tr_lineN_continuous(U16 RingNo, U16 GroupNo, const I32* DistArray, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec) { return -4; }
    I16 rpi_mnet_m1a_group_ta_lineN_continuous(U16 RingNo, U16 GroupNo, const I32* PosArray, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec) { return -4; }
    I16 rpi_mnet_m1a_group_sr_lineN_continuous(U16 RingNo, U16 GroupNo, const I32* DistArray, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec) { return -4; }
    I16 rpi_mnet_m1a_group_sa_lineN_continuous(U16 RingNo, U16 GroupNo, const I32* PosArray, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec) { return -4; }
    I16 rpi_mnet_m1a_group_tr_arc(U16 RingNo, U16 GroupNo, U16 AxIP, U16 AyIP, I32 OffsetCx, I32 OffsetCy, I32 OffsetEx, I32 OffsetEy, I16 Dir, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec) { return -4; }
    I16 rpi_mnet_m1a_group_ta_arc(U16 RingNo, U16 GroupNo, U16 AxIP, U16 AyIP, I32 Cx, I32 Cy, I32 Ex, I32 Ey, I16 Dir, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec) { return -4; }
    I16 rpi_mnet_m1a_group_sr_arc(U16 RingNo, U16 GroupNo, U16 AxIP, U16 AyIP, I32 OffsetCx, I32 OffsetCy, I32 OffsetEx, I32 OffsetEy, I16 Dir, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec) { return -4; }
    I16 rpi_mnet_m1a_group_sa_arc(U16 RingNo, U16 GroupNo, U16 AxIP, U16 AyIP, I32 Cx, I32 Cy, I32 Ex, I32 Ey, I16 Dir, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec) { return -4; }
    I16 rpi_mnet_m1a_group_tr_arc_continuous(U16 RingNo, U16 GroupNo, U16 AxIP, U16 AyIP, I32 OffsetCx, I32 OffsetCy, I32 OffsetEx, I32 OffsetEy, I16 Dir, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec) { return -4; }
    I16 rpi_mnet_m1a_group_ta_arc_continuous(U16 RingNo, U16 GroupNo, U16 AxIP, U16 AyIP, I32 Cx, I32 Cy, I32 Ex, I32 Ey, I16 Dir, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec) { return -4; }
    I16 rpi_mnet_m1a_group_sr_arc_continuous(U16 RingNo, U16 GroupNo, U16 AxIP, U16 AyIP, I32 OffsetCx, I32 OffsetCy, I32 OffsetEx, I32 OffsetEy, I16 Dir, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec) { return -4; }
    I16 rpi_mnet_m1a_group_sa_arc_continuous(U16 RingNo, U16 GroupNo, U16 AxIP, U16 AyIP, I32 Cx, I32 Cy, I32 Ex, I32 Ey, I16 Dir, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec) { return -4; }
    I16 rpi_mnet_m1a_group_tr_arc3(U16 RingNo, U16 GroupNo, U16 AxIP, U16 AyIP, U16 AzIP, I32 OffsetCx, I32 OffsetCy, I32 OffsetEx, I32 OffsetEy, I32 DistZ, I16 Dir, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec) { return -4; }
    I16 rpi_mnet_m1a_group_ta_arc3(U16 RingNo, U16 GroupNo, U16 AxIP, U16 AyIP, U16 AzIP, I32 Cx, I32 Cy, I32 Ex, I32 Ey, I32 PosZ, I16 Dir, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec) { return -4; }
    I16 rpi_mnet_m1a_group_sr_arc3(U16 RingNo, U16 GroupNo, U16 AxIP, U16 AyIP, U16 AzIP, I32 OffsetCx, I32 OffsetCy, I32 OffsetEx, I32 OffsetEy, I32 DistZ, I16 Dir, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec) { return -4; }
    I16 rpi_mnet_m1a_group_sa_arc3(U16 RingNo, U16 GroupNo, U16 AxIP, U16 AyIP, U16 AzIP, I32 Cx, I32 Cy, I32 Ex, I32 Ey, I32 PosZ, I16 Dir, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec) { return -4; }
    I16 rpi_mnet_m1a_group_tr_arc3_continuous(U16 RingNo, U16 GroupNo, U16 AxIP, U16 AyIP, U16 AzIP, I32 OffsetCx, I32 OffsetCy, I32 OffsetEx, I32 OffsetEy, I32 DistZ, I16 Dir, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec) { return -4; }
    I16 rpi_mnet_m1a_group_ta_arc3_continuous(U16 RingNo, U16 GroupNo, U16 AxIP, U16 AyIP, U16 AzIP, I32 Cx, I32 Cy, I32 Ex, I32 Ey, I32 PosZ, I16 Dir, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec) { return -4; }
    I16 rpi_mnet_m1a_group_sr_arc3_continuous(U16 RingNo, U16 GroupNo, U16 AxIP, U16 AyIP, U16 AzIP, I32 OffsetCx, I32 OffsetCy, I32 OffsetEx, I32 OffsetEy, I32 DistZ, I16 Dir, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec) { return -4; }
    I16 rpi_mnet_m1a_group_sa_arc3_continuous(U16 RingNo, U16 GroupNo, U16 AxIP, U16 AyIP, U16 AzIP, I32 Cx, I32 Cy, I32 Ex, I32 Ey, I32 PosZ, I16 Dir, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec) { return -4; }
    I16 rpi_mnet_m1a_group_set_holding_start(U16 RingNo, U16 GroupNo, U8 Enabled) { return -4; }
    I16 rpi_mnet_m1a_group_start(U16 RingNo, U16 GroupNo) { return -4; }
    I16 rpi_mnet_m1a_group_emg_stop(U16 RingNo, U16 GroupNo) { return -4; }
    I16 rpi_mnet_m1a_group_imd_stop(U16 RingNo, U16 GroupNo) { return -4; }
    I16 rpi_mnet_m1a_group_sd_stop(U16 RingNo, U16 GroupNo) { return -4; }
    I16 rpi_mnet_m1a_get_hardware_info(U16 RingNo, U16 SlaveIP, U32* VendorID, U16* DeviceID) { return -4; }
    I16 rpi_mnet_m1a_recovery_from_EEPROM(U16 RingNo, U16 SlaveIP) { return -4; }
    I16 rpi_mnet_m1a_backup_to_EEPROM(U16 RingNo, U16 SlaveIP) { return -4; }