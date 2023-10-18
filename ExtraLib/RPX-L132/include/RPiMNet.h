#ifndef RPIMNET_H_
#define RPIMNET_H_

#include "type_def.h"

////////////////////////////////////////////////////////////////////////////////
//                                Error Code                                  //
////////////////////////////////////////////////////////////////////////////////
// Motionnet Error Codes
#define ERR_NoError                            0
#define ERR_MNET_Ring_Used                    -1
#define ERR_Invalid_Ring                      -2
#define ERR_Invalid_Slave                     -3
#define ERR_Invalid_Hardware                  -4
#define ERR_Invalid_Sync_Object               -5
#define ERR_RingCommunicationError            -8
#define ERR_RingCommunicationTimeOut          -9
#define ERR_DataFifoFull                     -10
#define ERR_DataCommunicationFailed          -11
#define ERR_InvalidMaxVel                    -16
#define ERR_ObsoleteFunction                 -34
#define ERR_G94_RECEIVE_TimeOut              -36
#define ERR_G94_CPURead                      -37
#define ERR_G94_MsgCommunication             -38
#define ERR_G94_MsgSize                      -39
#define ERR_G94_CommTimeOut                  -40
#define ERR_InvalidAxisNo                    -50
#define ERR_InvalidParameter_1               -51
#define ERR_InvalidParameter_2               -52
#define ERR_InvalidParameter_3               -53
#define ERR_InvalidParameter_4               -54
#define ERR_InvalidParameter_5               -55
#define ERR_InvalidParameter_6               -56
#define ERR_InvalidParameter_7               -57
#define ERR_InvalidParameter_8               -58
#define ERR_InvalidParameter_9               -59
#define ERR_InvalidParameter_10              -60
#define ERR_InvalidParameter_11              -61
#define ERR_InvalidParameter_12              -62
#define ERR_InvalidParameter_13              -63
#define ERR_FailGetDeviceTable               -74
#define ERR_NoDeviceFound                    -75
#define ERR_SlowDownPointError              -101
#define ERR_M1A_InvalidCheckCode            -103
#define ERR_M1A_NotInitializedYet           -105
#define ERR_AI8_RegisterWriteFail           -111
#define ERR_AI8_RegisterReadFail            -112
#define ERR_AI8_DeviceTypeNotCorrect        -113
#define ERR_AI8_IncorrectChannelNo          -114
#define ERR_AI8_IncorrectValueOfGain        -115
#define ERR_AI8_DeviceNotEnable             -116
#define ERR_AI8_ChannelNotEnable            -117
#define ERR_AI8_MOSI_Busy                   -118
#define ERR_AI8_MISO_Busy                   -119
#define ERR_AI8_Read_EEPROM                 -120
#define ERR_AI8_AdjustFail                  -121
#define ERR_AO4_RegisterWriteFail           -131
#define ERR_AO4_RegisterReadFail            -132
#define ERR_AO4_RegisterReadTimeout         -133
#define ERR_AO4_DeviceTypeNotCorrect        -134
#define ERR_AO4_ChannelNoNotCorrect         -135
#define ERR_AO4_OutputValueNotCorrect       -136
#define ERR_AO4_CoarseGainValueNotCorrect   -137
#define ERR_AO4_FineGainValueNotCorrect     -138
#define ERR_AO4_OffsetValueNotCorrect       -139
#define ERR_AO4_OutputVoltageNotCorrect     -140
#define ERR_AO4_FlashReadFail               -141
#define ERR_AO4_FlashWriteFail              -142
#define ERR_AO4_FlashEraseFail              -143
#define ERR_FunctionNotSupport              -151
#define ERR_InvalidDeviceType               -152
#define ERR_GetDLLPath                      -170
#define ERR_GetDLLVersion                   -171
#define ERR_LoadFileFailed                  -172
#define ERR_AutoCmpSetFifoFailed            -173
#define ERR_InvalidOperation                -174
#define ERR_OperationFailed                 -175

////////////////////////////////////////////////////////////////////////////////
//                           Slave Type Definition                            //
////////////////////////////////////////////////////////////////////////////////
typedef enum {
    // Axis
    G9103_M1X1 = 0xA2,
    G9003_M1X1 = 0xA3,
    G9004_M304T = 0xA8,
    G9004_M4A2 = 0xA9,
    // DIO (32 In/Out)
    G9002_Q32 = 0xB0,
    G9002_I16Q16 = 0xB2,
    G9002_I32 = 0xB4,
    // DIO (64 In/Out)
    G9004_I64 = 0xBC,
    G9004_I32Q32 = 0xBD,
    G9004_Q64 = 0xBE,
    // DIO (96 In/Out)
    G9004_I96 = 0xB5,
    G9004_I80Q16 = 0xB6,
    G9004_I64Q32 = 0xB7,
    G9004_I48Q48 = 0xB8,
    G9004_I32Q64 = 0xB9,
    G9004_I16Q80 = 0xBA,
    G9004_Q96 = 0xBB,
    // DIO (16 In/Out)
    G9102_I16 = 0xC0,
    G9102_I8Q8 = 0xC4,
    G9102_Q16 = 0xC7,
    G9205_I16 = 0xC8,
    G9205_I8Q8 = 0xCC,
    G9205_Q16 = 0xCF,
    // Misc.
    G9004_A104 = 0xD0,  // 106-A104
    G9004_A180 = 0xD1,  // 106-A180
    G9004_P144 = 0xD2,  // 108-P144
    G9004_C144 = 0xD3,  // 108-C144
    G9004_C144F = 0xD6, // 108-C144F
    G9004_A222 = 0xD4,  // 108-A222
    G9004_A204 = 0xD5,  // 108-A204
    G9004_A322 = 0xD7,  // 108-A322
    G9004_MSG = 0xE0,
    G9004_UNKNOWN = 0xFF,
} ESlaveType;

////////////////////////////////////////////////////////////////////////////////
//                            Function Declaration                            //
////////////////////////////////////////////////////////////////////////////////
#ifdef __cplusplus
extern "C" {
#endif
    I16 rpi_mnet_open(void);
    I16 rpi_mnet_close(void);
    I16 rpi_mnet_reset_ring(U16 RingNo);
    I16 rpi_mnet_start_ring(U16 RingNo);
    I16 rpi_mnet_stop_ring(U16 RingNo);
    I16 rpi_mnet_set_ring_config(U16 RingNo, U16 BaudRate);
    I16 rpi_mnet_get_ring_count(U16 *Count);
    I16 rpi_mnet_get_ring_active_table(U16 RingNo, U32 DevTable[2]);
    I16 rpi_mnet_get_slave_type(U16 RingNo, U16 SlaveIP, U8 *SlaveType);
    I16 rpi_mnet_get_msg_slave_type(U16 RingNo, U16 SlaveIP, U16 *Type);
    I16 rpi_mnet_get_ring_status(U16 RingNo, U16 *Status);
    I16 rpi_mnet_get_com_status(U16 RingNo);
    I16 rpi_mnet_get_ring_error_counter(U16 RingNo, U16 *ErrCount);
    I16 rpi_mnet_reset_ring_error_counter(U16 RingNo);
    I16 rpi_mnet_get_communication_error_flag(U16 RingNo, U16 SlaveIP, U8 *IsError);
    I16 rpi_mnet_reset_communication_error_flag(U16 RingNo, U16 SlaveIP);
    I16 rpi_mnet_get_communication_error_table(U16 RingNo, U32 ErrTable[2]);
    I16 rpi_mnet_reset_communication_error_table(U16 RingNo);
    I16 rpi_mnet_set_io_error_check(U16 RingNo, U8 Enabled);
    I16 rpi_mnet_io_input(U16 RingNo, U16 SlaveIP, U8 PortNo);
    I16 rpi_mnet_io_output(U16 RingNo, U16 SlaveIP, U8 PortNo, U8 Val);
    I16 rpi_mnet_bit_io_input(U16 RingNo, U16 SlaveIP, U8 PortNo, U8 BitNo, U8 *OnOff);
    I16 rpi_mnet_bit_io_output(U16 RingNo, U16 SlaveIP, U8 PortNo, U8 BitNo, U8 OnOff);
    I16 rpi_mnet_bit_io_toggle(U16 RingNo, U16 SlaveIP, U8 PortNo, U8 BitNo);
    I16 rpi_mnet_a222_initial(U16 RingNo, U16 SlaveIP);
    I16 rpi_mnet_a222_get_fpga_version(U16 RingNo, U16 SlaveIP, U16 *Ver);
    I16 rpi_mnet_a222_get_adc_version(U16 RingNo, U16 SlaveIP, U16 *Ver);
    I16 rpi_mnet_a222_get_adc_error(U16 RingNo, U16 SlaveIP, U8 *Error);
    I16 rpi_mnet_a222_reset_adc(U16 RingNo, U16 SlaveIP);
    I16 rpi_mnet_a222_start_analog_input(U16 RingNo, U16 SlaveIP);
    I16 rpi_mnet_a222_stop_analog_input(U16 RingNo, U16 SlaveIP);
    I16 rpi_mnet_a222_set_analog_input_enable(U16 RingNo, U16 SlaveIP, U8 Enabled);
    I16 rpi_mnet_a222_set_analog_input_type(U16 RingNo, U16 SlaveIP, U8 Type);
    I16 rpi_mnet_a222_set_analog_input_gain(U16 RingNo, U16 SlaveIP, U8 Gain);
    I16 rpi_mnet_a222_get_analog_input(U16 RingNo, U16 SlaveIP, U16 ChannelNo, F64 *Data);
    I16 rpi_mnet_a222_get_analog_input_ex(U16 RingNo, U16 SlaveIP, U16 ChannelNo, F64 *Data, U16 *Val);
    I16 rpi_mnet_a222_get_analog_input_all(U16 RingNo, U16 SlaveIP, F64 Data[8]);
    I16 rpi_mnet_a222_get_analog_input_all_ex(U16 RingNo, U16 SlaveIP, F64 Data[8], U16 Val[8]);
    I16 rpi_mnet_a222_get_analog_input_value(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U16 *Val);
    I16 rpi_mnet_a222_get_analog_input_value_all(U16 RingNo, U16 SlaveIP, U16 Val[8]);
    I16 rpi_mnet_a222_set_analog_output(U16 RingNo, U16 SlaveIP, U16 ChannelNo, F64 Volt);
    I16 rpi_mnet_a222_set_analog_output_all(U16 RingNo, U16 SlaveIP, const F64 Volt[4]);
    I16 rpi_mnet_a222_set_analog_output_value(U16 RingNo, U16 SlaveIP, U16 ChannelNo, I16 Val);
    I16 rpi_mnet_a222_set_analog_output_value_all(U16 RingNo, U16 SlaveIP, const I16 Val[4]);
    I16 rpi_mnet_a322_initial(U16 RingNo, U16 SlaveIP);
    I16 rpi_mnet_a322_get_fpga_version(U16 RingNo, U16 SlaveIP, U16 *Ver);
    I16 rpi_mnet_a322_get_fw_version(U16 RingNo, U16 SlaveIP, U32 *Ver);
    I16 rpi_mnet_a322_start_analog_input(U16 RingNo, U16 SlaveIP);
    I16 rpi_mnet_a322_stop_analog_input(U16 RingNo, U16 SlaveIP);
    I16 rpi_mnet_a322_set_analog_input_enable(U16 RingNo, U16 SlaveIP, U8 EnabledChannels);
    I16 rpi_mnet_a322_get_analog_input_enable(U16 RingNo, U16 SlaveIP, U8 *EnabledChannels);
    I16 rpi_mnet_a322_set_analog_input_range(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U8 Range);
    I16 rpi_mnet_a322_get_analog_input_range(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U8 *Range);
    I16 rpi_mnet_a322_set_analog_input_average_cycles(U16 RingNo, U16 SlaveIP, U16 Cycles);
    I16 rpi_mnet_a322_get_analog_input_average_cycles(U16 RingNo, U16 SlaveIP, U16 *Cycles);
    I16 rpi_mnet_a322_get_analog_input_value(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U16 *Val);
    I16 rpi_mnet_a322_get_analog_input_value_all(U16 RingNo, U16 SlaveIP, U16 Val[8]);
    I16 rpi_mnet_a322_get_analog_input(U16 RingNo, U16 SlaveIP, U16 ChannelNo, F64 *Volt);
    I16 rpi_mnet_a322_get_analog_input_ex(U16 RingNo, U16 SlaveIP, U16 ChannelNo, F64 *Volt, U16 *Val);
    I16 rpi_mnet_a322_get_analog_input_all(U16 RingNo, U16 SlaveIP, F64 Volt[8]);
    I16 rpi_mnet_a322_get_analog_input_all_ex(U16 RingNo, U16 SlaveIP, F64 Volt[8], U16 Val[8]);
    I16 rpi_mnet_a322_set_analog_output_range(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U8 Range);
    I16 rpi_mnet_a322_get_analog_output_range(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U8 *Range);
    I16 rpi_mnet_a322_set_analog_output_value(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U16 Val);
    I16 rpi_mnet_a322_set_analog_output_value_all(U16 RingNo, U16 SlaveIP, const U16 Val[4]);
    I16 rpi_mnet_a322_set_analog_output(U16 RingNo, U16 SlaveIP, U16 ChannelNo, F64 Volt);
    I16 rpi_mnet_a322_set_analog_output_ex(U16 RingNo, U16 SlaveIP, U16 ChannelNo, F64 Volt, U16 *Val);
    I16 rpi_mnet_a322_set_analog_output_all(U16 RingNo, U16 SlaveIP, const F64 Volt[4]);
    I16 rpi_mnet_a322_set_analog_output_all_ex(U16 RingNo, U16 SlaveIP, const F64 Volt[4], U16 Val[4]);
    I16 rpi_mnet_a204_initial(U16 RingNo, U16 SlaveIP);
    I16 rpi_mnet_a204_get_firmware_version(U16 RingNo, U16 SlaveIP, U16 *Ver);
    I16 rpi_mnet_a204_set_analog_output(U16 RingNo, U16 SlaveIP, U16 ChannelNo, F64 Volt);
    I16 rpi_mnet_a204_set_analog_output_ex(U16 RingNo, U16 SlaveIP, U16 ChannelNo, F64 Volt, I16 *Val);
    I16 rpi_mnet_a204_set_analog_output_all(U16 RingNo, U16 SlaveIP, const F64 Volt[4]);
    I16 rpi_mnet_a204_set_analog_output_all_ex(U16 RingNo, U16 SlaveIP, const F64 Volt[4], I16 Val[4]);
    I16 rpi_mnet_a204_set_analog_output_value(U16 RingNo, U16 SlaveIP, U16 ChannelNo, I16 Val);
    I16 rpi_mnet_a204_set_analog_output_value_all(U16 RingNo, U16 SlaveIP, const I16 Val[4]);
    I16 rpi_mnet_ai8_initial(U16 RingNo, U16 SlaveIP);
    I16 rpi_mnet_ai8_get_hardware_info(U16 RingNo, U16 SlaveIP, U8 *DeviceID, U8 *VHDL_Version);
    I16 rpi_mnet_ai8_set_cycle_time(U16 RingNo, U16 SlaveIP, U8 SetValue);
    I16 rpi_mnet_ai8_enable_device(U16 RingNo, U16 SlaveIP, U8 EnableDevice);
    I16 rpi_mnet_ai8_enable_channel(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U8 Enable);
    I16 rpi_mnet_ai8_set_channel_gain(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U8 Gain);
    I16 rpi_mnet_ai8_get_value(U16 RingNo, U16 SlaveIP, U16 ChannelNo, I16 *Value);
    I16 rpi_mnet_ai8_get_value_all(U16 RingNo, U16 SlaveIP, I16 Value[8]);
    I16 rpi_mnet_ai8_get_voltage(U16 RingNo, U16 SlaveIP, U16 ChannelNo, F64 *Voltage);
    I16 rpi_mnet_ai8_get_voltage_all(U16 RingNo, U16 SlaveIP, F64 Voltage[8]);
    I16 rpi_mnet_ao4_initial(U16 RingNo, U16 SlaveIP);
    I16 rpi_mnet_ao4_get_hardware_info(U16 RingNo, U16 SlaveIP, U8 *DeviceID, U8 *VHDL_Version);
    I16 rpi_mnet_ao4_reset_DAC(U16 RingNo, U16 SlaveIP);
    I16 rpi_mnet_ao4_clear_output_all(U16 RingNo, U16 SlaveIP);
    I16 rpi_mnet_ao4_set_output(U16 RingNo, U16 SlaveIP, U8 ChannelNo, I16 SetValue);
    I16 rpi_mnet_ao4_set_voltage(U16 RingNo, U16 SlaveIP, U8 ChannelNo, F64 Voltage);
    I16 rpi_mnet_ao4_set_voltage1(U16 RingNo, U16 SlaveIP, U8 ChannelNo, F64 Voltage, I16 *Value);
    I16 rpi_mnet_ao4_set_output_all(U16 RingNo, U16 SlaveIP, I16 SetValue1, I16 SetValue2, I16 SetValue3, I16 SetValue4);
    I16 rpi_mnet_ao4_set_voltage_all(U16 RingNo, U16 SlaveIP, F64 Voltage1, F64 Voltage2, F64 Voltage3, F64 Voltage4);
    I16 rpi_mnet_ao4_set_voltage_all1(U16 RingNo, U16 SlaveIP, F64 Voltage1, F64 Voltage2, F64 Voltage3, F64 Voltage4, I16 *Value1, I16 *Value2, I16 *Value3, I16 *Value4);
    I16 rpi_mnet_ao4_set_coarse_gain(U16 RingNo, U16 SlaveIP, U8 ChannelNo, I16 SetValue);
    I16 rpi_mnet_c144_initial(U16 RingNo, U16 SlaveIP);
    I16 rpi_mnet_c144_get_firmware_version(U16 RingNo, U16 SlaveIP, U16 *Ver);
    I16 rpi_mnet_c144_get_do(U16 RingNo, U16 SlaveIP, U8 *Val);
    I16 rpi_mnet_c144_set_do(U16 RingNo, U16 SlaveIP, U8 Val);
    I16 rpi_mnet_c144_set_encoder_mode(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U16 Mode);
    I16 rpi_mnet_c144_set_pls_iptmode(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U16 PlsInMode, U16 PlsInDir);
    I16 rpi_mnet_c144_get_encoder(U16 RingNo, U16 SlaveIP, U16 ChannelNo, I32 *EncPos);
    I16 rpi_mnet_c144_get_encoder_all(U16 RingNo, U16 SlaveIP, I32 EncPos[4]);
    I16 rpi_mnet_c144_set_encoder(U16 RingNo, U16 SlaveIP, U16 ChannelNo, I32 EncPos);
    I16 rpi_mnet_c144_set_auto_compare_source(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U16 SrcEncNo);
    I16 rpi_mnet_c144_get_auto_compare_count(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U16 *Count);
    I16 rpi_mnet_c144_get_auto_compare_count_all(U16 RingNo, U16 SlaveIP, U16 Count[4]);
    I16 rpi_mnet_c144_get_auto_compare_count_ex(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U32 *Count);
    I16 rpi_mnet_c144_get_auto_compare_count_all_ex(U16 RingNo, U16 SlaveIP, U32 Count[4]);
    I16 rpi_mnet_c144_get_auto_compare_status(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U16 *OnOff);
    I16 rpi_mnet_c144_get_auto_compare_status_all(U16 RingNo, U16 SlaveIP, U16 OnOff[4]);
    I16 rpi_mnet_c144_set_auto_compare_trigger(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U16 Level, U16 Width);
    I16 rpi_mnet_c144_set_auto_compare_function(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U8 Dir, I32 StrPos, I32 Interval, U16 TrgCnt);
    I16 rpi_mnet_c144_set_auto_compare_function_ex(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U8 Dir, I32 StrPos, I32 Interval, U32 TrgCnt);
    I16 rpi_mnet_c144_set_auto_compare_table(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U8 Dir, U16 Size, const I32 *Table);
    I16 rpi_mnet_c144_get_auto_compare_table_capacity(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U16 *Size);
    I16 rpi_mnet_c144_start_auto_compare(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U16 OnOff);
    I16 rpi_mnet_c144_force_trigger_output(U16 RingNo, U16 SlaveIP, U16 ChannelNo);
    I16 rpi_mnet_p144_initial(U16 RingNo, U16 SlaveIP);
    I16 rpi_mnet_p144_get_firmware_version(U16 RingNo, U16 SlaveIP, U16 *Ver);
    I16 rpi_mnet_p144_set_count(U16 RingNo, U16 SlaveIP, U16 ChannelNo, I32 Count);
    I16 rpi_mnet_p144_get_count(U16 RingNo, U16 SlaveIP, U16 ChannelNo, I32 *Count);
    I16 rpi_mnet_p144_get_count_all(U16 RingNo, U16 SlaveIP, I32 Count[4]);
    I16 rpi_mnet_p144_get_frequency(U16 RingNo, U16 SlaveIP, U16 ChannelNo, F64 *Freq);
    I16 rpi_mnet_p144_get_frequency_all(U16 RingNo, U16 SlaveIP, F64 Freq[4]);
    I16 rpi_mnet_p144_set_pulse_level(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U8 Level);
    I16 rpi_mnet_p144_get_pulse_width(U16 RingNo, U16 SlaveIP, U16 ChannelNo, F64 *Width);
    I16 rpi_mnet_p144_get_pulse_width_all(U16 RingNo, U16 SlaveIP, F64 Width[4]);
    I16 rpi_mnet_p144_set_filter(U16 RingNo, U16 SlaveIP, U16 ChannelNo, U16 Filter);
    I16 rpi_mnet_p144_get_do(U16 RingNo, U16 SlaveIP, U8 *Val);
    I16 rpi_mnet_p144_set_do_bit(U16 RingNo, U16 SlaveIP, U16 BitNo, U8 Mode, I32 Val);
    I16 rpi_mnet_m1a_initial(U16 RingNo, U16 SlaveIP);
    I16 rpi_mnet_m1a_set_pls_outmode(U16 RingNo, U16 SlaveIP, U16 pls_outmode);
    I16 rpi_mnet_m1a_set_pls_iptmode(U16 RingNo, U16 SlaveIP, U16 pls_iptmode, U16 pls_iptdir);
    I16 rpi_mnet_m1a_set_feedback_src(U16 RingNo, U16 SlaveIP, U16 FbkSrc);
    I16 rpi_mnet_m1a_set_abs_reference(U16 RingNo, U16 SlaveIP, U16 Ref);
    I16 rpi_mnet_m1a_set_abs_ref_encoder_ignore_backlash(U16 RingNo, U16 SlaveIP, U8 OnOff);
    I16 rpi_mnet_m1a_set_direction_change_delay(U16 RingNo, U16 SlaveIP, U8 Enabled);
    I16 rpi_mnet_m1a_set_move_ratio(U16 RingNo, U16 SlaveIP, F32 MoveRatio);
    I16 rpi_mnet_m1a_set_alm(U16 RingNo, U16 SlaveIP, U16 alm_logic, U16 alm_mode);
    I16 rpi_mnet_m1a_set_inp(U16 RingNo, U16 SlaveIP, U16 inp_enable, U16 inp_logic);
    I16 rpi_mnet_m1a_set_el_mode(U16 RingNo, U16 SlaveIP, U16 el_mode);
    I16 rpi_mnet_m1a_set_sd(U16 RingNo, U16 SlaveIP, I16 sd_enable, I16 sd_logic, I16 sd_latch, I16 sd_mode);
    I16 rpi_mnet_m1a_set_erc(U16 RingNo, U16 SlaveIP, U16 erc_logic, U16 erc_on_time, U16 erc_off_time);
    I16 rpi_mnet_m1a_set_erc_on(U16 RingNo, U16 SlaveIP, I16 on_off);
    I16 rpi_mnet_m1a_set_auto_erc(U16 RingNo, U16 SlaveIP, I16 Enable);
    I16 rpi_mnet_m1a_set_svon(U16 RingNo, U16 SlaveIP, U16 on_off);
    I16 rpi_mnet_m1a_set_ralm(U16 RingNo, U16 SlaveIP, U16 on_off);
    I16 rpi_mnet_m1a_set_ltc_logic(U16 RingNo, U16 SlaveIP, U16 ltc_logic);
    I16 rpi_mnet_m1a_set_mechanical_input_filter(U16 RingNo, U16 SlaveIP, U16 on_off);
    I16 rpi_mnet_m1a_set_backlash(U16 RingNo, U16 SlaveIP, U16 Value, U16 Enable, U16 CntSrc);
    I16 rpi_mnet_m1a_dio_output(U16 RingNo, U16 SlaveIP, U16 DoNo, U16 on_off);
    I16 rpi_mnet_m1a_dio_input(U16 RingNo, U16 SlaveIP, U16 DiNo);
    I16 rpi_mnet_m1a_get_dio(U16 RingNo, U16 SlaveIP, U8 *Val);
    I16 rpi_mnet_m1a_get_io_status(U16 RingNo, U16 SlaveIP, U32 *IO_status);
    I16 rpi_mnet_m1a_load_motion_file(U16 RingNo, U16 SlaveIP, const char *FilePath);
    I16 rpi_mnet_m1a_get_command(U16 RingNo, U16 SlaveIP, I32 *Cmd);
    I16 rpi_mnet_m1a_set_command(U16 RingNo, U16 SlaveIP, I32 Cmd);
    I16 rpi_mnet_m1a_reset_command(U16 RingNo, U16 SlaveIP);
    I16 rpi_mnet_m1a_get_position(U16 RingNo, U16 SlaveIP, I32 *Pos);
    I16 rpi_mnet_m1a_set_position(U16 RingNo, U16 SlaveIP, I32 Pos);
    I16 rpi_mnet_m1a_reset_position(U16 RingNo, U16 SlaveIP);
    I16 rpi_mnet_m1a_get_error_counter(U16 RingNo, U16 SlaveIP, I32 *ErrCnt);
    I16 rpi_mnet_m1a_set_error_counter(U16 RingNo, U16 SlaveIP, I32 ErrCnt);
    I16 rpi_mnet_m1a_reset_error_counter(U16 RingNo, U16 SlaveIP);
    I16 rpi_mnet_m1a_get_rest_command(U16 RingNo, U16 SlaveIP, I32 *Cmd);
    I16 rpi_mnet_m1a_enable_command_ring_counter(U16 RingNo, U16 SlaveIP, U32 RingCounter);
    I16 rpi_mnet_m1a_disable_command_ring_counter(U16 RingNo, U16 SlaveIP);
    I16 rpi_mnet_m1a_enable_position_ring_counter(U16 RingNo, U16 SlaveIP, U32 RingCounter);
    I16 rpi_mnet_m1a_disable_position_ring_counter(U16 RingNo, U16 SlaveIP);
    I16 rpi_mnet_m1a_get_current_speed(U16 RingNo, U16 SlaveIP, U32 *Speed);
    I16 rpi_mnet_m1a_get_command_without_backlash(U16 RingNo, U16 SlaveIP, I32 *Cmd);
    I16 rpi_mnet_m1a_get_position_without_backlash(U16 RingNo, U16 SlaveIP, I32 *Pos);
    I16 rpi_mnet_m1a_set_tmove_speed(U16 RingNo, U16 SlaveIP, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec);
    I16 rpi_mnet_m1a_set_smove_speed(U16 RingNo, U16 SlaveIP, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec);
    I16 rpi_mnet_m1a_set_smove_speed_ex(U16 RingNo, U16 SlaveIP, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec, U32 SVacc, U32 SVdec);
    I16 rpi_mnet_m1a_start_r_move(U16 RingNo, U16 SlaveIP, I32 Dist);
    I16 rpi_mnet_m1a_start_a_move(U16 RingNo, U16 SlaveIP, I32 Pos);
    I16 rpi_mnet_m1a_start_tr_move(U16 RingNo, U16 SlaveIP, I32 Dist, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec);
    I16 rpi_mnet_m1a_start_ta_move(U16 RingNo, U16 SlaveIP, I32 Pos, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec);
    I16 rpi_mnet_m1a_start_sr_move(U16 RingNo, U16 SlaveIP, I32 Dist, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec);
    I16 rpi_mnet_m1a_start_sa_move(U16 RingNo, U16 SlaveIP, I32 Pos, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec);
    I16 rpi_mnet_m1a_v_move(U16 RingNo, U16 SlaveIP, U8 Dir);
    I16 rpi_mnet_m1a_tv_move(U16 RingNo, U16 SlaveIP, U8 Dir, U32 StrVel, U32 MaxVel, F32 Tacc);
    I16 rpi_mnet_m1a_sv_move(U16 RingNo, U16 SlaveIP, U8 Dir, U32 StrVel, U32 MaxVel, F32 Tacc);
    I16 rpi_mnet_m1a_sd_stop(U16 RingNo, U16 SlaveIP);
    I16 rpi_mnet_m1a_sd_stop_ex(U16 RingNo, U16 SlaveIP, F32 Tdec);
    I16 rpi_mnet_m1a_imd_stop(U16 RingNo, U16 SlaveIP);
    I16 rpi_mnet_m1a_emg_stop(U16 RingNo, U16 SlaveIP);
    I16 rpi_mnet_m1a_motion_done(U16 RingNo, U16 SlaveIP, U16 *MoSt);
    I16 rpi_mnet_m1a_triangle_correction(U16 RingNo, U16 SlaveIP, U16 OnOff);
    I16 rpi_mnet_m1a_set_home_config(U16 RingNo, U16 SlaveIP, U16 home_mode, U16 org_logic, U16 ez_logic, U16 ez_count, U16 erc_out);
    I16 rpi_mnet_m1a_start_home_move(U16 RingNo, U16 SlaveIP, U8 Dir);
    I16 rpi_mnet_m1a_start_home_search(U16 RingNo, U16 SlaveIP, U8 Dir, I32 ORGOffset);
    I16 rpi_mnet_m1a_start_home_escape(U16 RingNo, U16 SlaveIP, U8 Dir);
    I16 rpi_mnet_m1a_start_search_ez(U16 RingNo, U16 SlaveIP, U8 Dir, U16 EzCount);
    I16 rpi_mnet_m1a_home_move(U16 RingNo, U16 SlaveIP, U8 Dir, U32 StrVel, U32 MaxVel, F32 Tacc);
    I16 rpi_mnet_m1a_home_search(U16 RingNo, U16 SlaveIP, U8 Dir, U32 StrVel, U32 MaxVel, F32 Tacc, I32 ORGOffset);
    I16 rpi_mnet_m1a_home_escape(U16 RingNo, U16 SlaveIP, U8 Dir, U32 StrVel, U32 MaxVel, F32 Tacc);
    I16 rpi_mnet_m1a_search_ez(U16 RingNo, U16 SlaveIP, U8 Dir, U16 EzCount, U32 StrVel, U32 MaxVel, F32 Tacc);
    I16 rpi_mnet_m1a_set_comparator_mode(U16 RingNo, U16 SlaveIP, I16 CompNo, I16 CmpSrc, I16 CmpMethod, I16 CmpAction);
    I16 rpi_mnet_m1a_set_comparator_data(U16 RingNo, U16 SlaveIP, I16 CompNo, I32 Pos);
    I16 rpi_mnet_m1a_set_trigger_comparator(U16 RingNo, U16 SlaveIP, U16 CmpSrc, U16 CmpMethod);
    I16 rpi_mnet_m1a_set_trigger_comparator_data(U16 RingNo, U16 SlaveIP, I32 Data);
    I16 rpi_mnet_m1a_set_auto_trigger(U16 RingNo, U16 SlaveIP, U16 CmpSrc, U16 CmpMethod, U16 Interval, U16 on_off);
    I16 rpi_mnet_m1a_get_comparator_data(U16 RingNo, U16 SlaveIP, I16 CompNo, I32 *Pos);
    I16 rpi_mnet_m1a_set_soft_limit(U16 RingNo, U16 SlaveIP, I32 PLimit, I32 MLimit);
    I16 rpi_mnet_m1a_enable_soft_limit(U16 RingNo, U16 SlaveIP, U8 Action);
    I16 rpi_mnet_m1a_disable_soft_limit(U16 RingNo, U16 SlaveIP);
    I16 rpi_mnet_m1a_set_latch_signal(U16 RingNo, U16 SlaveIP, U8 LtcSignal);
    I16 rpi_mnet_m1a_get_latch_data(U16 RingNo, U16 SlaveIP, I16 LatchNo, I32 *Pos);
    I16 rpi_mnet_m1a_v_change(U16 RingNo, U16 SlaveIP, U32 NewVel, F32 Time);
    I16 rpi_mnet_m1a_fix_speed_range(U16 RingNo, U16 SlaveIP, U32 MaxVel);
    I16 rpi_mnet_m1a_unfix_speed_range(U16 RingNo, U16 SlaveIP);
    I16 rpi_mnet_m1a_get_speed_range(U16 RingNo, U16 SlaveIP, U32 *MinVel, U32 *MaxVel);
    I16 rpi_mnet_m1a_start_p_change(U16 RingNo, U16 SlaveIP, I32 NewPos);
    I16 rpi_mnet_m1a_start_d_change(U16 RingNo, U16 SlaveIP, I32 NewDist);
    I16 rpi_mnet_m1a_get_preregister_status(U16 RingNo, U16 SlaveIP, U8* Full);
    I16 rpi_mnet_m1a_get_preregister_depth(U16 RingNo, U16 SlaveIP, U8* Depth);
    I16 rpi_mnet_m1a_cancel_preregister(U16 RingNo, U16 SlaveIP);
    I16 rpi_mnet_m1a_shift_preregister(U16 RingNo, U16 SlaveIP);
    I16 rpi_mnet_m1a_check_int(U16 RingNo, U16 SlaveIP, U8* IntStatus);
    I16 rpi_mnet_m1a_check_error(U16 RingNo, U16 SlaveIP, U32* ErrorStatus);
    I16 rpi_mnet_m1a_check_event(U16 RingNo, U16 SlaveIP, U32* EventStatus);
    I16 rpi_mnet_m1a_set_int_factor(U16 RingNo, U16 SlaveIP, U32 int_factor);
    I16 rpi_mnet_m1a_set_group(U16 RingNo, U16 GroupNo, const U16* AxisArray, U16 AxisCount);
    I16 rpi_mnet_m1a_group_reset_command(U16 RingNo, U16 GroupNo);
    I16 rpi_mnet_m1a_group_reset_position(U16 RingNo, U16 GroupNo);
    I16 rpi_mnet_m1a_group_continuous_move(U16 RingNo, U16 GroupNo, U16 Enable);
    I16 rpi_mnet_m1a_group_tr_lineN(U16 RingNo, U16 GroupNo, const I32* DistArray, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec);
    I16 rpi_mnet_m1a_group_ta_lineN(U16 RingNo, U16 GroupNo, const I32* PosArray, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec);
    I16 rpi_mnet_m1a_group_sr_lineN(U16 RingNo, U16 GroupNo, const I32* DistArray, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec);
    I16 rpi_mnet_m1a_group_sa_lineN(U16 RingNo, U16 GroupNo, const I32* PosArray, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec);
    I16 rpi_mnet_m1a_group_tr_lineN_continuous(U16 RingNo, U16 GroupNo, const I32* DistArray, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec);
    I16 rpi_mnet_m1a_group_ta_lineN_continuous(U16 RingNo, U16 GroupNo, const I32* PosArray, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec);
    I16 rpi_mnet_m1a_group_sr_lineN_continuous(U16 RingNo, U16 GroupNo, const I32* DistArray, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec);
    I16 rpi_mnet_m1a_group_sa_lineN_continuous(U16 RingNo, U16 GroupNo, const I32* PosArray, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec);
    I16 rpi_mnet_m1a_group_tr_arc(U16 RingNo, U16 GroupNo, U16 AxIP, U16 AyIP, I32 OffsetCx, I32 OffsetCy, I32 OffsetEx, I32 OffsetEy, I16 Dir, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec);
    I16 rpi_mnet_m1a_group_ta_arc(U16 RingNo, U16 GroupNo, U16 AxIP, U16 AyIP, I32 Cx, I32 Cy, I32 Ex, I32 Ey, I16 Dir, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec);
    I16 rpi_mnet_m1a_group_sr_arc(U16 RingNo, U16 GroupNo, U16 AxIP, U16 AyIP, I32 OffsetCx, I32 OffsetCy, I32 OffsetEx, I32 OffsetEy, I16 Dir, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec);
    I16 rpi_mnet_m1a_group_sa_arc(U16 RingNo, U16 GroupNo, U16 AxIP, U16 AyIP, I32 Cx, I32 Cy, I32 Ex, I32 Ey, I16 Dir, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec);
    I16 rpi_mnet_m1a_group_tr_arc_continuous(U16 RingNo, U16 GroupNo, U16 AxIP, U16 AyIP, I32 OffsetCx, I32 OffsetCy, I32 OffsetEx, I32 OffsetEy, I16 Dir, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec);
    I16 rpi_mnet_m1a_group_ta_arc_continuous(U16 RingNo, U16 GroupNo, U16 AxIP, U16 AyIP, I32 Cx, I32 Cy, I32 Ex, I32 Ey, I16 Dir, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec);
    I16 rpi_mnet_m1a_group_sr_arc_continuous(U16 RingNo, U16 GroupNo, U16 AxIP, U16 AyIP, I32 OffsetCx, I32 OffsetCy, I32 OffsetEx, I32 OffsetEy, I16 Dir, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec);
    I16 rpi_mnet_m1a_group_sa_arc_continuous(U16 RingNo, U16 GroupNo, U16 AxIP, U16 AyIP, I32 Cx, I32 Cy, I32 Ex, I32 Ey, I16 Dir, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec);
    I16 rpi_mnet_m1a_group_tr_arc3(U16 RingNo, U16 GroupNo, U16 AxIP, U16 AyIP, U16 AzIP, I32 OffsetCx, I32 OffsetCy, I32 OffsetEx, I32 OffsetEy, I32 DistZ, I16 Dir, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec);
    I16 rpi_mnet_m1a_group_ta_arc3(U16 RingNo, U16 GroupNo, U16 AxIP, U16 AyIP, U16 AzIP, I32 Cx, I32 Cy, I32 Ex, I32 Ey, I32 PosZ, I16 Dir, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec);
    I16 rpi_mnet_m1a_group_sr_arc3(U16 RingNo, U16 GroupNo, U16 AxIP, U16 AyIP, U16 AzIP, I32 OffsetCx, I32 OffsetCy, I32 OffsetEx, I32 OffsetEy, I32 DistZ, I16 Dir, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec);
    I16 rpi_mnet_m1a_group_sa_arc3(U16 RingNo, U16 GroupNo, U16 AxIP, U16 AyIP, U16 AzIP, I32 Cx, I32 Cy, I32 Ex, I32 Ey, I32 PosZ, I16 Dir, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec);
    I16 rpi_mnet_m1a_group_tr_arc3_continuous(U16 RingNo, U16 GroupNo, U16 AxIP, U16 AyIP, U16 AzIP, I32 OffsetCx, I32 OffsetCy, I32 OffsetEx, I32 OffsetEy, I32 DistZ, I16 Dir, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec);
    I16 rpi_mnet_m1a_group_ta_arc3_continuous(U16 RingNo, U16 GroupNo, U16 AxIP, U16 AyIP, U16 AzIP, I32 Cx, I32 Cy, I32 Ex, I32 Ey, I32 PosZ, I16 Dir, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec);
    I16 rpi_mnet_m1a_group_sr_arc3_continuous(U16 RingNo, U16 GroupNo, U16 AxIP, U16 AyIP, U16 AzIP, I32 OffsetCx, I32 OffsetCy, I32 OffsetEx, I32 OffsetEy, I32 DistZ, I16 Dir, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec);
    I16 rpi_mnet_m1a_group_sa_arc3_continuous(U16 RingNo, U16 GroupNo, U16 AxIP, U16 AyIP, U16 AzIP, I32 Cx, I32 Cy, I32 Ex, I32 Ey, I32 PosZ, I16 Dir, U32 StrVel, U32 MaxVel, F32 Tacc, F32 Tdec);
    I16 rpi_mnet_m1a_group_set_holding_start(U16 RingNo, U16 GroupNo, U8 Enabled);
    I16 rpi_mnet_m1a_group_start(U16 RingNo, U16 GroupNo);
    I16 rpi_mnet_m1a_group_emg_stop(U16 RingNo, U16 GroupNo);
    I16 rpi_mnet_m1a_group_imd_stop(U16 RingNo, U16 GroupNo);
    I16 rpi_mnet_m1a_group_sd_stop(U16 RingNo, U16 GroupNo);
    I16 rpi_mnet_m1a_get_hardware_info(U16 RingNo, U16 SlaveIP, U32* VendorID, U16* DeviceID);
    I16 rpi_mnet_m1a_recovery_from_EEPROM(U16 RingNo, U16 SlaveIP);
    I16 rpi_mnet_m1a_backup_to_EEPROM(U16 RingNo, U16 SlaveIP);
#ifdef __cplusplus
}
#endif

#endif /* RPIMNET_H_ */
