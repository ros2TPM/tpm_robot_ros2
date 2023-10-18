#include "RPiL132.h"

    I16 rpi_master_initialize(void) { return -8018; }
    I16 rpi_master_finalize(void) { return -8018; }
    I16 rpi_master_get_library_version(U32* Version) { return -8018; }
    I16 rpi_l132_get_hardware_version(U16* Version) { return -8018; }
    I16 rpi_l132_get_fpga_version(U16* Version) { return -8018; }
    I16 rpi_l132_get_firmware_version(U32* Version) { return -8018; }
    I16 rpi_l132_get_rotary_switch_position(U8* Pos) { return -8018; }
    I16 rpi_l132_read_local_input(U8* Val) { return -8018; }
    I16 rpi_l132_read_local_output(U8* Val) { return -8018; }
    I16 rpi_l132_write_local_output(U8 Val) { return -8018; }
    I16 rpi_l132_write_local_output_bit(U8 Bit, U8 Val) { return -8018; }
    I16 rpi_l132_toggle_local_output_bit(U8 Bit) { return -8018; }
    I16 rpi_l132_set_encoder_mode(U8 Channel, U8 Mode) { return -8018; }
    I16 rpi_l132_get_encoder(U8 Channel, I32* EncPos) { return -8018; }
    I16 rpi_l132_set_encoder(U8 Channel, I32 EncPos) { return -8018; }
    I16 rpi_l132_set_auto_compare_source(U8 CmpNo, U8 SrcEncNo) { return -8018; }
    I16 rpi_l132_get_auto_compare_count(U8 CmpNo, U32* Count) { return -8018; }
    I16 rpi_l132_get_auto_compare_status(U8 CmpNo, U8* Status) { return -8018; }
    I16 rpi_l132_set_auto_compare_trigger(U8 CmpNo, U8 Level, U16 Width) { return -8018; }
    I16 rpi_l132_set_auto_compare_function(U8 CmpNo, U8 Dir, I32 StrPos, I32 Interval, U32 TrgCnt) { return -8018; }
    I16 rpi_l132_set_auto_compare_table(U8 CmpNo, U8 Dir, U16 Size, const I32* Table) { return -8018; }
    I16 rpi_l132_start_auto_compare(U8 CmpNo, U8 OnOff) { return -8018; }
    I16 rpi_l132_force_trigger_output(U8 CmpNo) { return -8018; }
    I16 rpi_l132_get_auto_compare_fifo_capacity(U8 CmpNo, U16* Capacity) { return -8018; }
    I16 rpi_l132_assign_auto_compare_fifo(U8 Assign) { return -8018; }
    I16 rpi_l132_start_fifo_latch(U8 LtcNo, U8 SrcEncNo, U8 Edge) { return -8018; }
    I16 rpi_l132_stop_fifo_latch(U8 LtcNo) { return -8018; }
    I16 rpi_l132_get_fifo_latch_data_length(U8 LtcNo, U16* Length) { return -8018; }
    I16 rpi_l132_get_fifo_latch_data(U8 LtcNo, U16 MaxSize, U16* Size, I32* LtcTable) { return -8018; }
    I16 rpi_l132_set_fifo_latch_filter(U8 LtcNo, U16 Filter) { return -8018; }