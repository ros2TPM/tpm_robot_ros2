#ifndef RPIL132_H_
#define RPIL132_H_

#include "type_def.h"


enum RPI_ERROR_CODE
{
    RPIERR_SUCCESS = 0,
    RPIERR_INIT_FAILED = -8001,
    RPIERR_COMM_FAILED = -8002,
    RPIERR_COMM_TIMEOUT = -8003,
    RPIERR_INVALID_CRC = -8004,
    RPIERR_INVALID_DATA_SIZE = -8005,
    RPIERR_INVALID_CHECKSUM = -8006,
    RPIERR_INVALID_FUNC = -8007,
    RPIERR_INVALID_PACKET = -8008,
    RPIERR_OPEN_FILE_FAILED = -8010,
    RPIERR_FWUPDATE_NOT_FINISHED_YET = -8011,
    RPIERR_FWUPDATE_NOT_STARTED_YET = -8012,
    RPIERR_FWUPDATE_WRONG_PASSWORD = -8013,
    RPIERR_FWUPDATE_INVALID_FILESIZE = -8014,
    RPIERR_FWUPDATE_INVALID_CHECKSUM = -8015,
    RPIERR_FWUPDATE_FLASH_FULL = -8016,
    RPIERR_FWUPDATE_PROG_ERROR = -8017,
    RPIERR_NOT_SUPPORTED = -8018,
    RPIERR_INVALID_ARGUMENT_1 = -8021,
    RPIERR_INVALID_ARGUMENT_2 = -8022,
    RPIERR_INVALID_ARGUMENT_3 = -8023,
    RPIERR_INVALID_ARGUMENT_4 = -8024,
    RPIERR_INVALID_ARGUMENT_5 = -8025,
};

#ifdef __cplusplus
extern "C" {
#endif
    I16 rpi_master_initialize(void);
    I16 rpi_master_finalize(void);
    I16 rpi_master_get_library_version(U32* Version);
    I16 rpi_l132_get_hardware_version(U16* Version);
    I16 rpi_l132_get_fpga_version(U16* Version);
    I16 rpi_l132_get_firmware_version(U32* Version);
    I16 rpi_l132_get_rotary_switch_position(U8* Pos);
    I16 rpi_l132_read_local_input(U8* Val);
    I16 rpi_l132_read_local_output(U8* Val);
    I16 rpi_l132_write_local_output(U8 Val);
    I16 rpi_l132_write_local_output_bit(U8 Bit, U8 Val);
    I16 rpi_l132_toggle_local_output_bit(U8 Bit);
    I16 rpi_l132_set_encoder_mode(U8 Channel, U8 Mode);
    I16 rpi_l132_get_encoder(U8 Channel, I32* EncPos);
    I16 rpi_l132_set_encoder(U8 Channel, I32 EncPos);
    I16 rpi_l132_set_auto_compare_source(U8 CmpNo, U8 SrcEncNo);
    I16 rpi_l132_get_auto_compare_count(U8 CmpNo, U32* Count);
    I16 rpi_l132_get_auto_compare_status(U8 CmpNo, U8* Status);
    I16 rpi_l132_set_auto_compare_trigger(U8 CmpNo, U8 Level, U16 Width);
    I16 rpi_l132_set_auto_compare_function(U8 CmpNo, U8 Dir, I32 StrPos, I32 Interval, U32 TrgCnt);
    I16 rpi_l132_set_auto_compare_table(U8 CmpNo, U8 Dir, U16 Size, const I32* Table);
    I16 rpi_l132_start_auto_compare(U8 CmpNo, U8 OnOff);
    I16 rpi_l132_force_trigger_output(U8 CmpNo);
    I16 rpi_l132_get_auto_compare_fifo_capacity(U8 CmpNo, U16* Capacity);
    I16 rpi_l132_assign_auto_compare_fifo(U8 Assign);
    I16 rpi_l132_start_fifo_latch(U8 LtcNo, U8 SrcEncNo, U8 Edge);
    I16 rpi_l132_stop_fifo_latch(U8 LtcNo);
    I16 rpi_l132_get_fifo_latch_data_length(U8 LtcNo, U16* Length);
    I16 rpi_l132_get_fifo_latch_data(U8 LtcNo, U16 MaxSize, U16* Size, I32* LtcTable);
    I16 rpi_l132_set_fifo_latch_filter(U8 LtcNo, U16 Filter);
#ifdef __cplusplus
}
#endif

#endif /* RPIL132_H_ */
