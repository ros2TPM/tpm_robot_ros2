#ifndef DEF_RIDT_H
#define DEF_RIDT_H

#include <stdint.h>

#define MAX_DIO_SLV_PER_ROBOT   8
#define MAX_AXIS_PER_ROBOT   6

typedef float FLT;

typedef struct {
    FLT axis[MAX_AXIS_PER_ROBOT]; //robc_get_axis
    FLT pose[MAX_AXIS_PER_ROBOT]; //robc_get_pose
    uint32_t bufDepth;                 //robc_get_buffer_depth
    uint32_t nowCmdId;                 //robc_get_now_cmd_id
    int32_t lastErr;                  //robc_get_last_error
} ST_RIDT_ROBC, ST_RIDT_ROBC_t;

typedef struct {
    uint32_t ioSts;
    int32_t encPos;
} ST_RIDT_MNET_M1A, ST_RIDT_MNET_M1A_t;

typedef struct {
    uint8_t port[12];
} ST_RIDT_MNET_DIO, ST_RIDT_MNET_DIO_t;

typedef struct RIDT{
    ST_RIDT_ROBC_t robc;
    ST_RIDT_MNET_M1A_t m1a[MAX_AXIS_PER_ROBOT];
    ST_RIDT_MNET_DIO_t dio[MAX_DIO_SLV_PER_ROBOT];
} ST_RIDT, ST_RIDT_t;


#endif //DEF_RIDT_H