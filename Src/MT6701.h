#ifndef MT6701_H
#define MT6701_H

#include "main.h"
#include <stdint.h>

#define MT6701_RESOLUTION      16384.0f
#define MT6701_FULL_SCALE_RAW  16384U

typedef enum
{
    MT6701_OK = 0,
    MT6701_HAL_ERROR,
    MT6701_CRC_ERROR,
    MT6701_NULL_ERROR
} MT6701_Status_t;

typedef struct
{
    uint16_t raw_angle;      /* 0..16383 */
    uint8_t magnetic_status; /* Mg[3:0] from SSI frame */
    uint8_t crc;
    uint8_t crc_ok;
} MT6701_Frame_t;

typedef struct
{
    uint8_t rx[3];
    uint32_t packet;
    uint16_t raw_shift10;
    uint16_t raw_shift9;
    uint16_t raw_shift8;
    uint16_t raw_shift7;
    uint8_t magnetic_status;
    uint8_t received_crc;
    uint8_t calculated_crc;
    uint8_t crc_ok;
} MT6701_Debug_t;

void MT6701_Init(void);
MT6701_Status_t MT6701_ReadFrame(MT6701_Frame_t *frame);
MT6701_Status_t MT6701_ReadDebug(MT6701_Debug_t *debug);
MT6701_Status_t MT6701_ReadRawAngle(uint16_t *raw_angle);
MT6701_Status_t MT6701_ReadRawAngleNoCrc(uint16_t *raw_angle);
MT6701_Status_t MT6701_ReadAngleDeg(float *angle_deg);
MT6701_Status_t MT6701_ReadAngleDegNoCrc(float *angle_deg);
float MT6701_RawToDeg(uint16_t raw_angle);

#endif
