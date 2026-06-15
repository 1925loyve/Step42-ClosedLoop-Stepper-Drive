#include "MT6701.h"

extern SPI_HandleTypeDef hspi1;

static void MT6701_CsLow(void)
{
    HAL_GPIO_WritePin(MT6701_CSN_GPIO_Port, MT6701_CSN_Pin, GPIO_PIN_RESET);
}

static void MT6701_CsHigh(void)
{
    HAL_GPIO_WritePin(MT6701_CSN_GPIO_Port, MT6701_CSN_Pin, GPIO_PIN_SET);
}

static uint8_t MT6701_CalcCrc6(uint32_t data18)
{
    uint8_t crc = 0;

    for (int8_t i = 17; i >= 0; i--)
    {
        uint8_t bit = (data18 >> i) & 0x01U;
        uint8_t msb = (crc >> 5) & 0x01U;

        crc = (uint8_t)((crc << 1) & 0x3FU);
        if ((msb ^ bit) != 0U)
        {
            crc ^= 0x03U; /* x^6 + x + 1 */
        }
    }

    return crc & 0x3FU;
}

void MT6701_Init(void)
{
    MT6701_CsHigh();
}

MT6701_Status_t MT6701_ReadFrame(MT6701_Frame_t *frame)
{
    uint8_t rx[3] = {0};
    HAL_StatusTypeDef hal_status;

    if (frame == 0)
    {
        return MT6701_NULL_ERROR;
    }

    MT6701_CsLow();
    hal_status = HAL_SPI_Receive(&hspi1, rx, sizeof(rx), 10);
    MT6701_CsHigh();

    if (hal_status != HAL_OK)
    {
        return MT6701_HAL_ERROR;
    }

    uint32_t packet = ((uint32_t)rx[0] << 16) |
                      ((uint32_t)rx[1] << 8) |
                      ((uint32_t)rx[2]);
    uint32_t crc_data = (packet >> 6) & 0x3FFFFU;
    uint8_t received_crc = packet & 0x3FU;
    uint8_t calculated_crc = MT6701_CalcCrc6(crc_data);

    frame->raw_angle = (uint16_t)((packet >> 10) & 0x3FFFU);
    frame->magnetic_status = (uint8_t)((packet >> 6) & 0x0FU);
    frame->crc = received_crc;
    frame->crc_ok = (received_crc == calculated_crc) ? 1U : 0U;

    if (frame->crc_ok == 0U)
    {
        return MT6701_CRC_ERROR;
    }

    return MT6701_OK;
}

MT6701_Status_t MT6701_ReadRawAngle(uint16_t *raw_angle)
{
    MT6701_Frame_t frame;
    MT6701_Status_t status;

    if (raw_angle == 0)
    {
        return MT6701_NULL_ERROR;
    }

    status = MT6701_ReadFrame(&frame);
    if (status != MT6701_OK)
    {
        return status;
    }

    *raw_angle = frame.raw_angle;
    return MT6701_OK;
}

MT6701_Status_t MT6701_ReadRawAngleNoCrc(uint16_t *raw_angle)
{
    MT6701_Frame_t frame;
    MT6701_Status_t status;

    if (raw_angle == 0)
    {
        return MT6701_NULL_ERROR;
    }

    status = MT6701_ReadFrame(&frame);
    if ((status != MT6701_OK) && (status != MT6701_CRC_ERROR))
    {
        return status;
    }

    *raw_angle = frame.raw_angle;
    return MT6701_OK;
}

MT6701_Status_t MT6701_ReadAngleDeg(float *angle_deg)
{
    uint16_t raw_angle;
    MT6701_Status_t status;

    if (angle_deg == 0)
    {
        return MT6701_NULL_ERROR;
    }

    status = MT6701_ReadRawAngle(&raw_angle);
    if (status != MT6701_OK)
    {
        return status;
    }

    *angle_deg = MT6701_RawToDeg(raw_angle);
    return MT6701_OK;
}

MT6701_Status_t MT6701_ReadAngleDegNoCrc(float *angle_deg)
{
    uint16_t raw_angle;
    MT6701_Status_t status;

    if (angle_deg == 0)
    {
        return MT6701_NULL_ERROR;
    }

    status = MT6701_ReadRawAngleNoCrc(&raw_angle);
    if (status != MT6701_OK)
    {
        return status;
    }

    *angle_deg = MT6701_RawToDeg(raw_angle);
    return MT6701_OK;
}

float MT6701_RawToDeg(uint16_t raw_angle)
{
    return ((float)(raw_angle & 0x3FFFU)) * 360.0f / MT6701_RESOLUTION;
}
