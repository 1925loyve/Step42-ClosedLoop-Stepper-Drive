

#include "Tmc2209.h"
#include <string.h>
#include "delay.h"
#include "led.h"
#include "main.h"

extern UART_HandleTypeDef huart1;

/*
 *
 * 这份工程当前实际只写 4 个核心寄存器：
 * 1. GCONF       (0x00)  全局功能开关
 * 2. IHOLD_IRUN  (0x10)  保持/运行电流
 * 3. VACTUAL     (0x22)  内部速度模式，这里固定为 0，不使用
 * 4. CHOPCONF    (0x6C)  细分/斩波核心配置
 */
static const uint32_t TMC2209_GCONF_BASE_VALUE = 0x00000180;
/*
 * CHOPCONF.MRES = 8 -> full-step external STEP input.
 * Change bits 24..27 to switch microstep resolution for precision tests.
 */
static const uint32_t TMC2209_CHOPCONF_VALUE = 0x13000053;
static const uint32_t TMC2209_IHOLD_IRUN_VALUE = 0x00060C04;
static const uint32_t TMC2209_VACTUAL_VALUE = 0x00000000;
static uint8_t tmc2209_active_addr = 0U;
static uint8_t tmc2209_chopper_mode = TMC2209_CHOPPER_SILENT;

static uint8_t TMC2209_MicrostepsToMres(uint16_t microsteps, uint8_t *mres)
{
    if (mres == 0)
    {
        return 0;
    }

    switch (microsteps)
    {
    case 256:
        *mres = 0;
        return 1;
    case 128:
        *mres = 1;
        return 1;
    case 64:
        *mres = 2;
        return 1;
    case 32:
        *mres = 3;
        return 1;
    case 16:
        *mres = 4;
        return 1;
    case 8:
        *mres = 5;
        return 1;
    case 4:
        *mres = 6;
        return 1;
    case 2:
        *mres = 7;
        return 1;
    case 1:
        *mres = 8;
        return 1;
    default:
        return 0;
    }
}

static void TMC2209_UART_Clear(void)
{
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    __HAL_UART_CLEAR_FEFLAG(&huart1);
    __HAL_UART_CLEAR_NEFLAG(&huart1);
    __HAL_UART_CLEAR_PEFLAG(&huart1);
    __HAL_UART_FLUSH_DRREGISTER(&huart1);
}

static void TMC2209_UART_RxEnable(uint8_t enable)
{
    if (enable)
    {
        SET_BIT(huart1.Instance->CR1, USART_CR1_RE);
    }
    else
    {
        CLEAR_BIT(huart1.Instance->CR1, USART_CR1_RE);
    }
}

void tmc_calc_crc(uint8_t *dat, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < (len - 1); i++)
    {
        uint8_t currentByte = dat[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if ((crc >> 7) ^ (currentByte & 0x01))
            {
                crc = (crc << 1) ^ 0x07;
            }
            else
            {
                crc = (crc << 1);
            }
            currentByte >>= 1;
        }
    }
    dat[len - 1] = crc;
}

void TMC2209_WriteReg(uint8_t address, uint8_t reg, uint32_t val)
{
    uint8_t txBuffer[8];
    txBuffer[0] = 0x05;
    txBuffer[1] = address;
    txBuffer[2] = reg | 0x80;
    txBuffer[3] = (val >> 24) & 0xFF;
    txBuffer[4] = (val >> 16) & 0xFF;
    txBuffer[5] = (val >> 8) & 0xFF;
    txBuffer[6] = val & 0xFF;
    tmc_calc_crc(txBuffer, 8);

    TMC2209_UART_Clear();
    TMC2209_UART_RxEnable(0);
    HAL_UART_Transmit(&huart1, txBuffer, 8, 10);
    TMC2209_UART_RxEnable(1);
    TMC2209_UART_Clear();
}

uint8_t TMC2209_ReadReg_Robust(uint8_t tmc_addr, uint8_t reg, uint32_t *val_out)
{
    uint8_t txBuffer[4];
    uint8_t rxBuffer[24];

    txBuffer[0] = 0x05;
    txBuffer[1] = tmc_addr;
    txBuffer[2] = reg;
    tmc_calc_crc(txBuffer, 4);

    TMC2209_UART_Clear();
    memset(rxBuffer, 0, sizeof(rxBuffer));

    TMC2209_UART_RxEnable(0);
    HAL_UART_Transmit(&huart1, txBuffer, 4, 10);
    TMC2209_UART_RxEnable(1);

    if (HAL_UART_Receive(&huart1, rxBuffer, 8, 50) != HAL_OK)
    {
        TMC2209_UART_Clear();
        return 0;
    }

    // 滑动窗口搜索特征头 (0x05 0xFF)
    for (uint8_t i = 0; i <= (8 - 8); i++)
    {
        // 特征：当前是 0x05，下一个是 0xFF (主机地址)
        if (rxBuffer[i] == 0x05 && rxBuffer[i + 1] == 0xFF)
        {
            if (rxBuffer[i + 2] != reg)
            {
                continue;
            }

            // 5.2 提取这 8 个字节进行 CRC 校验
            uint8_t potential_packet[8];
            memcpy(potential_packet, &rxBuffer[i], 8);

            uint8_t received_crc = potential_packet[7];
            tmc_calc_crc(potential_packet, 8);

            if (received_crc == potential_packet[7])
            {
                *val_out = 0;
                *val_out |= (uint32_t)potential_packet[3] << 24;
                *val_out |= (uint32_t)potential_packet[4] << 16;
                *val_out |= (uint32_t)potential_packet[5] << 8;
                *val_out |= (uint32_t)potential_packet[6];
                return 1; // 成功
            }
        }
    }

    return 0;
}

uint8_t Tmc2209_Init(uint8_t tmc_addr)
{
    tmc2209_active_addr = tmc_addr;
    delay_ms(100);
    tmc2209_chopper_mode = TMC2209_CHOPPER_SILENT;
    TMC2209_WriteReg(tmc_addr, TMC2209_GCONF, TMC2209_GCONF_BASE_VALUE);
    TMC2209_WriteReg(tmc_addr, TMC2209_CHOPCONF, TMC2209_CHOPCONF_VALUE);
    TMC2209_WriteReg(tmc_addr, TMC2209_IHOLD_IRUN, TMC2209_IHOLD_IRUN_VALUE);
    TMC2209_WriteReg(tmc_addr, TMC2209_VACTUAL, TMC2209_VACTUAL_VALUE);

    delay_ms(50);

    uint32_t read_val = 0;
    uint8_t retry = 0;
    uint8_t read_ok = 0;

    const uint32_t expected_gconf_mask = 0x00000180;
    while (retry < 10)
    {
        if (TMC2209_ReadReg_Robust(tmc_addr, TMC2209_GCONF, &read_val))
        {
            if ((read_val & expected_gconf_mask) == expected_gconf_mask)
            {
                read_ok = 1;
                break;
            }
        }
        retry++;
        delay_ms(200);
    }

    if (read_ok)
    {
        return 1;
    }

    return 0;
}

uint8_t Tmc2209_SetMicrosteps(uint16_t microsteps)
{
    uint8_t mres;
    uint32_t chopconf;
    uint32_t read_val = 0;

    if (TMC2209_MicrostepsToMres(microsteps, &mres) == 0)
    {
        return 0;
    }

    chopconf = (TMC2209_CHOPCONF_VALUE & ~(0x0FUL << 24)) | (((uint32_t)mres) << 24);
    TMC2209_WriteReg(tmc2209_active_addr, TMC2209_CHOPCONF, chopconf);
    delay_ms(2);

    if (TMC2209_ReadReg_Robust(tmc2209_active_addr, TMC2209_CHOPCONF, &read_val) == 0)
    {
        return 0;
    }

    return (((read_val >> 24) & 0x0FU) == mres) ? 1U : 0U;
}

uint8_t Tmc2209_SetChopperMode(uint8_t mode)
{
    uint32_t value;
    uint32_t read_val = 0U;

    if ((mode != TMC2209_CHOPPER_SILENT) &&
        (mode != TMC2209_CHOPPER_TORQUE))
    {
        return 0U;
    }

    value = TMC2209_GCONF_BASE_VALUE;
    if (mode == TMC2209_CHOPPER_TORQUE)
    {
        value |= (1UL << 2); /* GCONF.en_spreadCycle */
    }

    TMC2209_WriteReg(tmc2209_active_addr, TMC2209_GCONF, value);
    delay_ms(2);
    if (TMC2209_ReadReg_Robust(tmc2209_active_addr, TMC2209_GCONF, &read_val) == 0U)
    {
        return 0U;
    }
    if (((read_val >> 2) & 0x01U) != (uint32_t)mode)
    {
        return 0U;
    }

    tmc2209_chopper_mode = mode;
    return 1U;
}

uint8_t Tmc2209_GetChopperMode(void)
{
    return tmc2209_chopper_mode;
}

uint8_t Tmc2209_InitAuto(uint8_t *detected_addr)
{
    uint32_t read_val = 0;

    delay_ms(100);

    for (uint8_t retry = 0; retry < 3; retry++)
    {
        for (uint8_t addr = 0; addr < 4; addr++)
        {
            if (TMC2209_ReadReg_Robust(addr, TMC2209_GCONF, &read_val))
            {
                if (Tmc2209_Init(addr))
                {
                    if (detected_addr != 0)
                    {
                        *detected_addr = addr;
                    }
                    return 1;
                }
            }
            delay_ms(20);
        }
    }

    return 0;
}
