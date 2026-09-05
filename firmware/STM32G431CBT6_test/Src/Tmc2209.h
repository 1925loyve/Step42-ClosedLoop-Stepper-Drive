#ifndef TMC2209_H
#define TMC2209_H

#include <stdint.h>

#define TMC2209_GCONF      0x00
#define TMC2209_GSTAT      0x01
#define TMC2209_IFCNT      0x02
#define TMC2209_IHOLD_IRUN 0x10
#define TMC2209_TPOWERDOWN 0x11
#define TMC2209_TPWMTHRS   0x13
#define TMC2209_VACTUAL    0x22
#define TMC2209_CHOPCONF   0x6C
#define TMC2209_PWMCONF    0x70

#define TMC2209_CHOPPER_SILENT 0U
#define TMC2209_CHOPPER_TORQUE 1U

uint8_t Tmc2209_Init(uint8_t tmc_addr);
uint8_t Tmc2209_InitAuto(uint8_t *detected_addr);
uint8_t Tmc2209_SetMicrosteps(uint16_t microsteps);
uint8_t Tmc2209_SetChopperMode(uint8_t mode);
uint8_t Tmc2209_GetChopperMode(void);
uint8_t Tmc2209_SetCurrent(uint8_t hold_current, uint8_t run_current);
uint8_t Tmc2209_GetHoldCurrent(void);
uint8_t Tmc2209_GetRunCurrent(void);


#endif

