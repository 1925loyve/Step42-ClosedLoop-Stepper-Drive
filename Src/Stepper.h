#ifndef STEPPER_H
#define STEPPER_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>

// --- 定义运行模式 ---
typedef enum {
    MODE_POSITION = 0, // 位置模式：走固定步数
    MODE_SPEED    = 1  // 速度模式：PID追踪、连续运动
} Stepper_Mode_t;

typedef enum {
    DIR_FORWARD = 1,
    DIR_REVERSE = 0
} Stepper_Direction_t;

typedef enum{
    TMC_EN = 1,
    TMC_UNEN = 0
}Stepper_ENN;

typedef enum {
    STOP, ACCEL, DECEL, RUN
} Stepper_State_t;

typedef struct {
    // -- 硬件资源 --
    TIM_HandleTypeDef*  htim;
    uint32_t            tim_channel;
    GPIO_TypeDef*       dir_port;
    uint16_t            dir_pin;
    uint32_t            timer_freq; // 定时器频率 (Hz)

    // -- 模式控制 --
    Stepper_Mode_t      run_mode;   //运行模式
		Stepper_State_t     state;
    float               accel;
    
		// -- 位置模式专用变量 --
    uint32_t            decel_start_step;// 开始减速的步数位置
    uint32_t            steps_to_move;// 本次运动需要移动的总步数(正数)
    uint32_t            irq_call_count;// 中断调用次数 (等同于本次运动已走的步数)
    uint32_t            step_delay; // 当前步进延时 (ARR值)
    uint32_t            min_delay;  //最小延时 (对应最高速)
    float               decel;
    float               max_speed;
    volatile int32_t    target_pos;      // 目标绝对位置 (带符号)
		uint32_t            accel_count;     // 加速计数器

    // -- 速度模式专用变量 (核心) --
    volatile float      target_speed;  // 目标速度 (带符号, steps/s)
    volatile float      current_speed; // 当前实时速度 (带符号, steps/s)
    
    // -- 位置记录 --
    volatile int32_t    current_pos;
    volatile Stepper_Direction_t direction;

} Stepper_t;

// --- API 函数 ---
// 初始化增加 mode 参数
void Stepper_Init(Stepper_t *stepper, TIM_HandleTypeDef *htim, uint32_t tim_channel,
				  GPIO_TypeDef *dir_port, uint16_t dir_pin,Stepper_Mode_t mode);

void Stepper_EN(GPIO_TypeDef *en_port, uint16_t en_pin,Stepper_ENN FEN);

// 统一的配置函数 (在速度模式下，max_speed 即为目标速度)
void Stepper_Set_Speed_Profile(Stepper_t *stepper, float speed, float accel);

// 位置模式移动
void Stepper_Move(Stepper_t *stepper, int32_t relative_steps);

// 速度模式启动 (无参，仅用于开启中断)
void Stepper_Run_Speed(Stepper_t *stepper);

// 强制停止
void Stepper_Stop(Stepper_t *stepper);

// 中断处理
void Stepper_TIM_IRQ_Handler(Stepper_t *stepper);

#endif