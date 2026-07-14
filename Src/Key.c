#include "Key.h"

#define KEY_DEBOUNCE_MS 20U
#define KEY_LONG_PRESS_MS 1500U

typedef struct
{
    uint8_t last_raw;
    uint8_t stable;
    uint8_t long_reported;
    uint32_t changed_tick;
    uint32_t pressed_tick;
} Key_State_t;

static int8_t Key_Update(GPIO_TypeDef *port,
                         uint16_t pin,
                         Key_State_t *state,
                         uint32_t now,
                         int8_t short_event,
                         int8_t long_event)
{
    uint8_t raw = (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) ? 0U : 1U;

    if (raw != state->last_raw)
    {
        state->last_raw = raw;
        state->changed_tick = now;
    }

    if (((now - state->changed_tick) >= KEY_DEBOUNCE_MS) && (raw != state->stable))
    {
        state->stable = raw;
        if (raw == 0U)
        {
            state->pressed_tick = now;
            state->long_reported = 0U;
        }
        else if (state->long_reported == 0U)
        {
            return short_event;
        }
    }

    if ((state->stable == 0U) &&
        (state->long_reported == 0U) &&
        ((uint32_t)(now - state->pressed_tick) >= KEY_LONG_PRESS_MS))
    {
        state->long_reported = 1U;
        return long_event;
    }

    return 0;
}

int8_t Key_Scan(void)
{
    static Key_State_t key1 = {1U, 1U, 0U, 0U, 0U};
    static Key_State_t key2 = {1U, 1U, 0U, 0U, 0U};
    uint32_t now = HAL_GetTick();
    int8_t key;

    key = Key_Update(KEY1_GPIO_Port, KEY1_Pin, &key1, now,
                     KEY_EVENT_HOME_SHORT, KEY_EVENT_PROGRAM_LONG);
    if (key != 0)
    {
        return key;
    }

    key = Key_Update(KEY2_GPIO_Port, KEY2_Pin, &key2, now,
                     KEY_EVENT_ENABLE_SHORT, KEY_EVENT_STOP_LONG);
    if (key != 0)
    {
        return key;
    }

    return 0;
}
