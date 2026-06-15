#include "Key.h"

#define KEY_DEBOUNCE_MS 20U

static int8_t Key_Update(GPIO_TypeDef *port,
                         uint16_t pin,
                         uint8_t *last_raw,
                         uint8_t *stable,
                         uint32_t *changed_tick,
                         uint32_t now,
                         int8_t event)
{
    uint8_t raw = (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) ? 0U : 1U;

    if (raw != *last_raw)
    {
        *last_raw = raw;
        *changed_tick = now;
    }

    if (((now - *changed_tick) >= KEY_DEBOUNCE_MS) && (raw != *stable))
    {
        *stable = raw;
        if (raw == 0U)
        {
            return event;
        }
    }

    return 0;
}

int8_t Key_Scan(void)
{
    static uint8_t key1_last_raw = 1U;
    static uint8_t key2_last_raw = 1U;
    static uint8_t key1_stable = 1U;
    static uint8_t key2_stable = 1U;
    static uint32_t key1_changed_tick = 0U;
    static uint32_t key2_changed_tick = 0U;
    uint32_t now = HAL_GetTick();
    int8_t key;

    key = Key_Update(KEY1_GPIO_Port, KEY1_Pin, &key1_last_raw, &key1_stable, &key1_changed_tick, now, 1);
    if (key != 0)
    {
        return key;
    }

    key = Key_Update(KEY2_GPIO_Port, KEY2_Pin, &key2_last_raw, &key2_stable, &key2_changed_tick, now, 2);
    if (key != 0)
    {
        return key;
    }

    return 0;
}
