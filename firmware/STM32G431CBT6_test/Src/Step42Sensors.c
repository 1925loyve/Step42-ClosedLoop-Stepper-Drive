#include "Step42Sensors.h"
#include "main.h"
#include <math.h>

#define TEMP_SAMPLE_INTERVAL_MS (100UL)
#define TEMP_SAMPLE_COUNT (16U)
#define TEMP_ADC_FULL_SCALE (4095.0f)
#define TEMP_FIXED_RESISTOR_OHM (10000.0f)
#define TEMP_NTC_R25_OHM (10000.0f)
#define TEMP_NTC_BETA (3450.0f)
#define TEMP_T25_K (298.15f)
#define TEMP_WARNING_C (55.0f)
#define TEMP_SHUTDOWN_C (65.0f)
#define TEMP_HYSTERESIS_C (5.0f)

extern ADC_HandleTypeDef hadc1;

static uint32_t last_sample_ms = 0U;
static uint16_t adc_raw = 0U;
static float temperature_c = 25.0f;
static uint8_t sensor_flags = 0U;

void Step42Sensors_Init(void)
{
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) == HAL_OK)
    {
        sensor_flags = 0U;
    }
}

void Step42Sensors_Process(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t sum = 0U;
    uint32_t count = 0U;
    uint32_t i;
    float resistance;
    float inverse_temp;

    if ((uint32_t)(now - last_sample_ms) < TEMP_SAMPLE_INTERVAL_MS)
    {
        return;
    }
    last_sample_ms = now;

    for (i = 0U; i < TEMP_SAMPLE_COUNT; i++)
    {
        if (HAL_ADC_Start(&hadc1) != HAL_OK)
        {
            continue;
        }
        if (HAL_ADC_PollForConversion(&hadc1, 2U) == HAL_OK)
        {
            sum += HAL_ADC_GetValue(&hadc1);
            count++;
        }
        HAL_ADC_Stop(&hadc1);
    }

    if (count == 0U)
    {
        sensor_flags &= (uint8_t)~STEP42_SENSOR_TEMP_VALID;
        return;
    }

    adc_raw = (uint16_t)(sum / count);
    if ((adc_raw < 4U) || (adc_raw > 4091U))
    {
        sensor_flags &= (uint8_t)~STEP42_SENSOR_TEMP_VALID;
        return;
    }

    resistance = TEMP_FIXED_RESISTOR_OHM * (float)adc_raw /
                 (TEMP_ADC_FULL_SCALE - (float)adc_raw);
    inverse_temp = (1.0f / TEMP_T25_K) +
                   (logf(resistance / TEMP_NTC_R25_OHM) / TEMP_NTC_BETA);
    temperature_c = (1.0f / inverse_temp) - 273.15f;
    sensor_flags |= STEP42_SENSOR_TEMP_VALID;

    if (temperature_c >= TEMP_SHUTDOWN_C)
    {
        sensor_flags |= STEP42_SENSOR_TEMP_WARNING | STEP42_SENSOR_TEMP_SHUTDOWN;
    }
    else
    {
        if (temperature_c <= (TEMP_SHUTDOWN_C - TEMP_HYSTERESIS_C))
        {
            sensor_flags &= (uint8_t)~STEP42_SENSOR_TEMP_SHUTDOWN;
        }
        if (temperature_c >= TEMP_WARNING_C)
        {
            sensor_flags |= STEP42_SENSOR_TEMP_WARNING;
        }
        else if (temperature_c <= (TEMP_WARNING_C - TEMP_HYSTERESIS_C))
        {
            sensor_flags &= (uint8_t)~STEP42_SENSOR_TEMP_WARNING;
        }
    }
}

float Step42Sensors_GetTemperatureC(void)
{
    return temperature_c;
}

uint16_t Step42Sensors_GetAdcRaw(void)
{
    return adc_raw;
}

uint8_t Step42Sensors_GetFlags(void)
{
    return sensor_flags;
}
