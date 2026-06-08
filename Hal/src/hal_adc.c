/*
 * hal_adc.c
 *
 * ADC driver for battery voltage measurement.
 * Channel 8 (PA12) is the battery measurement input.
 */

#include "hal_adc.h"
#include "hal_bsp.h"
#include "stm32wlxx.h"

static volatile bool adc_flag;
static uint16_t      adc_value;

ADC_HandleTypeDef hadc;

void HAL_ADC_vInit(void)
{
    hadc.Instance                   = ADC;
    hadc.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV2;
    hadc.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hadc.Init.LowPowerAutoWait      = DISABLE;
    hadc.Init.LowPowerAutoPowerOff  = DISABLE;
    hadc.Init.ContinuousConvMode    = DISABLE;
    hadc.Init.NbrOfConversion       = 1;
    hadc.Init.DiscontinuousConvMode = DISABLE;
    hadc.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc.Init.DMAContinuousRequests = DISABLE;
    hadc.Init.Overrun               = ADC_OVR_DATA_PRESERVED;
    /* 79.5 cycles: required for the high-impedance VREFINT internal channel
     * to settle, and it also removes the undershoot on the high-impedance
     * battery divider. Both channels use common sampling time 1. */
    hadc.Init.SamplingTimeCommon1   = ADC_SAMPLETIME_79CYCLES_5;
    hadc.Init.SamplingTimeCommon2   = ADC_SAMPLETIME_79CYCLES_5;
    hadc.Init.OversamplingMode      = DISABLE;
    hadc.Init.TriggerFrequencyMode  = ADC_TRIGGER_FREQ_HIGH;

    if (HAL_ADC_Init(&hadc) != HAL_OK)
        Error_Handler();

    /* Enable the VREFINT internal channel path up front so its buffer is
     * stable long before the first battery sample (sampling runs every 10 s,
     * well past the ~12 us VREFINT stabilisation time). HAL_ADC_ConfigChannel
     * for the regular battery channel leaves this internal path untouched. */
    LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(hadc.Instance),
                                   LL_ADC_PATH_INTERNAL_VREFINT);
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *adcHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (adcHandle->Instance == ADC)
    {
        __HAL_RCC_ADC_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_InitStruct.Pin  = BSP_BAT_MEAS_VOLT_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(BSP_BAT_MEAS_VOLT_PORT, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(ADC_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(ADC_IRQn);
    }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *adcHandle)
{
    if (adcHandle->Instance == ADC)
    {
        __HAL_RCC_ADC_CLK_DISABLE();
        HAL_GPIO_DeInit(BSP_BAT_MEAS_VOLT_PORT, BSP_BAT_MEAS_VOLT_PIN);
        HAL_NVIC_DisableIRQ(ADC_IRQn);
    }
}

void HAL_ADC_vEnable(void)      { ADC_Enable(&hadc); }
bool HAL_ADC_bIsEnabled(void)   { return LL_ADC_IsEnabled(hadc.Instance) == 1; }
void HAL_ADC_vDisable(void)     { ADC_Disable(&hadc); }
bool HAL_ADC_bGetInterruptFlag(void)    { return adc_flag; }
void HAL_ADC_vClearInterruptFlag(void)  { adc_flag = false; }

void HAL_ADC_vSelectChannel(hal_adc_channel_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    switch (channel)
    {
        case BAT_VOLTAGE_CHANNEL:
            sConfig.Channel = BSP_BAT_ADC_VOLTAGE_CHANNEL;
            break;
        case VREFINT_CHANNEL:
            sConfig.Channel = ADC_CHANNEL_VREFINT;
            break;
        default:
            break;
    }
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
    HAL_ADC_ConfigChannel(&hadc, &sConfig);
}

void HAL_ADC_vStartConversion(hal_adc_channel_t channel)
{
    HAL_ADC_vSelectChannel(channel);
    HAL_ADC_vClearInterruptFlag();
    HAL_ADC_Start_IT(&hadc);
}

uint16_t HAL_ADC_u16GetResult(void)
{
    return adc_value;
}

uint16_t HAL_ADC_u16VddaFromVrefint(uint16_t u16VrefintRaw)
{
    if (u16VrefintRaw == 0U)
        return 0U;

    /* __LL_ADC_CALC_VREFANALOG_VOLTAGE uses the factory VREFINT calibration
     * (trimmed at VDDA = 3.0 V) to recover the true VDDA from a VREFINT
     * conversion, independent of the nominal rail (1.8 V here). */
    return (uint16_t)__LL_ADC_CALC_VREFANALOG_VOLTAGE(u16VrefintRaw,
                                                      LL_ADC_RESOLUTION_12B);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *my_hadc)
{
    UNUSED(my_hadc);
    adc_flag  = true;
    adc_value = (uint16_t)HAL_ADC_GetValue(&hadc);
}
