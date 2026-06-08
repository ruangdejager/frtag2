/*
 * hal_adc.c
 *
 * ADC driver — battery (PA12/IN8), VSOLAR (PB3/IN2), RSENSE (PB4/IN3).
 *
 * The peripheral is shared. Use HAL_ADC_vLock / HAL_ADC_vUnlock around
 * each full enable → convert → disable sequence.
 */

#include "hal_adc.h"
#include "hal_bsp.h"
#include "stm32wlxx.h"
#include "FreeRTOS.h"
#include "task.h"

static volatile bool   adc_flag;
static uint16_t        adc_value;
static osMutexId_t     adc_mutex;

ADC_HandleTypeDef hadc;

void HAL_ADC_vInit(void)
{
    hadc.Instance                   = ADC;
    /* PCLK is 48 MHz; /2 would give 24 MHz, above the STM32WL ADC's 16 MHz
     * limit. The low-impedance battery divider tolerated the overclock but the
     * high-impedance VREFINT channel converted ~2 %% high, corrupting the VDDA
     * estimate. /4 = 12 MHz keeps the ADC in spec. */
    hadc.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
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
    /* 160.5 cycles (~13 us at the 12 MHz ADC clock): the high-impedance
     * VREFINT internal channel needs a long settling window or it converts
     * high and corrupts the VDDA estimate. Both channels use common time 1. */
    hadc.Init.SamplingTimeCommon1   = ADC_SAMPLETIME_160CYCLES_5;
    hadc.Init.SamplingTimeCommon2   = ADC_SAMPLETIME_160CYCLES_5;
    hadc.Init.OversamplingMode      = DISABLE;
    hadc.Init.TriggerFrequencyMode  = ADC_TRIGGER_FREQ_HIGH;

    if (HAL_ADC_Init(&hadc) != HAL_OK)
        Error_Handler();

    /* Calibrate the ADC offset. The STM32WL SAR ADC powers up uncalibrated and
     * carries a stable few-percent error until this runs — which is what made
     * VREFINT read ~2 % high and corrupted the VDDA estimate. Must run with the
     * ADC disabled (it is, right after init). HAL_ADC_vInit() also runs after
     * each STOP2 wake, so this re-calibrates whenever the ADC has lost power. */
    if (HAL_ADCEx_Calibration_Start(&hadc) != HAL_OK)
        Error_Handler();

    /* Enable the VREFINT internal channel path up front so its buffer is
     * stable long before the first sample (sampling runs every 10 s, well past
     * the ~12 us VREFINT stabilisation time). HAL_ADC_ConfigChannel for the
     * regular battery/solar channels leaves this internal path untouched. */
    LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(hadc.Instance),
                                   LL_ADC_PATH_INTERNAL_VREFINT);

    /* The ADC is shared between the battery and solar workers — serialize.
     * Create the mutex ONCE. HAL_ADC_vInit() also runs on every STOP2 wake
     * (from HAL_SYSTEM_vEnterStop2, in the tickless-idle path with interrupts
     * masked). Re-creating the mutex there would (a) leak the previous one
     * every second until the heap is exhausted and configASSERT traps, and
     * (b) call pvPortMalloc from a masked-interrupt critical section, which is
     * illegal. The guard keeps creation on the boot path only. */
    if (adc_mutex == NULL)
    {
        adc_mutex = osMutexNew(NULL);
        configASSERT(adc_mutex != NULL);
    }
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

        __HAL_RCC_GPIOB_CLK_ENABLE();

        GPIO_InitStruct.Pin  = BSP_VSOLAR_MEAS_PIN;
        HAL_GPIO_Init(BSP_VSOLAR_MEAS_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Pin  = BSP_RSENSE_MEAS_PIN;
        HAL_GPIO_Init(BSP_RSENSE_MEAS_PORT, &GPIO_InitStruct);

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
        case VSOLAR_VOLTAGE_CHANNEL:
            sConfig.Channel = BSP_VSOLAR_ADC_CHANNEL;
            break;
        case RSENSE_VOLTAGE_CHANNEL:
            sConfig.Channel = BSP_RSENSE_ADC_CHANNEL;
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

void HAL_ADC_vLock(void)   { osMutexAcquire(adc_mutex, osWaitForever); }
void HAL_ADC_vUnlock(void) { osMutexRelease(adc_mutex); }
