/*
 * hal_gpio.c
 *
 * GPIO initialisation for the frtag2 custom board.
 * All GPIOs are brought to a known low-power state at boot.
 * Peripheral-specific pins (UART TX/RX, SPI SCK/MISO/MOSI) are
 * initialised by their respective HAL drivers.
 */

#include "hal_gpio.h"
#include "hal_bsp.h"

#include "stm32wlxx.h"

static bool bLedsAllowed = true;

void HAL_GPIO_vInit(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* LEDs */
    BSP_LED_Init(LED_RED);
    BSP_LED_Init(LED_YELLOW);

    /* GNSS power enable — single pin replaces 3-pin GPS power from frtag */
    HAL_GPIO_vInitOutput(BSP_GNSS_ON_PORT, BSP_GNSS_ON_PIN, GPIO_PIN_RESET);

    /* Accelerometer chip-select */
#ifdef ENABLE_MOVE
    HAL_GPIO_vInitOutput(BSP_ACC_CS_PORT, BSP_ACC_CS_PIN, GPIO_PIN_SET);
#else
    HAL_GPIO_vInitOutput(BSP_ACC_CS_PORT, BSP_ACC_CS_PIN, GPIO_PIN_RESET);
#endif
    /* ACC SPI SCK/MISO/MOSI are initialised by hal_spi */

    /* Battery measurement */
    HAL_GPIO_vInitOutput(BSP_BAT_BIAS_ENABLE_PORT, BSP_BAT_BIAS_ENABLE_PIN, GPIO_PIN_RESET);
    /* BAT_MEAS_VOLT is configured as analog by hal_adc */

    /* Farmranger interrupt output */
    HAL_GPIO_vInitOutput(BSP_FR_GPIO_INT_PORT, BSP_FR_GPIO_INT_PIN, GPIO_PIN_RESET);

    /* RF switch control — start LOW (RX/OFF) */
    HAL_GPIO_vInitOutput(BSP_RF_SW_CTRL_PORT, BSP_RF_SW_CTRL_PIN, GPIO_PIN_RESET);

    /* Device role sense */
    HAL_GPIO_vInitInput(BSP_ROLE_BIT0_PORT, BSP_ROLE_BIT0_PIN, GPIO_PULLDOWN);
}

/* Peripheral alternate-function pins that must not be left driven/floating in
 * STOP2 (they would leak through the bus or an idle external device). Parked as
 * analog Hi-Z before sleep and restored on wake. SPI1 = ACC, SPI2 = flash,
 * USART2 = debug, USART1 = GPS/Farmranger.
 * NOT included (intentionally held across STOP2 to keep externals in a known
 * state): PA0 GNSS_ON=low, PA4 BAT_BIAS=low, PA9 RF_SW=low, PA11 ACC_CS=high,
 * PA15 FLASH_CS=high, PB5/PB8 LEDs=low. ADC pins (PA12/PB3/PB4) are already
 * analog. GPIO state is retained in STOP2, so driven pins keep their level.
 *
 * PB12 (ROLE_BIT0) is also kept in the analog sleep mask: it is read once at
 * boot (DEVICE_DISCOVERY_vConfigDeviceRole) and tristated immediately after, so
 * it is already analog by the time we sleep. On a PRIMARY the strap is tied
 * HIGH, so leaving it input-pulldown would sink ~VDD/R_pd (~80 uA) continuously;
 * parking it here is belt-and-braces in case any future path re-drives it. */
/* PA13/PA14 are the SWD pins (SWDIO/SWCLK). They keep their reset-default AF0
 * with an internal pull-up (PA13) / pull-down (PA14). If the debug header carries
 * an opposing external pull, the two form a divider that leaks continuously
 * (~1.8V/(40k internal + 10k external) ~= 36 uA). Park them analog (no pull)
 * for STOP2 so neither internal pull is driven. Debug still works from reset up
 * to the first sleep; a deployed tag needs no SWD. */
#define HAL_GPIO_SLEEP_ANALOG_GPIOA  (BSP_ACC_SCK_PIN  | BSP_DEBUG_UART_TX_PIN | \
                                      BSP_DEBUG_UART_RX_PIN | BSP_FLASH_MISO_PIN | \
                                      BSP_ACC_MISO_PIN | BSP_ACC_MOSI_PIN | \
                                      BSP_FLASH_SCK_PIN | BSP_FLASH_MOSI_PIN | \
                                      GPIO_PIN_13 | GPIO_PIN_14)
#define HAL_GPIO_SLEEP_ANALOG_GPIOB  (BSP_GPS_UART_TX_PIN | BSP_GPS_UART_RX_PIN | \
                                      BSP_ROLE_BIT0_PIN)
#define HAL_GPIO_SLEEP_ANALOG_GPIOC  (BSP_ACC_INT_PIN)

void HAL_GPIO_vOnSleep(void)
{
    /* Park the peripheral AF pins as analog BEFORE gating the port clocks
     * (register writes need the clock on). */
    HAL_GPIO_vInitAnalogNoPull(GPIOA, HAL_GPIO_SLEEP_ANALOG_GPIOA);
    HAL_GPIO_vInitAnalogNoPull(GPIOB, HAL_GPIO_SLEEP_ANALOG_GPIOB);
    HAL_GPIO_vInitAnalogNoPull(GPIOC, HAL_GPIO_SLEEP_ANALOG_GPIOC);

    __HAL_RCC_GPIOA_CLK_DISABLE();
    __HAL_RCC_GPIOB_CLK_DISABLE();
    __HAL_RCC_GPIOC_CLK_DISABLE();
    __HAL_RCC_GPIOH_CLK_DISABLE();
}

void HAL_GPIO_OnWake(void)
{
    /* Only the GPIO port clocks are restored here — the minimum the idle 1 Hz
     * wake needs (LED indicator, role strap, and the lazy SPI restore that may
     * follow). No peripheral pins are re-armed on the wake path: a module that
     * needs the debug UART, SPI or ADC brings it up itself when it acquires the
     * sleep lock (DEBUG_vInit / the SPI select chokepoints / HAL_ADC_vLock), so
     * an idle wake that uses none of them pays nothing.
     *
     * The debug UART pins in particular are NOT re-armed here: while a sleep
     * lock is held the device stays on the LIGHT WFI path (no STOP2, no pin
     * parking), so DEBUG_vInit's pin setup survives the whole active window;
     * and during idle the 1 Hz UART marker is suppressed, so the pins are not
     * needed. USART1 (GPS/Farmranger) and ADC pins are likewise left to their
     * owners. */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
}

bool HAL_GPIO_bLedsAllowed(void)
{
    return bLedsAllowed;
}

void HAL_GPIO_vAllowLeds(bool allow)
{
    bLedsAllowed = allow;
}

void HAL_GPIO_vInitIntPullup(GPIO_TypeDef *GPIOx, uint32_t gpio_pin)
{
    GPIO_InitTypeDef gpio;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Mode  = GPIO_MODE_IT_RISING_FALLING;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Pin   = gpio_pin;
    HAL_GPIO_Init(GPIOx, &gpio);
}

void HAL_GPIO_vInitOutput(GPIO_TypeDef *GPIOx, uint32_t gpio_pin, GPIO_PinState gpio_pinstate)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Pin   = gpio_pin;
    HAL_GPIO_Init(GPIOx, &gpio);
    HAL_GPIO_WritePin(GPIOx, gpio_pin, gpio_pinstate);
}

void HAL_GPIO_vInitInput(GPIO_TypeDef *GPIOx, uint32_t gpio_pin, uint32_t gpio_pull)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = gpio_pull;
    gpio.Pin   = gpio_pin;
    HAL_GPIO_Init(GPIOx, &gpio);
}

void HAL_GPIO_vInitAnalogNoPull(GPIO_TypeDef *GPIOx, uint32_t gpio_pin)
{
    GPIO_InitTypeDef gpio;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Mode  = GPIO_MODE_ANALOG;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Pin   = gpio_pin;
    HAL_GPIO_Init(GPIOx, &gpio);
}
