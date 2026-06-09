/*
 * Flash.c
 *
 * AT25EU0041A SPI NOR flash device driver.
 * Follows the fr9 flash.c command pattern (JEDEC-compatible command set).
 *
 * All SPI transactions use the driver macros in Flash_Driver.h which
 * route through the HAL SPI2 layer in hal_spi.h.
 */

#include "Flash.h"
#include "Flash_Config.h"
#include "Flash_Driver.h"
#include "dbg_log.h"
#include "cmsis_os2.h"

/* --------------------------------------------------------------------------
 * Internal helpers
 * -------------------------------------------------------------------------- */

static void FLASH_vWaitReady(void)
{
    while (FLASH_bDeviceBusy())
        osDelay(1);
}

static void FLASH_vWriteEnable(void)
{
    uint8_t cmd = FLASH_CMD_WRITE_ENABLE;
    FLASH_DRIVER_vSelect();
    FLASH_DRIVER_vWrite(&cmd, 1);
    FLASH_DRIVER_vDeselect();
}

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

void FLASH_vInit(void)
{
    FLASH_vReleaseDeepPowerDown();
    osDelay(1);

    /* Read and report the raw JEDEC ID so a mismatch shows what came back:
     * 00s => no clock/data (AF or wiring), FFs => MISO idle / CS not asserting,
     * other => CPOL/CPHA or wrong part. */
    uint8_t cmd   = FLASH_CMD_JEDEC_ID;
    uint8_t id[3] = {0};
    FLASH_DRIVER_vSelect();
    FLASH_DRIVER_vWrite(&cmd, 1);
    FLASH_DRIVER_vRead(id, 3);
    FLASH_DRIVER_vDeselect();

    if (id[0] != FLASH_MANUFACTURER_ID)
        DBG("FLASH: JEDEC ID mismatch - got %02X %02X %02X (expected mfr %02X)\r\n",
            id[0], id[1], id[2], FLASH_MANUFACTURER_ID);
    else
        DBG("FLASH: JEDEC ID %02X %02X %02X OK\r\n", id[0], id[1], id[2]);

    FLASH_vChipErase();

}

bool FLASH_bDeviceBusy(void)
{
    return (FLASH_u8ReadStatusReg() & FLASH_STATUS_WIP) != 0U;
}

uint8_t FLASH_u8ReadStatusReg(void)
{
    uint8_t cmd = FLASH_CMD_READ_STATUS;
    uint8_t status;
    FLASH_DRIVER_vSelect();
    FLASH_DRIVER_vWrite(&cmd, 1);
    FLASH_DRIVER_vRead(&status, 1);
    FLASH_DRIVER_vDeselect();
    return status;
}

bool FLASH_bVerifyDevice(void)
{
    uint8_t cmd    = FLASH_CMD_JEDEC_ID;
    uint8_t id[3]  = {0};
    FLASH_DRIVER_vSelect();
    FLASH_DRIVER_vWrite(&cmd, 1);
    FLASH_DRIVER_vRead(id, 3);
    FLASH_DRIVER_vDeselect();
    return id[0] == FLASH_MANUFACTURER_ID;
}

void FLASH_vRead(uint32_t addr, uint8_t *buf, uint16_t len)
{
    uint8_t cmd[4] = {
        FLASH_CMD_READ,
        (uint8_t)((addr >> 16) & 0xFFU),
        (uint8_t)((addr >>  8) & 0xFFU),
        (uint8_t)((addr      ) & 0xFFU),
    };
    FLASH_vWaitReady();
    FLASH_DRIVER_vSelect();
    FLASH_DRIVER_vWrite(cmd, 4);
    FLASH_DRIVER_vRead(buf, len);
    FLASH_DRIVER_vDeselect();
}

void FLASH_vPageWrite(uint32_t addr, const uint8_t *buf, uint16_t len)
{
    uint8_t cmd[4] = {
        FLASH_CMD_PAGE_PROGRAM,
        (uint8_t)((addr >> 16) & 0xFFU),
        (uint8_t)((addr >>  8) & 0xFFU),
        (uint8_t)((addr      ) & 0xFFU),
    };
    FLASH_vWaitReady();
    FLASH_vWriteEnable();
    FLASH_DRIVER_vSelect();
    FLASH_DRIVER_vWrite(cmd, 4);
    FLASH_DRIVER_vWrite(buf, len);
    FLASH_DRIVER_vDeselect();
}

void FLASH_vSectorErase(uint32_t addr)
{
    uint8_t cmd[4] = {
        FLASH_CMD_SECTOR_ERASE,
        (uint8_t)((addr >> 16) & 0xFFU),
        (uint8_t)((addr >>  8) & 0xFFU),
        (uint8_t)((addr      ) & 0xFFU),
    };
    FLASH_vWaitReady();
    FLASH_vWriteEnable();
    FLASH_DRIVER_vSelect();
    FLASH_DRIVER_vWrite(cmd, 4);
    FLASH_DRIVER_vDeselect();
}

void FLASH_vBlockErase(uint32_t addr)
{
    uint8_t cmd[4] = {
        FLASH_CMD_BLOCK_ERASE,
        (uint8_t)((addr >> 16) & 0xFFU),
        (uint8_t)((addr >>  8) & 0xFFU),
        (uint8_t)((addr      ) & 0xFFU),
    };
    FLASH_vWaitReady();
    FLASH_vWriteEnable();
    FLASH_DRIVER_vSelect();
    FLASH_DRIVER_vWrite(cmd, 4);
    FLASH_DRIVER_vDeselect();
}

void FLASH_vChipErase(void)
{
    uint8_t cmd = FLASH_CMD_CHIP_ERASE;
    FLASH_vWaitReady();
    FLASH_vWriteEnable();
    FLASH_DRIVER_vSelect();
    FLASH_DRIVER_vWrite(&cmd, 1);
    FLASH_DRIVER_vDeselect();
    FLASH_vWaitReady();
}

void FLASH_vDeepPowerDown(void)
{
    uint8_t cmd = FLASH_CMD_DEEP_PWR_DOWN;
    FLASH_DRIVER_vSelect();
    FLASH_DRIVER_vWrite(&cmd, 1);
    FLASH_DRIVER_vDeselect();
}

void FLASH_vReleaseDeepPowerDown(void)
{
    uint8_t cmd = FLASH_CMD_RESUME;
    FLASH_DRIVER_vSelect();
    FLASH_DRIVER_vWrite(&cmd, 1);
    FLASH_DRIVER_vDeselect();
}
