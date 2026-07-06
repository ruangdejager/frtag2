/*
 * MicroSD.c
 *
 * MicroSD card driver, SPI mode, raw 512-byte block access (no filesystem).
 * Shares the SPI2 bus + PA15 CS with the NOR-flash footprint; the two are
 * mutually exclusive hardware (storage_config.h), so this whole TU is empty
 * unless STORAGE_BACKEND_MICROSD is selected.
 *
 * Init handshake clocks at ~250 kHz (SPI mandates <=400 kHz until the card is
 * ready); block I/O then runs at the flash data rate (~0.5 MHz), which is the
 * known-good speed for this shared bus layout and is ample for the log + 1 Hz
 * accelerometer write loads.
 */

#include "storage_config.h"

#ifdef STORAGE_BACKEND_MICROSD

#include "MicroSD.h"
#include "MicroSD_Config.h"
#include "MicroSD_Driver.h"

#include "dbg_log.h"
#include "cmsis_os2.h"

/* -------------------------------------------------------------------------- */

static bool         bReady;
static bool         bBlockAddressing;   /* true = SDHC/SDXC (block addr), false = SDSC (byte addr) */
static uint32_t     u32BlockCount;

/* The SD card is on the shared SPI2 bus and, unlike the flash, is accessed by
 * two tasks: the DbgLog consumer (text log) and the Movement worker (1 Hz acc
 * data). This mutex serialises whole block transactions so the two can't
 * interleave on the wire. */
static osMutexId_t  xSdMutex;

static void SD_vLock(void)   { if (xSdMutex != NULL) (void)osMutexAcquire(xSdMutex, osWaitForever); }
static void SD_vUnlock(void) { if (xSdMutex != NULL) (void)osMutexRelease(xSdMutex); }

/* -------------------------------------------------------------------------- */

static uint8_t SD_u8Xchg(uint8_t u8Tx)
{
    uint8_t u8Rx = 0xFFU;
    MICROSD_DRIVER_vWriteRead(&u8Tx, &u8Rx, 1U);
    return u8Rx;
}

static void SD_vClock(uint16_t u16Bytes)
{
    while (u16Bytes-- != 0U)
        (void)SD_u8Xchg(0xFFU);
}

static void SD_vDeselectIdle(void)
{
    MICROSD_DRIVER_vDeselect();
    (void)SD_u8Xchg(0xFFU);   /* release MISO with one trailing clock */
}

/* Wait until the card releases the line (0xFF = not busy). */
static bool SD_bWaitReady(void)
{
    for (uint32_t i = 0U; i < SD_READY_WAIT_RETRIES; i++)
        if (SD_u8Xchg(0xFFU) == 0xFFU)
            return true;
    return false;
}

/* Issue a command; return the R1 response (bit7 clear) or 0xFF on no reply. */
static uint8_t SD_u8SendCmd(uint8_t u8Cmd, uint32_t u32Arg, uint8_t u8Crc)
{
    (void)SD_u8Xchg(0xFFU);   /* flush */

    (void)SD_u8Xchg((uint8_t)(0x40U | u8Cmd));
    (void)SD_u8Xchg((uint8_t)(u32Arg >> 24));
    (void)SD_u8Xchg((uint8_t)(u32Arg >> 16));
    (void)SD_u8Xchg((uint8_t)(u32Arg >> 8));
    (void)SD_u8Xchg((uint8_t)(u32Arg));
    (void)SD_u8Xchg(u8Crc);

    uint8_t u8R1 = 0xFFU;
    for (uint8_t i = 0U; i < 8U; i++)
    {
        u8R1 = SD_u8Xchg(0xFFU);
        if ((u8R1 & 0x80U) == 0U)
            break;
    }
    return u8R1;
}

/* Application command: CMD55 followed by the ACMD. */
static uint8_t SD_u8SendACmd(uint8_t u8Cmd, uint32_t u32Arg)
{
    (void)SD_u8SendCmd(SD_CMD55_APP_CMD, 0U, 0x01U);
    return SD_u8SendCmd(u8Cmd, u32Arg, 0x01U);
}

/* Read the CSD and derive the card capacity in 512-byte blocks (0 on error). */
static uint32_t SD_u32ReadCapacity(void)
{
    if (SD_u8SendCmd(SD_CMD9_SEND_CSD, 0U, 0x01U) != SD_R1_READY)
        return 0U;

    uint8_t u8Tok = 0xFFU;
    for (uint32_t i = 0U; i < SD_TOKEN_WAIT_RETRIES; i++)
    {
        u8Tok = SD_u8Xchg(0xFFU);
        if (u8Tok != 0xFFU)
            break;
    }
    if (u8Tok != SD_TOKEN_START_BLOCK)
        return 0U;

    uint8_t au8Csd[16];
    for (uint8_t i = 0U; i < 16U; i++)
        au8Csd[i] = SD_u8Xchg(0xFFU);
    (void)SD_u8Xchg(0xFFU);   /* CRC */
    (void)SD_u8Xchg(0xFFU);

    if ((au8Csd[0] >> 6) == 1U)
    {
        /* CSD version 2.0 (SDHC/SDXC): capacity = (C_SIZE + 1) * 512 KB. */
        uint32_t u32CSize = ((uint32_t)(au8Csd[7] & 0x3FU) << 16)
                          | ((uint32_t)au8Csd[8] << 8)
                          | (uint32_t)au8Csd[9];
        return (u32CSize + 1U) * 1024U;   /* 512 KB / 512 B = 1024 blocks */
    }

    /* CSD version 1.0 (SDSC). */
    uint32_t u32CSize = ((uint32_t)(au8Csd[6] & 0x03U) << 10)
                      | ((uint32_t)au8Csd[7] << 2)
                      | ((uint32_t)au8Csd[8] >> 6);
    uint8_t  u8CMult    = (uint8_t)(((au8Csd[9] & 0x03U) << 1) | (au8Csd[10] >> 7));
    uint8_t  u8ReadBlLen = (uint8_t)(au8Csd[5] & 0x0FU);
    uint32_t u32BlockNr  = (u32CSize + 1U) << (u8CMult + 2U);
    uint32_t u32BlockLen = 1UL << u8ReadBlLen;
    uint64_t u64Bytes    = (uint64_t)u32BlockNr * u32BlockLen;
    return (uint32_t)(u64Bytes / SD_BLOCK_SIZE);
}

/* -------------------------------------------------------------------------- */

bool MICROSD_vInit(void)
{
    bReady           = false;
    bBlockAddressing = false;
    u32BlockCount    = 0U;

    if (xSdMutex == NULL)
        xSdMutex = osMutexNew(NULL);

    /* No other task touches the SD bus until init returns, so the command
     * sequence below runs without taking the mutex. */

    /* SD power-up: VDD must be stable for >=250 ms before the init clocks.
     * The MCU boots in <50 ms, so we pay the remainder here. */
    osDelay(250U);

    /* >=74 clocks with CS de-asserted at <=400 kHz to enter SPI mode.
     * Use 20 bytes (160 clocks) — some cards need more than the 74-clock minimum. */
    MICROSD_DRIVER_vSetSpeed(SPI_BAUDRATEPRESCALER_128);   /* ~250 kHz */
    MICROSD_DRIVER_vDeselect();
    SD_vClock(20U);                                        /* 160 clocks */

    bool bIdle = false;
    uint8_t u8LastR1 = 0xFFU;
    for (uint8_t i = 0U; i < 10U; i++)
    {
        /* Deassert CS, give one trailing clock, then reassert before each
         * CMD0 attempt — required by the SD SPI spec for proper retry. */
        MICROSD_DRIVER_vDeselect();
        SD_vClock(1U);
        MICROSD_DRIVER_vSelect();

        u8LastR1 = SD_u8SendCmd(SD_CMD0_GO_IDLE_STATE, 0U, 0x95U);
        (void)SD_u8Xchg(0xFFU);   /* trailing clock after R1 */

        if (u8LastR1 == SD_R1_IDLE_STATE)
        {
            bIdle = true;
            break;
        }
        osDelay(10U);
    }
    if (!bIdle)
    {
        SD_vDeselectIdle();
        DBG_LOG("MicroSD: CMD0 (go-idle) failed (last R1=0x%02X)\r\n",
                (unsigned)u8LastR1);
        return false;
    }

    /* CMD8 distinguishes v2 cards and validates the voltage range. */
    bool    bV2 = false;
    uint8_t u8R1 = SD_u8SendCmd(SD_CMD8_SEND_IF_COND, SD_CMD8_ARG, 0x87U);
    if ((u8R1 & SD_R1_ILLEGAL_CMD) == 0U)
    {
        uint8_t au8Trailer[4];
        for (uint8_t i = 0U; i < 4U; i++)
            au8Trailer[i] = SD_u8Xchg(0xFFU);
        bV2 = (au8Trailer[3] == SD_CMD8_PATTERN);
    }

    /* ACMD41 (with HCS for v2) until the card leaves idle. */
    uint32_t u32Acmd41Arg = bV2 ? SD_ARG_HCS : 0U;
    bool     bInited = false;
    for (uint32_t i = 0U; i < SD_INIT_ACMD41_RETRIES; i++)
    {
        if (SD_u8SendACmd(SD_ACMD41_SEND_OP_COND, u32Acmd41Arg) == SD_R1_READY)
        {
            bInited = true;
            break;
        }
        osDelay(1);
    }
    if (!bInited)
    {
        SD_vDeselectIdle();
        DBG_LOG("MicroSD: ACMD41 init timeout\r\n");
        return false;
    }

    /* CMD58 OCR -> CCS bit (block vs byte addressing) for v2 cards. */
    if (bV2 && (SD_u8SendCmd(SD_CMD58_READ_OCR, 0U, 0x01U) == SD_R1_READY))
    {
        uint8_t au8Ocr[4];
        for (uint8_t i = 0U; i < 4U; i++)
            au8Ocr[i] = SD_u8Xchg(0xFFU);
        bBlockAddressing = ((uint32_t)au8Ocr[0] & 0x40U) != 0U;   /* OCR bit30 (CCS) */
    }

    /* SDSC cards address by byte and need an explicit 512-byte block length. */
    if (!bBlockAddressing)
        (void)SD_u8SendCmd(SD_CMD16_SET_BLOCKLEN, SD_BLOCK_SIZE, 0x01U);

    u32BlockCount = SD_u32ReadCapacity();

    SD_vDeselectIdle();

    /* Data phase at the flash rate (known-good on this shared bus). */
    MICROSD_DRIVER_vSetSpeed(SPI_BAUDRATEPRESCALER_64);

    if (u32BlockCount <= SD_ACC_LBA_START)
    {
        DBG_LOG("MicroSD: card too small (%lu blocks <= %lu log region)\r\n",
                (unsigned long)u32BlockCount, (unsigned long)SD_ACC_LBA_START);
        return false;
    }

    bReady = true;
    DBG_LOG("MicroSD: ready (%s, %lu blocks)\r\n",
            bBlockAddressing ? "SDHC/XC" : "SDSC", (unsigned long)u32BlockCount);
    return true;
}

bool MICROSD_bIsReady(void)
{
    return bReady;
}

uint32_t MICROSD_u32BlockCount(void)
{
    return u32BlockCount;
}

static bool SD_bReadBlockImpl(uint32_t u32Lba, uint8_t *pu8Buf)
{
    if (!bReady)
        return false;

    uint32_t u32Addr = bBlockAddressing ? u32Lba : (u32Lba * SD_BLOCK_SIZE);

    MICROSD_DRIVER_vSelect();

    if (SD_u8SendCmd(SD_CMD17_READ_SINGLE, u32Addr, 0x01U) != SD_R1_READY)
    {
        SD_vDeselectIdle();
        return false;
    }

    uint8_t u8Tok = 0xFFU;
    for (uint32_t i = 0U; i < SD_TOKEN_WAIT_RETRIES; i++)
    {
        u8Tok = SD_u8Xchg(0xFFU);
        if (u8Tok != 0xFFU)
            break;
    }
    if (u8Tok != SD_TOKEN_START_BLOCK)
    {
        SD_vDeselectIdle();
        return false;
    }

    MICROSD_DRIVER_vRead(pu8Buf, SD_BLOCK_SIZE);
    (void)SD_u8Xchg(0xFFU);   /* CRC */
    (void)SD_u8Xchg(0xFFU);

    SD_vDeselectIdle();
    return true;
}

static bool SD_bWriteBlockImpl(uint32_t u32Lba, const uint8_t *pu8Buf)
{
    if (!bReady)
        return false;

    uint32_t u32Addr = bBlockAddressing ? u32Lba : (u32Lba * SD_BLOCK_SIZE);

    MICROSD_DRIVER_vSelect();

    if (SD_u8SendCmd(SD_CMD24_WRITE_SINGLE, u32Addr, 0x01U) != SD_R1_READY)
    {
        SD_vDeselectIdle();
        return false;
    }

    (void)SD_u8Xchg(0xFFU);                 /* one-byte gap before the token */
    (void)SD_u8Xchg(SD_TOKEN_START_BLOCK);
    MICROSD_DRIVER_vWrite(pu8Buf, SD_BLOCK_SIZE);
    (void)SD_u8Xchg(0xFFU);                 /* dummy CRC */
    (void)SD_u8Xchg(0xFFU);

    uint8_t u8Resp = SD_u8Xchg(0xFFU);
    if ((u8Resp & SD_DATA_RESP_MASK) != SD_DATA_RESP_ACCEPTED)
    {
        SD_vDeselectIdle();
        return false;
    }

    if (!SD_bWaitReady())   /* wait out the internal programming */
    {
        SD_vDeselectIdle();
        return false;
    }

    SD_vDeselectIdle();
    return true;
}

bool MICROSD_bReadBlock(uint32_t u32Lba, uint8_t *pu8Buf)
{
    SD_vLock();
    bool bOk = SD_bReadBlockImpl(u32Lba, pu8Buf);
    SD_vUnlock();
    return bOk;
}

bool MICROSD_bWriteBlock(uint32_t u32Lba, const uint8_t *pu8Buf)
{
    SD_vLock();
    bool bOk = SD_bWriteBlockImpl(u32Lba, pu8Buf);
    SD_vUnlock();
    return bOk;
}

void MICROSD_vIdle(void)
{
    SD_vLock();
    SD_vDeselectIdle();
    SD_vUnlock();
}

#endif /* STORAGE_BACKEND_MICROSD */
