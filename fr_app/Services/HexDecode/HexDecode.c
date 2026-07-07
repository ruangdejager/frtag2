/*
 * HexDecode.c
 *
 * Streaming "VS,<n>" + Intel HEX decoder. See HexDecode.h.
 *
 * Port notes (fr9 fota.c / hex_ln_to_bin.c):
 *  - The nibble-by-nibble line parser is the fr9's HEXLN2BIN_bConvert()
 *    verbatim in behaviour; only naming and state packaging changed.
 *  - All decoder state lives in one struct so a checkpoint is a struct copy.
 *  - fr9's per-byte variant silently reset on errors and relied on a caller
 *    timeout; here a fatal error latches an ERROR state and returns false so
 *    the transfer state machine can retry or abort deterministically.
 */

#include "HexDecode.h"
#include "OtaStore_Config.h"

#include <string.h>
#include <stdlib.h>

/* ---- Line/stream limits (fr9 parity) ---- */
#define HEXDECODE_LINE_BUF_LEN   48U   /* ":10...." data line = 43 chars + CRLF */
#define HEXDECODE_BIN_BUF_LEN    16U   /* max data bytes per HEX line           */
#define HEXDECODE_VS_DIGITS_MAX  5U    /* line count "VS,<= 5 digits>"          */

/* ---- Intel HEX record types ---- */
#define HEXDECODE_REC_DATA           0x00U
#define HEXDECODE_REC_EOF            0x01U
#define HEXDECODE_REC_EXT_SEG_ADDR   0x02U
#define HEXDECODE_REC_EXT_LIN_ADDR   0x04U
#define HEXDECODE_REC_LIN_START_ADDR 0x05U

/* ---- Stream states ---- */
typedef enum {
    HEXDECODE_STREAM_HEADER = 0,   /* waiting for "VS,<n>\r\n"        */
    HEXDECODE_STREAM_BODY,         /* decoding HEX lines              */
    HEXDECODE_STREAM_DONE,         /* EOF seen, line count matched    */
    HEXDECODE_STREAM_ERROR         /* fatal; init/restore to recover  */
} HexDecodeStream_e;

/* ---- Full decoder state (one struct => checkpoint is a copy) ---- */
typedef struct {
    HexDecodeStream_e eStream;

    /* line assembly */
    uint8_t  au8Line[HEXDECODE_LINE_BUF_LEN];
    uint8_t  u8LineIdx;

    /* header / progress */
    uint16_t u16LinesTotal;
    uint16_t u16LinesCount;
    uint32_t u32StopAddr;
    uint32_t u32DataBytes;

    /* Intel HEX line parser (fr9 hex_ln_to_bin globals, struct-packaged) */
    uint32_t u32ExtSegAddr;
    uint32_t u32ExtLinAddr;
} HexDecodeState_t;

static HexDecodeState_t tState;
static HexDecodeState_t tCheckpoint;
static HexDecodeWriteFn pfnWriteSink;

/* -------------------------------------------------------------------------- */

static uint8_t HEXDECODE_u8Nibble(uint8_t u8Char)
{
    if (u8Char >= '0' && u8Char <= '9') return (uint8_t)(u8Char - '0');
    if (u8Char >= 'A' && u8Char <= 'F') return (uint8_t)(u8Char - 'A' + 10U);
    if (u8Char >= 'a' && u8Char <= 'f') return (uint8_t)(u8Char - 'a' + 10U);
    return 0xFFU;
}

/* Parse one complete ":llaaaatt<data>cc" line from tState.au8Line.
 * On success fills the outputs and returns true; false = malformed line. */
static bool HEXDECODE_bParseLine(uint8_t *pu8Type, uint32_t *pu32Addr,
                                 uint8_t *pu8Data, uint8_t *pu8DataLen)
{
    const uint8_t *p = tState.au8Line;
    uint8_t u8Len = tState.u8LineIdx;

    /* Strip trailing CR/LF. */
    while (u8Len > 0U && (p[u8Len - 1U] == '\r' || p[u8Len - 1U] == '\n'))
        u8Len--;

    /* ':' + count(2) + addr(4) + type(2) + checksum(2) = 11 chars minimum,
     * and the payload must be whole hex-digit pairs. */
    if (u8Len < 11U || p[0] != ':' || ((u8Len - 1U) % 2U) != 0U)
        return false;

    uint8_t au8Bin[HEXDECODE_BIN_BUF_LEN + 5U];   /* count+addr+type+data+crc */
    uint8_t u8BinLen = (uint8_t)((u8Len - 1U) / 2U);
    if (u8BinLen > sizeof(au8Bin))
        return false;

    uint8_t u8Sum = 0U;
    for (uint8_t i = 0U; i < u8BinLen; i++)
    {
        uint8_t hi = HEXDECODE_u8Nibble(p[1U + 2U * i]);
        uint8_t lo = HEXDECODE_u8Nibble(p[2U + 2U * i]);
        if (hi > 0x0FU || lo > 0x0FU)
            return false;
        au8Bin[i] = (uint8_t)((hi << 4) | lo);
        u8Sum = (uint8_t)(u8Sum + au8Bin[i]);
    }

    /* Intel HEX record checksum: all bytes (incl. checksum) sum to zero. */
    if (u8Sum != 0U)
        return false;

    uint8_t u8DataLen = au8Bin[0];
    if (u8DataLen > HEXDECODE_BIN_BUF_LEN || (u8DataLen + 5U) != u8BinLen)
        return false;

    *pu8Type    = au8Bin[3];
    *pu32Addr   = ((uint32_t)au8Bin[1] << 8) | au8Bin[2];
    *pu8DataLen = u8DataLen;
    memcpy(pu8Data, &au8Bin[4], u8DataLen);
    return true;
}

/* Handle one complete, checksum-verified HEX line. Returns false on fatal. */
static bool HEXDECODE_bHandleBodyLine(void)
{
    uint8_t  u8Type;
    uint32_t u32Addr;
    uint8_t  au8Data[HEXDECODE_BIN_BUF_LEN];
    uint8_t  u8DataLen;

    if (!HEXDECODE_bParseLine(&u8Type, &u32Addr, au8Data, &u8DataLen))
        return false;

    switch (u8Type)
    {
        case HEXDECODE_REC_DATA:
        {
            /* Absolute address = record address + active segment bases
             * (fr9 parity: ext-linear and ext-segment are exclusive). */
            uint32_t u32Abs = u32Addr + tState.u32ExtSegAddr + tState.u32ExtLinAddr;

            if (u32Abs < OTA_APP_BASE_ADDR)
                return false;   /* below the app region (bootloader space) */

            uint32_t u32Offset = u32Abs - OTA_APP_BASE_ADDR;
            if ((u32Offset + u8DataLen) > OTA_APP_MAX_SIZE)
                return false;   /* image exceeds the scratchpad / app region */

            if (pfnWriteSink == NULL || !pfnWriteSink(u32Offset, au8Data, u8DataLen))
                return false;

            uint32_t u32Last = u32Abs + u8DataLen - 1UL;
            if (u32Last > tState.u32StopAddr)
                tState.u32StopAddr = u32Last;
            tState.u32DataBytes += u8DataLen;
            tState.u16LinesCount++;
            break;
        }

        case HEXDECODE_REC_EXT_SEG_ADDR:
            tState.u32ExtSegAddr = (((uint32_t)au8Data[0] << 8) | au8Data[1]) << 4;
            tState.u32ExtLinAddr = 0UL;
            tState.u16LinesCount++;
            break;

        case HEXDECODE_REC_EXT_LIN_ADDR:
            tState.u32ExtLinAddr = (((uint32_t)au8Data[0] << 8) | au8Data[1]) << 16;
            tState.u32ExtSegAddr = 0UL;
            tState.u16LinesCount++;
            break;

        case HEXDECODE_REC_LIN_START_ADDR:
            tState.u16LinesCount++;   /* informational; nothing to program */
            break;

        case HEXDECODE_REC_EOF:
            tState.u16LinesCount++;
            if (tState.u16LinesCount != tState.u16LinesTotal)
                return false;   /* lines went missing somewhere upstream */
            tState.eStream = HEXDECODE_STREAM_DONE;
            break;

        default:
            return false;   /* unknown record type */
    }
    return true;
}

/* Handle the "VS,<n>" header line. Returns false on a malformed header. */
static bool HEXDECODE_bHandleHeaderLine(void)
{
    tState.au8Line[tState.u8LineIdx] = '\0';

    char *s1 = strstr((char *)tState.au8Line, "VS,");
    if (s1 == NULL)
        return false;
    s1 += 3;

    char *s2 = strstr(s1, "\r");
    if (s2 == NULL || (size_t)(s2 - s1) > HEXDECODE_VS_DIGITS_MAX || s2 == s1)
        return false;

    *s2 = '\0';
    tState.u16LinesTotal = (uint16_t)atoi(s1);
    if (tState.u16LinesTotal == 0U)
        return false;

    tState.eStream = HEXDECODE_STREAM_BODY;
    return true;
}

/* --------------------------------------------------------------------------
 * HEXDECODE_vInit
 * -------------------------------------------------------------------------- */
void HEXDECODE_vInit(HexDecodeWriteFn pfnWrite)
{
    memset(&tState, 0, sizeof(tState));
    tState.eStream = HEXDECODE_STREAM_HEADER;
    pfnWriteSink   = pfnWrite;
    tCheckpoint    = tState;
}

/* --------------------------------------------------------------------------
 * HEXDECODE_bOnByte
 * -------------------------------------------------------------------------- */
bool HEXDECODE_bOnByte(uint8_t u8Byte)
{
    if (tState.eStream == HEXDECODE_STREAM_ERROR)
        return false;
    if (tState.eStream == HEXDECODE_STREAM_DONE)
        return true;    /* trailing bytes after EOF are ignored (fr9 parity) */

    /* Assemble a line; overlong lines are fatal (fr9 silently dropped them,
     * but a 48-char cap can only be exceeded by corruption). */
    if (tState.u8LineIdx >= (HEXDECODE_LINE_BUF_LEN - 1U))
    {
        tState.eStream = HEXDECODE_STREAM_ERROR;
        return false;
    }
    tState.au8Line[tState.u8LineIdx++] = u8Byte;

    if (u8Byte != '\n')
        return true;    /* line not complete yet */

    bool bOk = (tState.eStream == HEXDECODE_STREAM_HEADER)
             ? HEXDECODE_bHandleHeaderLine()
             : HEXDECODE_bHandleBodyLine();

    /* Reset line assembly for the next line. */
    tState.u8LineIdx = 0U;
    memset(tState.au8Line, 0, sizeof(tState.au8Line));

    if (!bOk)
        tState.eStream = HEXDECODE_STREAM_ERROR;
    return bOk;
}

/* -------------------------------------------------------------------------- */

bool HEXDECODE_bDone(void)
{
    return (tState.eStream == HEXDECODE_STREAM_DONE);
}

uint32_t HEXDECODE_u32StopAddr(void)
{
    return tState.u32StopAddr;
}

uint32_t HEXDECODE_u32DataBytes(void)
{
    return tState.u32DataBytes;
}

/* --------------------------------------------------------------------------
 * Checkpoint / restore — block-retry support
 * -------------------------------------------------------------------------- */
void HEXDECODE_vCheckpoint(void)
{
    tCheckpoint = tState;
}

void HEXDECODE_vRestore(void)
{
    tState = tCheckpoint;
}
