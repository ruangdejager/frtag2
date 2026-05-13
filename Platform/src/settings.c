/*
 * settings.c
 *
 * Runtime settings module.
 * Settings are initialised from compile-time defaults (settings_def) and
 * held in RAM.  Accessor functions operate on raw byte offsets so the
 * storage layout can be changed by updating settings_default.h alone.
 *
 * Persistent storage (flash/EEPROM) is mediated through weak HAL stubs
 * HAL_STORAGE_vReadSettings / HAL_STORAGE_vWriteSettings which the
 * application can override.
 */

#include "settings.h"
#include "math_func.h"
#include "platform.h"
#include "str.h"

static settings_struct_t currentSettings = {0};

static char     holding_buffer[64];
static uint16_t holding_buffer_length = 64;

void SETTINGS_vInit(void)
{
    SETTINGS_vReadValues();
}

void SETTINGS_vReadValues(void)
{
    currentSettings = settings_def;
}

/* ---- uint8_t ------------------------------------------------------------- */
uint8_t SETTINGS_u8GetByteAtOffset(uint32_t offset)
{
    return *(uint8_t *)((uint8_t *)&currentSettings + offset);
}

void SETTINGS_vSetByteAtOffset(uint32_t offset, uint8_t value)
{
    *(uint8_t *)((uint8_t *)&currentSettings + offset) = value;
}

void SETTINGS_vResetByteAtOffset(uint32_t offset)
{
    *(uint8_t *)((uint8_t *)&currentSettings + offset) =
        *(uint8_t *)((uint8_t *)&SETTINGS_CONF_DEFAULT_STRUCT + offset);
}

/* ---- uint16_t ------------------------------------------------------------ */
uint16_t SETTINGS_u16GetShortAtOffset(uint32_t offset)
{
    return *(uint16_t *)((uint8_t *)&currentSettings + offset);
}

void SETTINGS_vSetShortAtOffset(uint32_t offset, uint16_t value)
{
    *(uint16_t *)((uint8_t *)&currentSettings + offset) = value;
}

void SETTINGS_vResetShortAtOffset(uint32_t offset)
{
    *(uint16_t *)((uint8_t *)&currentSettings + offset) =
        *(uint16_t *)((uint8_t *)&SETTINGS_CONF_DEFAULT_STRUCT + offset);
}

/* ---- uint32_t ------------------------------------------------------------ */
uint32_t SETTINGS_u32GetWordAtOffset(uint32_t offset, uint32_t extra)
{
    return *(uint32_t *)((uint8_t *)&currentSettings + offset + extra);
}

void SETTINGS_vSetWordAtOffset(uint32_t offset, uint32_t extra, uint32_t value)
{
    *(uint32_t *)((uint8_t *)&currentSettings + offset + extra) = value;
}

void SETTINGS_vResetWordAtOffset(uint32_t offset, uint32_t extra)
{
    *(uint32_t *)((uint8_t *)&currentSettings + offset + extra) =
        *(uint32_t *)((uint8_t *)&SETTINGS_CONF_DEFAULT_STRUCT + offset + extra);
}

/* ---- strings ------------------------------------------------------------- */
void SETTINGS_vSetStringAtOffset(uint32_t offset, size_t length_at_offset,
                                  const char *new_value, size_t dest_length)
{
    uint32_t min_length = (dest_length < length_at_offset) ? dest_length : length_at_offset;
    memset((uint8_t *)&currentSettings + offset, 0, length_at_offset);
    memcpy((uint8_t *)&currentSettings + offset, new_value, min_length);
}

char *SETTINGS_vGetStringAtOffset(uint32_t offset, size_t length_at_offset,
                                   char *buf, size_t length)
{
    if (buf == NULL)
    {
        memset(holding_buffer, '\0', holding_buffer_length);
        memcpy(holding_buffer, (char *)((uint8_t *)&currentSettings + offset), length_at_offset);
        return holding_buffer;
    }
    memset(buf, '\0', length);
    memcpy(buf, (char *)((uint8_t *)&currentSettings + offset),
           min(length_at_offset, length));
    return NULL;
}

bool SETTINGS_bCheckStringAtOffset(uint32_t offset, size_t length_at_offset, bool mayBeEmpty)
{
    return STR_bIsValid(SETTINGS_vGetStringAtOffset(offset, length_at_offset, NULL, 0),
                        (uint8_t)length_at_offset, mayBeEmpty);
}

void SETTINGS_vResetStringAtOffset(uint32_t offset, size_t length)
{
    uint8_t *source = (uint8_t *)&SETTINGS_CONF_DEFAULT_STRUCT + offset;
    uint8_t *dest   = (uint8_t *)&currentSettings + offset;
    memset(dest, 0, length);
    memcpy(dest, source, length);
}

/* ---- bool ---------------------------------------------------------------- */
void SETTINGS_vSetBoolAtOffset(uint32_t offset, uint32_t extra, bool value)
{
    *(bool *)((uint8_t *)&currentSettings + offset + extra) = value;
}

bool SETTINGS_bGetBoolAtOffset(uint32_t offset, uint32_t extra)
{
    return *(bool *)((uint8_t *)&currentSettings + offset + extra);
}

void SETTINGS_vResetBoolAtOffset(uint32_t offset, uint32_t extra)
{
    *(bool *)((uint8_t *)&currentSettings + offset + extra) =
        *(bool *)((uint8_t *)&SETTINGS_CONF_DEFAULT_STRUCT + offset + extra);
}

/* ---- struct pointer ------------------------------------------------------ */
void *SETTINGS_vpGetStructureAtOffset(uint32_t offset)
{
    return (void *)&currentSettings + offset;
}

settings_struct_t *SETTINGS_pGetSettingsStruct(void)
{
    return &currentSettings;
}

/* ---- Weak HAL storage stubs --------------------------------------------- */
uint32_t __attribute__((weak)) HAL_STORAGE_vReadSettings(uint32_t *length)
{
    (void)length;
    return 0;
}

uint32_t __attribute__((weak)) HAL_STORAGE_vWriteSettings(uint32_t length)
{
    (void)length;
    return 0;
}
