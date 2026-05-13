/*
 * settings.h
 *
 * Runtime settings module.
 * Settings live in RAM and are initialised from the compile-time defaults
 * in settings_default.h.  Accessor macros translate field names to byte
 * offsets so the underlying storage can be changed without touching callers.
 */

#ifndef SETTINGS_H_
#define SETTINGS_H_

#include <stdint.h>
#include <stddef.h>
#include "settings_default.h"

typedef Settings_EE settings_struct_t;

extern Settings_EE settings_def;

#define SETTINGS_CONF_DEFAULT_STRUCT    settings_def

void SETTINGS_vInit(void);
void SETTINGS_vReadValues(void);

/* uint8_t */
uint8_t  SETTINGS_u8GetByteAtOffset(uint32_t offset);
void     SETTINGS_vSetByteAtOffset(uint32_t offset, uint8_t value);
bool     SETTINGS_bCheckByteAtOffset(uint32_t offset);
void     SETTINGS_vResetByteAtOffset(uint32_t offset);

/* uint16_t */
uint16_t SETTINGS_u16GetShortAtOffset(uint32_t offset);
void     SETTINGS_vSetShortAtOffset(uint32_t offset, uint16_t value);
bool     SETTINGS_bCheckShortAtOffset(uint32_t offset);
void     SETTINGS_vResetShortAtOffset(uint32_t offset);

/* uint32_t */
uint32_t SETTINGS_u32GetWordAtOffset(uint32_t offset, uint32_t extra);
void     SETTINGS_vSetWordAtOffset(uint32_t offset, uint32_t extra, uint32_t value);
bool     SETTINGS_bCheckWordAtOffset(uint32_t offset, uint32_t extra, bool is_signed);
void     SETTINGS_vResetWordAtOffset(uint32_t offset, uint32_t extra);

/* strings */
void  SETTINGS_vSetStringAtOffset(uint32_t offset, size_t length_at_offset, const char *new_value, size_t length);
char *SETTINGS_vGetStringAtOffset(uint32_t offset, size_t length_at_offset, char *buf, size_t length);
bool  SETTINGS_bCheckStringAtOffset(uint32_t offset, size_t length_at_offset, bool mayBeEmpty);
void  SETTINGS_vResetStringAtOffset(uint32_t offset, size_t length);

/* bool */
void SETTINGS_vSetBoolAtOffset(uint32_t offset, uint32_t extra, bool value);
bool SETTINGS_bGetBoolAtOffset(uint32_t offset, uint32_t extra);
bool SETTINGS_bCheckBoolAtOffset(uint32_t offset, uint32_t extra);
void SETTINGS_vResetBoolAtOffset(uint32_t offset, uint32_t extra);

/* struct pointer */
void *SETTINGS_vpGetStructureAtOffset(uint32_t offset);

settings_struct_t *SETTINGS_pGetSettingsStruct(void);

/* ---- Helper macros ---- */
#define ADR(parameter)          (offsetof(settings_struct_t, parameter))
#define PARAM_SIZE(parameter)   ((size_t)sizeof(((settings_struct_t *)0)->parameter))

/* uint8_t */
#define SETTINGS_u8GetByte(p)           SETTINGS_u8GetByteAtOffset(ADR(p))
#define SETTINGS_vSetByte(p, v)         SETTINGS_vSetByteAtOffset(ADR(p), v)
#define SETTINGS_bCheckByte(p)          SETTINGS_bCheckByteAtOffset(ADR(p))
#define SETTINGS_vResetByte(p)          SETTINGS_vResetByteAtOffset(ADR(p))

/* uint16_t */
#define SETTINGS_u16GetShort(p)         SETTINGS_u16GetShortAtOffset(ADR(p))
#define SETTINGS_vSetShort(p, v)        SETTINGS_vSetShortAtOffset(ADR(p), v)
#define SETTINGS_bCheckShort(p)         SETTINGS_bCheckShortAtOffset(ADR(p))
#define SETTINGS_vResetShort(p)         SETTINGS_vResetShortAtOffset(ADR(p))

/* uint32_t */
#define SETTINGS_u32GetWord(p)          SETTINGS_u32GetWordAtOffset(ADR(p), 0)
#define SETTINGS_vSetWord(p, v)         SETTINGS_vSetWordAtOffset(ADR(p), 0, v)
#define SETTINGS_bCheckWord(p)          SETTINGS_bCheckWordAtOffset(ADR(p), 0, false)
#define SETTINGS_bCheckSignedWord(p)    SETTINGS_bCheckWordAtOffset(ADR(p), 0, true)
#define SETTINGS_vResetWord(p)          SETTINGS_vResetWordAtOffset(ADR(p), 0)

/* uint32_t with extra offset (iterating over repeated structs) */
#define SETTINGS_u32GetWordExtra(st, p, e)      SETTINGS_u32GetWordAtOffset(offsetof(st, p), e)
#define SETTINGS_vSetWordExtra(st, p, e, v)     SETTINGS_vSetWordAtOffset(offsetof(st, p), e, v)
#define SETTINGS_bCheckWordExtra(st, p, e)      SETTINGS_bCheckWordAtOffset(offsetof(st, p), e, false)
#define SETTINGS_vResetWordExtra(st, p, e)      SETTINGS_vResetWordAtOffset(offsetof(st, p), e)

/* string */
#define SETTINGS_pauGetString(p)                SETTINGS_vGetStringAtOffset(ADR(p), PARAM_SIZE(p), NULL, 0)
#define SETTINGS_pauGetStringInto(p, b, l)      SETTINGS_vGetStringAtOffset(ADR(p), PARAM_SIZE(p), b, l)
#define SETTINGS_vSetString(p, v, l)            SETTINGS_vSetStringAtOffset(ADR(p), PARAM_SIZE(p), v, l)
#define SETTINGS_vResetString(p)                SETTINGS_vResetStringAtOffset(ADR(p), PARAM_SIZE(p))
#define SETTINGS_bCheckString(p, e)             SETTINGS_bCheckStringAtOffset(ADR(p), PARAM_SIZE(p), e)

/* bool */
#define SETTINGS_bGetBool(p)                    SETTINGS_bGetBoolAtOffset(ADR(p), 0)
#define SETTINGS_vSetBool(p, v)                 SETTINGS_vSetBoolAtOffset(ADR(p), 0, v)
#define SETTINGS_vResetBool(p)                  SETTINGS_vResetBoolAtOffset(ADR(p), 0)
#define SETTINGS_bCheckBool(p)                  SETTINGS_bCheckBoolAtOffset(ADR(p), 0)

#define SETTINGS_bGetBoolExtra(st, p, e)        SETTINGS_bGetBoolAtOffset(offsetof(st, p), e)
#define SETTINGS_vSetBoolExtra(st, p, e, v)     SETTINGS_vSetBoolAtOffset(offsetof(st, p), e, v)
#define SETTINGS_bCheckBoolExtra(st, p, e)      SETTINGS_bCheckBoolAtOffset(offsetof(st, p), e)
#define SETTINGS_vResetBoolExtra(st, p, e)      SETTINGS_vResetBoolAtOffset(offsetof(st, p), e)

/* struct pointer */
#define SETTINGS_vpGetStructure(p)              SETTINGS_vpGetStructureAtOffset(ADR(p))

#endif /* SETTINGS_H_ */
