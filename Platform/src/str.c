/*
 * str.c
 *
 * String utility helpers.
 */

#include "str.h"
#include <string.h>
#include <ctype.h>

/*
 * Returns true if pacStr contains a NUL terminator within u8StrBufLen
 * bytes and every character before the NUL is printable.
 * If bMayBeEmpty is false, an empty string is rejected.
 */
bool STR_bIsValid(char *pacStr, uint8_t u8StrBufLen, bool bMayBeEmpty)
{
    uint8_t i;

    for (i = 0; i < u8StrBufLen; i++)
        if (pacStr[i] == 0) break;

    if (i == u8StrBufLen) return false;
    if (i == 0)           return bMayBeEmpty;

    for (i = 0; i < strlen(pacStr); i++)
        if (!isprint((unsigned char)pacStr[i])) return false;

    return true;
}

/*
 * memmem — find first occurrence of needle in haystack.
 * Returns pointer to start of match or NULL.
 */
void *memmem(const void *haystack, size_t hlen, const void *needle, size_t nlen)
{
    int         needle_first;
    const void *p    = haystack;
    size_t      plen = hlen;

    if (!nlen) return NULL;

    needle_first = *(unsigned char *)needle;

    while (plen >= nlen &&
           (p = memchr(p, needle_first, plen - nlen + 1)))
    {
        if (!memcmp(p, needle, nlen))
            return (void *)p;
        p++;
        plen = hlen - (p - haystack);
    }
    return NULL;
}
