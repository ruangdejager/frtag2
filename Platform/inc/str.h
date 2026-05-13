/*
 * str.h
 *
 * String utility helpers.
 */

#ifndef STR_H_
#define STR_H_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

bool  STR_bIsValid(char *pacStr, uint8_t u8StrBufLen, bool bMayBeEmpty);
void *memmem(const void *haystack, size_t hlen, const void *needle, size_t nlen);

#endif /* STR_H_ */
