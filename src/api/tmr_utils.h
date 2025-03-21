#ifndef _TMR_UTILS_H
#define _TMR_UTILS_H

/**
 *  @file tmr_utils.h
 *  @brief Mercury API - generic utilities
 *  @author Nathan Williams
 *  @date 12/1/2009
 */

/*
 * Copyright (c) 2009 ThingMagic, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdint.h>
#include <stddef.h>

#ifdef  __cplusplus
extern "C" {
#endif

/* Macros for working with values embedded in uint8_t arrays (msg) */
/* Absolute-value get */
#define GETU8AT(msg, i) ( \
  ((msg)[(i)])          )

#define GETU16AT(msg, i) ( \
  ((msg)[(i)  ] <<  8)   | \
  ((msg)[(i)+1] <<  0)   )

#define GETU24AT(msg, i) ( \
  ((msg)[(i)  ] <<  16)  | \
  ((msg)[(i)+1] <<   8)  | \
  ((msg)[(i)+2] <<   0)  )

#define GETU32AT(msg, i) ( \
  ((msg)[(i)  ] <<  24)  | \
  ((msg)[(i)+1] <<  16)  | \
  ((msg)[(i)+2] <<   8)  | \
  ((msg)[(i)+3] <<   0)  )

/* Get and update index to next position */
#define GETU8(msg, i)   ((msg)[(i)++])
#define GETU16(msg, i)  (i+=2, GETU16AT((msg), i-2))
#define GETU24(msg, i)  (i+=3, GETU24AT((msg), i-3))
#define GETU32(msg, i)  (i+=4, GETU32AT((msg), i-4))

/* Set and update index to next position */
#define SETU8(msg, i, u8val) do {      \
  (msg)[(i)++] = (u8val)      & 0xff;  \
} while (0)

#define SETU16(msg, i, u16val) do {    \
  uint16_t _tmp = (u16val);            \
  (msg)[(i)++] =(uint8_t) (_tmp >>  8) & 0xff;  \
  (msg)[(i)++] =(uint8_t)(_tmp >>  0) & 0xff;  \
} while (0)

#define SETU32(msg, i, u32val) do {    \
  uint32_t _tmp = (u32val);            \
  (msg)[(i)++] = (uint8_t)(_tmp >> 24) & 0xff;  \
  (msg)[(i)++] = (uint8_t)(_tmp >> 16) & 0xff;  \
  (msg)[(i)++] = (uint8_t)(_tmp >>  8) & 0xff;  \
  (msg)[(i)++] = (uint8_t)(_tmp >>  0) & 0xff;  \
} while (0)

/* Append a value to our list structures, which have both
 * a allocated-space value (max) and a length-of-underlying-list
 * value (len). Len can exceed max, which indicates to the caller
 * that there was not enough space in the passed-in list structure
 * to store the entire list.
 */
#define LISTAPPEND(l, value) do {         \
  (l)->len++;                             \
  if ((l)->len <= (l)->max)               \
    (l)->list[(l)->len - 1] = (value);    \
} while (0)


/* Macros for working with large bitmasks made up of arrays of uint32_t */
#define BITGET(array, number) (((array)[(number)/32] >> ((number)&31)) & 1)
#define BITSET(array, number) ((array)[(number)/32] |= (1 << ((number)&31)))
#define BITCLR(array, number) ((array)[(number)/32] &= ~(1 << ((number)&31)))

#define numberof(x) (sizeof((x))/sizeof((x)[0]))

#ifndef TMR_USE_HOST_C_LIBRARY
void *tm_memcpy(void *dest, const void *src, size_t n);
char *tm_strcpy(char *dest, const char *src);
char *tm_strchr(const char *s, int c);

#undef memcpy
#undef strcpy
#undef strchr

#define memcpy tm_memcpy
#define strcpy tm_strcpy
#define strchr tm_strchr
#endif

int tm_strcasecmp(const char *s1, const char *s2);
#define strcasecmp tm_strcasecmp

void tm_gettime_consistent(uint32_t *high, uint32_t *low);
uint32_t tm_time_subtract(uint32_t end, uint32_t start);
int tm_u8s_per_bits(int bitCount);

#ifdef __cplusplus
}
#endif

#endif /* _TMR_UTILS_H */
