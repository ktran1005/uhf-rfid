#ifndef _OSDEP_H
#define _OSDEP_H

/**
 *  @file osdep.h
 *  @brief Mercury API - OS dependencies
 *  @author Nathan Williams
 *  @date 2/24/2010
 */

/*
 * Copyright (c) 2010 ThingMagic, Inc.
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

#ifdef  __cplusplus
extern "C" {
#endif

/* These are functions that must be provided for the library to operate */

uint64_t tmr_gettime();

/* The time functions collectively return a 64-bit counter in units of
 * milliseconds. Both methods are used when timestamping events such
 * as tag reads. For controlling elapsed time, only the lower one is
 * used, so If your platform does not support more than 32 bits of
 * millisecond counting, returning 0 from the high method will not
 * cause problems internal to the library.
 */

/**
 * Return the low 32-bits of a system millisecond counter. This is
 * used internally for operation timing and to timestamp tag reads.
 */
uint32_t tmr_gettime_low();

/**
 * Return the high 32-bits of a system millisecond counter. This is
 * used to timestamp tag reads.
 */
uint32_t tmr_gettime_high();

/**
 * Suspend operation for a given duration.
 * @param sleepms The number of milliseconds to sleep for.
 */
void tmr_sleep(uint32_t sleepms);

#ifdef __cplusplus
}
#endif

#endif /* _OSDEP_H */
