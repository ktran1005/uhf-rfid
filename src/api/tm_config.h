/* ex: set tabstop=2 shiftwidth=2 expandtab cindent: */
#ifndef _TM_CONFIG_H
#define _TM_CONFIG_H

/**
 *  @file tm_config.h
 *  @brief Mercury API - Build Configuration
 *  @author Nathan Williams
 *  @date 10/20/2009
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

#ifdef  __cplusplus
extern "C" {
#endif

/**
 * API version number
 */
#define TMR_VERSION "1.9.1.82"

/**
 * Define this to enable support for devices that support the serial
 * reader command set (M5e, M6e, Vega, etc.)
 */
#define TMR_ENABLE_SERIAL_READER

/**
 * Define this to enable support for local serial port access via
 * native interfaces (COM1 on Windows, /dev/ttyS0, /dev/ttyUSB0 on
 * Linux, etc.).
 */
#define TMR_ENABLE_SERIAL_TRANSPORT_NATIVE

/**
 * The longest possible name for a reader.
 * If set to 0, the device will be opened immediately upon invocation of TMR_create().
 */
#define TMR_MAX_READER_NAME_LENGTH 64

/**
 * The maximum number of protocols supported in a multiprotocol search command
 */
#define TMR_MAX_SERIAL_MULTIPROTOCOL_LENGTH 5

/**
 * The maximum size of the Queue, used to share the streamed messages
 * between the do_background_reads thread and parse_tag_reads thread.
 */
#define TMR_MAX_QUEUE_SLOTS 10

/** 
 * Number of bytes to allocate for embedded data return
 * in each TagReadData.
 */
#define TMR_MAX_EMBEDDED_DATA_LENGTH 32

/**
 * Define this to enable support for serial transport over LLRP.
 * (Not yet available for Windows)
 */
#ifndef WIN32
#define TMR_ENABLE_SERIAL_TRANSPORT_LLRP
#endif

/**
 * Define this to enable support for the ISO180006B protocol parameters
 * and access commands
 */
#define TMR_ENABLE_ISO180006B

/**
 * Define this to enable support for RQL-based fixed readers.
 */
#undef TMR_ENABLE_RQL_READER

/**
 * Define this to enable support for background reads using native threads.
 * This feature is also available for windows (using pthreads-win32)
 */

#define TMR_ENABLE_BACKGROUND_READS


/**
 * Define this to include TMR_strerror().
 */
#define TMR_ENABLE_ERROR_STRINGS

/**
 * Define this to include TMR_paramName() and TMR_paramID().
 */
#define TMR_ENABLE_PARAM_STRINGS

/**
 * Define the largest number serial reader antenna ports that will be supported
 */
#define TMR_SR_MAX_ANTENNA_PORTS (16)

/**
 * Define when compiling on a big-endian host platform to enable some
 * endian optimizations. Without this, no endianness will be assumed.
 */ 
#undef TMR_BIG_ENDIAN_HOST

/**
 * Define this to enable API-side tag read deduplication.  Under
 * certain conditions, the module runs out of buffer space to detect
 * reads of a previously-seen EPC.  
 */
#define TMR_ENABLE_API_SIDE_DEDUPLICATION

/**
 * Default read filter timeout.
 */
#define TMR_DEFAULT_READ_FILTER_TIMEOUT -1

#ifdef __cplusplus
}
#endif

#endif /* _TM_CONFIG_H */
