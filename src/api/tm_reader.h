/* ex: set tabstop=2 shiftwidth=2 expandtab cindent: */
#ifndef _TM_READER_H
#define _TM_READER_H
/**
 *  @file tm_reader.h
 *  @brief Mercury API Reader Interface
 *  @author Brian Fiegel
 *  @date 4/18/2009
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

#include "tm_config.h"

#if defined(TMR_ENABLE_SERIAL_READER) && !defined(TMR_ENABLE_RQL_READER)
#define TMR__SERIAL_ONLY
#endif

#ifdef TMR_ENABLE_BACKGROUND_READS
#include <pthread.h>
#include <semaphore.h>
#endif

typedef struct TMR_Reader TMR_Reader;

#include "tmr_types.h"
#include "tmr_status.h"

#include "tmr_gpio.h"
#include "tmr_params.h"
#include "tmr_read_plan.h"
#include "tmr_tag_auth.h"
#include "tmr_tag_data.h"
#include "tmr_tag_lock_action.h"
#include "tmr_tag_protocol.h"
#include "tmr_tagop.h"

#include "tmr_serial_reader.h"

#ifdef  __cplusplus
extern "C" {
#endif


#ifndef DOXYGEN_IGNORE

typedef struct TMR_readParams
{
#ifdef TMR_ENABLE_BACKGROUND_READS
  uint32_t asyncOnTime;
  uint32_t asyncOffTime;
#endif
  TMR_ReadPlan defaultReadPlan;
  TMR_ReadPlan *readPlan;
} TMR_readParams;

typedef struct TMR_tagOpParams
{
  uint8_t antenna;
  TMR_TagProtocol protocol;
}TMR_tagOpParams;

typedef enum TMR_ReaderType
{
  TMR_READER_TYPE_INVALID,
  TMR_READER_TYPE_RQL,
  TMR_READER_TYPE_SERIAL,
} TMR_ReaderType;

#endif /* DOXYGEN_IGNORE */

/** Type of functions to be registered as read callbacks */
typedef void (*TMR_ReadListener)(TMR_Reader *reader, const TMR_TagReadData *t,
                                 void *cookie);
/**
 * User-allocated structure containing the callback pointer and the
 * value to pass to that callback.
 */
typedef struct TMR_ReadListenerBlock
{
  /** Pointer to callback function */
  TMR_ReadListener listener;
  /** Value to pass to callback function */
  void *cookie;
  /** @private */
  struct TMR_ReadListenerBlock *next;
} TMR_ReadListenerBlock;

/** Type of functions to be registered as read error callbacks */
typedef void (*TMR_ReadExceptionListener)(TMR_Reader *reader, TMR_Status error, void *cookie);
/**
 * User-allocated structure containing the callback pointer and the
 * value to pass to that callback.
 */
typedef struct TMR_ReadExceptionListenerBlock
{
  /** Pointer to callback function */
  TMR_ReadExceptionListener listener;
  /** Value to pass to callback function */
  void *cookie;
  /** @private */
  struct TMR_ReadExceptionListenerBlock *next;
} TMR_ReadExceptionListenerBlock;

/** Type of functions to be registered as transport callbacks */
typedef void (*TMR_TransportListener) (bool tx, uint32_t dataLen,
                                       const uint8_t data[], uint32_t timeout,
                                       void *cookie);
/**
 * User-allocated structure containing the callback pointer and the
 * value to pass to that callback.
 */
typedef struct TMR_TransportListenerBlock
{
  /** Pointer to callback function */
  TMR_TransportListener listener;
  /** Value to pass to callback function */
  void *cookie;
  /** @private */
  struct TMR_TransportListenerBlock *next;
} TMR_TransportListenerBlock;

/** Type of functions to be registered as Status read callbacks */
typedef void (*TMR_StatusListener)(TMR_Reader *reader, const TMR_SR_StatusReport report[],
                                 void *cookie);
/**
 * User-allocated structure containing the callback pointer and the
 * value to pass to that callback.
 */
typedef struct TMR_StatusListenerBlock
{
  /** Pointer to callback function */
  TMR_StatusListener listener;
  /** Value to pass to callback function */
  void *cookie;
  /** @private */
  struct TMR_StatusListenerBlock *next;
} TMR_StatusListenerBlock;

/**
 * Private: should not be used by user level application.
 */
typedef struct TMR_Queue_tagReads
{
  uint8_t *tagEntry;
  uint8_t bufPointer;
  bool isStatusResponse;
  struct TMR_Queue_tagReads  *next;
}TMR_Queue_tagReads;

/**
 * @defgroup reader Reader
 *
 * TMR_Reader encapsulates a connection to a ThingMagic RFID reader
 * device and provides an interface to perform RFID operations such as
 * reading tags and writing tag IDs.  
 *
 * Reads can be done on demand, with the TMR_read() function, or
 * continuously in the background with the TMR_startReading()
 * function.  Background reads notify registered listeners of tags
 * that are read.
 *
 * Operations which take an argument for a tag to operate on may
 * optionally be passed a @c NULL argument. This lets the reader choose
 * what tag to use, but may not work if multiple tags are
 * present. This use is recommended only when exactly one tag is known
 * to be in range.
 */

/**
 * @ingroup reader
 * @struct TMR_Reader
 * 
 * The structure that represents a RFID reader. The user should
 * allocate a TMR_Reader structure and initialize it with
 * TMR_create().
 */
struct TMR_Reader
{
  /** @privatesection */

  enum TMR_ReaderType readerType;
  bool connected;
  TMR_TransportListenerBlock *transportListeners;

  TMR_readParams readParams;
  TMR_tagOpParams tagOpParams;
  char uri[TMR_MAX_READER_NAME_LENGTH];
  
  union
  {
    TMR_SR_SerialReader serialReader;
    /*
     *TMR_RQL_RqlReader rqlReader;
     */
  }u;

#ifdef TMR_ENABLE_BACKGROUND_READS
  bool backgroundSetup, backgroundEnabled, backgroundRunning;
  bool parserSetup, parserEnabled, parserRunning;
  sem_t queue_slots, queue_length;
  pthread_mutex_t queue_lock;
  pthread_mutex_t backgroundLock;
  pthread_mutex_t parserLock;
  pthread_cond_t backgroundCond;
  pthread_cond_t parserCond;
  pthread_t backgroundReader;
  pthread_t backgroundParser;
  pthread_mutex_t listenerLock;  
  TMR_ReadListenerBlock *readListeners;
  TMR_ReadExceptionListenerBlock *readExceptionListeners;
  TMR_StatusListenerBlock *statusListeners;
  TMR_SR_StatusType streamStats;
  TMR_Queue_tagReads *tagReadQueue;
#endif


  /* Level 1 */
#ifndef TMR__SERIAL_ONLY
  TMR_Status (*connect)(struct TMR_Reader *reader);
  TMR_Status (*destroy)(struct TMR_Reader *reader);
  TMR_Status (*read)(struct TMR_Reader *reader, uint32_t timeoutMs, int32_t *tagCount);
  TMR_Status (*hasMoreTags)(struct TMR_Reader *reader);
  TMR_Status (*getNextTag)(struct TMR_Reader *reader, TMR_TagReadData *read);
  TMR_Status (*writeTag)(struct TMR_Reader *reader, const TMR_TagFilter *filter, const TMR_TagData *data);
#endif /* TMR__SERIAL_ONLY */
  TMR_Status (*paramGet)(struct TMR_Reader *reader, TMR_Param key, void *value);
  TMR_Status (*paramSet)(struct TMR_Reader *reader, TMR_Param key, const void *value);
#ifndef TMR__SERIAL_ONLY
  /* Level 2 */
  TMR_Status (*killTag)(struct TMR_Reader *reader, const TMR_TagFilter *filter, const TMR_TagAuthentication *auth);
  TMR_Status (*lockTag)(struct TMR_Reader *reader, const TMR_TagFilter *filter, TMR_TagLockAction *action);
  TMR_Status (*tagopExecute)(struct TMR_Reader *reader, TMR_TagOp *tagop);
  TMR_Status (*readTagMemBytes)(struct TMR_Reader *reader, const TMR_TagFilter *filter, 
                               uint32_t bank, uint32_t address,
                               uint16_t count, uint8_t data[]);
  TMR_Status (*readTagMemWords)(struct TMR_Reader *reader, const TMR_TagFilter *filter,
                               uint32_t bank, uint32_t address,
                               uint16_t count, uint16_t *data);
  TMR_Status (*writeTagMemBytes)(struct TMR_Reader *reader, const TMR_TagFilter *filter,
                               uint32_t bank, uint32_t address,
                               uint16_t count, const uint8_t data[]);
  TMR_Status (*writeTagMemWords)(struct TMR_Reader *reader, const TMR_TagFilter *filter,
                               uint32_t bank, uint32_t address,
                               uint16_t count, const uint16_t data[]);
  TMR_Status (*gpoSet)(struct TMR_Reader *reader, uint8_t count, const TMR_GpioPin state[]);
  TMR_Status (*gpiGet)(struct TMR_Reader *reader, uint8_t *count, TMR_GpioPin state[]);
  TMR_Status (*firmwareLoad)(TMR_Reader *reader, void *cookie, TMR_FirmwareDataProvider provider);
#endif /* TMR__SERIAL_ONLY */
};

/**
 * @ingroup reader
 * The TMR_Reader struct being initialized should be preallocated by
 * the user.  No memory is allocated during the initialization
 * process, and the reader is not contacted at this point.
 *
 * @param reader The TMR_Reader structure to initialize
 * @param deviceUri an identifier for the reader to connect to, with
 * a URI syntax. The scheme can be \c eapi for the embedded
 * module protocol, \c rql for the request query language, or
 * \c tmr to guess. The remainder of the URI identifies the
 * stream that the protocol will be spoken over, either a local host
 * serial port device or a TCP network port.
 * Example URIs are:
 *  - tmr://192.168.10.8/ - Reader at TCP/IP address 192.168.10.8, RQL assumed
 *  - rql://192.168.10.8/ - RQL reader at TCP/IP address 192.168.10.8
 *  - tmr:///dev/ttyS1    - reader on serial port /dev/ttyS1, EAPI (serial) assumed
 *  - eapi:///COM2        - EAPI (serial) reader on serial port COM2
 *  - llrp+eapi://192.168.10.8 - EAPI tunneled over LLRP to a reader at
 * TCP/IP address 192.168.10.8
 *
 * @test Call with valid URI should not fail.
 * @test Call with invalid URI should fail.
 */
TMR_Status TMR_create(TMR_Reader *reader, const char* deviceUri);

 /**
 * @ingroup reader
  *
  * This function is a version of TMR_create() that dynamically
  * allocates the TMR_Reader structure and stores the pointer in
  * *reader.
  * 
  * @param[out] readerPointer Pointer to the TMR_Reader* in which to place the
  * pointer to the newly allocated reader structure.
  @ @param deviceUri As described in TMR_create().
  *
  * @test Call with valid URI should not fail.
  * @test Call with invalid URI should fail.
  * @test Multiple calls with different URIs should return distinct objects.
  */
TMR_Status TMR_create_alloc(TMR_Reader **readerPointer, const char* deviceUri);


/**
 * @ingroup reader
 * Establishes the connection to the reader at the URI specified in
 * the TMR_create() call. The existence of a reader at the address is
 * verified and the reader is brought into a state appropriate for
 * performing RF operations.
 *
 * @param reader The reader to connect 
 *
 * 
 * @test Connecting to existing reader should succeed.
 * @test Connecting to nonexisting reader should fail.
 */
TMR_Status TMR_connect(struct TMR_Reader *reader);

/**
 * @ingroup reader
 * Closes the connection to the reader and releases any resources
 * that have been consumed by the reader structure.
 *
 * @param reader The reader to shut down.
 *
 * @test Destroying existing connected reader should succed.
 * @test Destroying existing unconnected reader should succeed.
 * @test Destroying already-destroyed reader should fail.
 * @test Destroying reader allocated by TMR_create_alloc() should
 * free the allocated memory.
 */
TMR_Status TMR_destroy(TMR_Reader *reader);

/**
 * @ingroup reader
 * Search for tags for a fixed duration. Once this returns, the tags
 * are retrieved by calling TMR_getNextTag() until TMR_hasMoreTags()
 * returns false.
 *
 * @param reader The reader being operated on
 * @param timeoutMs The number of milliseconds to search for tags
 * @param[out] tagCount The number of tags read, or -1 if the number
 * is unknown. If NULL, no value will be stored.
 *
 * @test Call should fail if reader is not connected.
 * @test If call is successful as per TMR_Error, *tagCount should be
 * modified and contain a positive value.
 */
TMR_Status TMR_read(TMR_Reader *reader, uint32_t timeoutMs, int32_t *tagCount);

/**
 * @ingroup reader
 * @retval TMR_SUCCESS if there are more tags to be retrieved from the most recent TMR_read() operation.
 * @retval TMR_ERROR_NO_TAGS if no more tags are available for retrieval.
 * @retval Other TMR_ERRORs when appropriate.
 *
 * @param reader The reader being operated on
 * 
 * @test Call should return false before a read() call.
 * @test Call should return false after a read() call that set
 * *tagCount to zero.
 * @test Call should return true after a read() call that set
 * *tagCount to a nonzero value.
 * @test Call should return true through exactly as many getNextTag()
 * calls as specified by *tagCount.
 */
TMR_Status TMR_hasMoreTags(TMR_Reader *reader);

/**
 * @ingroup reader
 * Fetches the next tag from the last TMR_read() operation.
 *
 * @param reader The reader being operated on
 * @param[out] tagData The TMR_TagReadData structure to fill.
 * 
 * @test Call should fail before a read() call.
 * @test Call should fail after a read() call that set *tagCount to
 * zero.
 * @test Call should fill in a tag after a read() call that set
 * *tagCount to a nonzero value.
 * @test Call should succeed exactly as many times as specified by
 * *tagCount.
 */
TMR_Status TMR_getNextTag(TMR_Reader *reader, TMR_TagReadData *tagData);

/**
 * @ingroup reader
 * Wrapper routine that searches for tags, allocates space for the
 * results, and returns the results in a single array.
 * 
 * @param reader The reader being operated on
 * @param timeoutMs The number of milliseconds to search for tags
 * @param[out] tagCount The number of tags found and the size of the allocated array.
 * @param[out] result The array of tag read data.
 *
 * @test Call should fail if reader is not connected.
 * @test If call is successful as per TMR_Error, *tagCount should be
 * modified and contain a positive value, and *result should be
 * modified and contain the corresponding number of tag read values.
 *
 * Out param "result" should be freed after using it.
 */

TMR_Status TMR_readIntoArray(struct TMR_Reader *reader, uint32_t timeoutMs, int32_t *tagCount, TMR_TagReadData *result[]);

/**
 * @deprecated This method is deprecated 
 *
 * Write a new EPC to an existing tag.  
 * 
 * The @c /reader/tagop/antenna parameter controls which antenna will
 * be used for the write operation. The @c /reader/tagop/protocol
 * parameter controls which protocol will be used.
 * 
 * @param reader The reader being operated on
 * @param target The existing tag to write.
 * @param data The EPC to write to the tag
 * @note target is not supported on serial or Astra readers and only
 * TagData filters are supported on other fixed readers.
 *
 * @return TMR_ERROR_NO_TAG_FOUND if no tag matching the filter could be found.
 *
 * @test Call should fail if reader is not connected.
 * @test Upon success, a subsequent TMR_read() should find the new tag ID.
 */
TMR_Status TMR_writeTag(TMR_Reader *reader, const TMR_TagFilter *target, TMR_TagData *data);

/**
 * @deprecated This method is deprecated
 *
 * Sends a kill command to a tag.
 *
 * @param reader The reader being operated on.
 * @param target The tag to kill, or @c NULL to kill the first tag seen.
 * @param auth The authentication needed to kill the tag.
 *
 * @return TMR_ERROR_NO_TAG_FOUND if no tag matching the filter could be found.
 */
TMR_Status TMR_killTag(TMR_Reader *reader, TMR_TagFilter *target, TMR_TagAuthentication *auth);


/**
 * @deprecated This method is deprecated
 *
 * Sends a lock command to a tag.
 *
 * @param reader The reader being operated on.
 * @param target The tag to kill, or @c NULL to kill the first tag seen.
 * @param action The locking action to apply.
 *
 * @return TMR_ERROR_NO_TAG_FOUND if no tag matching the filter could be found.
 */
TMR_Status TMR_lockTag(TMR_Reader *reader, TMR_TagFilter *target, TMR_TagLockAction *action);


/**
 * @ingroup reader
 * Read 8-bit bytes from the memory bank of a tag.
 *
 * @param reader The reader being operated on.
 * @param target The tag to read from, or @c NULL.
 * @param bank The tag memory bank to read from.
 * @param byteAddress The byte address to start reading at.
 * @param byteCount The number of bytes to read
 * @param[out] data The read data.
 */
TMR_Status TMR_readTagMemBytes(TMR_Reader *reader, TMR_TagFilter *target, 
                               uint32_t bank, uint32_t byteAddress, 
                               uint16_t byteCount, uint8_t data[]);

/**
 * @ingroup reader
 * Read 16-bit words from the memory bank of a tag.
 *
 * @param reader The reader being operated on.
 * @param target The tag to read from, or @c NULL.
 * @param bank The tag memory bank to read from.
 * @param wordAddress The word address to start reading at.
 * @param wordCount The number of words to read
 * @param[out] data The read data.
 */
TMR_Status TMR_readTagMemWords(TMR_Reader *reader, TMR_TagFilter *target, 
                               uint32_t bank, uint32_t wordAddress, 
                               uint16_t wordCount, uint16_t data[]);

/**
 * @deprecated This method is deprecated
 *
 * Write 8-bit bytes to the memory bank of a tag. If the tag's
 * fundamental memory unit is larger than 8 bits, trying to write
 * sub-unit quantities will produce an error.
 *
 * @param reader The reader to operate on.
 * @param target The tag to write to, or @c NULL.
 * @param bank The tag memory bank to write to.
 * @param byteAddress The byte address to start writing to.
 * @param byteCount The number of bytes to write.
 * @param data The bytes to write.
 */
TMR_Status TMR_writeTagMemBytes(TMR_Reader *reader, TMR_TagFilter *target,
                                uint32_t bank, uint32_t byteAddress,
                                uint16_t byteCount, const uint8_t data[]);

/**
 * @deprecated This method is deprecated
 *
 * Write 16-bit words to the memory bank of a tag. If the tag's
 * fundamental memory unit is larger than 16 bits, trying to write
 * sub-unit quantities will produce an error.
 *
 * @param reader The reader to operate on.
 * @param target The tag to write to, or @c NULL.
 * @param bank The tag memory bank to write to.
 * @param wordAddress The word address to start writing to.
 * @param wordCount The number of words to write.
 * @param data The words to write.
 */
TMR_Status TMR_writeTagMemWords(TMR_Reader *reader, TMR_TagFilter *target,
                                uint32_t bank, uint32_t wordAddress,
                                uint16_t wordCount, const uint16_t data[]);


/**
 * @ingroup reader
 * Set the state of some GPO pins.
 *
 * @param reader The reader to operate on.
 * @param count The length of the state array.
 * @param state Array of reader pins and the state to set them to.
 */
TMR_Status TMR_gpoSet(TMR_Reader *reader, uint8_t count, const TMR_GpioPin state[]);

/**
 * @ingroup reader
 * Get the state of all GPI pins.
 *
 * @param reader The reader to operate on.
 * @param[in] count Pointer to the allocated length of the state array.
 * @param[out] count Pointer to the number of entries in the state array.
 * @param[out] state Array of reader pins and the state to set them to.
 */
TMR_Status TMR_gpiGet(TMR_Reader *reader, uint8_t *count, TMR_GpioPin state[]);

/**
 * @ingroup reader
 * The TMR_firmwareLoad() method attempts to install firmware on the
 * reader, then restart and reinitialize.
 * 
 * @param reader The reader to operate on
 * @param cookie Value to pass to the callback function.
 * @param provider Callback function to provide firmware data.
 */
TMR_Status TMR_firmwareLoad(TMR_Reader *reader, void *cookie, TMR_FirmwareDataProvider provider);

/**
 * Cookie for state storage of a TMR_memoryProvider.
 */
typedef struct TMR_memoryCookie {
  /** Pointer to the start of the firmware image in memory */
  uint8_t *firmwareStart;
  /** Size of the firmware image */
  uint32_t firmwareSize;
} TMR_memoryCookie;

/**
 * This function can be used as a callback to TMR_firmwareLoad when
 * the firmware image exists in system memory. The cookie is a pointer
 * to a TMR_memoryCookie that is initialized with the start of the firmware
 * image in memory and its size.
 *
 * @param cookie TMR_memoryCookie value.
 * @param[in] size The number of bytes requested.
 * @param[out] size The number of bytes returned.
 * @param data Pointer to buffer to fill.
 * @return true if there is more firmware data to return.
 */
bool TMR_memoryProvider(void *cookie, uint16_t *size, uint8_t *data);

/**
 * This function can be used as a callback to TMR_firmwareLoad when
 * the firmware image exists as a file. 
 *
 * @param cookie FILE * pointing to the file to read from.
 * @param[in] size The number of bytes requested.
 * @param[out] size The number of bytes returned.
 * @param data Pointer to buffer to fill.
 * @return true if there is more firmware data to return.
 */
bool TMR_fileProvider(void *cookie, uint16_t *size, uint8_t *data);

/**
 * @ingroup reader
 * Set the value of a reader parameter.
 * 
 * @param reader The reader to operate on.
 * @param key The string identifying the parameter.
 * @param value The new value to use for the parameter. The caller may
 * not alter the value after passing it to this function.
 *
 * @test Call should fail if key is an invalid parameter name.
 * @test Call should fail if key is valid but unsupported by connected reader.
 * @test Call should succeed if key matches parameter name, independent of case.
 */
TMR_Status TMR_paramSet(struct TMR_Reader *reader, TMR_Param key, const void *value);

/**
 * @ingroup reader
 * Get the value of a reader parameter.
 *
 * @param reader The reader to operate on.
 * @param key The string identifying the parameter.

 * @param[out] value Pointer to the parameter value. For list types
 * (including TMR_String), the caller must allocate the pointed-to
 * space and set the size of the allocated space in the max field, and
 * the function will write into that space and set the actual length
 * of the data. If the list or string is larger than will fit in the
 * pointed-to space, the length will be larger than the allocated
 * space - be careful iterating over the results if there is a chance
 * of not passing in a large enough structure.
 *
 * @test Call should fail if key is an invalid parameter name.
 * @test Call should fail if key is valid but unsupported by connected reader.
 * @test Call should succeed if key matches parameter name, independent of case.
 */
TMR_Status TMR_paramGet(struct TMR_Reader *reader, TMR_Param key, void *value);


/**
 * Get a list of the parameters available
 *
 * Supported Parameters:
 * @li /reader/antenna/checkPort
 * @li /reader/antenna/connectedPortList
 * @li /reader/antenna/portList
 * @li /reader/antenna/portSwitchGpos
 * @li /reader/antenna/settlingTimeList
 * @li /reader/antenna/txRxMap
 * @li /reader/baudRate
 * @li /reader/commandTimeout
 * @li /reader/extendedEpc
 * @li /reader/gen2/BLF
 * @li /reader/gen2/accessPassword
 * @li /reader/gen2/q
 * @li /reader/gen2/session
 * @li /reader/gen2/tagEncoding
 * @li /reader/gen2/target
 * @li /reader/gen2/tari
 * @li /reader/gen2/writeMode
 * @li /reader/gpio/inputList
 * @li /reader/gpio/outputList
 * @li /reader/iso180006b/BLF
 * @li /reader/licenseKey
 * @li /reader/powerMode
 * @li /reader/radio/enablePowerSave
 * @li /reader/radio/enableSJC
 * @li /reader/radio/portReadPowerList
 * @li /reader/radio/portWritePowerList
 * @li /reader/radio/powerMax
 * @li /reader/radio/powerMin
 * @li /reader/radio/readPower
 * @li /reader/radio/temperature
 * @li /reader/radio/writePower
 * @li /reader/read/asyncOffTime
 * @li /reader/read/asyncOnTime
 * @li /reader/read/plan
 * @li /reader/region/hopTable
 * @li /reader/region/hopTime
 * @li /reader/region/id
 * @li /reader/region/lbt/enable
 * @li /reader/region/supportedRegions
 * @li /reader/statistics
 * @li /reader/status/antennaEnable
 * @li /reader/status/frequencyEnable
 * @li /reader/status/temperatureEnable
 * @li /reader/tagReadData/enableReadFilter
 * @li /reader/tagReadData/readFilterTimeout
 * @li /reader/tagReadData/recordHighestRssi
 * @li /reader/tagReadData/reportRssiInDbm
 * @li /reader/tagReadData/tagopFailures
 * @li /reader/tagReadData/tagopSuccesses
 * @li /reader/tagReadData/uniqueByAntenna
 * @li /reader/tagReadData/uniqueByData
 * @li /reader/tagReadData/uniqueByProtocol
 * @li /reader/tagop/antenna
 * @li /reader/tagop/protocol
 * @li /reader/transportTimeout
 * @li /reader/uri
 * @li /reader/userConfig
 * @li /reader/userMode
 * @li /reader/version/hardware
 * @li /reader/version/model
 * @li /reader/version/productGroup
 * @li /reader/version/productGroupID
 * @li /reader/version/serial
 * @li /reader/version/software
 * @li /reader/version/supportedProtocols
 *
 * @ingroup reader
 *
 * @param reader The reader to operate on.
 * @param[out] keys Pointer to the list of parameters. The caller must
 * allocate space for the parameters.
 * @param[in] len Pointer to the allocated length of the space pointed to by keys.
 * @param[out] len Pointer to the number of parameters supported by
 * the reader, which may be larger or smaller than the provided
 * array. If it is larger than the provided array, the list is
 * truncated.
 */
TMR_Status TMR_paramList(struct TMR_Reader *reader, TMR_Param *keys, uint32_t *len);

/**
 * @ingroup reader
 * Return the string name corresponding to a TMR_Param value.
 * Only present if TMR_ENABLE_PARAM_STRINGS is set.
 *
 * @param key The parameter ID.
 * @return The corresponding string name.
 */
const char* TMR_paramName(TMR_Param key);

/**
 * @ingroup reader
 * Return the TMR_Param ID corresponding to a string name.
 * Only present if TMR_ENABLE_PARAM_STRINGS is set.
 *
 * @param name the parameter 
 * @return The TMR_Param value of the name, or TMR_PARAM_NONE if no such
 * parameter exists.
 */
TMR_Param TMR_paramID(const char *name);


/**
 * @ingroup reader
 * 
 * Add a listener to the list of functions that will be called for
 * each message sent to or recieved from the reader.
 *
 * @param reader The reader to operate on.
 * @param block A structure containing a pointer to the listener
 * function and a user-supplied cookie value to pass to the function
 * when called.
 */
TMR_Status TMR_addTransportListener(TMR_Reader *reader, TMR_TransportListenerBlock *block);


/**
 * @ingroup reader
 * 
 * Remove a listener from the list of functions that will be called
 * for each message sent to or recieved from the reader.
 *
 * @param reader The reader to operate on.
 * @param block A structure containing a pointer to the listener
 * function and a user-supplied cookie value to pass to the function
 * when called.
 */
TMR_Status TMR_removeTransportListener(TMR_Reader *reader, TMR_TransportListenerBlock *block);

/**
 * @ingroup reader
 * Add a listener to the list of functions that will be called for
 * each background tag read.
 *
 * @param reader The reader to operate on.
 * @param block A structure containing a pointer to the listener
 * function and a user-supplied cookie value to pass to the function
 * when called.
 */
TMR_Status TMR_addReadListener(struct TMR_Reader *reader,
                               TMR_ReadListenerBlock *block);

/**
 * @ingroup reader
 * Remove a listener from the list of functions that will be called
 * for each background tag read.
 *
 * @param reader The reader to operate on.
 * @param block A structure containing a pointer to the listener
 * function and a user-supplied cookie value to pass to the function
 * when called.
 */
TMR_Status TMR_removeReadListener(struct TMR_Reader *reader,
                                  TMR_ReadListenerBlock *block);

/**
 * @ingroup reader
 * Add a listener to the list of functions that will be called for
 * each error that occurs during background tag reading.
 *
 * @param reader The reader to operate on.
 * @param block A structure containing a pointer to the listener
 * function and a user-supplied cookie value to pass to the function
 * when called.
 */
TMR_Status TMR_addReadExceptionListener(struct TMR_Reader *reader,
                                        TMR_ReadExceptionListenerBlock *block);

/**
 * @ingroup reader
 * Remove a listener from the list of functions that will be called
 * for each error that occurs during background tag reading.
 *
 * @param reader The reader to operate on.
 * @param block A structure containing a pointer to the listener
 * function and a user-supplied cookie value to pass to the function
 * when called.
 */
TMR_Status TMR_removeReadExceptionListener(struct TMR_Reader *reader,
                                           TMR_ReadExceptionListenerBlock *block);

/**
 * @ingroup reader
 * Add a listener to the list of functions that will be called for
 * each background status response.
 *
 * @param reader The reader to operate on.
 * @param block A structure containing a pointer to the listener
 * function and a user-supplied cookie value to pass to the function
 * when called.
 */
TMR_Status TMR_addStatusListener(struct TMR_Reader *reader,
                               TMR_StatusListenerBlock *block);

/**
 * @ingroup reader
 * Remove a listener from the list of functions that will be called
 * for each background status response.
 *
 * @param reader The reader to operate on.
 * @param block A structure containing a pointer to the listener
 * function and a user-supplied cookie value to pass to the function
 * when called.
 */
TMR_Status TMR_removeStatusListener(struct TMR_Reader *reader,
                                  TMR_StatusListenerBlock *block);


/**
 * @ingroup reader
 * Start reading tags in the background. The tags found will be passed
 * to the registered read listeners, and any errors that occur during
 * reading will be passed to the registered exception
 * listeners. Reading will continue until stopReading() is called.
 * The read listeners will be called from a background thread.
 *
 * @param reader The reader to operate on.
 */

TMR_Status TMR_startReading(struct TMR_Reader *reader);

/**
 * @ingroup reader
 * Stop reading tags in the background. This function will wait until
 * the reader has stopped.
 *
 * @param reader The reader to operate on.
 */
TMR_Status TMR_stopReading(struct TMR_Reader *reader);

/**
 * This function will convert the error codes into proper error messages.
 *
 * @param reader The reader which resulted in error
 * @param status The status code which is to be converted
 */
const char *TMR_strerr(TMR_Reader *reader, TMR_Status status);


#ifndef DOXYGEN_IGNORE

TMR_Status TMR_reader_init_internal(struct TMR_Reader *reader);
/*TMR_Status TMR_init_statusListenerBlock(TMR_StatusListenerBlock *slb, TMR_StatusListener listener,
                                        void *cookie, TMR_SR_StatusContentFlags statusFlags);*/

void TMR__notifyTransportListeners(TMR_Reader *reader, bool tx, 
                                   uint32_t dataLen, uint8_t *data,
                                   int timeout);

#ifdef TMR__SERIAL_ONLY
#define TMR_connect(reader) (TMR_SR_connect(reader))
#define TMR_destroy(reader) (TMR_SR_destroy(reader))
#define TMR_read(reader, timeoutMs, tagCount) (TMR_SR_read((reader),(timeoutMs),(tagCount)))
#define TMR_hasMoreTags(reader) (TMR_SR_hasMoreTags(reader))
#define TMR_getNextTag(reader, read) (TMR_SR_getNextTag((reader),(read)))
#define TMR_writeTag(reader, filter, data) (TMR_SR_writeTag((reader),(filter),(data)))
#define TMR_readTagMemWords(reader, target, bank, address, count, data) (TMR_SR_readTagMemWords((reader),(target),(bank),(address),(count),(data)))
#define TMR_readTagMemBytes(reader, target, bank, address, count, data) (TMR_SR_readTagMemBytes((reader),(target),(bank),(address),(count),(data)))
#define TMR_writeTagMemWords(reader, target, bank, address, count, data) (TMR_SR_writeTagMemWords((reader),(target),(bank),(address),(count),(data)))
#define TMR_writeTagMemBytes(reader, target, bank, address, count, data) (TMR_SR_writeTagMemBytes((reader),(target),(bank),(address),(count),(data)))
#define TMR_killTag(reader, filter, auth) (TMR_SR_killTag((reader),(filter),(auth)))
#define TMR_lockTag(reader, filter, action) (TMR_SR_lockTag((reader),(filter),(action)))
#define TMR_executeTagOp(reader, tagop, filter, data) (TMR_SR_executeTagOp((reader), (tagop), (filter), (data)))
#define TMR_gpoSet(reader, count, state) (TMR_SR_gpoSet((reader),(count),(state)))
#define TMR_gpiGet(reader, count, state) (TMR_SR_gpiGet((reader),(count),(state)))
#define TMR_firmwareLoad(reader, cookie, provider) (TMR_SR_firmwareLoad((reader),(cookie),(provider)))
#else

/** 
 * The fact that these functions are implemented via macros is an
 * implementation detail, not something for the public API
 * documentation.
 */
#define TMR_connect(reader) ((reader)->connect(reader))
#define TMR_destroy(reader) ((reader)->destroy(reader))
#define TMR_read(reader, timeoutMs, tagCount) ((reader)->read((reader),(timeoutMs),(tagCount)))
#define TMR_hasMoreTags(reader) ((reader)->hasMoreTags(reader))
#define TMR_getNextTag(reader, read) ((reader)->getNextTag((reader),(read)))
#define TMR_writeTag(reader, filter, data) ((reader)->writeTag((reader),(filter),(data)))
#define TMR_readTagMemWords(reader, target, bank, address, count, data) ((reader)->readTagMemWords((reader),(target),(bank),(address),(count),(data)))
#define TMR_readTagMemBytes(reader, target, bank, address, count, data) ((reader)->readTagMemBytes((reader),(target),(bank),(address),(count),(data)))
#define TMR_writeTagMemWords(reader, target, bank, address, count, data) ((reader)->writeTagMemWords((reader),(target),(bank),(address),(count),(data)))
#define TMR_writeTagMemBytes(reader, target, bank, address, count, data) ((reader)->writeTagMemBytes((reader),(target),(bank),(address),(count),(data)))
#define TMR_killTag(reader, filter, auth) ((reader)->killTag((reader),(filter),(auth)))
#define TMR_lockTag(reader, filter, action) ((reader)->lockTag((reader),(filter),(action)))
#define TMR_executeTagOp(reader, tagop, filter, data) (TMR_SR_executeTagOp((reader), (tagop), (filter), (data)))
#define TMR_gpoSet(reader, count, state) ((reader)->gpoSet((reader),(count),(state)))
#define TMR_gpiGet(reader, count, state) ((reader)->gpiGet((reader),(count),(state)))
#define TMR_firmwareLoad(reader, cookie, provider) ((reader)->firmwareLoad((reader),(cookie),(provider)))
#endif /* TMR__SERIAL_ONLY */
#endif /* DOXYGEN_IGNORE */

#ifdef  __cplusplus
}
#endif

#endif /* _TM_READER_H_ */
