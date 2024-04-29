/**
 *  @file tm_reader.c
 *  @brief Mercury API - top level implementation
 *  @author Nathan Williams
 *  @date 10/28/2009
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

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "tm_reader.h"
#include "tmr_utils.h"

#define EAPI_PREFIX "eapi://"
#define EAPI_PREFIX_LEN (sizeof(EAPI_PREFIX)-1)

#define TMR_PREFIX "tmr://"
#define TMR_PREFIX_LEN (sizeof(TMR_PREFIX)-1)

#define LLRP_EAPI_PREFIX "llrp+eapi://"
#define LLRP_EAPI_PREFIX_LEN (sizeof(LLRP_EAPI_PREFIX)-1)


TMR_Status
TMR_create_alloc(TMR_Reader **reader, const char* deviceUri)
{

  *reader = malloc(sizeof(**reader));
  if (NULL == *reader)
    return TMR_ERROR_OUT_OF_MEMORY;
  return TMR_create(*reader, deviceUri);
}


TMR_Status
TMR_create(TMR_Reader *reader, const char* deviceUri)
{
  TMR_Status ret;

  ret = TMR_ERROR_INVALID;

#ifdef TMR_ENABLE_SERIAL_READER
#ifdef TMR_ENABLE_SERIAL_TRANSPORT_NATIVE
  if ((strncmp(deviceUri, EAPI_PREFIX, EAPI_PREFIX_LEN) == 0)
      || ((strncmp(deviceUri, TMR_PREFIX, TMR_PREFIX_LEN) == 0)
          && *(deviceUri + TMR_PREFIX_LEN) == '/'))
  {
    const char *devname;

    if ('e' == deviceUri[0])
    {
      devname = deviceUri + EAPI_PREFIX_LEN ;
    }
    else
    {
      devname = deviceUri + TMR_PREFIX_LEN;
    }

    ret = TMR_SR_SerialTransportNativeInit(&reader->u.serialReader.transport, 
                                           &reader->u.serialReader.
                                           transportContext.nativeContext,
                                           devname);
    if (TMR_SUCCESS != ret)
    {
      return ret;
    }
    strncpy(reader->uri, deviceUri, TMR_MAX_READER_NAME_LENGTH); 
  }
  else
#endif /* TMR_ENABLE_SERIAL_TRANSPORT_NATIVE */
#ifdef TMR_ENABLE_SERIAL_TRANSPORT_LLRP
  if (strncmp(deviceUri, LLRP_EAPI_PREFIX, LLRP_EAPI_PREFIX_LEN) == 0)
  {
    const char *host, *port;
    char hostCopy[256];
    int portnum;

    host = deviceUri + LLRP_EAPI_PREFIX_LEN;
    port = strchr(host, ':');
    strcpy(hostCopy, host);
    if (port == NULL)
    {
      char *slash;
      portnum = 5084;
      slash = strchr(hostCopy, '/');
      if (slash)
      {
        *slash = '\0';
      }
    }
    else
    {
      portnum = atoi(port + 1);
      hostCopy[port - host] = '\0';
    }
    
    ret = TMR_SR_LlrpEapiTransportInit(&reader->u.serialReader.transport, 
                                       &reader->u.serialReader.
                                            transportContext.llrpContext,
                                       hostCopy, portnum, false);
    if (TMR_SUCCESS != ret)
    {
      return ret;
    }
  }
  else
#endif /* TMR_ENABLE_SERIAL_TRANSPORT_LLRP */
  {
    return ret;
  }

  return TMR_SR_SerialReader_init(reader);

#else /* TMR_ENABLE_SERIAL_READER */
  /* No readers supported! */
  return TMR_ERROR_INVALID;
#endif
}

TMR_Status
TMR_reader_init_internal(struct TMR_Reader *reader)
{
  reader->connected = false;
  reader->transportListeners = NULL;


  TMR_RP_init_simple(&reader->readParams.defaultReadPlan, 0, NULL, 
                     TMR_TAG_PROTOCOL_GEN2, 1);
  reader->readParams.readPlan = &reader->readParams.defaultReadPlan;
#ifdef TMR_ENABLE_BACKGROUND_READS
  reader->readParams.asyncOnTime = 250;
  reader->readParams.asyncOffTime = 0;
  pthread_mutex_init(&reader->backgroundLock, NULL);
  pthread_mutex_init(&reader->parserLock, NULL);
  pthread_cond_init(&reader->backgroundCond, NULL);
  pthread_cond_init(&reader->parserCond, NULL);
  pthread_mutex_init(&reader->listenerLock, NULL);
  pthread_mutex_init(&reader->queue_lock, NULL);
  reader->readListeners = NULL;
  reader->readExceptionListeners = NULL;
  reader->statusListeners = NULL;
  reader->backgroundSetup = false;
  reader->parserSetup = false;
  reader->backgroundEnabled = false;
  reader->parserEnabled = false;
  reader->tagReadQueue = NULL;
  reader->u.serialReader.isStatusResponse = false;  
  reader->streamStats = TMR_SR_STATUS_NONE;
#endif

  return TMR_SUCCESS;
}

#ifdef TMR_ENABLE_API_SIDE_DEDUPLICATION

static int
TMR_findDupTag(TMR_Reader *reader,
               TMR_TagReadData* newRead,
               TMR_TagReadData oldReads[], int32_t oldLength,
               bool uniqueByAntenna, bool uniqueByData, bool uniqueByProtocol)
{
  int i;

  for (i=0; i<oldLength; i++)
  {
    TMR_TagReadData* oldRead = &oldReads[i];
    TMR_TagData* oldTag = &oldRead->tag;
    TMR_TagData* newTag = &newRead->tag;

    if ((oldTag->epcByteCount != newTag->epcByteCount) ||
        (0 != memcmp(oldTag->epc, newTag->epc,
                     (oldTag->epcByteCount)*sizeof(uint8_t))))
    {
      continue;
    }
    if (uniqueByAntenna)
    {
      if (oldRead->antenna != newRead->antenna)
      {
        continue;
      }
    }
    if (uniqueByData)
    {
      if ((oldRead->data.len != newRead->data.len) ||
          (0 != memcmp(oldRead->data.list, newRead->data.list,
                       (oldRead->data.len)*sizeof(uint8_t))))
      {
        continue;
      }
    }
    if (uniqueByProtocol)
    {
      if (oldRead->tag.protocol != newRead->tag.protocol)
      {
        continue;
      }
    }
    /* No fields mismatched; this tag is a match */
    break;
  }

  return (i < oldLength) ? i : -1;
}

static void
TMR_updateDupTag(TMR_Reader* reader,
                 TMR_TagReadData* oldRead, TMR_TagReadData* newRead,
                 bool highestRssi)
{
  oldRead->readCount += newRead->readCount;

  if (highestRssi)
  {
    if (newRead->rssi > oldRead->rssi)
    {
      uint32_t saveCount = oldRead->readCount;

      memcpy(oldRead, newRead, sizeof(TMR_TagReadData));
      /* TODO: TagReadData.data field not yet supported, pending a
       * comprehensive strategy for dynamic memory allocation. */

      oldRead->readCount = saveCount;
    }
  }
}

#endif /* TMR_ENABLE_API_SIDE_DEDUPLICATION */

TMR_Status
TMR_readIntoArray(struct TMR_Reader *reader, uint32_t timeoutMs,
                  int32_t *tagCount, TMR_TagReadData *result[])
{
  int32_t tagsRead, count, alloc;
  TMR_TagReadData *results;
  TMR_Status ret;
  uint32_t startHi, startLo, nowHi, nowLo;
#ifdef TMR_ENABLE_API_SIDE_DEDUPLICATION
  bool uniqueByAntenna, uniqueByData, recordHighestRssi, uniqueByProtocol;
#endif /* TMR_ENABLE_API_SIDE_DEDUPLICATION */

#ifdef TMR_ENABLE_API_SIDE_DEDUPLICATION
  {
    bool bval;

    ret = TMR_paramGet(reader, TMR_PARAM_TAGREADDATA_UNIQUEBYANTENNA, &bval);
    if (TMR_ERROR_NOT_FOUND == ret) { bval = false; }
    else if (TMR_SUCCESS != ret) { return ret; }
    uniqueByAntenna = bval;

    ret = TMR_paramGet(reader, TMR_PARAM_TAGREADDATA_UNIQUEBYDATA, &bval);
    if (TMR_ERROR_NOT_FOUND == ret) { bval = false; }
    else if (TMR_SUCCESS != ret) { return ret; }
    uniqueByData = bval;

    ret = TMR_paramGet(reader, TMR_PARAM_TAGREADDATA_UNIQUEBYPROTOCOL, &bval);
    if (TMR_ERROR_NOT_FOUND == ret) { bval = false; }
    else if (TMR_SUCCESS != ret) { return ret; }
    uniqueByProtocol = bval;

    ret = TMR_paramGet(reader, TMR_PARAM_TAGREADDATA_RECORDHIGHESTRSSI, &bval);
    if (TMR_ERROR_NOT_FOUND == ret) { bval = false; }
    else if (TMR_SUCCESS != ret) { return ret; }
    recordHighestRssi = bval;
  }
#endif /* TMR_ENABLE_API_SIDE_DEDUPLICATION */

  tagsRead = 0;
  alloc = 0;
  results = NULL;

  tm_gettime_consistent(&startHi, &startLo);
  do 
  {

    ret = TMR_read(reader, timeoutMs, &count);
    if ((TMR_SUCCESS != ret) && (TMR_ERROR_TAG_ID_BUFFER_FULL != ret))
    {
      goto out;
    }

    if (0 == count)
    {
      goto out;
    }
    else if (-1 == count) /* Unknown - streaming */
    {
      alloc += 4;
    }
    else
    {
      alloc += count;
    }

    {
      TMR_TagReadData *newResults;
      newResults = realloc(results, alloc * sizeof(*results));
      if (NULL == newResults)
      {
        ret = TMR_ERROR_OUT_OF_MEMORY;
        goto out;
      }
      results = newResults;
    }
    while (TMR_SUCCESS == TMR_hasMoreTags(reader))
    {
      if (tagsRead == alloc)
      {
        TMR_TagReadData *newResults;
        alloc *= 2;
        newResults = realloc(results, alloc * sizeof(*results));
        if (NULL == newResults)
        {
          ret = TMR_ERROR_OUT_OF_MEMORY;
          goto out;
        }
        results = newResults;
      }
      TMR_TRD_init(&results[tagsRead]);
      ret = TMR_getNextTag(reader, &results[tagsRead]);
      if (TMR_SUCCESS != ret)
      {
        goto out;
      }
#ifndef TMR_ENABLE_API_SIDE_DEDUPLICATION
      tagsRead++;
#else
      /* Search array for record duplicating the one just fetched.
       * If no dup found, commit fetched tag by incrementing tag count.
       * If dup found, copy last record to found position, don't advance count.
       */
      if (true == reader->u.serialReader.enableReadFiltering)
      {
        TMR_TagReadData* last = &results[tagsRead];
        int dupIndex = TMR_findDupTag(reader, last, results, tagsRead,
                        uniqueByAntenna, uniqueByData, uniqueByProtocol);
        if (-1 == dupIndex)
          {
            tagsRead++;
          }
        else
          {
            TMR_updateDupTag(reader, &results[dupIndex], last,
                             recordHighestRssi);
          }
      }
#endif /* TMR_ENABLE_API_SIDE_DEDUPLICATION */
    }

    tm_gettime_consistent(&nowHi, &nowLo);
  }
  while (tm_time_subtract(nowLo, startLo) < timeoutMs);

out:
  if (NULL != tagCount)
    *tagCount = tagsRead;
  *result = results;
  return ret;
}

TMR_Status
TMR_paramSet(struct TMR_Reader *reader, TMR_Param key, const void *value)
{
  TMR_Status ret;

  ret = TMR_SUCCESS;

  switch (key)
  {
#ifdef TMR_ENABLE_BACKGROUND_READS
  case TMR_PARAM_READ_ASYNCOFFTIME:
    reader->readParams.asyncOffTime = *(uint32_t *)value;
    break;
  case TMR_PARAM_READ_ASYNCONTIME:
    reader->readParams.asyncOnTime = *(uint32_t *)value;
    break;
#endif
  default:
    ret = reader->paramSet(reader, key, value);
  }
  return ret;
}


TMR_Status
TMR_paramGet(struct TMR_Reader *reader, TMR_Param key, void *value)
{
  TMR_Status ret;

  ret = TMR_SUCCESS;

  switch (key)
  {
  case TMR_PARAM_READ_PLAN:
  {
    TMR_ReadPlan *plan = value;
    *plan = *reader->readParams.readPlan;
    break;
  }
#ifdef TMR_ENABLE_BACKGROUND_READS
  case TMR_PARAM_READ_ASYNCOFFTIME:
    *(uint32_t *)value = reader->readParams.asyncOffTime;
    break;
  case TMR_PARAM_READ_ASYNCONTIME:
    *(uint32_t *)value = reader->readParams.asyncOnTime;
    break;
#endif
  default:
    ret = reader->paramGet(reader, key, value);
  }
  return ret;
}


TMR_Status
TMR_addTransportListener(TMR_Reader *reader, TMR_TransportListenerBlock *b)
{

  b->next = reader->transportListeners;
  reader->transportListeners = b;

  return TMR_SUCCESS;
}


TMR_Status
TMR_removeTransportListener(TMR_Reader *reader, TMR_TransportListenerBlock *b)
{
  TMR_TransportListenerBlock *block, **prev;

  prev = &reader->transportListeners;
  block = reader->transportListeners;
  while (NULL != block)
  {
    if (block == b)
    {
      *prev = block->next;
      break;
    }
    prev = &block->next;
    block = block->next;
  }
  if (block == NULL)
  {
    return TMR_ERROR_INVALID;
  }

  return TMR_SUCCESS;
}


void
TMR__notifyTransportListeners(TMR_Reader *reader, bool tx, 
                              uint32_t dataLen, uint8_t *data,
                              int timeout)
{
  TMR_TransportListenerBlock *block;

  block = reader->transportListeners;
  while (NULL != block)
  {
    block->listener(tx, dataLen, data, timeout, block->cookie);
    block = block->next;
  }
}

bool
TMR_memoryProvider(void *cookie, uint16_t *size, uint8_t *data)
{
  TMR_memoryCookie *mc;
  int len;

  mc = cookie;

  if (0 == mc->firmwareSize)
  {
    return false;
  }

  if (*size > mc->firmwareSize)
  {
    *size =(uint16_t) mc->firmwareSize;
  }

  len = *size;

  memcpy(data, mc->firmwareStart, len);
  
  mc->firmwareSize -= len;
  mc->firmwareStart += len;

  return true;
}

bool
TMR_fileProvider(void *cookie, uint16_t *size, uint8_t *data)
{
  FILE *fp;
  size_t len;

  fp = cookie;

  len = fread(data, 1, *size, fp);
  if (0 == len)
  {
    return false;
  }
  
  *size = (uint16_t) len;
  return true;
}


/**
 * Initialize TMR_TagReadData with default values.
 * The tagData buffer will be initialized to TMR_MAX_EMBEDDED_DATA_LENGTH
 * which can be found in tm_config.h.  
 * If this value is zero, then the buffer is pointed to NULL.
 * @param trd Pointer to the TMR_TagReadData structure to initialize
 */
TMR_Status
TMR_TRD_init(TMR_TagReadData *trd)
{
#if TMR_MAX_EMBEDDED_DATA_LENGTH
  trd->data.list = trd->_dataList;
#else
  trd->data.list = NULL;
#endif
  trd->data.max = TMR_MAX_EMBEDDED_DATA_LENGTH;
  trd->data.len = 0;

  return TMR_SUCCESS;
}

/**
 * Initialize a TMR_TagReadData with the provided data storage area.
 *
 * @param trd Pointer to the TMR_TagReadData structure to initialize
 * @param size The number of bytes pointed to
 * @param buf Pointer to the uint8_t storage area
 */
TMR_Status
TMR_TRD_init_data(TMR_TagReadData *trd, uint16_t size, uint8_t *buf)
{
  trd->data.max = size;
  trd->data.len = 0;
  trd->data.list = buf;

  return TMR_SUCCESS;
}

/**
 * Initialize a TMR_Filter structure as tag data (EPC) filter with the
 * provided tag (which is copied).
 * 
 * @param filter Pointer to the filter structure to initialize
 * @param tag TMR_TagData to use as the filter value
 */
TMR_Status
TMR_TF_init_tag(TMR_TagFilter *filter, TMR_TagData *tag)
{
  
  filter->type = TMR_FILTER_TYPE_TAG_DATA;
  filter->u.tagData = *tag;
  
  return TMR_SUCCESS;
}

/**
 * Initialize a TMR_Filter structure as a Gen2 select filter with the
 * provided parameters.
 * 
 * @param filter Pointer to the filter structure to initialize
 * @param invert Whether to invert the result of the select
 * @param bank The memory bank on the tag to compare with the data
 * @param bitPointer The bit address of the tag data to compare
 * @param maskBitLength The length of the data to compare
 * @param mask The data to compare
 */
TMR_Status
TMR_TF_init_gen2_select(TMR_TagFilter *filter, bool invert, TMR_GEN2_Bank bank,
                        uint32_t bitPointer, uint16_t maskBitLength,
                        uint8_t *mask)
{
  
  filter->type = TMR_FILTER_TYPE_GEN2_SELECT;
  filter->u.gen2Select.invert = invert;
  filter->u.gen2Select.bank = bank;
  filter->u.gen2Select.bitPointer = bitPointer;
  filter->u.gen2Select.maskBitLength = maskBitLength;
  filter->u.gen2Select.mask = mask;

  return TMR_SUCCESS;
}


#ifdef TMR_ENABLE_ISO180006B
/**
 * Initialize a TMR_Filter structure as an ISO180006B select filter with the
 * provided parameters.
 * 
 * @param filter Pointer to the filter structure to initialize
 * @param invert Whether to invert the result of the select
 * @param op The operation to use to compare the provided data with the tag data
 * @param address The address of the 8 bytes of data on the tag to compare 
 * @param mask 8-bit mask of the bytes to compare
 * @param wordData The data to compare to the tag data
 */
TMR_Status TMR_TF_init_ISO180006B_select(TMR_TagFilter *filter, bool invert,
                                         TMR_ISO180006B_SelectOp op,
                                         uint8_t address, uint8_t mask,
                                         uint8_t wordData[8])
{

  filter->type = TMR_FILTER_TYPE_ISO180006B_SELECT;
  filter->u.iso180006bSelect.invert = invert;
  filter->u.iso180006bSelect.op = op;
  filter->u.iso180006bSelect.address = address;
  filter->u.iso180006bSelect.mask = mask;
  memcpy(filter->u.iso180006bSelect.data, wordData, 8);
  
  return TMR_SUCCESS;
}
#endif /* TMR_ENABLE_ISO180006B */


bool
TMR_TF_match(TMR_TagFilter *filter, TMR_TagData *tag)
{
  uint32_t i, bitAddr;
  bool match;
  TMR_GEN2_Select *sel;

  if (TMR_FILTER_TYPE_GEN2_SELECT != filter->type)
  {
    return false;
  }

  if (TMR_TAG_PROTOCOL_GEN2 != tag->protocol)
  {
    return false;
  }

  sel = &filter->u.gen2Select;

  if (TMR_GEN2_BANK_EPC != sel->bank)
  {
    /*
     * Can't perform non-EPC matches, since we don't have the rest of
     * the tag data.
     */
    return false;
  }

  i = 0;
  bitAddr = sel->bitPointer;
  /*
   * Matching against the CRC and PC does not have defined
   * behavior; see section 6.3.2.11.1.1 of Gen2 version 1.2.0.
   * We choose to let it match, because that's simple.
   */
  bitAddr -= 32;
  if (bitAddr < 0)
  {
    i -= bitAddr;
    bitAddr = 0;
  }

  match = true;
  for (; i < sel->maskBitLength; i++, bitAddr++)
  {
    if (bitAddr >(uint32_t) (tag->epcByteCount * 8))
    {
      match = false;
      break;
    }
    /* Extract the relevant bit from both the EPC and the mask. */
    if (((tag->epc[bitAddr / 8] >> (7 - (bitAddr & 7))) & 1) !=
        ((sel->mask[i / 8] >> (7 - (i & 7))) & 1))
    {
      match = false;
      break;
    }
  }
  if (sel->invert)
    match = match ? false : true;

  return match;
}


/**
 * Initialize a TMR_TagAuthentication structure as a Gen2 password.
 *
 * @param auth Pointer to the structure to initialize.
 * @param password The password 32-bit Gen2 password value.
 */
TMR_Status
TMR_TA_init_gen2(TMR_TagAuthentication *auth, TMR_GEN2_Password password)
{

  auth->type = TMR_AUTH_TYPE_GEN2_PASSWORD;
  auth->u.gen2Password = password;

  return TMR_SUCCESS;
}

/**
 * Initialize a TMR_ReadPlan structure as a simple read plan with the
 * provided parameters.  
 *
 * Only the mandatory elements are parameters to
 * this function. The optional elements, filters and tag operations,
 * can be set with TMR_RP_set_filter() and TMR_RP_set_tagop().
 *
 * @param plan Pointer to the read plan to initialize.
 * @param antennaCount Number of antennas in antenna list. A
 * zero-length list requests the reader to use all antennas known to
 * be connected at the time of the read operation.
 * @param antennaList Pointer to antenna numbers.
 * @param protocol Protocol
 * @param weight Weight.
 */
TMR_Status
TMR_RP_init_simple(TMR_ReadPlan *plan, uint8_t antennaCount,
                   uint8_t *antennaList, TMR_TagProtocol protocol,
                   uint32_t weight) 
{
  
  plan->type = TMR_READ_PLAN_TYPE_SIMPLE;
  plan->u.simple.antennas.max = antennaCount;
  plan->u.simple.antennas.len = antennaCount;
  plan->u.simple.antennas.list = antennaList;
  plan->u.simple.protocol = protocol;
  plan->u.simple.filter = NULL;
  plan->u.simple.tagop = NULL;
  plan->weight = weight;
  
  return TMR_SUCCESS;
}

/**
 * Set the filter of a simple read plan.
 *
 * @param plan Pointer to the read plan
 * @param filter Pointer to the filter 
 */
TMR_Status
TMR_RP_set_filter(TMR_ReadPlan *plan, TMR_TagFilter *filter)
{

  if (TMR_READ_PLAN_TYPE_SIMPLE != plan->type)
    return TMR_ERROR_INVALID;

  plan->u.simple.filter = filter;

  return TMR_SUCCESS;
}


/**
 * Set the tagop of a simple read plan.
 *
 * @param plan Pointer to the read plan
 * @param tagop Pointer to the tagop 
 */
TMR_Status
TMR_RP_set_tagop(TMR_ReadPlan *plan, TMR_TagOp *tagop)
{

  if (TMR_READ_PLAN_TYPE_SIMPLE != plan->type)
    return TMR_ERROR_INVALID;

  plan->u.simple.tagop = tagop;

  return TMR_SUCCESS;
}

/**
 * Initialize a TMR_ReadPlan structure as a multi-read plan with the
 * provided parameters.
 *
 * @param plan Pointer to the read plan to initialize.
 * @param plans Array of pointers to read plans to include
 * @param planCount Number of elements in array
 * @param weight Weight.
 */
TMR_Status
TMR_RP_init_multi(TMR_ReadPlan *plan, TMR_ReadPlan **plans, uint8_t planCount,
                  uint32_t weight)
{
  plan->type = TMR_READ_PLAN_TYPE_MULTI;
  plan->u.multi.plans = plans;
  plan->u.multi.planCount = planCount;
  plan->u.multi.totalWeight = 0;
  plan->weight = weight;

  return TMR_SUCCESS;
}


/**
 * Initialize a TMR_TagLockAction as a Gen2 lock action with the
 * provided parameters.
 *
 * @param lockAction Pointer to the structure to initialize.
 * @param mask mask
 * @param action action
 */
TMR_Status
TMR_TLA_init_gen2(TMR_TagLockAction *lockAction, uint16_t mask, uint16_t action)
{

  lockAction->type = TMR_LOCK_ACTION_TYPE_GEN2_LOCK_ACTION;
  lockAction->u.gen2LockAction.mask = mask;
  lockAction->u.gen2LockAction.action = action;

  return TMR_SUCCESS;
}


#ifdef TMR_ENABLE_ISO180006B
/**
 * Initialize a TMR_TagLockAction as an ISO180006B lock action with the
 * provided parameters.
 *
 * @param lockAction Pointer to the structure to initialize.
 * @param address The byte to lock
 */
TMR_Status
TMR_TLA_init_ISO180006B(TMR_TagLockAction *lockAction, uint8_t address)
{

  lockAction->type = TMR_LOCK_ACTION_TYPE_ISO180006B_LOCK_ACTION;
  lockAction->u.iso180006bLockAction.address = address;

  return TMR_SUCCESS;
}

#endif /* TMR_ENABLE_ISO180006B */


/**
 * Initialize a TMR_TagOp as a GEN2 EPC write operation with the
 * provided parameters.
 * 
 * @param tagop Pointer to the tagop structure to initialize.
 * @param epc EPC to write
 */
TMR_Status
TMR_TagOp_init_GEN2_WriteTag(TMR_TagOp *tagop, TMR_TagData* epc)
{
  tagop->type = TMR_TAGOP_GEN2_WRITETAG;
  tagop->u.gen2.u.writeTag.epcptr = epc;  /* Takes pointer to EPC; doesn't make an actual copy */

  return TMR_SUCCESS;
}

/**
 * Initialize a TMR_TagOp as a GEN2 data read operation with the
 * provided parameters.
 * 
 * @param tagop Pointer to the tagop structure to initialize.
 * @param bank Memory bank from which to read
 * @param wordAddress Word address of location in bank from which to read
 * @param len Number of words to read
 */
TMR_Status
TMR_TagOp_init_GEN2_ReadData(TMR_TagOp *tagop, TMR_GEN2_Bank bank,
                             uint32_t wordAddress, uint8_t len)
{

  tagop->type = TMR_TAGOP_GEN2_READDATA;
  tagop->u.gen2.u.readData.bank = bank;
  tagop->u.gen2.u.readData.wordAddress = wordAddress;
  tagop->u.gen2.u.readData.len = len;

  return TMR_SUCCESS;
}

/**
 * Initialize a TMR_TagOp as a ISO18000B data read operation with the
 * provided parameters.
 * 
 * @param tagop Pointer to the tagop structure to initialize.
 * @param byteAddress Address of location in bank from which to read
 * @param len Number of bytes to read
 */
TMR_Status
TMR_TagOp_init_ISO180006B_ReadData(TMR_TagOp *tagop, uint8_t byteAddress, uint8_t len)
{

  tagop->type = TMR_TAGOP_ISO180006B_READDATA;
  tagop->u.iso180006b.u.readData.byteAddress = byteAddress;
  tagop->u.iso180006b.u.readData.len = len;

  return TMR_SUCCESS;
}


/**
 * Initialize a TMR_TagOp as a GEN2 data write operation with the
 * provided parameters.
 * 
 * @param tagop Pointer to the tagop structure to initialize.
 * @param bank Memory bank to write into
 * @param wordAddress Word address of location to begin write
 * @param data Data to write
 */
TMR_Status
TMR_TagOp_init_GEN2_WriteData(TMR_TagOp *tagop, TMR_GEN2_Bank bank,
                              uint32_t wordAddress, TMR_uint16List *data)
{
  tagop->type = TMR_TAGOP_GEN2_WRITEDATA;
  tagop->u.gen2.u.writeData.bank = bank;
  tagop->u.gen2.u.writeData.wordAddress = wordAddress;
  tagop->u.gen2.u.writeData.data = *data; /* Copies pointer to the words adata but not data */

  return TMR_SUCCESS;
}

/**
 * Initialize a TMR_TagOp as a ISO180006B data write operation with the
 * provided parameters.
 * 
 * @param tagop Pointer to the tagop structure to initialize.
 * @param byteAddress  address of location to begin write
 * @param data Data to write
 */
TMR_Status
TMR_TagOp_init_ISO180006B_WriteData(TMR_TagOp *tagop, uint8_t byteAddress, TMR_uint8List *data)
{
  tagop->type = TMR_TAGOP_ISO180006B_WRITEDATA;
  tagop->u.iso180006b.u.writeData.byteAddress = byteAddress;
  tagop->u.iso180006b.u.writeData.data = *data;
  return TMR_SUCCESS;
}

/**
 * Initialize a TMR_TagOp as a GEN2 lock operation with the provided parameters.
 * @param tagop Pointer to the tagop structure to initialize.
 * @param mask mask
 * @param action action
 * @param accessPassword The password to use to lock the tag.
 */
TMR_Status
TMR_TagOp_init_GEN2_Lock(TMR_TagOp *tagop, uint16_t mask, uint16_t action, TMR_GEN2_Password accessPassword)
{

  tagop->type = TMR_TAGOP_GEN2_LOCK;
  tagop->u.gen2.u.lock.mask = mask;
  tagop->u.gen2.u.lock.action = action;
  tagop->u.gen2.u.lock.accessPassword = accessPassword;

  return TMR_SUCCESS;
}

/**
 * Initialize a TMR_TagOp as a ISO180006B lock operation with the provided parameters.
 * @param tagop Pointer to the tagop structure to initialize.
 * @param address The memory address of the byte to lock.
 */
TMR_Status
TMR_TagOp_init_ISO180006B_Lock(TMR_TagOp *tagop,  uint8_t address)
{

  tagop->type = TMR_TAGOP_ISO180006B_LOCK;
  tagop->u.iso180006b.u.lock.address = address;

  return TMR_SUCCESS;
}

/**
 * Initialize a TMR_TagOp as a GEN2 kill operation with the provided parameters.
 * @param tagop Pointer to the tagop structure to initialize.
 * @param killPassword tag kill password
 */
TMR_Status
TMR_TagOp_init_GEN2_Kill(TMR_TagOp *tagop, TMR_GEN2_Password killPassword)
{

  tagop->type = TMR_TAGOP_GEN2_KILL;
  tagop->u.gen2.u.kill.password = killPassword;

  return TMR_SUCCESS;
}


/**
 * Initialize a TMR_TagOp as a GEN2 BlockWrite operation with the provided parameters.
 * @param tagop Pointer to the tagop structure to initialize.
 * @param bank bank
 * @param wordPtr word pointer
 * @param data data (The length of the data specifies the word count)
 */
TMR_Status
TMR_TagOp_init_GEN2_BlockWrite(TMR_TagOp *tagop, TMR_GEN2_Bank bank, uint32_t wordPtr, TMR_uint16List *data)
{
  tagop->type = TMR_TAGOP_GEN2_BLOCKWRITE;
  tagop->u.gen2.u.blockWrite.bank = bank;
  tagop->u.gen2.u.blockWrite.wordPtr = wordPtr;
  tagop->u.gen2.u.blockWrite.data.len = data->len;
  tagop->u.gen2.u.blockWrite.data.list = data->list;
  tagop->u.gen2.u.blockWrite.data.max = data->max;
  return TMR_SUCCESS;
}

/**
 * Initialize a TMR_TagOp as a GEN2 BlockPermaLock operation with the provided parameters.
 * @param tagop Pointer to the tagop structure to initialize.
 * @param readLock readLock
 * @param bank bank
 * @param blockPtr block pointer
 * @param mask mask (The length of the mask specifies the block range)
 */
TMR_Status
TMR_TagOp_init_GEN2_BlockPermaLock(TMR_TagOp *tagop, uint8_t readLock, TMR_GEN2_Bank bank, uint32_t blockPtr, TMR_uint16List *mask)
{
  tagop->type = TMR_TAGOP_GEN2_BLOCKPERMALOCK;
  tagop->u.gen2.u.blockPermaLock.readLock = readLock;
  tagop->u.gen2.u.blockPermaLock.bank = bank;
  tagop->u.gen2.u.blockPermaLock.blockPtr = blockPtr;
  tagop->u.gen2.u.blockPermaLock.mask.len = mask->len;
  tagop->u.gen2.u.blockPermaLock.mask.list = mask->list;
  tagop->u.gen2.u.blockPermaLock.mask.max = mask->max;
  return TMR_SUCCESS;

}

/**
 * Initialize a TMR_TagOp as a Higgs2 Partial Load Image operation with the provided parameters.
 * @param tagop Pointer to the tagop structure to initialize.
 * @param killPassword kill password
 * @param accessPassword access password
 * @param epc EPC to write
 */
TMR_Status 
TMR_TagOp_init_GEN2_Alien_Higgs2_PartialLoadImage(TMR_TagOp *tagop, TMR_GEN2_Password killPassword,
                                       TMR_GEN2_Password accessPassword, TMR_TagData *epc)
{
  tagop->type = TMR_TAGOP_GEN2_ALIEN_HIGGS2_PARTIALLOADIMAGE;
  tagop->u.gen2.u.custom.chipType = TMR_SR_GEN2_ALIEN_HIGGS_SILICON;
  tagop->u.gen2.u.custom.u.alien.u.higgs2.u.partialLoadImage.accessPassword = accessPassword;
  tagop->u.gen2.u.custom.u.alien.u.higgs2.u.partialLoadImage.killPassword = killPassword;
  tagop->u.gen2.u.custom.u.alien.u.higgs2.u.partialLoadImage.epcptr = epc; /* Takes pointer to EPC; doesn't make an actual copy */

  return TMR_SUCCESS;
}

/**
 * Initialize a TMR_TagOp as a Higgs2 Full Load Image operation with the provided parameters.
 * @param tagop Pointer to the tagop structure to initialize.
 * @param killPassword kill password
 * @param accessPassword access password
 * @param lockBits Lock bits to write to the tag
 * @param pcWord PC word to write to the tag
 * @param epc EPC to write
 */
TMR_Status
TMR_TagOp_init_GEN2_Alien_Higgs2_FullLoadImage(TMR_TagOp *tagop, TMR_GEN2_Password killPassword,
                                    TMR_GEN2_Password accessPassword, uint16_t lockBits, 
                                    uint16_t pcWord, TMR_TagData *epc)
{
  tagop->type = TMR_TAGOP_GEN2_ALIEN_HIGGS2_FULLLOADIMAGE;
  tagop->u.gen2.u.custom.chipType = TMR_SR_GEN2_ALIEN_HIGGS_SILICON;
  tagop->u.gen2.u.custom.u.alien.u.higgs2.u.fullLoadImage.accessPassword = accessPassword;
  tagop->u.gen2.u.custom.u.alien.u.higgs2.u.fullLoadImage.killPassword = killPassword;
  tagop->u.gen2.u.custom.u.alien.u.higgs2.u.fullLoadImage.lockBits = lockBits;
  tagop->u.gen2.u.custom.u.alien.u.higgs2.u.fullLoadImage.pcWord = pcWord;
  tagop->u.gen2.u.custom.u.alien.u.higgs2.u.fullLoadImage.epcptr = epc; /* Takes pointer to EPC; doesn't make an actual copy */

  return TMR_SUCCESS;
}

/**
 * Initialize a TMR_TagOp as a Higgs3 Fast Load Image operation with the provided parameters.
 * @param tagop Pointer to the tagop structure to initialize.
 * @param currentAccessPassword The access password used to write to the tag
 * @param accessPassword access password
 * @param killPassword kill password
 * @param pcWord PC word to write to the tag
 * @param epc EPC to write
 */
TMR_Status TMR_TagOp_init_GEN2_Alien_Higgs3_FastLoadImage(TMR_TagOp *tagop, TMR_GEN2_Password currentAccessPassword,
                                    TMR_GEN2_Password accessPassword, TMR_GEN2_Password killPassword, 
                                    uint16_t pcWord, TMR_TagData *epc)
{
  tagop->type = TMR_TAGOP_GEN2_ALIEN_HIGGS3_FASTLOADIMAGE;
  tagop->u.gen2.u.custom.chipType = TMR_SR_GEN2_ALIEN_HIGGS3_SILICON;
  tagop->u.gen2.u.custom.u.alien.u.higgs3.u.fastLoadImage.currentAccessPassword = currentAccessPassword;
  tagop->u.gen2.u.custom.u.alien.u.higgs3.u.fastLoadImage.killPassword = killPassword;
  tagop->u.gen2.u.custom.u.alien.u.higgs3.u.fastLoadImage.accessPassword = accessPassword;
  tagop->u.gen2.u.custom.u.alien.u.higgs3.u.fastLoadImage.pcWord = pcWord;
  tagop->u.gen2.u.custom.u.alien.u.higgs3.u.fastLoadImage.epcptr = epc; /* Takes pointer to EPC; doesn't make an actual copy */

  return TMR_SUCCESS;
}

/**
 * Initialize a TMR_TagOp as a Higgs3 Load Image operation with the provided parameters.
 * @param tagop Pointer to the tagop structure to initialize.
 * @param currentAccessPassword The access password used to write to the tag
 * @param accessPassword access password
 * @param killPassword kill password
 * @param pcWord PC word to write to the tag
 * @param epcAndUserData Tag EPC and user data to write to the tag (76 bytes)
 */
TMR_Status TMR_TagOp_init_GEN2_Alien_Higgs3_LoadImage(TMR_TagOp *tagop, TMR_GEN2_Password currentAccessPassword,
                                    TMR_GEN2_Password accessPassword, TMR_GEN2_Password killPassword, 
                                    uint16_t pcWord, TMR_uint8List *epcAndUserData)
{
  tagop->type = TMR_TAGOP_GEN2_ALIEN_HIGGS3_LOADIMAGE;
  tagop->u.gen2.u.custom.chipType = TMR_SR_GEN2_ALIEN_HIGGS3_SILICON;
  tagop->u.gen2.u.custom.u.alien.u.higgs3.u.loadImage.currentAccessPassword = currentAccessPassword;
  tagop->u.gen2.u.custom.u.alien.u.higgs3.u.loadImage.killPassword = killPassword;
  tagop->u.gen2.u.custom.u.alien.u.higgs3.u.loadImage.accessPassword = accessPassword;
  tagop->u.gen2.u.custom.u.alien.u.higgs3.u.loadImage.pcWord = pcWord;
  tagop->u.gen2.u.custom.u.alien.u.higgs3.u.loadImage.epcAndUserData = epcAndUserData; /* Takes pointer epcAndUserData; doesn't make an actual copy */

  return TMR_SUCCESS;
}

/**
 * Initialize a TMR_TagOp as a Higgs3 Block Read Lock operation with the provided parameters.
 * @param tagop Pointer to the tagop structure to initialize.
 * @param accessPassword access password
 * @param lockBits A bitmask of bits to lock
 */
TMR_Status TMR_TagOp_init_GEN2_Alien_Higgs3_BlockReadLock(TMR_TagOp *tagop, TMR_GEN2_Password accessPassword, uint8_t lockBits)
{
  tagop->type = TMR_TAGOP_GEN2_ALIEN_HIGGS3_BLOCKREADLOCK;
  tagop->u.gen2.u.custom.chipType = TMR_SR_GEN2_ALIEN_HIGGS3_SILICON;
  tagop->u.gen2.u.custom.u.alien.u.higgs3.u.blockReadLock.accessPassword = accessPassword;
  tagop->u.gen2.u.custom.u.alien.u.higgs3.u.blockReadLock.lockBits = lockBits;
  
  return TMR_SUCCESS;
}

/**
 * Initialize a TMR_TagOp as a NXP G2I set read protect operation with the provided parameters.
 * @param tagop Pointer to the tagop structure to initialize.
 * @param accessPassword access password
 */
TMR_Status TMR_TagOp_init_GEN2_NXP_G2I_SetReadProtect(TMR_TagOp *tagop, TMR_GEN2_Password accessPassword)
{
  tagop->type = TMR_TAGOP_GEN2_NXP_SETREADPROTECT;
  tagop->u.gen2.u.custom.chipType = TMR_SR_GEN2_NXP_G2I_SILICON;
  tagop->u.gen2.u.custom.u.nxp.u.setReadProtect.accessPassword = accessPassword;
  return TMR_SUCCESS;
}

/**
 * Initialize a TMR_TagOp as a NXP G2X set read protect operation with the provided parameters.
 * @param tagop Pointer to the tagop structure to initialize.
 * @param accessPassword access password
 */
TMR_Status TMR_TagOp_init_GEN2_NXP_G2X_SetReadProtect(TMR_TagOp *tagop, TMR_GEN2_Password accessPassword)
{
  tagop->type = TMR_TAGOP_GEN2_NXP_SETREADPROTECT;
  tagop->u.gen2.u.custom.chipType = TMR_SR_GEN2_NXP_G2X_SILICON;
  tagop->u.gen2.u.custom.u.nxp.u.setReadProtect.accessPassword = accessPassword;
  return TMR_SUCCESS;
}

/**
 * Initialize a TMR_TagOp as a NXP G2I reset read protect operation with the provided parameters.
 * @param tagop Pointer to the tagop structure to initialize.
 * @param accessPassword access password
 */
TMR_Status TMR_TagOp_init_GEN2_NXP_G2I_ResetReadProtect(TMR_TagOp *tagop, TMR_GEN2_Password accessPassword)
{
  tagop->type = TMR_TAGOP_GEN2_NXP_RESETREADPROTECT;
  tagop->u.gen2.u.custom.chipType = TMR_SR_GEN2_NXP_G2I_SILICON;
  tagop->u.gen2.u.custom.u.nxp.u.resetReadProtect.accessPassword = accessPassword;
  return TMR_SUCCESS;
}

/**
 * Initialize a TMR_TagOp as a NXP G2X reset read protect operation with the provided parameters.
 * @param tagop Pointer to the tagop structure to initialize.
 * @param accessPassword access password
 */
TMR_Status TMR_TagOp_init_GEN2_NXP_G2X_ResetReadProtect(TMR_TagOp *tagop, TMR_GEN2_Password accessPassword)
{
  tagop->type = TMR_TAGOP_GEN2_NXP_RESETREADPROTECT;
  tagop->u.gen2.u.custom.chipType = TMR_SR_GEN2_NXP_G2X_SILICON;
  tagop->u.gen2.u.custom.u.nxp.u.resetReadProtect.accessPassword = accessPassword;
  return TMR_SUCCESS;
}

/**
 * Initialize a TMR_TagOp as a NXP G2I Change EAS operation with the provided parameters.
 * @param tagop Pointer to the tagop structure to initialize.
 * @param accessPassword access password
 * @param resetEAS   -  if true, Reset EAS
 *                      if false, Set EAS
 */
TMR_Status TMR_TagOp_init_GEN2_NXP_G2I_ChangeEAS(TMR_TagOp *tagop, TMR_GEN2_Password accessPassword, bool resetEAS)
{
  tagop->type = TMR_TAGOP_GEN2_NXP_CHANGEEAS;
  tagop->u.gen2.u.custom.chipType = TMR_SR_GEN2_NXP_G2I_SILICON;
  tagop->u.gen2.u.custom.u.nxp.u.changeEAS.accessPassword = accessPassword;
  tagop->u.gen2.u.custom.u.nxp.u.changeEAS.reset = resetEAS;

  return TMR_SUCCESS;
}

/**
 * Initialize a TMR_TagOp as a NXP G2X Change EAS operation with the provided parameters.
 * @param tagop Pointer to the tagop structure to initialize.
 * @param accessPassword access password
 * @param resetEAS   -  if true, Reset EAS
 *                      if false, Set EAS
 */
TMR_Status TMR_TagOp_init_GEN2_NXP_G2X_ChangeEAS(TMR_TagOp *tagop, TMR_GEN2_Password accessPassword, bool resetEAS)
{
  tagop->type = TMR_TAGOP_GEN2_NXP_CHANGEEAS;
  tagop->u.gen2.u.custom.chipType = TMR_SR_GEN2_NXP_G2X_SILICON;
  tagop->u.gen2.u.custom.u.nxp.u.changeEAS.accessPassword = accessPassword;
  tagop->u.gen2.u.custom.u.nxp.u.changeEAS.reset = resetEAS;

  return TMR_SUCCESS;
}

/**
 * Initialize a TMR_TagOp as a NXP G2I EAS alarm operation with the provided parameters.
 * @param tagop Pointer to the tagop structure to initialize.
 * @param dr Gen2 divide ratio to use
 * @param m Gen2 M(tag encoding) parameter to use
 * @param trExt txExt Gen2 TrExt value to use
 */
TMR_Status TMR_TagOp_init_GEN2_NXP_G2I_EASAlarm(TMR_TagOp *tagop, TMR_GEN2_DivideRatio dr, TMR_GEN2_TagEncoding m, TMR_GEN2_TrExt trExt)
{
  tagop->type = TMR_TAGOP_GEN2_NXP_EASALARM;
  tagop->u.gen2.u.custom.chipType = TMR_SR_GEN2_NXP_G2I_SILICON;
  tagop->u.gen2.u.custom.u.nxp.u.EASAlarm.dr = dr;
  tagop->u.gen2.u.custom.u.nxp.u.EASAlarm.m = m;
  tagop->u.gen2.u.custom.u.nxp.u.EASAlarm.trExt = trExt;
  
  return TMR_SUCCESS;
}

/**
 * Initialize a TMR_TagOp as a NXP G2X EAS alarm operation with the provided parameters.
 * @param tagop Pointer to the tagop structure to initialize.
 * @param dr Gen2 divide ratio to use
 * @param m Gen2 M(tag encoding) parameter to use
 * @param trExt txExt Gen2 TrExt value to use
 */
TMR_Status TMR_TagOp_init_GEN2_NXP_G2X_EASAlarm(TMR_TagOp *tagop, TMR_GEN2_DivideRatio dr, TMR_GEN2_TagEncoding m, TMR_GEN2_TrExt trExt)
{
  tagop->type = TMR_TAGOP_GEN2_NXP_EASALARM;
  tagop->u.gen2.u.custom.chipType = TMR_SR_GEN2_NXP_G2X_SILICON;
  tagop->u.gen2.u.custom.u.nxp.u.EASAlarm.dr = dr;
  tagop->u.gen2.u.custom.u.nxp.u.EASAlarm.m = m;
  tagop->u.gen2.u.custom.u.nxp.u.EASAlarm.trExt = trExt;
  
  return TMR_SUCCESS;
}

/**
 * Initialize a TMR_TagOp as a NXP G2i Calibrate operation with the provided parameters.
 *
 * @param tagop Pointer to the tagop structure to initialize.
 * @param accessPassword access password
 */
TMR_Status TMR_TagOp_init_GEN2_NXP_G2I_Calibrate(TMR_TagOp *tagop, TMR_GEN2_Password accessPassword)
{
  tagop->type = TMR_TAGOP_GEN2_NXP_CALIBRATE;
  tagop->u.gen2.u.custom.chipType = TMR_SR_GEN2_NXP_G2I_SILICON;
  tagop->u.gen2.u.custom.u.nxp.u.calibrate.accessPassword = accessPassword;

  return TMR_SUCCESS;
}

/**
 * Initialize a TMR_TagOp as a NXP G2x Calibrate operation with the provided parameters.
 *
 * @param tagop Pointer to the tagop structure to initialize.
 * @param accessPassword access password
 */
TMR_Status TMR_TagOp_init_GEN2_NXP_G2X_Calibrate(TMR_TagOp *tagop, TMR_GEN2_Password accessPassword)
{
  tagop->type = TMR_TAGOP_GEN2_NXP_CALIBRATE;
  tagop->u.gen2.u.custom.chipType = TMR_SR_GEN2_NXP_G2X_SILICON;
  tagop->u.gen2.u.custom.u.nxp.u.calibrate.accessPassword = accessPassword;

  return TMR_SUCCESS;
}

/**
 * Initialize a TMR_TagOp as a NXP G2i Change Config operation with the provided parameters.
 * @param tagop Pointer to the tagop structure to initialize.
 * @param accessPassword access password
 * @param configWord ConfigWord to write to the tag.
 */
TMR_Status TMR_TagOp_init_GEN2_NXP_G2I_ChangeConfig(TMR_TagOp *tagop, TMR_GEN2_Password accessPassword, TMR_NXP_ConfigWord configWord)
{
  tagop->type = TMR_TAGOP_GEN2_NXP_CHANGECONFIG;
  tagop->u.gen2.u.custom.chipType = TMR_SR_GEN2_NXP_G2I_SILICON;
  tagop->u.gen2.u.custom.u.nxp.u.changeConfig.accessPassword = accessPassword;
  tagop->u.gen2.u.custom.u.nxp.u.changeConfig.configWord = configWord;

  return TMR_SUCCESS;
}

/**
 * Initialize a TMR_TagOp as a NXP G2x Change Config operation with the provided parameters.
 * @param tagop Pointer to the tagop structure to initialize.
 * @param accessPassword access password
 * @param configWord ConfigWord to write to the tag.
 */
TMR_Status TMR_TagOp_init_GEN2_NXP_G2X_ChangeConfig(TMR_TagOp *tagop, TMR_GEN2_Password accessPassword, TMR_NXP_ConfigWord configWord)
{
  tagop->type = TMR_TAGOP_GEN2_NXP_CHANGECONFIG;
  tagop->u.gen2.u.custom.chipType = TMR_SR_GEN2_NXP_G2X_SILICON;
  tagop->u.gen2.u.custom.u.nxp.u.changeConfig.accessPassword = accessPassword;
  tagop->u.gen2.u.custom.u.nxp.u.changeConfig.configWord = configWord;

  return TMR_SUCCESS;
}

/**
 * Initialize a TMR_TagOp as a Monza4 QT Read/Write operation with the provided parameters.
 * @param tagop Pointer to the tagop structure to initialize.
 * @param accessPassword access password
 * @param controlByte The QT control Byte 
 * @param payload The QT payload
 */
TMR_Status 
TMR_TagOp_init_GEN2_Impinj_Monza4_QTReadWrite(TMR_TagOp *tagop, TMR_GEN2_Password accessPassword,
                                  TMR_Monza4_ControlByte controlByte, TMR_Monza4_Payload payload)
{
  tagop->type = TMR_TAGOP_GEN2_IMPINJ_MONZA4_QTREADWRITE;
  tagop->u.gen2.u.custom.chipType = TMR_SR_GEN2_IMPINJ_MONZA4_SILICON;
  tagop->u.gen2.u.custom.u.impinj.u.monza4.u.qtReadWrite.accessPassword = accessPassword;
  tagop->u.gen2.u.custom.u.impinj.u.monza4.u.qtReadWrite.controlByte = controlByte;
  tagop->u.gen2.u.custom.u.impinj.u.monza4.u.qtReadWrite.payload = payload;

  return TMR_SUCCESS;
}

/**
 * Initialize TMR_UserConfigOp to set the default category.
 * @param config pointer to the TMR_UserConfigOp structure to initialize
 * @param op user configuration opeartion (save, restore or clear)
 */
TMR_Status
TMR_init_UserConfigOp(TMR_SR_UserConfigOp *config, TMR_SR_UserConfigOperation op)
{
  config->category = TMR_SR_ALL;
  config->op       = op;

  return TMR_SUCCESS;
}

/**
 *  Initialize TMR_NXP_ConfigWord to set the default value.
 *  If the instance of the above structure is created as 'static',
 *  then no need to call this constructor.
 *  @param configWord Instance of TMR_NXP_ConfigWord
 */
TMR_Status 
TMR_init_GEN2_NXP_G2I_ConfigWord(TMR_NXP_ConfigWord *configWord)
{
  configWord->data = 0x0000;
  return TMR_SUCCESS;
}

/**
 *  Initialize TMR_Monza4_ControlByte to set the default value to 0.
 *  If the instance of the above structure is created as 'static' or
 *  if it is already initialized to 0, then no need to call this constructor.
 *  @param controlByte Instance of TMR_Monza4_ControlByte
 */
TMR_Status
TMR_init_GEN2_Impinj_Monza4_ControlByte(TMR_Monza4_ControlByte *controlByte)
{
  controlByte->data = 0x00;
  return TMR_SUCCESS;
}

/**
 *  Initialize TMR_Monza4_Payload to set the default value to 0.
 *  If the instance of the above structure is created as 'static' or
 *  if it is already initialized to 0, then no need to call this constructor.
 *  @param payload Instance of TMR_Monza4_Payload
 */
TMR_Status
TMR_init_GEN2_Impinj_Monza4_Payload(TMR_Monza4_Payload *payload)
{
  payload->data = 0x0000;
  return TMR_SUCCESS;
}
