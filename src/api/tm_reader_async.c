/**
 *  @file tm_reader_async.c
 *  @brief Mercury API - background reading implementation
 *  @author Nathan Williams
 *  @date 11/18/2009
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
#ifdef TMR_ENABLE_BACKGROUND_READS

#include <stdlib.h>
#include <pthread.h>
#include <semaphore.h>
#include <time.h>

#ifndef WIN32
#include <sys/time.h>
#endif

#include "tm_reader.h"
#include "serial_reader_imp.h"
#include "osdep.h"
#include "tmr_utils.h"

static void *do_background_reads(void *arg);
static void *parse_tag_reads(void *arg);
static void TMR_stopContinuousRead(TMR_Reader *reader);
static void process_async_response(TMR_Reader *reader);

TMR_Status
TMR_startReading(struct TMR_Reader *reader)
{
  int ret;

  /* If model is M6e, and asyncOffTime is 0, only then use streaming */
  if ((TMR_SR_MODEL_M6E == reader->u.serialReader.versionInfo.hardware[0]) &&
        (reader->readParams.asyncOffTime == 0))
  {
    bool value = false;
    ret = TMR_SR_cmdSetReaderConfiguration(reader, TMR_SR_CONFIGURATION_ENABLE_READ_FILTER, &value);
    if (TMR_SUCCESS != ret)
    {
      return ret;
    }

    /** Background parser thread initialization
     *
     * Only M6e supports Streaming, and in case of other readers
     * we still use pseudo-async mechanism for continuous read.
     * To achieve continuous reading, create a parser thread
     */
    pthread_mutex_lock(&reader->parserLock);
    
    if (false == reader->parserSetup)
    {
      ret = pthread_create(&reader->backgroundParser, NULL,
                       parse_tag_reads, reader);
      if (0 != ret)
      {
        pthread_mutex_unlock(&reader->parserLock);
        return TMR_ERROR_NO_THREADS;
      }
      pthread_detach(reader->backgroundParser);
      reader->parserSetup = true;
    }

    reader->parserEnabled = true;

    /** Initialize semaphores only for the first time
     *  These semaphores are used only in case of streaming
     */
    sem_init(&reader->queue_length, 0, 0);
    sem_init(&reader->queue_slots, 0, TMR_MAX_QUEUE_SLOTS);
    /* Enable streaming */
    reader->u.serialReader.useStreaming = true;
    pthread_cond_signal(&reader->parserCond);
    pthread_mutex_unlock(&reader->parserLock);
  }

  /* Background reader thread initialization */
  pthread_mutex_lock(&reader->backgroundLock);

  if (false == reader->backgroundSetup)
  {
    ret = pthread_create(&reader->backgroundReader, NULL,
                         do_background_reads, reader);
    if (0 != ret)
    {
      pthread_mutex_unlock(&reader->backgroundLock);
      return TMR_ERROR_NO_THREADS;
    }
    pthread_detach(reader->backgroundReader);
    reader->backgroundSetup = true;
  }

  reader->backgroundEnabled = true;

  reader->u.serialReader.tagopFailureCount = 0;
  reader->u.serialReader.tagopSuccessCount = 0;

  pthread_cond_signal(&reader->backgroundCond);
  pthread_mutex_unlock(&reader->backgroundLock);

  /* End of Background reader thread initialization */

  return TMR_SUCCESS;
}

TMR_Status
TMR_stopReading(struct TMR_Reader *reader)
{
  /* De-activate the background reader thread */
  pthread_mutex_lock(&reader->backgroundLock);

  if (false == reader->backgroundSetup)
  {
    pthread_mutex_unlock(&reader->backgroundLock);
    return TMR_SUCCESS;
  }

  reader->backgroundEnabled = false;
  while (true == reader->backgroundRunning)
  {
    pthread_cond_wait(&reader->backgroundCond, &reader->backgroundLock);
  }
  pthread_mutex_unlock(&reader->backgroundLock);
  /* End of background reader de-activation */

  if (true == reader->u.serialReader.useStreaming)
  {
    /** In case of streaming on M6e, need to send stop continuous read command */
    TMR_stopContinuousRead(reader);

    pthread_mutex_lock(&reader->parserLock);
    /* De-activate background parser thread */
    reader->parserEnabled = false;
    /* release parser thread from waiting */
    sem_post(&reader->queue_length);
    while (true == reader->parserRunning)
    {
      pthread_cond_wait(&reader->parserCond, &reader->parserLock);
    }
    pthread_mutex_unlock(&reader->parserLock);
    /* End of background parser de-activation */

    /* Destroy the semaphores */
    sem_destroy(&reader->queue_length);
    sem_destroy(&reader->queue_slots);
    
    /* disable streaming */
    reader->u.serialReader.useStreaming = false;
  }

  return TMR_SUCCESS;
}

static void
notify_exception_listeners(TMR_Reader *reader, TMR_Status status)
{
  TMR_ReadExceptionListenerBlock *relb;

  pthread_mutex_lock(&reader->listenerLock);
  relb = reader->readExceptionListeners;
  while (relb)
  {
    relb->listener(reader, status, relb->cookie);
    relb = relb->next;
  }
  pthread_mutex_unlock(&reader->listenerLock);
}

TMR_Queue_tagReads *
dequeue(TMR_Reader *reader)
{
  TMR_Queue_tagReads *tagRead;
  pthread_mutex_lock(&reader->queue_lock);
  tagRead = reader->tagReadQueue;
  reader->tagReadQueue = reader->tagReadQueue->next;
  pthread_mutex_unlock(&reader->queue_lock);
  return(tagRead);
}


void enqueue(TMR_Reader *reader, TMR_Queue_tagReads *tagRead)
{
  pthread_mutex_lock(&reader->queue_lock);
  tagRead->next = reader->tagReadQueue;   /* Insert at head of list */
  reader->tagReadQueue = tagRead;
  pthread_mutex_unlock(&reader->queue_lock);
}

static void *
parse_tag_reads(void *arg)
{
  TMR_Reader *reader;
  TMR_Queue_tagReads *tagRead;
  uint16_t flags = 0;
  TMR_TagReadData trd;

  reader = arg;  

  while (1)
  {
    pthread_mutex_lock(&reader->parserLock);
    reader->parserRunning = false;
    pthread_cond_broadcast(&reader->parserCond);
    while (false == reader->parserEnabled)
    {
      pthread_cond_wait(&reader->parserCond, &reader->parserLock);
    }
    reader->parserRunning = true;
    pthread_mutex_unlock(&reader->parserLock);

    TMR_TRD_init(&trd);
    
    /**
     * Wait until queue_length is more than zero,
     * i.e., Queue should have atleast one tagRead to process
     */
    sem_wait(&reader->queue_length);

    if (NULL != reader->tagReadQueue)
    {
      /**
       * At this point there is a tagEntry in the queue
       * dequeue it and parse it.
       */          
      tagRead = dequeue(reader);
      if (false == tagRead->isStatusResponse)
      {
        /* Tag Buffer stream response */
        TMR_ReadListenerBlock *rlb;
    
        /* In case of streaming the flags always start at position 8*/
        flags = GETU16AT(tagRead->tagEntry, 8);
        TMR_SR_parseMetadataFromMessage(reader, &trd, flags, &tagRead->bufPointer, tagRead->tagEntry);
        if (TMR_READER_TYPE_SERIAL == reader->readerType)
        {
          TMR_SR_postprocessReaderSpecificMetadata(&trd, &reader->u.serialReader);
        }
        trd.reader = reader;

        /* notify tag read to listener */
        pthread_mutex_lock(&reader->listenerLock);
        rlb = reader->readListeners;
        while (rlb)
        {
          rlb->listener(reader, &trd, rlb->cookie);
          rlb = rlb->next;
        }
        pthread_mutex_unlock(&reader->listenerLock);
      }
      else
      {
        /* A status stream response */
        TMR_StatusListenerBlock *slb;
        uint8_t index = 0, j;
        TMR_SR_StatusReport report[TMR_SR_STATUS_MAX];

        int offset = tagRead->bufPointer;
        uint16_t flags;

        /* Get status content flags */
        flags = GETU16(tagRead->tagEntry, offset);

        if (0 != (flags & TMR_SR_STATUS_FREQUENCY))
        {
          report[index].type = TMR_SR_STATUS_FREQUENCY;
          report[index].u.fsr.freq = (uint32_t)(GETU24(tagRead->tagEntry, offset));
          index ++;
        }
        if (0 != (flags & TMR_SR_STATUS_TEMPERATURE))
        {
          report[index].type = TMR_SR_STATUS_TEMPERATURE;
          report[index].u.tsr.temp = GETU8(tagRead->tagEntry, offset);
          index ++;
        }
        if (0 != (flags & TMR_SR_STATUS_ANTENNA))
        {
          uint8_t tx, rx;
          report[index].type = TMR_SR_STATUS_ANTENNA;
          tx = GETU8(tagRead->tagEntry, offset);
          rx = GETU8(tagRead->tagEntry, offset);

          if (TMR_READER_TYPE_SERIAL == reader->readerType)
          {
            for (j = 0; j < reader->u.serialReader.txRxMap->len; j++)
            {
              if ((rx == reader->u.serialReader.txRxMap->list[j].rxPort) && (tx == reader->u.serialReader.txRxMap->list[j].txPort))
              {
                report[index].u.asr.ant = reader->u.serialReader.txRxMap->list[j].antenna;
                break;
              }
            }
          }
          index ++;
        }

        report[index].type = TMR_SR_STATUS_NONE;
      
        /* notify status response to listener */
        pthread_mutex_lock(&reader->listenerLock);
        slb = reader->statusListeners;
        while (slb)
        {
          slb->listener(reader, report, slb->cookie);
          slb = slb->next;
        }
        pthread_mutex_unlock(&reader->listenerLock);
      }
      
      /* Free the memory */
      free(tagRead->tagEntry);
      free(tagRead);
    
      /* Now, increment the queue_slots as we have removed one entry */
      sem_post(&reader->queue_slots);
    }
  }
  return NULL;
}

static void
TMR_stopContinuousRead(TMR_Reader *reader)
{
  TMR_Status ret;
  uint8_t msg[TMR_SR_MAX_PACKET_SIZE];
  uint8_t i, op;
  
  i = 2;
  op = TMR_SR_OPCODE_MULTI_PROTOCOL_TAG_OP;
  SETU8(msg, i, op);
  SETU16(msg, i, 0); /* Timeout, Currently ignored */
  SETU8(msg, i, (uint8_t)0x02); /* option - stop continuous reading */

  msg[1]=i - 3;  /* Install length */
  TMR_SR_sendMessage(reader, msg, &op, reader->u.serialReader.commandTimeout);

  /* Wait until the response for stop continuous read is received */
  do
  {
    ret = TMR_hasMoreTags(reader);
    if (TMR_SUCCESS == ret)
    {
      process_async_response(reader);
    }
    else
    {
      if (TMR_ERROR_IS_COMM(ret))
      {
        notify_exception_listeners(reader, ret);
        /** 
         * In case of timeout error or CRC error, flush the transport buffer.
         * this avoids receiving of junk response.
         */
        /* Handling this fix for serial reader now */
        if (TMR_READER_TYPE_SERIAL == reader->readerType)
        {
		      reader->u.serialReader.transport.flush(&reader->u.serialReader.transport);
        }
        break;
      }

      switch (ret)
      {
        case TMR_ERROR_NO_TAGS:
        case TMR_ERROR_NO_TAGS_FOUND:
        case TMR_ERROR_PARSE:
        case TMR_ERROR_MSG_WRONG_NUMBER_OF_DATA:
          break;
        default:
          notify_exception_listeners(reader, ret);
          break;
      }
    }
  } while (((reader->u.serialReader.bufResponse[2] != 0x2F) || (TMR_ERROR_NO_TAGS != ret)) && 
                    (TMR_ERROR_MSG_WRONG_NUMBER_OF_DATA != ret));
  
  {
    bool value = reader->u.serialReader.enableReadFiltering;
    ret = TMR_SR_cmdSetReaderConfiguration(reader, TMR_SR_CONFIGURATION_ENABLE_READ_FILTER, &value);
    if (TMR_SUCCESS != ret)
    {
      notify_exception_listeners(reader, ret);
    }
  }
}

static void
process_async_response(TMR_Reader *reader)
{
  TMR_Queue_tagReads *tagRead;

  /* Decrement Queue slots */
  sem_wait(&reader->queue_slots);

  tagRead = (TMR_Queue_tagReads *) malloc(sizeof(TMR_Queue_tagReads));
  tagRead->tagEntry = (uint8_t *) malloc(TMR_SR_MAX_PACKET_SIZE); /* size of bufResponse */
  memcpy(tagRead->tagEntry, reader->u.serialReader.bufResponse, TMR_SR_MAX_PACKET_SIZE);
  tagRead->bufPointer = reader->u.serialReader.bufPointer;
  tagRead->isStatusResponse = reader->u.serialReader.isStatusResponse;
  /* Enqueue the tagRead into Queue */
  enqueue(reader, tagRead);
  /* Increment queue_length */
  sem_post(&reader->queue_length);

  if (false == reader->u.serialReader.isStatusResponse)
  {
    reader->u.serialReader.tagsRemainingInBuffer--;
  }
}

static void *
do_background_reads(void *arg)
{
  TMR_Status ret;
  TMR_Reader *reader;
  uint32_t onTime, offTime;
  int32_t sleepTime;
  uint64_t end, now, difftime;
  bool trueAsyncflag = false;
  
  reader = arg;
    
  while (1)
  {
    /* Wait for reads to be enabled */
    pthread_mutex_lock(&reader->backgroundLock);
    reader->backgroundRunning = false;
    pthread_cond_broadcast(&reader->backgroundCond);
    while (false == reader->backgroundEnabled)
    {
      trueAsyncflag = false;
      pthread_cond_wait(&reader->backgroundCond, &reader->backgroundLock);
    }

    TMR_paramGet(reader, TMR_PARAM_READ_ASYNCONTIME, &onTime);
    TMR_paramGet(reader, TMR_PARAM_READ_ASYNCOFFTIME, &offTime);

    if (!trueAsyncflag)
    {
      ret = TMR_read(reader, onTime, NULL);
      if (TMR_SUCCESS != ret)
      {
        if ((TMR_ERROR_TIMEOUT == ret) || (TMR_ERROR_CRC_ERROR == ret) ||
              (TMR_ERROR_SYSTEM_UNKNOWN_ERROR == ret) || (TMR_ERROR_TM_ASSERT_FAILED == ret))
        {
          reader->u.serialReader.transport.flush(&reader->u.serialReader.transport);
          reader->backgroundEnabled = false;
        }
        notify_exception_listeners(reader, ret);
        pthread_mutex_unlock(&reader->backgroundLock);
        continue;
      }
      if(reader->u.serialReader.useStreaming)
      {
        /**
         * Set this flag, In case of true async reading
         * we have to send the command only once.
         */
        trueAsyncflag = true;
      }
    }

    reader->backgroundRunning = true;
    pthread_mutex_unlock(&reader->backgroundLock);

    if (true == reader->u.serialReader.useStreaming)
    {
      /**  
       * Streaming is enabled only in case of M6e, 
       * read till the end of stream.
       */            
      while (true)
      {
        ret = TMR_hasMoreTags(reader);
        if (TMR_SUCCESS == ret)
        {
          process_async_response(reader);
        }
        else if (TMR_ERROR_TAG_ID_BUFFER_FULL == ret)
        {
          /* In case of buffer full error, notify the exception */
          notify_exception_listeners(reader, ret);
          /**
           * Resubmit the search immediately, without user interaction
           * Resetting the trueAsyncFlag will send the continuous read command again.
           */
          ret = TMR_hasMoreTags(reader);
          trueAsyncflag = false;
          break;
        }
        else
        {
          if ((TMR_ERROR_TIMEOUT == ret) || (TMR_ERROR_CRC_ERROR == ret) ||
              (TMR_ERROR_SYSTEM_UNKNOWN_ERROR == ret) || (TMR_ERROR_TM_ASSERT_FAILED == ret))
          {
            notify_exception_listeners(reader, ret);
            /** 
             * In case of timeout error or CRC error, flush the transport buffer.
             * this avoids receiving of junk response.
             */
            if (TMR_ERROR_IS_COMM(ret) && (TMR_READER_TYPE_SERIAL == reader->readerType))
            {
              /* Handling this fix for serial reader now */
		          reader->u.serialReader.transport.flush(&reader->u.serialReader.transport);
            }
            /*Since we are in middle of search, stop the search */
            TMR_stopContinuousRead(reader);

            pthread_mutex_lock(&reader->backgroundLock);
            reader->backgroundEnabled = false;
            pthread_mutex_unlock(&reader->backgroundLock);

            pthread_mutex_lock(&reader->parserLock);
            reader->parserEnabled = false;
            sem_post(&reader->queue_length);
            pthread_mutex_unlock(&reader->parserLock);
          }
          else if ((TMR_ERROR_NO_TAGS_FOUND != ret) && (TMR_ERROR_NO_TAGS != ret))
          {
            /* Any exception other than 0x400 should be notified */
            notify_exception_listeners(reader, ret);
          }
          break;
        }
      }
    }
    else
    {
      /** 
       * On M5e and its variants, streaming is not supported
       * So still, retain the pseudo-async mechanism
       * Also, when asyncOffTime is non-zero the API should fallback to 
       * pseudo async mode.
       */
        
      end = tmr_gettime();

      while (TMR_SUCCESS == TMR_hasMoreTags(reader))
      {
        TMR_TagReadData trd;
        TMR_ReadListenerBlock *rlb;

        TMR_TRD_init(&trd);

        ret = TMR_getNextTag(reader, &trd);
        if (TMR_SUCCESS != ret)
        {
          pthread_mutex_lock(&reader->backgroundLock);
          reader->backgroundEnabled = false;
          pthread_mutex_unlock(&reader->backgroundLock);
          notify_exception_listeners(reader, ret);
          break;
        }

        pthread_mutex_lock(&reader->listenerLock);
        rlb = reader->readListeners;
        while (rlb)
        { 
          rlb->listener(reader, &trd, rlb->cookie);
          rlb = rlb->next;
        }
        pthread_mutex_unlock(&reader->listenerLock);
      }

      /* Wait for the asyncOffTime duration to pass */
      now = tmr_gettime();
      difftime = now - end;

      sleepTime = offTime - (uint32_t)difftime;
      if(sleepTime > 0)
      {
        tmr_sleep(sleepTime);
      }
    }    
  }
  return NULL;
}


TMR_Status
TMR_addReadListener(TMR_Reader *reader, TMR_ReadListenerBlock *b)
{

  if (0 != pthread_mutex_lock(&reader->listenerLock))
    return TMR_ERROR_TRYAGAIN;

  b->next = reader->readListeners;
  reader->readListeners = b;

  pthread_mutex_unlock(&reader->listenerLock);

  return TMR_SUCCESS;
}


TMR_Status
TMR_removeReadListener(TMR_Reader *reader, TMR_ReadListenerBlock *b)
{
  TMR_ReadListenerBlock *block, **prev;

  if (0 != pthread_mutex_lock(&reader->listenerLock))
    return TMR_ERROR_TRYAGAIN;

  prev = &reader->readListeners;
  block = reader->readListeners;
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

  pthread_mutex_unlock(&reader->listenerLock);

  if (block == NULL)
  {
    return TMR_ERROR_INVALID;
  }

  return TMR_SUCCESS;
}


TMR_Status
TMR_addReadExceptionListener(TMR_Reader *reader,
                             TMR_ReadExceptionListenerBlock *b)
{

  if (0 != pthread_mutex_lock(&reader->listenerLock))
    return TMR_ERROR_TRYAGAIN;

  b->next = reader->readExceptionListeners;
  reader->readExceptionListeners = b;

  pthread_mutex_unlock(&reader->listenerLock);

  return TMR_SUCCESS;
}


TMR_Status
TMR_removeReadExceptionListener(TMR_Reader *reader,
                                TMR_ReadExceptionListenerBlock *b)
{
  TMR_ReadExceptionListenerBlock *block, **prev;

  if (0 != pthread_mutex_lock(&reader->listenerLock))
    return TMR_ERROR_TRYAGAIN;

  prev = &reader->readExceptionListeners;
  block = reader->readExceptionListeners;
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

  pthread_mutex_unlock(&reader->listenerLock);

  if (block == NULL)
  {
    return TMR_ERROR_INVALID;
  }

  return TMR_SUCCESS;
}

TMR_Status
TMR_addStatusListener(TMR_Reader *reader, TMR_StatusListenerBlock *b)
{

  if (0 != pthread_mutex_lock(&reader->listenerLock))
    return TMR_ERROR_TRYAGAIN;

  b->next = reader->statusListeners;
  reader->statusListeners = b;

  /*reader->streamStats |= b->statusFlags & TMR_SR_STATUS_CONTENT_FLAGS_ALL;*/

  pthread_mutex_unlock(&reader->listenerLock);

  return TMR_SUCCESS;
}


TMR_Status
TMR_removeStatusListener(TMR_Reader *reader, TMR_StatusListenerBlock *b)
{
  TMR_StatusListenerBlock *block, **prev;

  if (0 != pthread_mutex_lock(&reader->listenerLock))
    return TMR_ERROR_TRYAGAIN;

  prev = &reader->statusListeners;
  block = reader->statusListeners;
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

  /* Remove the status flags requested by this listener and reframe */
  /*reader->streamStats = TMR_SR_STATUS_CONTENT_FLAG_NONE;
  {
    TMR_StatusListenerBlock *current;
    current = reader->statusListeners;
    while (NULL != current)
    {
      reader->streamStats |= current->statusFlags;
      current = current->next;
    }    
  }*/
  
  pthread_mutex_unlock(&reader->listenerLock);

  if (block == NULL)
  {
    return TMR_ERROR_INVALID;
  }

  return TMR_SUCCESS;
}

#endif /* TMR_ENABLE_BACKGROUND_READS */
