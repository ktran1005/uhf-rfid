/* ex: set tabstop=2 shiftwidth=2 expandtab cindent: */
#ifndef _TMR_SERIAL_READER_H
#define _TMR_SERIAL_READER_H
/**
 *  @file tmr_serial_reader.h
 *  @brief Mercury API - Serial Reader interface
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

#include "tmr_region.h"
#include "tmr_tag_protocol.h"
#include "tmr_serial_transport.h"

#define TMR_SR_MAX_PACKET_SIZE 256

#ifdef  __cplusplus
extern "C" {
#endif

/**
 * Defines the values for the parameter @c /reader/powerMode and the
 * return value and parameter to TMR_SR_cmdGetPowerMode and
 * TMR_SR_cmdSetPowerMode.
 */
typedef enum TMR_SR_PowerMode
{
  TMR_SR_POWER_MODE_MIN     = 0,
  TMR_SR_POWER_MODE_FULL    = 0,
  TMR_SR_POWER_MODE_MINSAVE = 1,
  TMR_SR_POWER_MODE_MEDSAVE = 2,
  TMR_SR_POWER_MODE_MAXSAVE = 3,
  TMR_SR_POWER_MODE_SLEEP   = 4,
  TMR_SR_POWER_MODE_MAX     = TMR_SR_POWER_MODE_SLEEP,
  TMR_SR_POWER_MODE_INVALID = TMR_SR_POWER_MODE_MAX + 1,
} TMR_SR_PowerMode;

/**
Operation Options for cmdSetProtocolLicenseKey
*/
typedef enum TMR_SR_SetProtocolLicenseOption
{
/**  Set Valid License Key */
TMR_SR_SET_LICENSE_KEY = 0x01,     
/**  Erase License Key */
TMR_SR_ERASE_LICENSE_KEY = 0x02, 
}TMR_SR_SetProtocolLicenseOption;

/** 
Operation Options for cmdSetUserProfile 
*/
typedef enum TMR_SR_UserConfigOperation
{          
/**  Save operation */
TMR_USERCONFIG_SAVE = 0x01,     
/**  Restore operation */
TMR_USERCONFIG_RESTORE = 0x02, 
/**  Verify operation */
TMR_USERCONFIG_VERIFY = 0x03,  
/** Clear operation  */
TMR_USERCONFIG_CLEAR = 0x04,           
}TMR_SR_UserConfigOperation; 

/**
Congfiguration category for cmdSetUserProfile
*/
typedef enum TMR_SR_UserConfigCategory
{
  /** All Configuration */
TMR_SR_ALL=0x01,   
}TMR_SR_UserConfigCategory;

/** 
The configuration type for cmdSetUserProfile
*/
typedef enum TMR_SR_UserConfigType
{
/** Firmware default configurations */
TMR_SR_FIRMWARE_DEFAULT=0x00,
/** Custom configurations */
TMR_SR_CUSTOM_CONFIGURATION=0x01,      
}TMR_SR_UserConfigType;

/**
 * The user profile operation structure
 */
typedef struct TMR_SR_UserConfigOp
{
  /** @privatesection */

  /** User config category */
  TMR_SR_UserConfigCategory category;
  /** User config operation */
  TMR_SR_UserConfigOperation op;
}TMR_SR_UserConfigOp;

/**
 * Defines the values for the parameter /reader/userMode and the
 * return value and parameter to TMR_SR_cmdGetUserMode() and
 * TMR_SR_cmdSetUserMode.
 */
typedef enum TMR_SR_UserMode
{
  TMR_SR_USER_MODE_MIN      = 0,
  TMR_SR_USER_MODE_UNSPEC   = 0,
  TMR_SR_USER_MODE_PRINTER  = 1,
  TMR_SR_USER_MODE_CONVEYOR = 2,
  TMR_SR_USER_MODE_PORTAL   = 3,
  TMR_SR_USER_MODE_HANDHELD = 4,
  TMR_SR_USER_MODE_MAX      = TMR_SR_USER_MODE_HANDHELD,
  TMR_SR_USER_MODE_INVALID  = TMR_SR_USER_MODE_MAX + 1,
}TMR_SR_UserMode;

/**
 * This represents the types of Q algorithms avaliable on the reader.
 */
typedef enum TMR_SR_GEN2_QType
{
  TMR_SR_GEN2_Q_MIN     = 0,
  TMR_SR_GEN2_Q_DYNAMIC = 0,
  TMR_SR_GEN2_Q_STATIC  = 1,
  TMR_SR_GEN2_Q_MAX     = TMR_SR_GEN2_Q_STATIC,
  TMR_SR_GEN2_Q_INVALID = TMR_SR_GEN2_Q_MAX + 1,
} TMR_SR_GEN2_QType;

/** Configuration of the static-Q algorithm. */
typedef struct TMR_SR_GEN2_QStatic
{
  /** The initial Q value to use. */
  uint8_t initialQ;
} TMR_SR_GEN2_QStatic;

/**
 * This represents a Q algorithm on the reader.
 */
typedef struct TMR_SR_GEN2_Q
{
  /** The type of Q algorithm. */
  TMR_SR_GEN2_QType type;
  union
  {
    /** Configuration of the static-Q algorithm. */
    TMR_SR_GEN2_QStatic staticQ;
  }u;
} TMR_SR_GEN2_Q;


/** An antenna port with an associated uint16_t value. */
typedef struct TMR_PortValue
{
  /** The port number */
  uint8_t port;
  /** The value */
  uint16_t value;
} TMR_PortValue;

/** List of TMR_PortValue values */
typedef struct TMR_PortValueList
{
  /** The array of values */
  TMR_PortValue *list;
  /** The number of entries there is space for in the array */
  uint8_t max;
  /** The number of entries in the list - may be larger than max, indicating truncated data. */
  uint8_t len;
} TMR_PortValueList;


/**
 * Mapping between an arbitrary "antenna" number and a TX/RX port
 * pair. Not all TX/RX pairings are valid for a device.
 */
typedef struct TMR_AntennaMap
{
  /** The antenna number - an arbitrary value. */
  uint8_t antenna;
  /** The device antenna port to use for transmission. */
  uint8_t txPort;
  /** The device antenna port to use for reception. */
  uint8_t rxPort;
} TMR_AntennaMap;

/** List of antenna mappings */
typedef struct TMR_AntennaMapList
{
  /** The array of values */
  TMR_AntennaMap *list;
  /** The number of entries there is space for in the array */
  uint8_t max;
  /** The number of entries in the list - may be larger than max, indicating truncated data. */
  uint8_t len;
} TMR_AntennaMapList;

/**
 * The version structure returned from cmdVersion().
 */
typedef struct TMR_SR_VersionInfo
{
  /** Bootloader version, as four 8-bit numbers */
  uint8_t bootloader[4];
  /** Hardware version. Opaque format */
  uint8_t hardware[4];
  /** Date app firmware was built, as BCD YYYYMMDD */
  uint8_t fwDate[4];
  /** App firmware version, as four 8-bit numbers */
  uint8_t fwVersion[4];
  /** Bitmask of the protocols supported by the device (indexed by TMR_TagProtocol values minus one).  */
  uint32_t protocols;
} TMR_SR_VersionInfo;

/**
 * The serial reader structure.
 */
typedef struct TM_SR_SerialReader
{
  /** @privatesection */

  /* Serial transport information */
  TMR_SR_SerialTransport transport;
  union
  {
#ifdef TMR_ENABLE_SERIAL_TRANSPORT_NATIVE
    TMR_SR_SerialPortNativeContext nativeContext;
#endif
#ifdef TMR_ENABLE_SERIAL_TRANSPORT_LLRP
    TMR_SR_LlrpEapiTransportContext llrpContext;
#endif
  } transportContext;

  /* User-configurable values */
  uint32_t baudRate;
  TMR_AntennaMapList *txRxMap;
  TMR_GEN2_Password gen2AccessPassword;
  uint32_t transportTimeout;
  uint32_t commandTimeout;
  TMR_Region regionId;

  /* Static storage for the default map */
  TMR_AntennaMap staticTxRxMapData[TMR_SR_MAX_ANTENNA_PORTS];
  TMR_AntennaMapList staticTxRxMap;

  /* Mostly-fixed information about the connected reader */
  TMR_SR_VersionInfo versionInfo;
  uint32_t portMask;
  bool useStreaming;

  /* Option to enable or disable the pre-amble */
  bool supportsPreAmble;

  /* Cache extendedEPC setting */
  bool extendedEPC;

  /* Cached values */
  TMR_SR_PowerMode powerMode;
  TMR_TagProtocol currentProtocol;
  int8_t gpioDirections;

#define TMR_PARAMWORDS ((1 + TMR_PARAM_MAX + 31) / 32)
  /* Large bitmask that stores whether each parameter's presence
   * is known or not.
   */
  uint32_t paramConfirmed[TMR_PARAMWORDS];
  /* Large bitmask that, if the corresponding bit in paramConfirmed is set,
   * stores whether each parameter is present or not.
   */
  uint32_t paramPresent[TMR_PARAMWORDS];

  /* Temporary storage during a read and subsequent fetch of tags */
  uint32_t readTimeLow, readTimeHigh;
  uint32_t searchTimeoutMs;
  
  /* Number of tags reported by module read command.
   * In streaming mode, exact quantity is unknown, so use
   * 0 if stream has ended, non-zero if end-of-stream has
   * not yet been detected. */
  int tagsRemaining;
  /* Buffer tag records fetched from module but not yet passed to caller. */
  uint8_t bufResponse[TMR_SR_MAX_PACKET_SIZE];
  /* bufResopnse read index */
  uint8_t bufPointer;
  /* Number of tag records in buffer but not yet passed to caller */
  uint8_t tagsRemainingInBuffer;
  /* Statistics response indicator */
  bool isStatusResponse;
  /*TMR opCode*/
  uint8_t opCode;
  /*Gen2 Q Value from previous tagop */
  TMR_SR_GEN2_Q oldQ;
  /*Gen2 WriteMode*/
  TMR_GEN2_WriteMode writeMode;
  /* Buffer to store additional error message */
  char errMsg[TMR_SR_MAX_PACKET_SIZE];
  /* Product group id */
  uint16_t productId;
  /* Tag operation success count */
  uint16_t tagopSuccessCount;
  /* Tag operation failure count */
  uint16_t tagopFailureCount;
  /* Enable read filtering */
  bool enableReadFiltering;
  /* Read filter timeout */
  uint32_t readFilterTimeout;
} TMR_SR_SerialReader;

/**
 * This object is returned from TMR_SR_cmdGetReaderStatistics.
 */
typedef struct TMR_SR_ReaderStatistics
{
  /** The number of entries in the arrays. */
  uint8_t numPorts;
  /** per-port RF on time, in milliseconds. */
  uint32_t rfOnTime[TMR_SR_MAX_ANTENNA_PORTS];
  /** The per-port noise floor with transmitter off */
  uint32_t noiseFloor[TMR_SR_MAX_ANTENNA_PORTS];
  /** The per-port noise floor while transmitting. */
  uint32_t noiseFloorTxOn[TMR_SR_MAX_ANTENNA_PORTS];
}TMR_SR_ReaderStatistics;

/**
 *  Object of this contains antenna status report
 */
typedef struct TMR_SR_AntennaStatusReport
{
  /** Antenna */
  uint8_t ant;
}TMR_SR_AntennaStatusReport;

/**
 * Object of this contains frequency status report
 */
typedef struct TMR_SR_FrequencyStatusReport
{
  /** RF carrier frequency */
  uint32_t freq;
}TMR_SR_FrequencyStatusReport;

/**
 * Object of this contains temperature status report.
 */
typedef struct TMR_SR_TemperatureStatusReport
{
  /** Temperature */
  int8_t temp;
}TMR_SR_TemperatureStatusReport;

/**
 *  Status report content flags
 */
typedef enum TMR_SR_StatusType
{
  /* None */
  TMR_SR_STATUS_NONE = 0x0000,
  /* Frequency */
  TMR_SR_STATUS_FREQUENCY = 0x0002,
  /* Temperature */
  TMR_SR_STATUS_TEMPERATURE = 0x0004,
  /* Current Antenna Ports */
  TMR_SR_STATUS_ANTENNA = 0x0008,
  /* All */
  TMR_SR_STATUS_ALL = 0x000E,
  TMR_SR_STATUS_MAX = 4,
}TMR_SR_StatusType;

/**
 * This object contains the information related to status reports
 * sent by the module during continuous reading
 */
typedef struct TMR_SR_StatusReport
{
  TMR_SR_StatusType type;
  union
  {
    /** Antenna status report */
    TMR_SR_AntennaStatusReport asr;
    /** Frequency status report */
    TMR_SR_FrequencyStatusReport fsr;
    /** Temperature status report */
    TMR_SR_TemperatureStatusReport tsr;
  } u;
}TMR_SR_StatusReport;


TMR_Status TMR_SR_connect(struct TMR_Reader *reader);
TMR_Status TMR_SR_destroy(struct TMR_Reader *reader);
TMR_Status TMR_SR_read(struct TMR_Reader *reader, uint32_t timeoutMs, int32_t *tagCount);
TMR_Status TMR_SR_hasMoreTags(struct TMR_Reader *reader);
TMR_Status TMR_SR_getNextTag(struct TMR_Reader *reader, TMR_TagReadData *read);
TMR_Status TMR_SR_executeTagOp(struct TMR_Reader *reader, TMR_TagOp *tagop, TMR_TagFilter *filter, TMR_uint8List *data);
TMR_Status TMR_SR_writeTag(struct TMR_Reader *reader, const TMR_TagFilter *filter, const TMR_TagData *data);
TMR_Status TMR_SR_killTag(struct TMR_Reader *reader, const TMR_TagFilter *filter, const TMR_TagAuthentication *auth);
TMR_Status TMR_SR_lockTag(struct TMR_Reader *reader, const TMR_TagFilter *filter, TMR_TagLockAction *action);
TMR_Status TMR_SR_readTagMemBytes(struct TMR_Reader *reader, const TMR_TagFilter *filter,
                              uint32_t bank, uint32_t address,
                              uint16_t count, uint8_t data[]);
TMR_Status TMR_SR_readTagMemWords(struct TMR_Reader *reader, const TMR_TagFilter *filter,
                              uint32_t bank, uint32_t address,
                              uint16_t count, uint16_t *data);
TMR_Status TMR_SR_writeTagMemBytes(struct TMR_Reader *reader, const TMR_TagFilter *filter,
                               uint32_t bank, uint32_t address,
                               uint16_t count, const uint8_t data[]);
TMR_Status TMR_SR_writeTagMemWords(struct TMR_Reader *reader, const TMR_TagFilter *filter,
                               uint32_t bank, uint32_t address,
                               uint16_t count, const uint16_t data[]);
TMR_Status TMR_SR_gpoSet(struct TMR_Reader *reader, uint8_t count, const TMR_GpioPin state[]);
TMR_Status TMR_SR_gpiGet(struct TMR_Reader *reader, uint8_t *count, TMR_GpioPin state[]);
TMR_Status TMR_SR_firmwareLoad(TMR_Reader *reader, void *cookie,
                               TMR_FirmwareDataProvider provider);
TMR_Status TMR_init_UserConfigOp(TMR_SR_UserConfigOp *config, TMR_SR_UserConfigOperation op);

/**
 * Initialize a serial reader. The reader->u.serialReader.transport
 * structure must be initialized before calling this.
 */
TMR_Status TMR_SR_SerialReader_init(TMR_Reader *reader);

#ifdef __cplusplus
}
#endif

#endif /* _TMR_SERIAL_READER_H */
