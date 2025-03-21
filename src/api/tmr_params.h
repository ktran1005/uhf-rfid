/* ex: set tabstop=2 shiftwidth=2 expandtab cindent: */
#ifndef _TMR_PARAMS_H
#define _TMR_PARAMS_H
/** 
 *  @file tmr_params.h
 *  @brief Mercury API - Reader parameter interface
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
 * Parameter keys for TMR_paramSet() and TMR_paramGet().  Each
 * parameter is listed with its associated string name and the
 * parameter type. A pointer to that type is passed to TMR_paramGet()
 * (for example, "TMR_String *" for /reader/version/model).
 */
typedef enum TMR_Param
{
  /** No such parameter - used as a return value from TMR_paramID().  */
  TMR_PARAM_NONE,
  TMR_PARAM_MIN,
  /** "/reader/baudRate", uint32_t */
  TMR_PARAM_BAUDRATE = TMR_PARAM_MIN,
  /** "/reader/commandTimeout", uint32_t */
  TMR_PARAM_COMMANDTIMEOUT,
  /** "/reader/transportTimeout", uint32_t */
  TMR_PARAM_TRANSPORTTIMEOUT,
  /** "/reader/powerMode", TMR_SR_PowerMode */
  TMR_PARAM_POWERMODE,
  /** "/reader/userMode", TMR_SR_UserMode */
  TMR_PARAM_USERMODE,
  /** "/reader/antenna/checkPort", bool  */
  TMR_PARAM_ANTENNA_CHECKPORT,
  /** "/reader/antenna/portList", TMR_uint8List  */
  TMR_PARAM_ANTENNA_PORTLIST,
  /** "/reader/antenna/connectedPortList", TMR_uint8List  */
  TMR_PARAM_ANTENNA_CONNECTEDPORTLIST,
  /** "/reader/antenna/portSwitchGpos", TMR_uint8List  */
  TMR_PARAM_ANTENNA_PORTSWITCHGPOS,
  /** "/reader/antenna/settlingTimeList", TMR_PortValueList  */
  TMR_PARAM_ANTENNA_SETTLINGTIMELIST,
  /** "/reader/antenna/txRxMap", TMR_AntennaMapList  */
  TMR_PARAM_ANTENNA_TXRXMAP,
  /** "/reader/gpio/inputList", TMR_uint8List */
  TMR_PARAM_GPIO_INPUTLIST,
  /** "/reader/gpio/outputList", TMR_uint8List */
  TMR_PARAM_GPIO_OUTPUTLIST,
  /** "/reader/gen2/accessPassword", TMR_GEN2_Password */
  TMR_PARAM_GEN2_ACCESSPASSWORD,
  /** "/reader/gen2/q", TMR_SR_GEN2_Q */
  TMR_PARAM_GEN2_Q,
  /** "/reader/gen2/tagEncoding", TMR_GEN2_TagEncoding */
  TMR_PARAM_GEN2_TAGENCODING,
  /** "/reader/gen2/session", TMR_GEN2_Session */
  TMR_PARAM_GEN2_SESSION,
  /** "/reader/gen2/target", TMR_GEN2_Target */
  TMR_PARAM_GEN2_TARGET,
  /** "/reader/gen2/BLF", TMR_Gen2_LinkFrequency */
  TMR_PARAM_GEN2_BLF,
  /** "/reader/gen2/tari", TMR_Gen2_Tari */
  TMR_PARAM_GEN2_TARI,
  /**"/reader/gen2/writeMode", TMR_Gen2_WriteMode*/
  TMR_PARAM_GEN2_WRITEMODE,
  /** "/reader/iso180006b/BLF", TMR_ISO180006B_LinkFrequency */
  TMR_PARAM_ISO180006B_BLF,
  /** "/reader/read/asyncOffTime", uint32_t */
  TMR_PARAM_READ_ASYNCOFFTIME,
  /** "/reader/read/asyncOnTime", uint32_t */
  TMR_PARAM_READ_ASYNCONTIME,
  /** "/reader/read/plan", TMR_ReadPlan */
  TMR_PARAM_READ_PLAN,
  /** "/reader/radio/enablePowerSave, bool **/
  TMR_PARAM_RADIO_ENABLEPOWERSAVE,
  /** "/reader/radio/powerMax", uint16_t */
  TMR_PARAM_RADIO_POWERMAX,
  /** "/reader/radio/powerMin", uint16_t */
  TMR_PARAM_RADIO_POWERMIN,
  /** "/reader/radio/portReadPowerList", TMR_PortValueList */
  TMR_PARAM_RADIO_PORTREADPOWERLIST,
  /** "/reader/radio/portWritePowerList", TMR_PortValueList */
  TMR_PARAM_RADIO_PORTWRITEPOWERLIST,
  /** "/reader/radio/readPower", uint16_t */
  TMR_PARAM_RADIO_READPOWER,
  /** "/reader/radio/writePower", uint16_t */
  TMR_PARAM_RADIO_WRITEPOWER,
  /** "/reader/radio/temperature", uint8_t */
  TMR_PARAM_RADIO_TEMPERATURE,
  /** "/reader/tagReadData/recordHighestRssi", bool */
  TMR_PARAM_TAGREADDATA_RECORDHIGHESTRSSI,
  /** "/reader/tagReadData/reportRssiInDbm", bool */
  TMR_PARAM_TAGREADDATA_REPORTRSSIINDBM,
  /** "/reader/tagReadData/uniqueByAntenna", bool */
  TMR_PARAM_TAGREADDATA_UNIQUEBYANTENNA,
  /** "/reader/tagReadData/uniqueByData", bool */
  TMR_PARAM_TAGREADDATA_UNIQUEBYDATA,
  /** "/reader/tagop/antenna", uint8_t */
  TMR_PARAM_TAGOP_ANTENNA,
  /** "/reader/tagop/protocol", TMR_Protocol */
  TMR_PARAM_TAGOP_PROTOCOL,
  /** "/reader/version/hardware", TMR_String */
  TMR_PARAM_VERSION_HARDWARE,
  /** "/reader/version/serial", TMR_String */
  TMR_PARAM_VERSION_SERIAL,
  /** "/reader/version/model", TMR_String */
  TMR_PARAM_VERSION_MODEL,
  /** "/reader/version/software", TMR_String */
  TMR_PARAM_VERSION_SOFTWARE,
  /** "/reader/version/supportedProtocols", TMR_TagProtocolList */
  TMR_PARAM_VERSION_SUPPORTEDPROTOCOLS,
  /** "/reader/region/id", TMR_Region */
  TMR_PARAM_REGION_ID,
  /** "/reader/region/supportedRegions", TMR_RegionList */
  TMR_PARAM_REGION_SUPPORTEDREGIONS,
  /** "/reader/region/hopTable", TMR_uint32List */
  TMR_PARAM_REGION_HOPTABLE,
  /** "/reader/region/hopTime", uint32_t */
  TMR_PARAM_REGION_HOPTIME,
  /** "/reader/region/lbt/enable", bool */
  TMR_PARAM_REGION_LBT_ENABLE,
  /** "/reader/licenseKey", TMR_uint8List */
  TMR_PARAM_LICENSE_KEY,
  /** "/reader/userConfig", TMR_UserConfigOption */
  TMR_PARAM_USER_CONFIG,
  /** "/reader/radio/enableSJC", bool */
  TMR_PARAM_RADIO_ENABLESJC,
  /** "/reader/extendedEpc", bool */
  TMR_PARAM_EXTENDEDEPC,
  /** "/reader/statistics", TMR_SR_ReaderStatistics */
  TMR_PARAM_READER_STATISTICS,
  /** "/reader/uri", TMR_String */
  TMR_PARAM_URI,
  /** "/reader/version/productGroupID", uint16_t */
  TMR_PARAM_PRODUCT_GROUP_ID,
  /** "/reader/version/productGroup", TMR_String */
  TMR_PARAM_PRODUCT_GROUP,
  /** "/reader/tagReadData/tagopSuccesses", uint16_t */
  TMR_PARAM_TAGREADATA_TAGOPSUCCESSCOUNT,
  /** "/reader/tagReadData/tagopFailures",  uint16_t */
  TMR_PARAM_TAGREADATA_TAGOPFAILURECOUNT,
  /** "/reader/status/antennaEnable", bool */
  TMR_PARAM_STATUS_ENABLE_ANTENNAREPORT,
  /** "/reader/status/frequencyEnable", bool */
  TMR_PARAM_STATUS_ENABLE_FREQUENCYREPORT,
  /** "/reader/status/temperatureEnable", bool */
  TMR_PARAM_STATUS_ENABLE_TEMPERATUREREPORT,
  /** "/reader/tagReadData/enableReadFilter", bool */
  TMR_PARAM_TAGREADDATA_ENABLEREADFILTER,
  /** "/reader/tagReadData/readFilterTimeout", uint16_t */
  TMR_PARAM_TAGREADDATA_READFILTERTIMEOUT,
  /** "/reader/tagReadData/uniqueByProtocol", bool */
  TMR_PARAM_TAGREADDATA_UNIQUEBYPROTOCOL,
  TMR_PARAM_END,
  TMR_PARAM_MAX = TMR_PARAM_END-1,

} TMR_Param;

#ifdef __cplusplus
}
#endif

#endif /* _TMR_PARAMS_H */
