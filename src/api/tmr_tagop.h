#ifndef _TMR_TAGOP_H
#define _TMR_TAGOP_H
/** 
 *  @file tmr_tagop.h
 *  @brief Mercury API - Tag Operations Interface
 *  @author Nathan Williams
 *  @date 1/8/2010
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

/** The type of a tag operation structure */
typedef enum TMR_TagOpType
{
  /** Gen2 EPC write */
  TMR_TAGOP_GEN2_WRITETAG,
  /** Gen2 memory read */
  TMR_TAGOP_GEN2_READDATA,
  /** Gen2 memory write */
  TMR_TAGOP_GEN2_WRITEDATA,
  /** Gen2 memory lock/unlock */
  TMR_TAGOP_GEN2_LOCK,
  /** Gen2 tag kill */
  TMR_TAGOP_GEN2_KILL,
  /** Gen2 tag block write */
  TMR_TAGOP_GEN2_BLOCKWRITE,
  /** Gen2 tag block permalock */
  TMR_TAGOP_GEN2_BLOCKPERMALOCK,
  /** Higgs2 Partial Load Image */
  TMR_TAGOP_GEN2_ALIEN_HIGGS2_PARTIALLOADIMAGE,
  /** Higgs2 Full Load Image */
  TMR_TAGOP_GEN2_ALIEN_HIGGS2_FULLLOADIMAGE,
  /** Higgs3 Fast Load Image */
  TMR_TAGOP_GEN2_ALIEN_HIGGS3_FASTLOADIMAGE,
  /** Higgs3 Load Image */
  TMR_TAGOP_GEN2_ALIEN_HIGGS3_LOADIMAGE,
  /** Higgs3 Block Read Lock */
  TMR_TAGOP_GEN2_ALIEN_HIGGS3_BLOCKREADLOCK,

  /** NXP set read protect */
  TMR_TAGOP_GEN2_NXP_SETREADPROTECT,
  /** NXP reset read protect */
  TMR_TAGOP_GEN2_NXP_RESETREADPROTECT,
  /** NXP Change EAS */
  TMR_TAGOP_GEN2_NXP_CHANGEEAS,
  /** NXP EAS Alarm */
  TMR_TAGOP_GEN2_NXP_EASALARM,
  /** NXP Calibrate */
  TMR_TAGOP_GEN2_NXP_CALIBRATE,
  /** NXP ChangeConfig */
  TMR_TAGOP_GEN2_NXP_CHANGECONFIG,

  /** Monza4 QT Read/Write*/
  TMR_TAGOP_GEN2_IMPINJ_MONZA4_QTREADWRITE,

  /** ISO180006B memory read */
  TMR_TAGOP_ISO180006B_READDATA,
  /** ISO180006B memory write */
  TMR_TAGOP_ISO180006B_WRITEDATA,
  /** ISO180006B memory lock/unlock */
  TMR_TAGOP_ISO180006B_LOCK,
  /** ISO180006B tag kill */

  /** List of tag operations */
  TMR_TAGOP_LIST
} TMR_TagOpType;

/** Parameters of a Gen2 EPC write operation */
typedef struct TMR_TagOp_GEN2_WriteTag
{
  /** Tag EPC */
  TMR_TagData* epcptr;
} TMR_TagOp_GEN2_WriteTag;

/** Parameters of a Gen2 memory read operation */
typedef struct TMR_TagOp_GEN2_ReadData
{
  /** Gen2 memory bank to read from */
  TMR_GEN2_Bank bank;
  /** Word address to start reading at */
  uint32_t wordAddress;
  /** Number of words to read */
  uint8_t len;
} TMR_TagOp_GEN2_ReadData;

/** Parameters of a Gen2 memory write operation */
typedef struct TMR_TagOp_GEN2_WriteData
{
  /** Gen2 memory bank to write to */
  TMR_GEN2_Bank bank;
  /** Word address to start writing at */
  uint32_t wordAddress;
  /** Data to write */
  TMR_uint16List data;
} TMR_TagOp_GEN2_WriteData;

/** Parameters of a Gen2 memory lock/unlock operation */
typedef struct TMR_TagOp_GEN2_Lock
{ 
  /** Bitmask indicating which lock bits to change */
  uint16_t mask;
  /** New values of each bit specified in the mask */
  uint16_t action;
  /** Access Password to use to lock the tag*/
  TMR_GEN2_Password accessPassword;
} TMR_TagOp_GEN2_Lock;

/** Parameters of a Gen2 tag kill operation */
typedef struct TMR_TagOp_GEN2_Kill
{
  /** Kill password to use to kill the tag */
  TMR_GEN2_Password password;
} TMR_TagOp_GEN2_Kill;

/** Parameters of a Gen2 tag Block Write operation */
typedef struct TMR_TagOp_GEN2_BlockWrite
{
  /** Gen2 memory bank to write to */
  TMR_GEN2_Bank bank; 
  /** The word address to start writing to */
  uint32_t wordPtr; 
  /** The data to write */
  TMR_uint16List data; 
}TMR_TagOp_GEN2_BlockWrite;

/** Parameters of a Gen2 tag Block PermaLock operation */
typedef struct TMR_TagOp_GEN2_BlockPermaLock
{
  /** Read lock status or write it? */
  uint8_t readLock; 
  /** Gen2 memory bank to lock */
  TMR_GEN2_Bank bank; 
  /** The starting word address to lock */
  uint32_t blockPtr;
  /** Mask: Which blocks to lock? */
  TMR_uint16List mask;
}TMR_TagOp_GEN2_BlockPermaLock;

/** Parameters of a Gen2 memory lock/unlock operation */
typedef struct TMR_TagOp_ISO180006B_Lock
{ 
  /** The memory address of the byte to lock */
  uint8_t address;
} TMR_TagOp_ISO180006B_Lock;

/** Parameters of an ISO180006B memory read operation */
typedef struct TMR_TagOp_ISO180006B_ReadData
{
  /** Byte address to start reading at */
  uint8_t byteAddress;
  /** Number of bytes to read */
  uint8_t len;
} TMR_TagOp_ISO180006B_ReadData;

/** Parameters of an ISO180006B memory write operation */
typedef struct TMR_TagOp_ISO180006B_WriteData
{
  /** Byte address to start writing at */
  uint8_t byteAddress;
  /** Data to write */
  TMR_uint8List data;
} TMR_TagOp_ISO180006B_WriteData;

/**
 *  Tagops for Gen2 custom commands
 **/

/** Parameters for Alien Higgs2, Partial Load Image command*/
typedef struct TMR_TagOp_GEN2_Alien_Higgs2_PartialLoadImage
{
  /** Kill password to write to the tag */
  TMR_GEN2_Password killPassword;
  /** Access password to write to the tag */
  TMR_GEN2_Password accessPassword;
  /** Tag EPC to write to the tag*/
  TMR_TagData* epcptr;
} TMR_TagOp_GEN2_Alien_Higgs2_PartialLoadImage;

/** Parameters for Alien Higgs2, Full Load Image command*/
typedef struct TMR_TagOp_GEN2_Alien_Higgs2_FullLoadImage
{
  /** Kill password to write to the tag */
  TMR_GEN2_Password killPassword;
  /** Access password to write to the tag */
  TMR_GEN2_Password accessPassword;
  /** Lock Bits */
  uint16_t lockBits;
  /** PC word */
  uint16_t pcWord;
  /** Tag EPC to write to the tag*/
  TMR_TagData* epcptr;
} TMR_TagOp_GEN2_Alien_Higgs2_FullLoadImage;

/** Parameters for Alien Higgs3, Fast Load Image command*/
typedef struct TMR_TagOp_GEN2_Alien_Higgs3_FastLoadImage
{
  /** Access password used to write to the tag */
  TMR_GEN2_Password currentAccessPassword;
  /** Kill password to write to the tag */
  TMR_GEN2_Password killPassword;
  /** Access password to write to the tag */
  TMR_GEN2_Password accessPassword;
  /** PC word */
  uint16_t pcWord;
  /** Tag EPC to write to the tag*/
  TMR_TagData* epcptr;
} TMR_TagOp_GEN2_Alien_Higgs3_FastLoadImage;


/** Parameters for Alien Higgs3, Load Image command*/
typedef struct TMR_TagOp_GEN2_Alien_Higgs3_LoadImage
{
  /** Access password used to write to the tag */
  TMR_GEN2_Password currentAccessPassword;
  /** Kill password to write to the tag */
  TMR_GEN2_Password killPassword;
  /** Access password to write to the tag */
  TMR_GEN2_Password accessPassword;
  /** PC word */
  uint16_t pcWord;
  /** Tag EPC and user data to write to the tag (76 bytes)*/
  TMR_uint8List *epcAndUserData;
} TMR_TagOp_GEN2_Alien_Higgs3_LoadImage;

/** Parameters for Alien Higgs3, Block Read Lock command*/
typedef struct TMR_TagOp_GEN2_Alien_Higgs3_BlockReadLock
{
  /** Access password to use to write to the tag, in case if the tag is already locked */
  TMR_GEN2_Password accessPassword;
  /** A bitmask of bits to lock */
  uint8_t lockBits;
} TMR_TagOp_GEN2_Alien_Higgs3_BlockReadLock;

/** Parameters for NXP, Set Read Protect command */
typedef struct TMR_TagOp_GEN2_NXP_SetReadProtect
{
  /** Access password to use to write to the tag */
  TMR_GEN2_Password accessPassword;
}TMR_TagOp_GEN2_NXP_SetReadProtect;

/** Parameters for NXP, Reset Read Protect Command*/
typedef struct TMR_TagOp_GEN2_NXP_ResetReadProtect
{
  /** Access password to use to write to the tag */
  TMR_GEN2_Password accessPassword;
} TMR_TagOp_GEN2_NXP_ResetReadProtect;

/** Parameters for NXP, Change EAS Command*/
typedef struct TMR_TagOp_GEN2_NXP_ChangeEAS
{
  /** Access password to use to write to the tag */
  TMR_GEN2_Password accessPassword;
  /** Reset or set EAS bit */
  bool reset;
} TMR_TagOp_GEN2_NXP_ChangeEAS;

/** Parameters for NXP, EAS alarm command */
typedef struct TMR_TagOp_GEN2_NXP_EASAlarm
{
  /** Gen2 divide ratio to use */
  TMR_GEN2_DivideRatio dr;
  /** Gen2 M parameter to use */
  TMR_GEN2_TagEncoding m;
  /** Gen2 TrExt value to use */
  TMR_GEN2_TrExt trExt;  
} TMR_TagOp_GEN2_NXP_EASAlarm;

/** Parameters for NXP, Calibration command */
typedef struct TMR_TagOp_GEN2_NXP_Calibrate
{
  /** Access password to use to write to the tag */
  TMR_GEN2_Password accessPassword;
} TMR_TagOp_GEN2_NXP_Calibrate;

/** Parameters for NXP, Change Config command */
typedef struct TMR_TagOp_GEN2_NXP_ChangeConfig
{
  /** Access password to use to write to the tag */
  TMR_GEN2_Password accessPassword;
  /** ConfigWord to write to the tag*/
  TMR_NXP_ConfigWord configWord;
}TMR_TagOp_GEN2_NXP_ChangeConfig;

/** Parameters for NXP, Change Config command */
typedef struct TMR_TagOp_GEN2_Impinj_Monza4_QTReadWrite
{
  /** Access password to use to write to the tag */
  TMR_GEN2_Password accessPassword;
  /** Control Byte to write to the tag*/
  TMR_Monza4_ControlByte controlByte;
  /** Payload */
  TMR_Monza4_Payload payload;    
}TMR_TagOp_GEN2_Impinj_Monza4_QTReadWrite;

/* Forward declaration for the benefit of TMR_TagOp_List */
typedef struct TMR_TagOp TMR_TagOp;

/** List of tag operations */
typedef struct TMR_TagOp_List
{
  /** Array of pointers to tag operations*/
  TMR_TagOp **list;
  /** Number of tag operations in list */
  uint16_t len;
} TMR_TagOp_List;

/** Sub-class for Gen2 Alien Higgs2 custom tag extensions */
typedef struct TMR_TagOp_GEN2_Alien_Higgs2
{
  union
  {
    TMR_TagOp_GEN2_Alien_Higgs2_PartialLoadImage partialLoadImage;
    TMR_TagOp_GEN2_Alien_Higgs2_FullLoadImage fullLoadImage;
  } u;
}TMR_TagOp_GEN2_Alien_Higgs2;

/** Sub-class for Gen2 Alien Higgs3 custom tag extensions */
typedef struct TMR_TagOp_GEN2_Alien_Higgs3
{
  union
  {
    TMR_TagOp_GEN2_Alien_Higgs3_FastLoadImage fastLoadImage;
    TMR_TagOp_GEN2_Alien_Higgs3_LoadImage loadImage;
    TMR_TagOp_GEN2_Alien_Higgs3_BlockReadLock blockReadLock;
  } u;
}TMR_TagOp_GEN2_Alien_Higgs3;

/** Sub-class for Gen2 Alien custom tag extensions */
typedef struct TMR_TagOp_GEN2_Alien
{
  union
  {
    TMR_TagOp_GEN2_Alien_Higgs2 higgs2;
    TMR_TagOp_GEN2_Alien_Higgs3 higgs3;
  } u;
}TMR_TagOp_GEN2_Alien;

/** Sub-class for Gen2 NXP custom tag extensions */
typedef struct TMR_TagOp_GEN2_NXP
{
  union
  {
    TMR_TagOp_GEN2_NXP_SetReadProtect setReadProtect;
    TMR_TagOp_GEN2_NXP_ResetReadProtect resetReadProtect;
    TMR_TagOp_GEN2_NXP_ChangeEAS changeEAS;
    TMR_TagOp_GEN2_NXP_EASAlarm EASAlarm;
    TMR_TagOp_GEN2_NXP_Calibrate calibrate;
    TMR_TagOp_GEN2_NXP_ChangeConfig changeConfig;
  }u;
}TMR_TagOp_GEN2_NXP;

/** Sub-class for Gen2 Impinj Monza4 custom tag extensions */
typedef struct TMR_TagOp_GEN2_Impinj_Monza4
{
  union
  {
    TMR_TagOp_GEN2_Impinj_Monza4_QTReadWrite qtReadWrite;
  } u;
}TMR_TagOp_GEN2_Impinj_Monza4;

/** Sub-class for Gen2 Impinj custom tag extensions */
typedef struct TMR_TagOp_GEN2_Impinj
{
  union
  {
    TMR_TagOp_GEN2_Impinj_Monza4 monza4;
  } u;
}TMR_TagOp_GEN2_Impinj;

/** Sub-class for Gen2 custom tagops */
typedef struct TMR_TagOp_GEN2_Custom
{
  TMR_SR_GEN2_SiliconType chipType;
  union
  {
    TMR_TagOp_GEN2_Alien alien;
    TMR_TagOp_GEN2_NXP nxp;
    TMR_TagOp_GEN2_Impinj impinj;
  } u;
}TMR_TagOp_GEN2_Custom;

/** Sub-class for ISO180006B tagops */
typedef struct TMR_TagOp_ISO180006B
{
  union
  {
    TMR_TagOp_ISO180006B_Lock lock;
    TMR_TagOp_ISO180006B_WriteData writeData;
    TMR_TagOp_ISO180006B_ReadData readData;
  } u;
}TMR_TagOp_ISO180006B;

/** Sub-class for Gen2 standard tagops */
typedef struct TMR_TagOp_GEN2
{
  union
  {
    TMR_TagOp_GEN2_WriteTag writeTag;
    TMR_TagOp_GEN2_WriteData writeData;
    TMR_TagOp_GEN2_ReadData readData;
    TMR_TagOp_GEN2_Lock lock;
    TMR_TagOp_GEN2_Kill kill;
    TMR_TagOp_GEN2_BlockWrite blockWrite;
    TMR_TagOp_GEN2_BlockPermaLock blockPermaLock;
    TMR_TagOp_GEN2_Custom custom;
  } u;
}TMR_TagOp_GEN2;

/** Tag operation data structure */
struct TMR_TagOp
{
  TMR_TagOpType type;
  union
  {
    TMR_TagOp_GEN2 gen2;
    TMR_TagOp_ISO180006B iso180006b;
    TMR_TagOp_List list;
  } u;
};

TMR_Status TMR_TagOp_init_GEN2_WriteTag(TMR_TagOp *tagop, TMR_TagData* epc);
TMR_Status TMR_TagOp_init_GEN2_ReadData(TMR_TagOp *tagop, TMR_GEN2_Bank bank,
                                        uint32_t wordAddress, uint8_t len);
TMR_Status TMR_TagOp_init_GEN2_WriteData(TMR_TagOp *tagop, TMR_GEN2_Bank bank,
                                         uint32_t wordAddress,
                                         TMR_uint16List *data);
TMR_Status TMR_TagOp_init_GEN2_Lock(TMR_TagOp *tagop, uint16_t mask,
                                    uint16_t action,  TMR_GEN2_Password accessPassword);
TMR_Status TMR_TagOp_init_GEN2_Kill(TMR_TagOp *tagop,
                                    TMR_GEN2_Password killPassword);
TMR_Status TMR_TagOp_init_GEN2_BlockWrite(TMR_TagOp *tagop, TMR_GEN2_Bank bank, uint32_t wordPtr, TMR_uint16List *data);
TMR_Status TMR_TagOp_init_GEN2_BlockPermaLock(TMR_TagOp *tagop, uint8_t readLock, TMR_GEN2_Bank bank, uint32_t blockPtr, TMR_uint16List* mask);

TMR_Status TMR_TagOp_init_ISO180006B_ReadData(TMR_TagOp *tagop, uint8_t byteAddress, uint8_t len);
TMR_Status TMR_TagOp_init_ISO180006B_WriteData(TMR_TagOp *tagop, uint8_t byteAddress, TMR_uint8List *data);
TMR_Status TMR_TagOp_init_ISO180006B_Lock(TMR_TagOp *tagop, uint8_t address);
TMR_Status TMR_TagOp_init_GEN2_Alien_Higgs2_PartialLoadImage(TMR_TagOp *tagop, TMR_GEN2_Password killPassword,
                                                  TMR_GEN2_Password accessPassword, TMR_TagData *epc);
TMR_Status TMR_TagOp_init_GEN2_Alien_Higgs2_FullLoadImage(TMR_TagOp *tagop, TMR_GEN2_Password killPassword,
                                    TMR_GEN2_Password accessPassword, uint16_t lockBits, 
                                    uint16_t pcWord, TMR_TagData *epc);
TMR_Status TMR_TagOp_init_GEN2_Alien_Higgs3_FastLoadImage(TMR_TagOp *tagop, TMR_GEN2_Password currentAccessPassword,
                                    TMR_GEN2_Password accessPassword, TMR_GEN2_Password killPassword, 
                                    uint16_t pcWord, TMR_TagData *epc);
TMR_Status TMR_TagOp_init_GEN2_Alien_Higgs3_LoadImage(TMR_TagOp *tagop, TMR_GEN2_Password currentAccessPassword,
                                    TMR_GEN2_Password accessPassword, TMR_GEN2_Password killPassword, 
                                    uint16_t pcWord, TMR_uint8List *epcAndUserData);
TMR_Status TMR_TagOp_init_GEN2_Alien_Higgs3_BlockReadLock(TMR_TagOp *tagop, TMR_GEN2_Password accessPassword, uint8_t lockBits);

TMR_Status TMR_TagOp_init_GEN2_NXP_G2I_SetReadProtect(TMR_TagOp *tagop, TMR_GEN2_Password accessPassword);
TMR_Status TMR_TagOp_init_GEN2_NXP_G2X_SetReadProtect(TMR_TagOp *tagop, TMR_GEN2_Password accessPassword);

TMR_Status TMR_TagOp_init_GEN2_NXP_G2I_ResetReadProtect(TMR_TagOp *tagop, TMR_GEN2_Password accessPassword);
TMR_Status TMR_TagOp_init_GEN2_NXP_G2X_ResetReadProtect(TMR_TagOp *tagop, TMR_GEN2_Password accessPassword);

TMR_Status TMR_TagOp_init_GEN2_NXP_G2I_ChangeEAS(TMR_TagOp *tagop, TMR_GEN2_Password accessPassword, bool resetEAS);
TMR_Status TMR_TagOp_init_GEN2_NXP_G2X_ChangeEAS(TMR_TagOp *tagop, TMR_GEN2_Password accessPassword, bool resetEAS);

TMR_Status TMR_TagOp_init_GEN2_NXP_G2I_EASAlarm(TMR_TagOp *tagop, TMR_GEN2_DivideRatio dr, TMR_GEN2_TagEncoding m, TMR_GEN2_TrExt trExt);
TMR_Status TMR_TagOp_init_GEN2_NXP_G2X_EASAlarm(TMR_TagOp *tagop, TMR_GEN2_DivideRatio dr, TMR_GEN2_TagEncoding m, TMR_GEN2_TrExt trExt);

TMR_Status TMR_TagOp_init_GEN2_NXP_G2I_Calibrate(TMR_TagOp *tagop, TMR_GEN2_Password accessPassword);
TMR_Status TMR_TagOp_init_GEN2_NXP_G2X_Calibrate(TMR_TagOp *tagop, TMR_GEN2_Password accessPassword);

TMR_Status TMR_TagOp_init_GEN2_NXP_G2I_ChangeConfig(TMR_TagOp *tagop, TMR_GEN2_Password accessPassword, TMR_NXP_ConfigWord configWord);
TMR_Status TMR_TagOp_init_GEN2_NXP_G2X_ChangeConfig(TMR_TagOp *tagop, TMR_GEN2_Password accessPassword, TMR_NXP_ConfigWord configWord);

TMR_Status TMR_TagOp_init_GEN2_Impinj_Monza4_QTReadWrite(TMR_TagOp *tagop, TMR_GEN2_Password accessPassword,
                                             TMR_Monza4_ControlByte controlByte, TMR_Monza4_Payload payload);
TMR_Status 
TMR_init_GEN2_NXP_G2I_ConfigWord(TMR_NXP_ConfigWord *configWord);

TMR_Status
TMR_init_GEN2_Impinj_Monza4_ControlByte(TMR_Monza4_ControlByte *controlByte);

TMR_Status
TMR_init_GEN2_Impinj_Monza4_Payload(TMR_Monza4_Payload *payload);

#ifdef __cplusplus
}
#endif

#endif /* _TMR_TAGOP_H */
