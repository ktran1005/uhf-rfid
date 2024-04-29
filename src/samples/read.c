/**
 * Sample program that reads tags for a fixed period of time (500ms)
 * and prints the tags found.
 * @file read.c
 */

#include <tm_reader.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

void errx(int exitval, const char *fmt, ...)
{
  va_list ap;

  va_start(ap, fmt);
  vfprintf(stderr, fmt, ap);

  exit(exitval);
}

void checkerr(TMR_Reader* rp, TMR_Status ret, int exitval, const char *msg)
{
  if (TMR_SUCCESS != ret)
  {
    errx(exitval, "Error %s: %s\n", msg, TMR_strerr(rp, ret));
  }
}

void serialPrinter(bool tx, uint32_t dataLen, const uint8_t data[],
                   uint32_t timeout, void *cookie)
{
  FILE *out = cookie;
  uint32_t i;

  fprintf(out, "%s", tx ? "Sending: " : "Received:");
  for (i = 0; i < dataLen; i++)
  {
    if (i > 0 && (i & 15) == 0)
    {
      fprintf(out, "\n         ");
    }
    fprintf(out, " %02x", data[i]);
  }
  fprintf(out, "\n");
}

int main(int argc, char *argv[])
{
  TMR_Reader r, *rp;
  TMR_Status ret;
  TMR_Region region;
  TMR_TransportListenerBlock tb;
  // TMR_ReadPlan plan;
  // uint8_t antennaList[] = {0x02, 0x01, 0x01};

  if (argc < 2)
  {
    errx(1, "Please provide reader URL, such as:\n"
           "tmr:///com4\n"
           "tmr://my-reader.example.com\n");
  }
  
  rp = &r;
  ret = TMR_create(rp, argv[1]);
  checkerr(rp, ret, 1, "creating reader");

  tb.listener = serialPrinter;
  tb.cookie = stdout;
#if 0
  TMR_addTransportListener(rp, &tb);
#endif

  // uint32_t baudRate = 9600;
  // ret = TMR_paramSet(rp, TMR_PARAM_BAUDRATE, &baudRate);
  // if (TMR_SUCCESS != ret)
  // {
  //   errx(1, "Failed to set baud rate: %s\n", TMR_strerr(rp, ret));
  // }

  ret = TMR_connect(rp);
  checkerr(rp, ret, 1, "connecting reader");
  printf("connect successfully\n");

  region = 0x0D;
  //ret = TMR_paramSet(rp, TMR_paramID("/reader/region/id"), &region);
  ret = TMR_paramSet(rp, TMR_PARAM_REGION_ID, &region);
  checkerr(rp, ret, 1, "setting region");
  printf("setting region succesfully\n");
  

  TMR_Status status;
  TMR_ReadPlan plan;
  uint8_t antennaList[] = {1}; // Specify the antenna number here
  uint8_t antennaCount = sizeof(antennaList)/sizeof(antennaList[0]);

  status = TMR_RP_init_simple(&plan, antennaCount, antennaList, TMR_TAG_PROTOCOL_GEN2, 1000);
  if (status != TMR_SUCCESS) {
    printf("Error setting read plan 1: %s\n", TMR_strerr(rp, status));
    return;
  }
  
  status = TMR_paramSet(rp, TMR_PARAM_READ_PLAN, &plan);
  if (status != TMR_SUCCESS) {
    printf("Error setting read plan 2: %s\n", TMR_strerr(rp, status));
    return;
  }

  // Enabling metadata
  // ret = TMR_paramSet(rp, TMR_TRD_METADATA_FLAG_DATA, &((uint16_t)(
  //   TMR_TRD_METADATA_FLAG_READCOUNT | 
  //   TMR_TRD_METADATA_FLAG_RSSI |
  //   TMR_TRD_METADATA_FLAG_PHASE |
  //   TMR_TRD_METADATA_FLAG_FREQUENCY |
  //   TMR_TRD_METADATA_FLAG_TIMESTAMP
  // )));
  
  // if (TMR_SUCCESS != ret)
  // {
  //   errx(1, "Error setting metadata flags: %s\n", TMR_strerr(rp, ret));
  // }


  
  // Read tags
  printf("trying to read data\n");
  status = TMR_read(rp, 500, NULL); // 500ms read time
  if (status != TMR_SUCCESS)
  {
    printf("Error reading tags: %s\n", TMR_strerr(rp, ret));
    return;
  }

  TMR_TagReadData trd;
  char epcStr[128];

  TMR_TRD_init(&trd);
  TMR_getNextTag(rp, &trd);
  TMR_bytesToHex(trd.tag.epc, trd.tag.epcByteCount, epcStr);

  printf("EPC: %s, RSSI: %d dBm\n", epcStr, trd.rssi);

  

  while (TMR_hasMoreTags(rp) == TMR_SUCCESS)
  {
    // TMR_TagReadData trd;
    TMR_getNextTag(rp, &trd);
    char epcStr[128];
    ret = TMR_getNextTag(rp, &trd);
    if (TMR_SUCCESS != ret) 
    {
      errx(1, "Error getting tag: %s\n", TMR_strerr(rp, ret));
    }
    TMR_bytesToHex(trd.tag.epc, trd.tag.epcByteCount, epcStr);
    printf("EPC: %s, RSSI: %d dBm\n", epcStr, trd.rssi);
  }

  TMR_destroy(rp);
  return 0;
}
