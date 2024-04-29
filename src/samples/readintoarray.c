/**
 * Sample program using C convenience function to create an array of tag reads
 * and print the tags found.
 * @file readintoarray.c
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

  ret = TMR_connect(rp);
  checkerr(rp, ret, 1, "connecting reader");

  region = TMR_REGION_NA;
/*  ret = TMR_paramSet(rp, TMR_paramID("/reader/region/id"), &region); */
  ret = TMR_paramSet(rp, TMR_PARAM_REGION_ID, &region);
  checkerr(rp, ret, 1, "setting region");
  
  {
    /* Simulate some extra antennas by enabling multiplexer */
    uint8_t list[] = {1,2};
    TMR_uint8List value;
    value.list = list;
    value.max = value.len = sizeof(list)/sizeof(uint8_t);
    ret = TMR_paramSet(rp, TMR_PARAM_ANTENNA_PORTSWITCHGPOS, &value);
    checkerr(rp, ret, 1, "setting portSwitchGpos");
    }
  {
    /* false -- Each antenna gets a separate record
     * true -- All antennas share a single record */
    bool value = true;
    ret = TMR_paramSet(rp, TMR_PARAM_TAGREADDATA_UNIQUEBYANTENNA,&value);
    checkerr(rp, ret, 1, "setting uniqueByAntenna");
    }
  {
    int32_t tagCount;
    TMR_TagReadData* tagReads;
    int i;

    ret = TMR_readIntoArray(rp, 500, &tagCount, &tagReads);
    checkerr(rp, ret, 1, "reading tags");

    printf("%d tags found.\n", tagCount);
    for (i=0; i<tagCount; i++)
    {
      TMR_TagReadData* trd = &tagReads[i];
      char epcStr[128];
      TMR_bytesToHex(trd->tag.epc, trd->tag.epcByteCount, epcStr);
      printf("%s", epcStr);
      printf(" ant:%d", trd->antenna);
      printf(" count:%d", trd->readCount);
      printf("\n");
    }
  }

  TMR_destroy(rp);
  return 0;
}
