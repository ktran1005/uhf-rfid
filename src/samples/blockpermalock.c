/**
* Sample program that perform BlockPermaLock
* @file BlockPermaLock.c
*/

#include <tm_reader.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include "serial_reader_imp.h"

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

void serialPrinter(bool tx,uint32_t dataLen, const uint8_t data[],uint32_t timeout, void *cookie)
{
  FILE *out = cookie;
  uint32_t i;

  fprintf(out, "%s", tx ? "Sending: " : "Received:");
  for (i = 0; i < dataLen; i++)
  {
    if (i > 0 && 
      (i & 15) == 0)
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
  ret = TMR_paramSet(rp, TMR_PARAM_REGION_ID, &region);
  checkerr(rp, ret, 1, "setting region");

  {
    TMR_TagOp tagop;
    TMR_uint16List mask;
    uint16_t maskData[] = { 0xFFFF };
    TMR_uint8List response;
    uint8_t responseData[16];

    mask.list = maskData;
    mask.max = mask.len = sizeof(maskData) / sizeof(maskData[0]);
    response.list = responseData;
    response.max = sizeof(responseData) / sizeof(responseData[0]);
    response.len = 0;

    ret = TMR_TagOp_init_GEN2_BlockPermaLock(&tagop, 0, TMR_GEN2_BANK_USER, 0, &mask);
    checkerr(rp, ret, 1, "creating BlockPermalock tagop");

    ret = TMR_executeTagOp(rp, &tagop, NULL, &response);
    checkerr(rp, ret, 1, "executing BlockPermalock tagop");

    printf("BlockPermalock succeeded\n");
    {
      int i;
      printf("PermaLock Bits:");
      for (i=0; i<response.len; i++)
      {
        printf(" %02x", response.list[i]);
      }
      printf("\n");
    }
  }

  TMR_destroy(rp);
  return 0;
}
