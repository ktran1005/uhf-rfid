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

  /* Read Plan */
  {
    TMR_ReadPlan plan;
    TMR_RP_init_simple(&plan, 0, NULL, TMR_TAG_PROTOCOL_GEN2, 1000);

    /* (Optional) Tag Filter
     * Not required to read TID, but useful for limiting target tags */
    if (0)  /* Change to "if (1)" to enable filter */
    {
      TMR_TagData td;
      TMR_TagFilter filt;
      td.protocol = TMR_TAG_PROTOCOL_GEN2;
      {
        int i = 0;
        td.epc[i++] = 0x01;
        td.epc[i++] = 0x23;
        td.epcByteCount = i;
      }
      ret = TMR_TF_init_tag(&filt, &td);
      checkerr(rp, ret, 1, "creating tag filter");
      ret = TMR_RP_set_filter(&plan, &filt);
      checkerr(rp, ret, 1, "setting tag filter");
    }

    /* Embedded Tagop */
    {
      TMR_TagOp op;
      ret = TMR_TagOp_init_GEN2_ReadData(&op, TMR_GEN2_BANK_TID, 0, 8);
      checkerr(rp, ret, 1, "creating tagop: GEN2 read data");
      ret = TMR_RP_set_tagop(&plan, &op);
      checkerr(rp, ret, 1, "setting tagop");
    }

    /* Commit read plan */
    ret = TMR_paramSet(rp, TMR_PARAM_READ_PLAN, &plan);
    checkerr(rp, ret, 1, "setting read plan");
  }

  ret = TMR_read(rp, 500, NULL);
  checkerr(rp, ret, 1, "reading tags");

  while (TMR_SUCCESS == TMR_hasMoreTags(rp))
  {
    TMR_TagReadData trd;
    uint8_t dataBuf[16];
    char epcStr[128];

    ret = TMR_TRD_init_data(&trd, sizeof(dataBuf)/sizeof(uint8_t), dataBuf);
    checkerr(rp, ret, 1, "creating tag read data");

    ret = TMR_getNextTag(rp, &trd);
    checkerr(rp, ret, 1, "fetching tag");

    TMR_bytesToHex(trd.tag.epc, trd.tag.epcByteCount, epcStr);
    printf("%s\n", epcStr);
    if (0 < trd.data.len)
    {
      char dataStr[128];
      TMR_bytesToHex(trd.data.list, trd.data.len, dataStr);
      printf("  data(%d): %s\n", trd.data.len, dataStr);
    }
  }

  TMR_destroy(rp);
  return 0;
}
