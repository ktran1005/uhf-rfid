/**
* Sample program that perform multi protocol read
* @file multiprotocolsearch.c
*/
#include "tm_reader.h"
#include "serial_reader_imp.h"
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

const char* protocolName(TMR_TagProtocol protocol)
{
	switch (protocol)
	{
	case TMR_TAG_PROTOCOL_NONE:
		return "NONE";
	case TMR_TAG_PROTOCOL_ISO180006B:
		return "ISO180006B";
	case TMR_TAG_PROTOCOL_GEN2:
		return "GEN2";
	case TMR_TAG_PROTOCOL_ISO180006B_UCODE:
		return "ISO180006B_UCODE";
	case TMR_TAG_PROTOCOL_IPX64:
		return "IPX64";
	case TMR_TAG_PROTOCOL_IPX256:
		return "IPX256";
	default:
		return "unknown";
	}
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
#define SUBPLAN_MAX (4)
    TMR_ReadPlan multiplan;
    TMR_ReadPlan subplans[SUBPLAN_MAX];
    TMR_ReadPlan* subplanPtrs[SUBPLAN_MAX];
    int subplanCount = 0;

    ret = TMR_RP_init_simple(&subplans[subplanCount++], 0, NULL, TMR_TAG_PROTOCOL_GEN2, 0);
    checkerr(rp, ret, 1, "creating GEN2 read plan");
    ret = TMR_RP_init_simple(&subplans[subplanCount++], 0, NULL, TMR_TAG_PROTOCOL_ISO180006B, 0);
    checkerr(rp, ret, 1, "creating ISO180006B read plan");
    ret = TMR_RP_init_simple(&subplans[subplanCount++], 0, NULL, TMR_TAG_PROTOCOL_IPX64, 0);
    checkerr(rp, ret, 1, "creating IPX64 read plan");
    ret = TMR_RP_init_simple(&subplans[subplanCount++], 0, NULL, TMR_TAG_PROTOCOL_IPX256, 0);
    checkerr(rp, ret, 1, "creating IPX256 read plan");
    {
      int i;
      for (i=0; i<subplanCount; i++)
      {
        subplanPtrs[i] = &subplans[i];
      }
    }
    ret = TMR_RP_init_multi(&multiplan, subplanPtrs, subplanCount, 0);
    checkerr(rp, ret, 1, "creating multi read plan");
    ret = TMR_paramSet(rp, TMR_PARAM_READ_PLAN, &multiplan);
    checkerr(rp, ret, 1, "setting read plan");

    ret = TMR_read(rp, 1000, NULL);
    if (TMR_SUCCESS != ret)
    {
      fprintf(stderr, "Error reading tags: %s\n", TMR_strerr(rp, ret));
      /* Don't exit, tags might still have been read before the error occurred. */
    }

    while (TMR_SUCCESS == TMR_hasMoreTags(rp))
    {
      TMR_TagReadData trd;
      char epcStr[128];

      ret = TMR_getNextTag(rp, &trd);
      checkerr(rp, ret, 1, "fetching tag");

      TMR_bytesToHex(trd.tag.epc, trd.tag.epcByteCount, epcStr);
      printf("%s %s\n", protocolName(trd.tag.protocol), epcStr);
    }
  }

  TMR_destroy(rp);
  return 0;
}

