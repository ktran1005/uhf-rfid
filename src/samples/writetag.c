/**
 * Sample program that writes an EPC to a tag
 * @file writetag.c
 */

#include <tm_reader.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#ifndef WIN32
#include <string.h>
#endif

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
    uint8_t epcData[] = {
      0x01, 0x23, 0x45, 0x67, 0x89, 0xAB,
      0xCD, 0xEF, 0x01, 0x23, 0x45, 0x67,
      };
    TMR_TagData epc;
    TMR_TagOp tagop;

	/* Set the tag EPC to a known value*/

    epc.epcByteCount = sizeof(epcData) / sizeof(epcData[0]);
    memcpy(epc.epc, epcData, epc.epcByteCount * sizeof(uint8_t));
    ret = TMR_TagOp_init_GEN2_WriteTag(&tagop, &epc);
    checkerr(rp, ret, 1, "initializing GEN2_WriteTag");

    ret = TMR_executeTagOp(rp, &tagop, NULL, NULL);
    checkerr(rp, ret, 1, "executing GEN2_WriteTag");

	{  /* Write Tag EPC with a select filter*/	  

	  uint8_t newEpcData[] = {
	    0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB,
		};
	  TMR_TagFilter filter;
	  TMR_TagData newEpc;
	  TMR_TagOp newtagop;

	  newEpc.epcByteCount = sizeof(newEpcData) / sizeof(newEpcData[0]);
      memcpy(newEpc.epc, newEpcData, newEpc.epcByteCount * sizeof(uint8_t));
	  
	  /* Initialize the new tagop to write the new epc*/
	  
	  ret = TMR_TagOp_init_GEN2_WriteTag(&newtagop, &newEpc);
	  checkerr(rp, ret, 1, "initializing GEN2_WriteTag");

      /* Initialize the filter with the original epc of the tag which is set earlier*/
	  ret = TMR_TF_init_tag(&filter, &epc);
	  checkerr(rp, ret, 1, "initializing TMR_TagFilter");

	  /* Execute the tag operation Gen2 writeTag with select filter applied*/
	  ret = TMR_executeTagOp(rp, &newtagop, &filter, NULL);
	  checkerr(rp, ret, 1, "executing GEN2_WriteTag");
	}
  }

  TMR_destroy(rp);
  return 0;
}
