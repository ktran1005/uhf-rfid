/**
 * Sample program that sets an access password on a tag and locks its EPC.
 * @file locktag.c
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
  TMR_TagReadData trd;
  TMR_TagFilter filter;
  TMR_TagOp tagop;
  TMR_TagOp *op = &tagop;
  char epcString[128];
  TMR_TransportListenerBlock tb;
  TMR_GEN2_Password accessPassword;
  TMR_GEN2_Session oldSession, newSession;
  
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
  ret = TMR_paramSet(rp, TMR_paramID("/reader/region/id"), &region);
  checkerr(rp, ret, 1, "setting region");

  ret = TMR_paramGet(rp, TMR_PARAM_GEN2_SESSION, &oldSession);
  checkerr(rp, ret, 1, "getting session");

  newSession = TMR_GEN2_SESSION_S0;
  printf("Changing to Session %d from Session %d\n", oldSession, newSession);
  ret = TMR_paramSet(rp, TMR_PARAM_GEN2_SESSION, &newSession);
  checkerr(rp, ret, 1, "setting session");
  
  ret = TMR_read(rp, 500, NULL);
  checkerr(rp, ret, 1, "reading tags");

  if (TMR_ERROR_NO_TAGS == TMR_hasMoreTags(rp))
  {
    errx(1, "No tags found for test\n");
  }

  ret = TMR_getNextTag(rp, &trd);
  checkerr(rp, ret, 1, "reading tags");

  TMR_bytesToHex(trd.tag.epc, trd.tag.epcByteCount, epcString);

  /* Initialize the filter */
  TMR_TF_init_tag(&filter, &trd.tag);
  
  accessPassword = 0x0;

  TMR_TagOp_init_GEN2_Lock(op, TMR_GEN2_LOCK_BITS_EPC, TMR_GEN2_LOCK_BITS_EPC, accessPassword);
  ret= TMR_SR_executeTagOp(rp,op, &filter, NULL);
  checkerr(rp, ret, 1, "locking tag");
  printf("Locked EPC of tag %s\n", epcString);

  TMR_TagOp_init_GEN2_Lock(op, TMR_GEN2_LOCK_BITS_EPC, 0, accessPassword);
  ret= TMR_SR_executeTagOp(rp,op, &filter, NULL);
  checkerr(rp, ret, 1, "unlocking tag");
  printf("Unlocked EPC of tag %s\n", epcString);

  printf("Restoring Session %d\n", oldSession);
  ret = TMR_paramSet(rp, TMR_PARAM_GEN2_SESSION, &oldSession);
  checkerr(rp, ret, 1, "restoring session");

  TMR_destroy(rp);
  return 0;
}
