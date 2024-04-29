/**
 * Sample program that reads tags on multiple readers and prints the tags found.
 * @file multireadasync.c
 */

#include <tm_reader.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#ifndef WIN32
#include <unistd.h>
#endif

typedef struct readerDesc
{
  char* uri;
  int idx;
} readerDesc;

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
  FILE *out = stdout;
  readerDesc *rdp = cookie;
  uint32_t i;

  fprintf(out, "%s %s", rdp->uri, tx ? "Sending: " : "Received:");
  for (i = 0; i < dataLen; i++)
  {
    if (i > 0 && (i & 15) == 0)
      fprintf(out, "\n         ");
    fprintf(out, " %02x", data[i]);
  }
  fprintf(out, "\n");
}

void callback(TMR_Reader *reader, const TMR_TagReadData *t, void *cookie);

int main(int argc, char *argv[])
{
  TMR_Reader *r;
  readerDesc *rd;
  int rcount;
  TMR_Reader *rp;
  TMR_Status ret;
  TMR_Region region;
  TMR_TransportListenerBlock *tb;
  TMR_ReadListenerBlock *rlb;
  int i;

  if (argc < 2)
  {
    errx(1, "Please provide reader URLs, such as:\n"
           "tmr:///com4\n"
           "tmr://my-reader.example.com\n");
  }
  
  rcount = argc-1;
  r = (TMR_Reader*) calloc(rcount, sizeof(TMR_Reader));
  rd = (readerDesc*) calloc(rcount, sizeof(readerDesc));
  tb = (TMR_TransportListenerBlock*) calloc(rcount, sizeof(TMR_TransportListenerBlock));
  rlb = (TMR_ReadListenerBlock*) calloc(rcount, sizeof(TMR_ReadListenerBlock));
  for (i=0; i<rcount; i++)
  {
    rp = &r[i];
    rd[i].uri = argv[i+1];
    ret = TMR_create(rp, rd[i].uri);
    checkerr(rp, ret, 1, "creating reader %s");

    rd[i].idx = i;
    printf("Created reader %d: %s\n", rd[i].idx, rd[i].uri);

    tb[i].listener = serialPrinter;
    tb[i].cookie = &rd[i];


#if 0
    TMR_addTransportListener(rp, &tb[i]);
#endif


    //TMR_SR_PowerMode pm = TMR_SR_POWER_MODE_FULL;
    //ret = TMR_paramSet(rp, TMR_PARAM_POWERMODE, &pm);

    ret = TMR_connect(rp);
    checkerr(rp, ret, 1, "connecting reader");

    region = TMR_REGION_NA;
    ret = TMR_paramSet(rp, TMR_PARAM_REGION_ID, &region);
    checkerr(rp, ret, 1, "setting region");

    rlb[i].listener = callback;
    rlb[i].cookie = &rd[i];

    ret = TMR_addReadListener(rp, &rlb[i]);
    checkerr(rp, ret, 1, "adding read listener");

    ret = TMR_startReading(rp);
    checkerr(rp, ret, 1, "starting reading");
  }

#ifndef WIN32
  sleep(5);
#else
  Sleep(5000);
#endif

  for (i=0; i<rcount; i++)
  {
    rp = &r[i];
    ret = TMR_stopReading(rp);
    checkerr(rp, ret, 1, "stopping reading");
    TMR_destroy(rp);
  }
  return 0;
}


void
callback(TMR_Reader *reader, const TMR_TagReadData *t, void *cookie)
{
  char epcStr[128];
  readerDesc *rdp = cookie;

  TMR_bytesToHex(t->tag.epc, t->tag.epcByteCount, epcStr);
  printf("%s: %s\n", rdp->uri, epcStr);
}
