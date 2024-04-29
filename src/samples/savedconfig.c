/**
 * Sample program that reads tags for a fixed period of time (500ms)
 * @file SavedConfig.c
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
  TMR_SR_UserConfigOp config;
  TMR_TagProtocol protocol;

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
  
  protocol = TMR_TAG_PROTOCOL_GEN2;
  ret = TMR_paramSet(rp, TMR_PARAM_TAGOP_PROTOCOL, &protocol);   // This to set the protocol
  checkerr(rp, ret, 1, "setting protocol");

  //Init UserConfigOp structure to save configuration
  TMR_init_UserConfigOp(&config, TMR_USERCONFIG_SAVE);
  ret = TMR_paramSet(rp, TMR_PARAM_USER_CONFIG, &config);
  checkerr(rp, ret, 1, "setting user configuration: save all configuration");
  printf("User config set option:save all configuration\n");

  //Init UserConfigOp structure to Restore all saved configuration parameters
  TMR_init_UserConfigOp(&config, TMR_USERCONFIG_RESTORE);
  ret = TMR_paramSet(rp, TMR_PARAM_USER_CONFIG, &config);
  checkerr(rp, ret, 1, "setting configuration: restore all saved configuration params");
  printf("User config set option:restore all saved configuration params\n");
  
  //Init UserConfigOp structure to verify all saved configuration parameters
  TMR_init_UserConfigOp(&config, TMR_USERCONFIG_VERIFY);
  ret = TMR_paramSet(rp, TMR_PARAM_USER_CONFIG, &config);
  checkerr(rp, ret, 1, "setting configuration: verify all saved configuration params");
  printf("User config set option:verify all configuration\n");


  // Get User Profile
  {
    TMR_Region region;
    TMR_TagProtocol proto;
    uint32_t baudrate;

    ret = TMR_paramGet(rp, TMR_PARAM_REGION_ID, &region);
    printf("Get user config success - option:Region\n");
    printf("%d\n", region);

    ret = TMR_paramGet(rp, TMR_PARAM_TAGOP_PROTOCOL, &proto);
    printf("Get user config success - option:Protocol\n");
    printf("%s\n", protocolName(proto));

    ret = TMR_paramGet(rp, TMR_PARAM_BAUDRATE, &baudrate);
    printf("Get user config success option:Baudrate\n");
    printf("%d\n", baudrate);
  }

  //Init UserConfigOp structure to reset/clear all configuration parameter
  TMR_init_UserConfigOp(&config, TMR_USERCONFIG_CLEAR);
  ret = TMR_paramSet(rp, TMR_PARAM_USER_CONFIG, &config);
  checkerr(rp, ret, 1, "setting user configuration option: reset all configuration parameters");
  printf("User config set option:reset all configuration parameters\n");

  TMR_destroy(rp);
  return 0;
}
