/**
 * Sample program that reads tags for a fixed period of time (500ms)
 * and prints the tags found.
 * @file read.c
 */

#include <tm_reader.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// #include <easy.h>
#include <curl/curl.h>

void sendPutRequest(const char *relative_time, const char *interrogator_time, const char *db_password, const char *freeform) {
    CURL *curl;
    CURLcode res;
    struct curl_slist *headers = NULL;
 
    // Initialize a CURL handle
    curl = curl_easy_init();
    if(curl) {
        // Create the JSON string
        char *jsonData = (char *)malloc(1024 * sizeof(char)); // Adjust size as needed
        if (!jsonData) {
            printf("Failed to allocate memory for JSON data.\n");
            return;
        }
 
        sprintf(jsonData, "[{\"data\":{\"relative_time\":\"%s\",\"interrogator_time\":\"%s\",\"db_password\":\"%s\",\"freeform\":\"%s\"}}]",
                relative_time, interrogator_time, db_password, freeform);
 
        // Set the URL for the PUT request
        curl_easy_setopt(curl, CURLOPT_URL, "https://localhost:5000/api/rssi");
 
        // Specify the PUT data
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, jsonData);
 
        // Set the content type header
        headers = curl_slist_append(headers, "Content-Type: application/json; charset=UTF-8");
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
 
        // Perform the PUT request
        res = curl_easy_perform(curl);
        if(res != CURLE_OK) {
            fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
        }
 
        // Cleanup
        free(jsonData);
        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);
    }
}

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
  printf("connected successfully\n");

  region = 0x0D;
  //ret = TMR_paramSet(rp, TMR_paramID("/reader/region/id"), &region);
  ret = TMR_paramSet(rp, TMR_PARAM_REGION_ID, &region);
  checkerr(rp, ret, 1, "setting region");
  printf("set region succesfully\n");
  

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
  
  //-----------------------------------------------------------------------
  // Version 1: Original Version

  // Read tags
  // printf("trying to read data\n");
  // status = TMR_read(rp, 500, NULL); // 500ms read time
  // if (status != TMR_SUCCESS)
  // {
  //   printf("Error reading tags: %s\n", TMR_strerr(rp, ret));
  //   return;
  // }


  // while (TMR_hasMoreTags(rp) == TMR_SUCCESS)
  // {
  //   TMR_TagReadData trd;
  //   TMR_getNextTag(rp, &trd);
  //   char epcStr[128];
  //   ret = TMR_getNextTag(rp, &trd);
  //   if (TMR_SUCCESS != ret) 
  //   {
  //     errx(1, "Error getting tag: %s\n", TMR_strerr(rp, ret));
  //   }
  //   TMR_bytesToHex(trd.tag.epc, trd.tag.epcByteCount, epcStr);
  //   printf("EPC: %s, RSSI: %d dBm\n", epcStr, trd.rssi);
  // }

  // TMR_destroy(rp);
  // return 0;
 
  // ---------------------------------------------------------------------

  // Version 2: Modified Version

  while (true)
  {
      // Read tags
      printf("trying to read data\n");
      status = TMR_read(rp, 1000, NULL); // 500ms read time
      if (status != TMR_SUCCESS)
      {
        printf("Error reading tags: %s\n", TMR_strerr(rp, ret));
        return;
      }
      TMR_TagReadData trd;
      char epcStr[128];
      ret = TMR_getNextTag(rp, &trd);
      // checkerr(rp, ret, 1, "fetching tag");
      if (TMR_SUCCESS != ret) 
      {
        printf("No tag detected....\n");
        continue;
      }

      TMR_bytesToHex(trd.tag.epc, trd.tag.epcByteCount, epcStr);
      printf("EPC: %s, RSSI: %d dBm\n", epcStr, trd.rssi);
      printf("Sending data to the database...\n");
      sendPutRequest("2024-04-19T12:00:00Z", "2024-04-19T12:00:05Z", "kapilrocks", "this is a test");
  }

  TMR_destroy(rp);
  return 0;
}
