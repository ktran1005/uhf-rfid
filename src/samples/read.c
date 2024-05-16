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
#include <curl/curl.h>

// Function to identify tags
void printLabel(const char *epc, char *label) {
    if (strcmp(epc, "E280116060000211EBDE7479") == 0) {
        // printf("GRN\n");
        sprintf(label, "%s", "GRN");
    } else if (strcmp(epc, "E280116060000211EBDE7469") == 0) {
        // printf("BLK\n");
        sprintf(label, "%s", "BLK");
    } else if (strcmp(epc, "E280116060000211EBDE7459") == 0) {
        // printf("RED\n");
        sprintf(label, "%s", "RED");
    } else if (strcmp(epc, "E280116060000211EBDE7489") == 0) {
        // printf("WHT\n");
        sprintf(label, "%s", "WHT");
    } else if (strcmp(epc, "E280116060000211EBDCDD2D") == 0) {
        // printf("SMR\n");
        sprintf(label, "%s", "SMR");
    } else if (strcmp(epc, "E280116060000211EBDCDD3D") == 0) {
        // printf("MAG\n");
        sprintf(label, "%s", "MAG");
    } else if (strcmp(epc, "E280116060000211EBDCDD4D") == 0) {
        // printf("KDT\n");
        sprintf(label, "%s", "KDT");
    } else if (strcmp(epc, "E280116060000211EBDCDD5D") == 0) {
        // printf("VPS\n");
        sprintf(label, "%s", "VPS");
    } else if (strcmp(epc, "E280116060000211EBDCDD6D") == 0) {
        // printf("BAD\n");
        sprintf(label, "%s", "BAD");
    } else if (strcmp(epc, "E280116060000211EBDCDD7D") == 0) {
        // printf("MLW\n");
        sprintf(label, "%s", "MLW");
    } else if (strcmp(epc, "E280116060000211EBDCDD8D") == 0) {
        // printf("JDZ\n");
        sprintf(label, "%s", "JDZ");
    } else if (strcmp(epc, "E280116060000211EBDCDD9D") == 0) {
        // printf("SPB\n");
        sprintf(label, "%s", "SPB");
    } else if (strcmp(epc, "E280116060000211EBDCDDAD") == 0) {
        // printf("PAT\n");
        sprintf(label, "%s", "PAT");
    } else if (strcmp(epc, "E280116060000211EBDCDDBD") == 0) {
        // printf("CEL\n");
        sprintf(label, "%s", "CEL");
    } else if (strcmp(epc, "E280116060000211EBDE7469") == 0) {
        // printf("Vishnu IPhone\n");
        sprintf(label, "%s", "VIP");
    } else if (strcmp(epc, "E280116060000211EBDDEB7F") == 0) {
        // printf("Vishnu Wallet\n");
        sprintf(label, "%s", "WAL");
    } else if (strcmp(epc, "E280116060000211EBDCDDCD") == 0) {
        // printf("BOB\n");
        sprintf(label, "%s", "BOB");
    } else if (strcmp(epc, "E280116060000211EBDCDDDD") == 0) {
        // printf("BIB\n");
        sprintf(label, "%s", "BIB");
    } else if (strcmp(epc, "E280116060000211EBDCDDED") == 0) {
        // printf("LAB\n");
        sprintf(label, "%s", "LAB");
    } else if (strcmp(epc, "E280116060000211EBDD090D") == 0) {
        // printf("FIB\n");
        sprintf(label, "%s", "FIB");
    } else if (strcmp(epc, "E280116060000211EBDD091D") == 0) {
        // printf("NIB\n");
        sprintf(label, "%s", "NIP");
    } else if (strcmp(epc, "E280116060000211EBDD092D") == 0) {
        // printf("TIB\n");
        sprintf(label, "%s", "TIB");
    } else if (strcmp(epc, "E280116060000211EBDD093D") == 0) {
        // printf("BIN\n");
        sprintf(label, "%s", "BIN");
    } else if (strcmp(epc, "E280116060000211EBDD094D") == 0) {
        // printf("SIB\n");
        sprintf(label, "%s", "SIB");
    } else if (strcmp(epc, "E280116060000211EBDD095D") == 0) {
        // printf("LIB\n");
        sprintf(label, "%s", "LIB");
    } else if (strcmp(epc, "E280116060000211EBDD096D") == 0) {
        // printf("DIB\n");
        sprintf(label, "%s", "DIB");
    } else {
        // printf("Unknown\n");
        sprintf(label, "%s", "Unknown");
    }
}

// Function to get the current epoch time in microseconds
long long get_epoch_time_microseconds() {
    struct timeval tv;

    // Get the current time
    if (gettimeofday(&tv, NULL) == -1) {
        perror("gettimeofday");
        return -1;
    }

    // Calculate the current epoch time in microseconds
    long long microseconds = (long long)tv.tv_sec * 1000000LL + (long long)tv.tv_usec;

    return microseconds;
}

// Function to send HTTP request
void sendPutRequest(const long long relative_time, const long long interrogator_time, const char *db_password, const char *freeform) {
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
       
        sprintf(jsonData, "[{\"data\":{\"relative_time\":\"%lld\",\"interrogator_time\":\"%lld\",\"db_password\":\"%s\",\"freeform\": %s}}]",
                relative_time, interrogator_time, db_password, freeform);
        
        printf("jsonData: %s\n", jsonData);

        // Set the URL for the PUT request
        curl_easy_setopt(curl, CURLOPT_URL, "https://localhost:5001/api/rssi");
 
        // Specify the PUT data
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, jsonData);
 
        // Set the content type header
        headers = curl_slist_append(headers, "Content-Type: application/json; charset=UTF-8");
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
	      curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "PUT");
        
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
  printf("connected successfully\n");

  region = 0x0D;
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
  long long start_time = get_epoch_time_microseconds();
  while (true)
  {
      char label[100] = {0};
      char *buffer = (char *)malloc(1024 * sizeof(char)); // Adjust size as needed;
      if (!buffer) {
            printf("Failed to allocate memory for buffer data.\n");
            return;
      }

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
    //   printf("Print Label: ");
      printLabel(epcStr, label);
      printf("Sending data to the database...\n");
      
      sprintf(buffer, "[{\"EPC\":\"%s\",\"rssi\":\"%d\", \"Label\":\"%s\"}]", epcStr, trd.rssi, label);
      
    //   printf("buffer: %s\n", buffer);
      sendPutRequest(get_epoch_time_microseconds() - start_time, get_epoch_time_microseconds(), "kapilrocks", buffer);
      free(buffer);
  }

  TMR_destroy(rp);
  return 0;
}
