
#pragma once

#define NUM_CLASSIFICATIONS 45 // each byte is 40 seconds (4 predictions x 10s per prediction) 30 minutes is 45 bytes
#define SIZE_OF_LOCATION  12 
#define SIZE_OF_ACTIVITY  49
#define SIZE_OF_RECORD  61 // 12 + 49 = 61

typedef struct
{
      long start_time;
      uint8_t activities[NUM_CLASSIFICATIONS];
}  activity_reading;

typedef struct
{
      long start_time;
      float lat;
      float lon;
}  location_reading;


typedef struct
{
    location_reading location;
    activity_reading activity;
    bool partially_sent; // Flag to indicate if one part has been sent
} combined_reading;


  

