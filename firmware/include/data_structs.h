
#pragma once

typedef struct
{
      long start_time;
      uint8_t activities[45];
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


#define SIZE_OF_LOCATION = 12; 
#define SIZE_OF_ACTIVITY = 49;
#define SIZE_OF_RECORD = SIZE_OF_LOCATION + SIZE_OF_ACTIVITY; // 12 + 49 = 61
  

