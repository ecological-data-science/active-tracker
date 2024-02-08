


#include <Wire.h>
#include "classifier.h"
#include "pico/stdlib.h"

bool IMU_ACTIVE = false;

Classifier classifier;

unsigned long imu_interval = 200; // update imu every 200ms for 5hz readings
unsigned long last_imu_time = 0;
void setup() {
  // setSDAPin(4);
  // setSCLPin(5);
  Wire.begin();

  Serial.begin(9600);
  while (!Serial)
     delay(10);


  if (!classifier.begin()) 
  {
    Serial.println("ERROR: classifier");
  }
  delay(500);
  Serial.println("\nSetup ok");

}


void loop() {

  if (!IMU_ACTIVE)
  {
    Serial.println("IMU activated");
    classifier.activate(0);
    IMU_ACTIVE = true;
  }
  if (IMU_ACTIVE)
    {
      if ((millis() - last_imu_time) >= imu_interval) //To stream at 5 Hz without using additional timers
      {
        last_imu_time = millis();
        IMU_ACTIVE = classifier.update();
      }
    }
  if (!IMU_ACTIVE)
  {
    
      classifier.deactivate();
      Serial.println("IMU deactivated");
      // classifier.latest_activity;
      delay(5000);
  }
  return;
}
