#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

// Variables for running average
unsigned long sampleCount = 0;

// Battery capacity in mAh
const float batteryCapacity_mAh = 3100.0;
float avgCurrent_mA = 0.0;

// Timer variables
unsigned long lastPrintTime = 0;
const unsigned long printInterval = 20000; // 20 seconds in milliseconds
void setup(void) 
{
  Serial.begin(115200);
  while (!Serial) {
      delay(1);
  }
    
  Serial.println("Hello!");
  
  if (! ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }

  Serial.println("Measuring voltage and current with INA219 ...");
}

void loop(void) 
{
  float current_mA = ina219.getCurrent_mA();
  
  avgCurrent_mA = (avgCurrent_mA * sampleCount + current_mA) / (sampleCount + 1);
  sampleCount++;
  

  
  // Only print every 20 seconds
  unsigned long currentMillis = millis();
  if (currentMillis - lastPrintTime >= printInterval) {
    float estimatedHours = (avgCurrent_mA > 0) ? (batteryCapacity_mAh / avgCurrent_mA) : 0;
    unsigned int estHrs = (unsigned int)estimatedHours;
    unsigned int estMins = (unsigned int)((estimatedHours - estHrs) * 60);
    float estimatedDays = estimatedHours / 24.0;

    lastPrintTime = currentMillis;
    // Print output
    Serial.print("Time: "); Serial.print(millis() / 1000); Serial.print("s    ");
    Serial.print("Current: "); Serial.print(current_mA); Serial.print(" mA    ");
    Serial.print("Avg current: "); Serial.print(avgCurrent_mA, 2); Serial.print(" mA    ");
    Serial.print("Est. Battery Life: "); 
    if (avgCurrent_mA > 0) {
      Serial.print(estHrs); Serial.print("h ");
    Serial.print(estMins); Serial.print("m    ");
      Serial.print("("); Serial.print(estimatedDays, 2); Serial.println(" days)");

    } 
    else {
      Serial.println("N/A");
    }
  }

  delay(10);
}
