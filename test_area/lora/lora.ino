/*
 *
 * GPS requisite functionality code
 *
 *
 */
#include <Wire.h>
#include "pico/stdlib.h"
//#include "pico/sleep.h"

#define SECRET_APP_KEY "434F4C494E5357494C44454245455354"


void sendmsg(const char *msg)
{

  unsigned long timeout = 1000;  // Timeout period in milliseconds (1 second)

  Serial1.println(msg);
  unsigned long startTime = millis();  // Record the starting time
  while (millis() - startTime < timeout)
  {
    while (Serial1.available()) {
      Serial.write(Serial1.read());
    }
   }

}


void setup() 
{


  Serial.begin(9600);
  Serial1.begin(9600); //Serial1 is the hardware serial port 

  delay(1000);   
  sendmsg("AT+ID=DevEui");

  delay(1000);   
  sendmsg("AT+ID=AppEui");

  delay(1000);   
  sendmsg("AT+ADR=?");
  delay(1000);
  char message[100];  // Make sure this buffer is large enough to hold the concatenated message
  sprintf(message, "AT+KEY=APPKEY,%s", SECRET_APP_KEY);
  sendmsg(message);  // Send the concatenated message
   
  delay(1000);   
  sendmsg("AT+MODE=LWOTAA");
  delay(1000);   
  sendmsg("AT+JOIN");
    delay(10000);   
  Serial1.println("staring main loop");

//Serial1.flush();  // Clear the buffer
//
//  Serial1.println("AT+ID=AppEui");
//  delay(1000);
//  // read the response
//  while (Serial1.available()) {
//    Serial.write(Serial1.read());
//  }
//
//  Serial.println("\nWaiting for 1 seconds");
//  delay(1000);        
//Serial1.flush();  // Clear the buffer
//
//  Serial1.print("AT+KEY=APPKEY,");
//  Serial1.println(SECRET_APP_KEY);
//  // delay(1000);
//  // read the response
//  while (Serial1.available()) {
//    Serial.write(Serial1.read());
//  }
//
//  Serial.println("\nWaiting for 5 seconds");
//  delay(1000);       
//    while (Serial1.available()) {
//    Serial.write(Serial1.read());
//  }
}
void loop() 
{
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  Serial.println("sending msg");

  sendmsg("AT+MSG=HELLO");
  
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW

  
  delay(1000*60*2);

}
