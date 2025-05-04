

#include "main.h"


// TODO: implement the following classes
GPS gps;
// Storage storage;
// Classifier classifier;
// Lora lora;
// LowPower lowPower;

bool GPS_ACTIVE = false;
bool IMU_ACTIVE = false;
bool LORA_ACTIVE = false;


unsigned long gps_run_time = 1000*60*10;  // gps will run for up to 10 minutes to get a fix
unsigned long lora_run_time = 1000*60*20; // lora will broadcast for up to 20 minutes every hour

unsigned long imu_interval = 200; // update imu every 200ms for 5hz readings
unsigned long last_imu_time = 0;

unsigned long gps_check_interval = 60*1000; // check the gps fix every minute  
unsigned long gps_last_check_time = 0;  

unsigned long start_time = 0;  
unsigned long lora_start_time = 0; 

unsigned int imu_count;
// location_reading latest_location;

int main() 
{
  // begin the watchdog timer
  bool success = setup();
  if (!success) 
  {
    printf("Setup failed\n");
    // sleep for 1 minute to allow the watchdog to reset
    sleep_ms(60*1000);
  }
  while (true) 
  {
    main_loop();
  }
}

bool setup() 
{
  stdio_init_all();
  sleep_ms(5000);
  printf("Starting setup ...\n");

  // initialize I2C using the default pins
  i2c_init(i2c0, 100 * 1000); // 100 kHz

  gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
  gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

  // initialize the GPS
  if (!gps.begin(i2c0)) 
  {
    printf("GPS initialization failed\n");
    return false;
  }

  // initialize the RTC

  // initialize the classifier

  // initialize the lora communication

  // flash the LED to indicate success

  // turn off all LEDs

  printf("Setup complete\n");

  return true;

}











void main_loop() 
{
  
    if ((!GPS_ACTIVE) && (!IMU_ACTIVE) && (!LORA_ACTIVE))
    {

      // // if nothing active then we are at the top of the hour so wake up the gps and imu and start the watchdog
      // gps.activate();
      // classifier.activate();
      // wdt.start();

      GPS_ACTIVE=true;
      IMU_ACTIVE=true;

    }


    // if (GPS_ACTIVE)
      // GPS_ACTIVE = gps.update();

    // if (IMU_ACTIVE)
    //   IMU_ACTIVE = classifier.update();


    if ((!IMU_ACTIVE) && (!LORA_ACTIVE))
    {
      // lora.activate();
      LORA_ACTIVE=true;
    }
    
    // if (LORA_ACTIVE)
    //   LORA_ACTIVE = lora.update();

    // wdt.clear();

    if ((!GPS_ACTIVE) && (!IMU_ACTIVE) && (!LORA_ACTIVE)) {      
      

      
      //wdt.setup(WDT_OFF);  //watchdog 

      // lowPower.activate(); 
      
    }

   
  


    
}




