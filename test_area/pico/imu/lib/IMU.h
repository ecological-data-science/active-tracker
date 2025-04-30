

#ifndef IMU_H
#define IMU_H

#include "sfe_bus.h"
#define ICM20649_I2CADDR_DEFAULT 0x68 ///< ICM20X default i2c address

/** Options for `enableAccelDLPF` */
typedef enum {
  ICM20X_ACCEL_FREQ_246_0_HZ = 0x1,
  ICM20X_ACCEL_FREQ_111_4_HZ = 0x2,
  ICM20X_ACCEL_FREQ_50_4_HZ = 0x3,
  ICM20X_ACCEL_FREQ_23_9_HZ = 0x4,
  ICM20X_ACCEL_FREQ_11_5_HZ = 0x5,
  ICM20X_ACCEL_FREQ_5_7_HZ = 0x6,
  ICM20X_ACCEL_FREQ_473_HZ = 0x7,
} icm20x_accel_cutoff_t;

/** Options for `enableGyroDLPF` */
typedef enum {
  ICM20X_GYRO_FREQ_196_6_HZ = 0x0,
  ICM20X_GYRO_FREQ_151_8_HZ = 0x1,
  ICM20X_GYRO_FREQ_119_5_HZ = 0x2,
  ICM20X_GYRO_FREQ_51_2_HZ = 0x3,
  ICM20X_GYRO_FREQ_23_9_HZ = 0x4,
  ICM20X_GYRO_FREQ_11_6_HZ = 0x5,
  ICM20X_GYRO_FREQ_5_7_HZ = 0x6,
  ICM20X_GYRO_FREQ_361_4_HZ = 0x7,

} icm20x_gyro_cutoff_t;
/** struct sensors_vec_s is used to return a vector in a common format. */
typedef struct {
    union {
        float v[3];
        struct {
            float x;
            float y;
            float z;
        };
        /* Orientation sensors */
        struct {
            float azimuth;
            float pitch;
            float roll;
        };
    };
    int8_t status;
    uint8_t reserved[3];
} sensors_vec_t;
/* Sensor event (36 bytes) */
/** struct sensor_event_s is used to provide a single sensor event in a common format. */
typedef struct
{
    int32_t version;                          /**< must be sizeof(struct sensors_event_t) */
    int32_t sensor_id;                        /**< unique sensor identifier */
    int32_t type;                             /**< sensor type */
    int32_t reserved0;                        /**< reserved */
    int32_t timestamp;                        /**< time is in milliseconds */
    union
    {
        float           data[4];
        sensors_vec_t   acceleration;         /**< acceleration values are in meter per second per second (m/s^2) */
        sensors_vec_t   magnetic;             /**< magnetic vector values are in micro-Tesla (uT) */
        sensors_vec_t   orientation;          /**< orientation values are in degrees */
        sensors_vec_t   gyro;                 /**< gyroscope values are in rad/s */
        float           temperature;          /**< temperature is in degrees centigrade (Celsius) */
        float           distance;             /**< distance in centimeters */
        float           light;                /**< light in SI lux units */
        float           pressure;             /**< pressure in hectopascal (hPa) */
        float           relative_humidity;    /**< relative humidity in percent */
        float           current;              /**< current in milliamps (mA) */
        float           voltage;              /**< voltage in volts (V) */
    };
} sensors_event_t;

class IMU {
public:
  IMU();
  virtual ~IMU(){};
  bool begin(i2c_inst_t *i2c, uint8_t deviceAddress = ICM20649_I2CADDR_DEFAULT);
  bool enableAccelDLPF(bool enable, icm20x_accel_cutoff_t cutoff_freq);
  bool enableGyrolDLPF(bool enable, icm20x_gyro_cutoff_t cutoff_freq);

  uint8_t getGyroRateDivisor(void);
  void setGyroRateDivisor(uint8_t new_gyro_divisor);

  uint16_t getAccelRateDivisor(void);
  void setAccelRateDivisor(uint16_t new_accel_divisor);

  void reset(void);
  bool getEvent(sensors_event_t *accel, sensors_event_t *gyro, sensors_event_t *temp);

private:
  void scaleValues(void);
  bool ping();

protected:

  float temperature, ///< Last reading's temperature (C)
      accX,          ///< Last reading's accelerometer X axis m/s^2
      accY,          ///< Last reading's accelerometer Y axis m/s^2
      accZ,          ///< Last reading's accelerometer Z axis m/s^2
      gyroX,         ///< Last reading's gyro X axis in rad/s
      gyroY,         ///< Last reading's gyro Y axis in rad/s
      gyroZ,         ///< Last reading's gyro Z axis in rad/s
      magX,          ///< Last reading's mag X axis in rad/s
      magY,          ///< Last reading's mag Y axis in rad/s
      magZ;          ///< Last reading's mag Z axis in rad/s

  i2c_inst_t *_i2cPort;
  uint8_t _address;
  SfeI2C _i2cBus;
  GNSSDeviceBus *_sfeBus;
};







#endif
