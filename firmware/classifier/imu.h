

#ifndef IMU_H
#define IMU_H

#define ICM20649_I2CADDR_DEFAULT 0x68 ///< ICM20X default i2c address
#define ICM20X_B0_ACCEL_XOUT_H 0x2D   ///< first byte of accel data
#define ICM20X_B0_REG_BANK_SEL 0x7F   ///< register bank selection register

#define ICM20X_B2_GYRO_SMPLRT_DIV 0x00    ///< Gyroscope data rate divisor
#define ICM20X_B2_GYRO_CONFIG_1 0x01      ///< Gyro config for range setting
#define ICM20X_B2_ACCEL_SMPLRT_DIV_1 0x10 ///< Accel data rate divisor MSByte
#define ICM20X_B2_ACCEL_SMPLRT_DIV_2 0x11 ///< Accel data rate divisor LSByte
#define ICM20X_B2_ACCEL_CONFIG_1 0x14     ///< Accel config for setting range

#define ICM20X_B0_PWR_MGMT_1 0x06 ///< primary power management register

#include "pico/stdlib.h"
#include "sfe_bus.h"
#define SENSORS_GRAVITY_EARTH (9.80665F) /**< Earth's gravity in m/s^2 */
#define SENSORS_DPS_TO_RADS (0.017453293F) /**< Degrees/s to rad/s multiplier */
/** The accelerometer data range */
typedef enum {
  ICM20649_ACCEL_RANGE_4_G,
  ICM20649_ACCEL_RANGE_8_G,
  ICM20649_ACCEL_RANGE_16_G,
  ICM20649_ACCEL_RANGE_30_G,
} icm20649_accel_range_t;

/** The gyro data range */
typedef enum {
  ICM20649_GYRO_RANGE_500_DPS,
  ICM20649_GYRO_RANGE_1000_DPS,
  ICM20649_GYRO_RANGE_2000_DPS,
  ICM20649_GYRO_RANGE_4000_DPS,
} icm20649_gyro_range_t;
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
/** struct sensor_event_s is used to provide a single sensor event in a common
 * format. */
typedef struct {
  int32_t version;   /**< must be sizeof(struct sensors_event_t) */
  int32_t sensor_id; /**< unique sensor identifier */
  int32_t type;      /**< sensor type */
  int32_t reserved0; /**< reserved */
  int32_t timestamp; /**< time is in milliseconds */
  union {
    float data[4];
    sensors_vec_t acceleration; /**< acceleration values are in meter per second
                                   per second (m/s^2) */
    sensors_vec_t
        magnetic; /**< magnetic vector values are in micro-Tesla (uT) */
    sensors_vec_t orientation; /**< orientation values are in degrees */
    sensors_vec_t gyro;        /**< gyroscope values are in rad/s */
    float temperature; /**< temperature is in degrees centigrade (Celsius) */
    float distance;    /**< distance in centimeters */
    float light;       /**< light in SI lux units */
    float pressure;    /**< pressure in hectopascal (hPa) */
    float relative_humidity; /**< relative humidity in percent */
    float current;           /**< current in milliamps (mA) */
    float voltage;           /**< voltage in volts (V) */
  };
} sensors_event_t;

class imu {
public:
  imu();
  virtual ~imu() {};
  bool begin(i2c_inst_t *i2c, uint8_t deviceAddress = ICM20649_I2CADDR_DEFAULT);
  bool enableAccelDLPF(bool enable, icm20x_accel_cutoff_t cutoff_freq);
  bool enableGyrolDLPF(bool enable, icm20x_gyro_cutoff_t cutoff_freq);

  uint8_t getGyroRateDivisor(void);
  void setGyroRateDivisor(uint8_t new_gyro_divisor);

  uint16_t getAccelRateDivisor(void);
  void setAccelRateDivisor(uint16_t new_accel_divisor);

  void reset(void);
  bool getEvent(sensors_event_t *accel, sensors_event_t *gyro,
                sensors_event_t *temp);
  void writeAccelRange(uint8_t new_accel_range);
  void writeGyroRange(uint8_t new_gyro_range);

private:
  void scaleValues(void);
  bool ping();

protected:
  uint8_t current_accel_range; ///< accelerometer range cache
  uint8_t current_gyro_range;  ///< gyro range cache
  float temperature,           ///< Last reading's temperature (C)
      accX,                    ///< Last reading's accelerometer X axis m/s^2
      accY,                    ///< Last reading's accelerometer Y axis m/s^2
      accZ,                    ///< Last reading's accelerometer Z axis m/s^2
      gyroX,                   ///< Last reading's gyro X axis in rad/s
      gyroY,                   ///< Last reading's gyro Y axis in rad/s
      gyroZ,                   ///< Last reading's gyro Z axis in rad/s
      magX,                    ///< Last reading's mag X axis in rad/s
      magY,                    ///< Last reading's mag Y axis in rad/s
      magZ;                    ///< Last reading's mag Z axis in rad/s

  i2c_inst_t *_i2cPort;
  uint8_t _address;
  SfeI2C _i2cBus;
  GNSSDeviceBus *_sfeBus;
  void _read(void);
  void _setBank(uint8_t bank_number);
  int16_t rawAccX, ///< temp variables
      rawAccY,     ///< temp variables
      rawAccZ,     ///< temp variables
      rawTemp,     ///< temp variables
      rawGyroX,    ///< temp variables
      rawGyroY,    ///< temp variables
      rawGyroZ,    ///< temp variables
      rawMagX,     ///< temp variables
      rawMagY,     ///< temp variables
      rawMagZ;     ///< temp variables
};

#endif
