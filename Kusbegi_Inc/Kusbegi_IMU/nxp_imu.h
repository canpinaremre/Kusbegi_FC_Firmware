
#include "stm32f4xx_hal.h"

#ifndef __NXP_IMU_H__
#define __NXP_IMU_H__

/*=========================================================================
    I2C ADDRESS/BITS AND SETTINGS
    -----------------------------------------------------------------------*/
/** 7-bit address for this sensor */
#define FXAS21002C_ADDRESS (0x21) // 0100001
/** Device ID for this sensor (used as a sanity check during init) */
#define FXAS21002C_ID (0xD7) // 1101 0111
/** Gyroscope sensitivity at 250dps */
#define GYRO_SENSITIVITY_250DPS (0.0078125F) // Table 35 of datasheet
/** Gyroscope sensitivity at 500dps */
#define GYRO_SENSITIVITY_500DPS (0.015625F)
/** Gyroscope sensitivity at 1000dps */
#define GYRO_SENSITIVITY_1000DPS (0.03125F)
/** Gyroscope sensitivity at 2000dps */
#define GYRO_SENSITIVITY_2000DPS (0.0625F)
/** 7-bit I2C address for this sensor */
#define FXOS8700_ADDRESS (0x1F) // 0011111
/** Device ID for this sensor (used as sanity check during init) */
#define FXOS8700_ID (0xC7) // 1100 0111

#define FXOS8700_ADDRESS_WRITE (0x3E)
#define FXOS8700_ADDRESS_READ (0x3F)
#define FXAS21002C_ADDRESS_WRITE (0x42)
#define FXAS21002C_ADDRESS_READ (0x43)
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
/*!
    Raw register addresses used to communicate with the sensor.
*/
typedef enum {
  GYRO_REGISTER_STATUS = 0x00,    /**< 0x00 */
  GYRO_REGISTER_OUT_X_MSB = 0x01, /**< 0x01 */
  GYRO_REGISTER_OUT_X_LSB = 0x02, /**< 0x02 */
  GYRO_REGISTER_OUT_Y_MSB = 0x03, /**< 0x03 */
  GYRO_REGISTER_OUT_Y_LSB = 0x04, /**< 0x04 */
  GYRO_REGISTER_OUT_Z_MSB = 0x05, /**< 0x05 */
  GYRO_REGISTER_OUT_Z_LSB = 0x06, /**< 0x06 */
  GYRO_REGISTER_WHO_AM_I =
      0x0C, /**< 0x0C (default value = 0b11010111, read only) */
  GYRO_REGISTER_CTRL_REG0 =
      0x0D, /**< 0x0D (default value = 0b00000000, read/write) */
  GYRO_REGISTER_CTRL_REG1 =
      0x13, /**< 0x13 (default value = 0b00000000, read/write) */
  GYRO_REGISTER_CTRL_REG2 =
      0x14, /**< 0x14 (default value = 0b00000000, read/write) */
} gyroRegisters_t;

typedef enum {
  FXOS8700_REGISTER_STATUS = 0x00,    /**< 0x00 */
  FXOS8700_REGISTER_OUT_X_MSB = 0x01, /**< 0x01 */
  FXOS8700_REGISTER_OUT_X_LSB = 0x02, /**< 0x02 */
  FXOS8700_REGISTER_OUT_Y_MSB = 0x03, /**< 0x03 */
  FXOS8700_REGISTER_OUT_Y_LSB = 0x04, /**< 0x04 */
  FXOS8700_REGISTER_OUT_Z_MSB = 0x05, /**< 0x05 */
  FXOS8700_REGISTER_OUT_Z_LSB = 0x06, /**< 0x06 */
  FXOS8700_REGISTER_WHO_AM_I =
      0x0D, /**< 0x0D (default value = 0b11000111, read only) */
  FXOS8700_REGISTER_XYZ_DATA_CFG = 0x0E, /**< 0x0E */
  FXOS8700_REGISTER_CTRL_REG1 =
      0x2A, /**< 0x2A (default value = 0b00000000, read/write) */
  FXOS8700_REGISTER_CTRL_REG2 =
      0x2B, /**< 0x2B (default value = 0b00000000, read/write) */
  FXOS8700_REGISTER_CTRL_REG3 =
      0x2C, /**< 0x2C (default value = 0b00000000, read/write) */
  FXOS8700_REGISTER_CTRL_REG4 =
      0x2D, /**< 0x2D (default value = 0b00000000, read/write) */
  FXOS8700_REGISTER_CTRL_REG5 =
      0x2E, /**< 0x2E (default value = 0b00000000, read/write) */
  FXOS8700_REGISTER_MSTATUS = 0x32,    /**< 0x32 */
  FXOS8700_REGISTER_MOUT_X_MSB = 0x33, /**< 0x33 */
  FXOS8700_REGISTER_MOUT_X_LSB = 0x34, /**< 0x34 */
  FXOS8700_REGISTER_MOUT_Y_MSB = 0x35, /**< 0x35 */
  FXOS8700_REGISTER_MOUT_Y_LSB = 0x36, /**< 0x36 */
  FXOS8700_REGISTER_MOUT_Z_MSB = 0x37, /**< 0x37 */
  FXOS8700_REGISTER_MOUT_Z_LSB = 0x38, /**< 0x38 */
  FXOS8700_REGISTER_OFF_X_MSB = 0x3F,

  FXOS8700_REGISTER_MCTRL_REG1 =
      0x5B, /**< 0x5B (default value = 0b00000000, read/write) */
  FXOS8700_REGISTER_MCTRL_REG2 =
      0x5C, /**< 0x5C (default value = 0b00000000, read/write) */
  FXOS8700_REGISTER_MCTRL_REG3 =
      0x5D, /**< 0x5D (default value = 0b00000000, read/write) */
} fxos8700Registers_t;
/*=========================================================================*/

/*=========================================================================
    OPTIONAL SPEED SETTINGS
    -----------------------------------------------------------------------*/
/*!
    Enum to define valid gyroscope range values
*/
typedef enum {
  GYRO_RANGE_250DPS = 250,   /**< 250dps */
  GYRO_RANGE_500DPS = 500,   /**< 500dps */
  GYRO_RANGE_1000DPS = 1000, /**< 1000dps */
  GYRO_RANGE_2000DPS = 2000  /**< 2000dps */
} gyroRange_t;

typedef enum {
  ACCEL_RANGE_2G = 0x00, /**< +/- 2g range */
  ACCEL_RANGE_4G = 0x01, /**< +/- 4g range */
  ACCEL_RANGE_8G = 0x02  /**< +/- 8g range */
} fxos8700AccelRange_t;


uint8_t init_nxp_imu(I2C_HandleTypeDef *hi2c);
uint8_t read_nxp_imu(I2C_HandleTypeDef *hi2c, float *gyro, float *accel, float *magno);


#endif /* __NXP_IMU_H__ */
