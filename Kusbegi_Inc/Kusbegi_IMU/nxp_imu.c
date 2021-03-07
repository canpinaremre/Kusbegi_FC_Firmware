#include <Kusbegi_Inc/Kusbegi_IMU/nxp_imu.h>

uint8_t init_nxp_imu(I2C_HandleTypeDef *hi2c){

	 uint8_t id;
	 uint8_t rslt;
	 rslt = HAL_I2C_Mem_Read(hi2c, (uint16_t)FXAS21002C_ADDRESS_READ, (uint16_t)GYRO_REGISTER_WHO_AM_I, 1, &id, 1, 1);
	 if(rslt != HAL_OK)
		 return rslt;

	 if(id != FXAS21002C_ID)
		 return 6;

	 rslt = HAL_I2C_Mem_Read(hi2c, (uint16_t)FXOS8700_ADDRESS_READ, (uint16_t)FXOS8700_REGISTER_WHO_AM_I, 1, &id, 1, 1);
	 if(rslt != HAL_OK)
	 		 return rslt;
	 if(id != FXOS8700_ID)
	 		 return 6;

	id = 0x00;
	rslt = HAL_I2C_Mem_Write(hi2c, (uint16_t) FXAS21002C_ADDRESS_WRITE,
			(uint16_t) GYRO_REGISTER_CTRL_REG1, 1, &id, 1, 1);
	id = (1 << 6);
	rslt = HAL_I2C_Mem_Write(hi2c, (uint16_t) FXAS21002C_ADDRESS_WRITE,
			(uint16_t) GYRO_REGISTER_CTRL_REG1, 1, &id, 1, 1);
	id = 0x00;
	rslt = HAL_I2C_Mem_Write(hi2c, (uint16_t) FXAS21002C_ADDRESS_WRITE,
			(uint16_t) GYRO_REGISTER_CTRL_REG0, 1, &id, 1, 1);
	id = 0x0E;
	rslt = HAL_I2C_Mem_Write(hi2c, (uint16_t) FXAS21002C_ADDRESS_WRITE,
			(uint16_t) GYRO_REGISTER_CTRL_REG1, 1, &id, 1, 1);



	id = 0x00;
	rslt = HAL_I2C_Mem_Write(hi2c, (uint16_t) FXOS8700_ADDRESS_WRITE,
			(uint16_t) FXOS8700_REGISTER_CTRL_REG1, 1, &id, 1, 1);

	id = 0x00;
	rslt = HAL_I2C_Mem_Write(hi2c, (uint16_t) FXOS8700_ADDRESS_WRITE,
			(uint16_t) FXOS8700_REGISTER_XYZ_DATA_CFG, 1, &id, 1, 1);

	id = 0x02;
	rslt = HAL_I2C_Mem_Write(hi2c, (uint16_t) FXOS8700_ADDRESS_WRITE,
			(uint16_t) FXOS8700_REGISTER_CTRL_REG2, 1, &id, 1, 1);

	id = 0x15;
	rslt = HAL_I2C_Mem_Write(hi2c, (uint16_t) FXOS8700_ADDRESS_WRITE,
			(uint16_t) FXOS8700_REGISTER_CTRL_REG1, 1, &id, 1, 1);

	//id = 0x9F;
	id = 0x1F;
	rslt = HAL_I2C_Mem_Write(hi2c, (uint16_t) FXOS8700_ADDRESS_WRITE,
			(uint16_t) FXOS8700_REGISTER_MCTRL_REG1, 1, &id, 1, 1);




	HAL_Delay(120);

	  return rslt;
}

uint8_t read_nxp_imu(I2C_HandleTypeDef *hi2c, float *gyro, float *accel, float *magno){

	uint8_t rslt;
	uint8_t data[6];
	int16_t data_read[3];


	rslt = HAL_I2C_Mem_Read(hi2c, (uint16_t) FXAS21002C_ADDRESS_READ,
			(uint16_t) GYRO_REGISTER_OUT_X_MSB, 1, &data[0], 6, 1);

	data_read[0] = (int16_t) ((data[0] << 8) | data[1]);
	data_read[1] = (int16_t) ((data[2] << 8) | data[3]);
	data_read[2] = (int16_t) ((data[4] << 8) | data[5]);

	gyro[0] = ((float) data_read[0]) / 16.0;
	gyro[1] = ((float) data_read[1]) / 16.0;
	gyro[2] = ((float) data_read[2]) / 16.0;

	rslt = HAL_I2C_Mem_Read(hi2c, (uint16_t) FXOS8700_ADDRESS_READ,
			(uint16_t) FXOS8700_REGISTER_OUT_X_MSB, 1, &data[0], 6, 1);

	data_read[0] = (int16_t) ((data[0] << 8) | data[1]);
	data_read[1] = (int16_t) ((data[2] << 8) | data[3]);
	data_read[2] = (int16_t) ((data[4] << 8) | data[5]);

	accel[0] = ((float) data_read[0]) * 0.000569767f;
	accel[1] = ((float) data_read[1]) * 0.000569767f;
	accel[2] = ((float) data_read[2]) * 0.000569767f;

	rslt = HAL_I2C_Mem_Read(hi2c, (uint16_t) FXOS8700_ADDRESS_READ,
			(uint16_t) FXOS8700_REGISTER_MOUT_X_MSB, 1, &data[0], 6, 1);

	data_read[0] = (int16_t) ((data[0] << 8) | data[1]);
	data_read[1] = (int16_t) ((data[2] << 8) | data[3]);
	data_read[2] = (int16_t) ((data[4] << 8) | data[5]);

	magno[0] = ((float) data_read[0]) / 1.0;
	magno[1] = ((float) data_read[1]) / 1.0;
	magno[2] = ((float) data_read[2]) / 1.0;



	return rslt;
}


