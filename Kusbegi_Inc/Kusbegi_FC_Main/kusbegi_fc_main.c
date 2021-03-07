/*
 * Kusbegi Flight Controller Main
 *
 * Yazar: İsmail Emre CANPINAR
 * @canpinaremre
 *
 * Açıklama: Uçuş kontrol kartının üzerinde çalışan fonksiyonların
 * FreeRTOS üzerinde çağırılmasını ve zamanlamalarını kontrol eder.
 *
 *
 */

#include <Kusbegi_Inc/Kusbegi_FC_Main/kusbegi_fc_main.h>

KSB_KUSBEGI Kusbegi;


void KUSBEGI_FC_InitKUSBEGI(UART_HandleTypeDef *Telemetry, I2C_HandleTypeDef *IMU,
		UART_HandleTypeDef *SBUS, UART_HandleTypeDef *GPS) {

	Kusbegi.com.GPS = GPS;
	Kusbegi.com.Telemetry = Telemetry;
	Kusbegi.com.IMU = IMU;
	Kusbegi.com.SBUS = SBUS;
}






void KUSBEGI_FC_UpdateIMU(){

	/*
	 * IMU başlangıç parametreleri ayarlanıp IMU başlatılır.
	 *
	 */
	init_nxp_imu(Kusbegi.com.IMU);

	Kusbegi.imu.off_magno[0] = 254;
	Kusbegi.imu.off_magno[1] = 84;
	Kusbegi.imu.off_magno[2] = 253;
	Kusbegi.imu.off_magno[3] = 56;
	Kusbegi.imu.off_magno[4] = 252;
	Kusbegi.imu.off_magno[5] = 78;

	HAL_I2C_Mem_Write(Kusbegi.com.IMU, (uint16_t) FXOS8700_ADDRESS_WRITE,
			(uint16_t) FXOS8700_REGISTER_OFF_X_MSB, 1, &Kusbegi.imu.off_magno[0], 6, 1);


	Kusbegi.imu.accel[0] = 0.0f;
	Kusbegi.imu.accel[1] = 0.0f;
	Kusbegi.imu.accel[2] = 0.0f;
	Kusbegi.imu.gyro[0] = 0.0f;
	Kusbegi.imu.gyro[1] = 0.0f;
	Kusbegi.imu.gyro[2] = 0.0f;
	Kusbegi.imu.magno[0] = 0.0f;
	Kusbegi.imu.magno[1] = 0.0f;
	Kusbegi.imu.magno[2] = 0.0f;


	while(true){
		Kusbegi.imu.elapsed_time = HAL_GetTick() - Kusbegi.imu.last_read_time;
		Kusbegi.imu.last_read_time = HAL_GetTick();

		read_nxp_imu(Kusbegi.com.IMU, Kusbegi.imu.gyro, Kusbegi.imu.accel, Kusbegi.imu.magno);

		/*
		 * KALİBRASYON MODU
		 */
//		uint8_t offset[6];
//		HAL_I2C_Mem_Read(Kusbegi.com.IMU, (uint16_t) FXOS8700_ADDRESS_READ,
//				(uint16_t) FXOS8700_REGISTER_OFF_X_MSB, 1,
//				&offset[0], 6, 1);
//
//		for(int i = 0;i<6;i++)
//			Kusbegi.imu.off_magno[i]= offset[i];


		Complementary_filter_YPR(&Kusbegi);


		osDelay(KUSBEGI_UPDATE_RATE_IMU);
	}
}
void KUSBEGI_FC_UpdateBarometer(){
	while(true){
			osDelay(KUSBEGI_UPDATE_RATE_Barometer);
		}
}
void KUSBEGI_FC_UpdatePID(){

	/*
	 * PID kontrolcü katsayı ve limitleri ayarlanır.
	 */

	/*
	 * Kp
	 */
	Kusbegi.pids.pid_yaw.Kp = PID_YAW_KP;
	Kusbegi.pids.pid_pitch.Kp = PID_ROLL_PITCH_KP;
	Kusbegi.pids.pid_roll.Kp = PID_ROLL_PITCH_KP;
	Kusbegi.pids.pid_altitude.Kp = PID_ALTITUDE_KP;

	/*
	 * Ki
	 */
	Kusbegi.pids.pid_yaw.Ki = PID_YAW_KI;
	Kusbegi.pids.pid_pitch.Ki = PID_ROLL_PITCH_KI;
	Kusbegi.pids.pid_roll.Ki = PID_ROLL_PITCH_KI;
	Kusbegi.pids.pid_altitude.Ki = PID_ALTITUDE_KI;

	/*
	 * Kd
	 */
	Kusbegi.pids.pid_yaw.Kd = PID_YAW_KD;
	Kusbegi.pids.pid_pitch.Kd = PID_ROLL_PITCH_KD;
	Kusbegi.pids.pid_roll.Kd = PID_ROLL_PITCH_KD;
	Kusbegi.pids.pid_altitude.Kd = PID_ALTITUDE_KD;

	/*
	 * Kontrolcü minimum değeri
	 */
	Kusbegi.pids.pid_yaw.limMin = PID_YAW_LIM_MIN;
	Kusbegi.pids.pid_pitch.limMin = PID_ROLL_PITCH_LIM_MIN;
	Kusbegi.pids.pid_roll.limMin = PID_ROLL_PITCH_LIM_MIN;
	Kusbegi.pids.pid_altitude.limMin = PID_ALTITUDE_LIM_MIN;

	/*
	 * Kontrolcü maksimum değeri
	 */
	Kusbegi.pids.pid_yaw.limMax = PID_YAW_LIM_MAX;
	Kusbegi.pids.pid_pitch.limMax = PID_ROLL_PITCH_LIM_MAX;
	Kusbegi.pids.pid_roll.limMax = PID_ROLL_PITCH_LIM_MAX;
	Kusbegi.pids.pid_altitude.limMax = PID_ALTITUDE_LIM_MAX;

	/*
	 * İntegral limiti
	 */
	Kusbegi.pids.pid_yaw.limInt = PID_YAW_LIM_INT;
	Kusbegi.pids.pid_pitch.limInt = PID_ROLL_PITCH_LIM_INT;
	Kusbegi.pids.pid_roll.limInt = PID_ROLL_PITCH_LIM_INT;
	Kusbegi.pids.pid_altitude.limInt = PID_ALTITUDE_LIM_INT;

	/*
	 * PID kontrolcüsü başlatılır.
	 */
	PIDController_Init(&Kusbegi.pids.pid_yaw);
	PIDController_Init(&Kusbegi.pids.pid_pitch);
	PIDController_Init(&Kusbegi.pids.pid_roll);
	PIDController_Init(&Kusbegi.pids.pid_altitude);

	while(true){

			PIDController_Update(&Kusbegi.pids.pid_yaw, Kusbegi.flight_mode.fm_sp.fm_yaw_sp, -Kusbegi.imu.gyro[2]);
			PIDController_Update(&Kusbegi.pids.pid_pitch, Kusbegi.flight_mode.fm_sp.fm_pitch_sp, Kusbegi.attitude.ypr[1]);
			PIDController_Update(&Kusbegi.pids.pid_roll, Kusbegi.flight_mode.fm_sp.fm_roll_sp, Kusbegi.attitude.ypr[2]);
			//TODO:Altitude

			osDelay(KUSBEGI_UPDATE_RATE_PID);
		}
}
void KUSBEGI_FC_UpdateReceiver(){


	while (true) {
		/*
		 * SBUS kullanılarak alıcının gönderdiği mesajlar okunup
		 * her hanal için PWM, failsafe ve frame_lost sinyalleri
		 * kayıt edilir.
		 */
		KUSBEGI_SBUS_ReadSBUS(&Kusbegi);
		Kusbegi.rc.status = KUSBEGI_RC_Update(&Kusbegi);


		osDelay(KUSBEGI_UPDATE_RATE_Receiver);
	}
}
void KUSBEGI_FC_UpdateTelemetry(){
	while(true){
			osDelay(KUSBEGI_UPDATE_RATE_Telemetry);
		}
}
void KUSBEGI_FC_UpdateLogger(){
	while(true){
			osDelay(KUSBEGI_UPDATE_RATE_Logger);
		}
}
void KUSBEGI_FC_UpdateMotorPWM(){
	while(true){

		if (Kusbegi.status.disarm_to_arm == true) {
			Kusbegi.motor_powers_PWM[0] = RC_PWM_MIN;
			Kusbegi.motor_powers_PWM[1] = RC_PWM_MIN;
			Kusbegi.motor_powers_PWM[2] = RC_PWM_MIN;
			Kusbegi.motor_powers_PWM[3] = RC_PWM_MIN;
			osDelay(1000);
			Kusbegi.status.disarm_to_arm = false;
		}
		else if (Kusbegi.status.arm && !Kusbegi.status.kill) {


			Kusbegi.motor_powers_PWM[0] =
					(uint16_t) (Kusbegi.pids.pid_altitude.out
							- Kusbegi.pids.pid_roll.out
							- Kusbegi.pids.pid_pitch.out
							+ Kusbegi.pids.pid_yaw.out);
			Kusbegi.motor_powers_PWM[1] =
					(uint16_t) (Kusbegi.pids.pid_altitude.out
							+ Kusbegi.pids.pid_roll.out
							+ Kusbegi.pids.pid_pitch.out
							+ Kusbegi.pids.pid_yaw.out);
			Kusbegi.motor_powers_PWM[2] =
					(uint16_t) (Kusbegi.pids.pid_altitude.out
							+ Kusbegi.pids.pid_roll.out
							- Kusbegi.pids.pid_pitch.out
							- Kusbegi.pids.pid_yaw.out);
			Kusbegi.motor_powers_PWM[3] =
					(uint16_t) (Kusbegi.pids.pid_altitude.out
							- Kusbegi.pids.pid_roll.out
							+ Kusbegi.pids.pid_pitch.out
							- Kusbegi.pids.pid_yaw.out);

			for(int i = 0; i<4;i++){
				if(Kusbegi.motor_powers_PWM[i] < RC_PWM_MIN)
					Kusbegi.motor_powers_PWM[i] = RC_PWM_MIN;
				else if(Kusbegi.motor_powers_PWM[i] > RC_PWM_MAX)
					Kusbegi.motor_powers_PWM[i] = RC_PWM_MAX;
			}

			/*
			 * TODO: PWM değerlerini gönder
			 */

		}
		else {
			Kusbegi.motor_powers_PWM[0] = 0;
			Kusbegi.motor_powers_PWM[1] = 0;
			Kusbegi.motor_powers_PWM[2] = 0;
			Kusbegi.motor_powers_PWM[3] = 0;
		}


			osDelay(KUSBEGI_UPDATE_RATE_MotorPWM);
		}
}
void KUSBEGI_FC_UpdateOffBoard(){
	while(true){
			osDelay(KUSBEGI_UPDATE_RATE_OffBoard);
		}
}
void KUSBEGI_FC_UpdateFlightMode(){
	while(true){

			if(Kusbegi.rc.status != RC_OK){
				Kusbegi.status.failsafe = true;
				Kusbegi.status.arm = false;
				Kusbegi.rc.state_arm = false;
			}

			else if(Kusbegi.rc.status == RC_OK){
				if(!Kusbegi.status.arm && Kusbegi.rc.state_arm){
					Kusbegi.status.disarm_to_arm = true;
				}
				Kusbegi.status.arm = Kusbegi.rc.state_arm;
				Kusbegi.status.failsafe = false;
			}



			if(Kusbegi.flight_mode.mode == fm_stabilize){
				Kusbegi.pids.pid_altitude.out = Kusbegi.rc.rc_channels_PWM[rc_throttle];
				Kusbegi.flight_mode.fm_sp.fm_yaw_sp = KUSBEGI_Map_Float(Kusbegi.rc.rc_channels_PWM[rc_yaw],(float)RC_PWM_MIN,(float)RC_PWM_MAX,-UAV_YAW_DPS_LIMIT,UAV_YAW_DPS_LIMIT);
				Kusbegi.flight_mode.fm_sp.fm_pitch_sp = KUSBEGI_Map_Float(Kusbegi.rc.rc_channels_PWM[rc_pitch],(float)RC_PWM_MIN,(float)RC_PWM_MAX,-UAV_PITCH_ROLL_LIMIT,UAV_PITCH_ROLL_LIMIT);
				Kusbegi.flight_mode.fm_sp.fm_roll_sp = KUSBEGI_Map_Float(Kusbegi.rc.rc_channels_PWM[rc_roll],(float)RC_PWM_MIN,(float)RC_PWM_MAX,-UAV_PITCH_ROLL_LIMIT,UAV_PITCH_ROLL_LIMIT);
			}




			osDelay(KUSBEGI_UPDATE_RATE_FlightMode);
		}
}
void KUSBEGI_FC_UpdateFlightTask(){
	while(true){
			osDelay(KUSBEGI_UPDATE_RATE_FlightTask);
		}
}
void KUSBEGI_FC_UpdateLEDs(){
	/*
	 * Önce başlangıç ayarlarını yapıyoruz.
	 */
	Kusbegi.order_of_leds = 0;
	HAL_GPIO_WritePin(GPIOD, LD3_ORANGE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, LD4_GREEN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, LD5_RED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, LD6_BLUE_Pin, GPIO_PIN_SET);
	while(true)
	{
		osDelay(KUSBEGI_UPDATE_RATE_LEDs);

		switch (Kusbegi.order_of_leds) {
			case 0:
				HAL_GPIO_WritePin(GPIOD, LD3_ORANGE_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOD, LD5_RED_Pin, GPIO_PIN_SET);
				break;
			case 1:
				HAL_GPIO_WritePin(GPIOD, LD5_RED_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOD, LD6_BLUE_Pin, GPIO_PIN_SET);
				break;
			case 2:
				HAL_GPIO_WritePin(GPIOD, LD6_BLUE_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOD, LD4_GREEN_Pin, GPIO_PIN_SET);
				break;
			case 3:
				HAL_GPIO_WritePin(GPIOD, LD4_GREEN_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOD, LD3_ORANGE_Pin, GPIO_PIN_SET);
				break;
			}
			Kusbegi.order_of_leds++;
			if (Kusbegi.order_of_leds == 4)
				Kusbegi.order_of_leds = 0;

	}
}

void KUSBEGI_FC_UpdatePositionEstimation(){
	while(true){
			osDelay(KUSBEGI_UPDATE_RATE_PositionEstimation);
		}
}
