/*
 * Kusbegi Flight Controller Main
 *
 * Yazar: İsmail Emre CANPINAR
 * @canpinaremre
 *
 * Açıklama: Uçuş kontrol kartının bütün global
 * değişkenlerini içerek KUSBEGI yapısını içerir.
 *
 *
 */

#ifndef __KUSBEGI_FC_GLOBAL_VALUES_KUSBEGI_H__
#define __KUSBEGI_FC_GLOBAL_VALUES_KUSBEGI_H__

#include "stm32f4xx_hal.h"
#include "Kusbegi_Inc/kusbegi_fc_parameters.h"

/*
 * Haberleşme için gerekli olan yapıların
 * tek bir yapı içerisinde birleştirilmesi.
 *
 * Init_Kusbegi fonksiyonu bu yapıya göre düzenlenmelidir.
 */
typedef struct {
	UART_HandleTypeDef *Telemetry;
	I2C_HandleTypeDef *IMU;
	UART_HandleTypeDef *SBUS;
	UART_HandleTypeDef *GPS;
} KSB_COMMUNICATION;

/*
 * SBUS okuma protokolü için gerekli değişkenleri
 * barındıran yapının tanımıdır.
 */
typedef struct {
	uint8_t msg_raw[25];
	uint8_t header_check_cnt;
	bool frame_lost;
	bool failsafe;
	bool connection_err;
} KSB_SBUS;

/*
 * Alıcıdan gelen verilerin işlenmesi
 * ve saklanması için oluşturulan yapının tanımıdır.
 *
 */
typedef struct {
	uint16_t rc_channels_raw[16];
	uint16_t rc_channels_PWM[16];
	uint16_t arm_cnt;
	uint16_t disarm_cnt;
	bool state_arm;
	RC_Status status;
	Flight_Modes state_mode;
} KSB_RC;

/*
 * IMU üzerinden gelen ham verileri depolayan yapıdır.
 */
typedef struct {
	float gyro[3];
	float accel[3];
	float magno[3];
	uint8_t off_magno[6];
	uint8_t off_accel[6];
	uint8_t off_gyro[6];

	uint32_t last_read_time; /* ms */
	uint32_t elapsed_time; /* ms */
} KSB_IMU;

typedef struct{
	float euler[3];
	float ypr[3];
	float yaw_dps;
}KSB_ATTITUDE;

/*
 * PID kontrolcüsü yapısı
 */
typedef struct {

	/*
	 * Kontrolcü katsayıları
	 */
	float Kp;
	float Ki;
	float Kd;


	/*
	 * Çıkış limitleri
	 */
	float limMin;
	float limMax;

	/*
	 * İntegral değerinin limitleri
	 */
	float limInt;

	/*
	 * Geçen zaman saniye cinsinden
	 */
	float deltaT;

	/*
	 * Kontrolcü hafızası
	 */
	float integrator;
	float prevError;
	float differentiator;
	float prevMeasurement;

	/*
	 * Kontrolcü çıktısı
	 */
	float out;

} KSB_PIDCONTROLLER;

typedef struct{
	float fm_yaw_sp;
	float fm_pitch_sp;
	float fm_roll_sp;
	float fm_altitude_sp;
}KSB_FLIGHT_MODE_SP;

typedef struct{
	KSB_FLIGHT_MODE_SP fm_sp;
	Flight_Modes mode;
}KSB_FLIGHT_MODE;

/*
 * PID kontrolcülerini bir arada tutan yapı
 */
typedef struct{
	KSB_PIDCONTROLLER pid_yaw;
	KSB_PIDCONTROLLER pid_pitch;
	KSB_PIDCONTROLLER pid_roll;
	KSB_PIDCONTROLLER pid_altitude;
}KSB_PIDS;

typedef struct{
	bool arm;
	bool kill;
	bool failsafe;
	bool disarm_to_arm;
}KSB_STATUS;


typedef struct{
	KSB_COMMUNICATION com;
	KSB_SBUS sbus;
	KSB_RC rc;
	KSB_IMU imu;
	KSB_ATTITUDE attitude;
	KSB_PIDS pids;
	KSB_FLIGHT_MODE flight_mode;
	uint16_t motor_powers_PWM[4];
	KSB_STATUS status;
	uint8_t order_of_leds;
}KSB_KUSBEGI;





#endif /* __KUSBEGI_FC_GLOBAL_VALUES_KUSBEGI_H__ */
