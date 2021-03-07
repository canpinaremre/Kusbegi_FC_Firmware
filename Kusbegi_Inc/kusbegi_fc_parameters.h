/*
 * Kusbegi Flight Controller Global Values
 *
 * Yazar: İsmail Emre CANPINAR
 * @canpinaremre
 *
 * Açıklama: Uçuş kontrol kartının parametrelerini içerir
 *
 *
 */



#include <Kusbegi_Inc/kusbegi.h>
#include "stm32f4xx_hal.h"
#include "main.h"
#include "cmsis_os.h"



#ifndef __KUSBEGI_FC_GLOBAL_VALUES_PARAMETERS_H__
#define __KUSBEGI_FC_GLOBAL_VALUES_PARAMETERS_H__

/*
 * İHA limitleri
 */
#define UAV_PITCH_ROLL_LIMIT 20.0f /* derece */
#define UAV_YAW_DPS_LIMIT 90.0f /*derece */

/*
 * PID Kontrolcü katsayı ve limitleri
 */
#define PID_ROLL_PITCH_KP  1.66f
#define PID_ROLL_PITCH_KI  0.55f
#define PID_ROLL_PITCH_KD  0.55f
#define PID_ROLL_PITCH_LIM_MIN -400.0f
#define PID_ROLL_PITCH_LIM_MAX  400.0f
#define PID_ROLL_PITCH_LIM_INT  10.00f
#define PID_YAW_KP  2.77f
#define PID_YAW_KI  2.77f
#define PID_YAW_KD  0.0f
#define PID_YAW_LIM_MIN -190.0f
#define PID_YAW_LIM_MAX  190.0f
#define PID_YAW_LIM_INT  11.50f
#define PID_ALTITUDE_KP  2.0f
#define PID_ALTITUDE_KI  0.5f
#define PID_ALTITUDE_KD  0.25f
#define PID_ALTITUDE_LIM_MIN -10.0f
#define PID_ALTITUDE_LIM_MAX  10.0f
#define PID_ALTITUDE_LIM_INT  5.0f


/*
 * KUSBEGI_FC_MAIN fonksiyonlarının yenilenme hızları
 * Milisaniye (ms)
 */
#define KUSBEGI_UPDATE_RATE_LEDs 500
#define KUSBEGI_UPDATE_RATE_IMU 5
#define KUSBEGI_UPDATE_RATE_Barometer 100
#define KUSBEGI_UPDATE_RATE_PID 10
#define KUSBEGI_UPDATE_RATE_Receiver 20
#define KUSBEGI_UPDATE_RATE_Telemetry 400
#define KUSBEGI_UPDATE_RATE_Logger 200
#define KUSBEGI_UPDATE_RATE_MotorPWM 19
#define KUSBEGI_UPDATE_RATE_OffBoard 100
#define KUSBEGI_UPDATE_RATE_FlightMode 19
#define KUSBEGI_UPDATE_RATE_FlightTask 100
#define KUSBEGI_UPDATE_RATE_PositionEstimation 50



/*
 * RC Kumanda ile ilgili parametreler
 * RC_CHANNEL_MAX en az 5 olabilir
 * Kumanda 4 ekseni ve mod ataması yapılmak zorundadır.
 * 5. kanaldan sonraki kanalları kullanmamak için yorum satırı olarak düzenlenir.
 */
#define RC_CHANNEL_MAX 			6

#define RC_CHANNEL_THROTTLE 	0
#define RC_CHANNEL_ROLL 		1
#define RC_CHANNEL_PITCH 		2
#define RC_CHANNEL_YAW	 		3
#define RC_CHANNEL_MODE	 		4
#define RC_CHANNEL_KILL	 		5
//#define RC_CHANNEL_RTL	 		6
//#define RC_CHANNEL_LAND	 		7
//#define RC_CHANNEL_OFFBOARD	 	8
//#define RC_CHANNEL_ARM			9

#define RC_ARM_DISARM_TOLARANCE	40 		//PWM
#define RC_STICK_ARM_TIME		1500 	//ms
#define RC_STICK_DISARM_TIME	600 	//ms

/*
 * DEĞİŞTİRİLEMEZ!
 * ARM ve DISARM sürelerinden hesaplanan döngü sayısıdır.
 */
#define RC_STICK_ARM_CNT		(RC_STICK_ARM_TIME / KUSBEGI_UPDATE_RATE_Receiver)
#define RC_STICK_DISARM_CNT		(RC_STICK_DISARM_TIME / KUSBEGI_UPDATE_RATE_Receiver)




/*
 * Filtre ve füzyon ile ilgili parametreler
 *
 */
#define COMPLEMENTARY_PR_ALFA (0.975F)
#define COMPLEMENTARY_YAW_ALFA (0.92F)


/*
 * Bool enum
 * false = 0
 * true = 1
 */
typedef enum {
	false = 0,
	true = 1
}bool;


/*
 * Alıcının durumunu belirten enum:
 * RC_OK: Alıcının okunmasında bir probmlem yok
 * conn_err: Bağlantıda hata var ve alıcıdan header düzgün okunamıyor.
 * failsafe: Alıcıya failsafe sinyali gelmiştir
 * framelost: Gelen paketlerde kayıp vardır.
 */
typedef enum{
	RC_OK = 1,
	rc_conn_err = -1,
	rc_failsafe = -2,
	rc_frame_lost = -3,
}RC_Status;


/*
 * Kumandanın kanallarının enum olarak belirlenmesi
 */
typedef enum{
	rc_throttle = 	RC_CHANNEL_THROTTLE,
	rc_yaw		 = 	RC_CHANNEL_YAW,
	rc_pitch 	 = 	RC_CHANNEL_PITCH,
	rc_roll     = 	RC_CHANNEL_ROLL,
	rc_mode 	 = 	RC_CHANNEL_MODE
#ifdef RC_CHANNEL_KILL
	,rc_kill_s  = 	RC_CHANNEL_KILL
#endif
#ifdef RC_CHANNEL_RTL
	,rc_rth 	 = 	RC_CHANNEL_RTL
#endif
#ifdef RC_CHANNEL_LAND
	,rc_land 	 = 	RC_CHANNEL_LAND
#endif
#ifdef RC_CHANNEL_OFFBOARD
	,rc_offboard = RC_CHANNEL_OFFBOARD
#endif
#ifdef RC_CHANNEL_ARM
	,rc_arm = RC_CHANNEL_ARM
#endif
}RC_Channel;

/*
 * Uçuş modlarını belirten enum
 */
typedef enum{
	fm_stabilize = 0,
	fm_alt_hold,
	fm_pos_hold,
	fm_auto,
	fm_rtl,
	fm_land,
	fm_failsafe
}Flight_Modes;


#endif /* __KUSBEGI_FC_GLOBAL_VALUES_PARAMETERS_H__ */
