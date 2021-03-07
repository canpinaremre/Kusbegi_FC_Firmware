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

#include <Kusbegi_Inc/kusbegi.h>
#include <Kusbegi_Inc/kusbegi_fc_parameters.h>
#include <Kusbegi_Inc/Kusbegi_Filter_And_Fusion/kusbegi_filter_and_fusion.h>
#include <Kusbegi_Inc/Kusbegi_IMU/nxp_imu.h>
#include <Kusbegi_Inc/Kusbegi_RC/kusbegi_rc.h>
#include <Kusbegi_Inc/Kusbegi_SBUS/kusbegi_sbus.h>
#include <Kusbegi_Inc/Kusbegi_PID/kusbegi_pid.h>
#include "stm32f4xx_hal.h"



#ifndef __KUSBEGI_FC_MAIN_H__
#define __KUSBEGI_FC_MAIN_H__





void KUSBEGI_FC_InitKUSBEGI(UART_HandleTypeDef *Telemetry, I2C_HandleTypeDef *IMU,
		UART_HandleTypeDef *SBUS, UART_HandleTypeDef *GPS);


void KUSBEGI_FC_UpdateIMU();
void KUSBEGI_FC_UpdateBarometer();
void KUSBEGI_FC_UpdatePID();
void KUSBEGI_FC_UpdateReceiver();
void KUSBEGI_FC_UpdateTelemetry();
void KUSBEGI_FC_UpdateLogger();
void KUSBEGI_FC_UpdateMotorPWM();
void KUSBEGI_FC_UpdateOffBoard();
void KUSBEGI_FC_UpdateFlightMode();
void KUSBEGI_FC_UpdateFlightTask();
void KUSBEGI_FC_UpdateLEDs();
void KUSBEGI_FC_UpdatePositionEstimation();



#endif /* __KUSBEGI_FC_MAIN_H__ */
