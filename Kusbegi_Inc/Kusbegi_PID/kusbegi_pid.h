/*
 *PID kontrolcüsü
 *Belirlenmiş kazanımlara göre PID algoritması koşturur.
 *
 *Önce başlatma fonskiyonu çağırılır ardından katsayılar
 *ve limitler belirlenir.
 *
 *Her döngüde güncelleme fonksiyonu çağıralarak istenilen çıktı bulunur.
 */

#include <Kusbegi_Inc/kusbegi.h>
#include <Kusbegi_Inc/kusbegi_fc_parameters.h>

#ifndef __KUSBEGI_PID_H__
#define __KUSBEGI_PID_H__



void PIDController_Init(KSB_PIDCONTROLLER *pid);
void PIDController_Reset(KSB_PIDCONTROLLER *pid);
float PIDController_Update(KSB_PIDCONTROLLER *pid, float setpoint, float measurement);




#endif /* __KUSBEGI_PID_H__ */
