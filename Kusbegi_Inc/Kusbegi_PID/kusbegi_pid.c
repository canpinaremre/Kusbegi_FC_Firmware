/*
 *PID kontrolcüsü
 *Belirlenmiş kazanımlara göre PID algoritması koşturur.
 *
 *Önce başlatma fonskiyonu çağırılır ardından katsayılar
 *ve limitler belirlenir.
 *
 *Her döngüde güncelleme fonksiyonu çağıralarak istenilen çıktı bulunur.
 */

#include "Kusbegi_Inc/Kusbegi_PID/kusbegi_pid.h"

/*
 * PID kontrolcüsünün başlatılması için gerekli ayarlamaların yapıldığı fonksiyondur.
 * İçerisine PIDController yapısı alır.
 */
void PIDController_Init(KSB_PIDCONTROLLER *pid){

	/*
	 *Değişkenler temizlenir.
	 */
	pid->integrator = 0.0f;
	pid->prevError  = 0.0f;

	pid->differentiator  = 0.0f;
	pid->prevMeasurement = 0.0f;

	pid->out = 0.0f;

	/*
	 * PID döngüsünün çalışma süresi KUSBEGI_UPDATE_RATE_PID olarak belirlenmiştir.
	 * Bu süre ms cinsinden olduğu için saniyeye çevirmek için 1000 ile bölünür.
	 */
	pid->deltaT = KUSBEGI_UPDATE_RATE_PID / 1000.0f;

}

/*
 * PID kontrolcüsünün integral ve çıkışını sıfırlayan fonksiyondur.
 * PID döngüsünün zamanı değiştikten sonra deltaT değerini güncellemek için de çağırılır.
 * İçerisine PIDController yapısı alır.
 */
void PIDController_Reset(KSB_PIDCONTROLLER *pid){

	pid->integrator = 0.0f;
	pid->out = 0.0f;
	pid->deltaT = KUSBEGI_UPDATE_RATE_PID / 1000.0f;
}

/*
 * PID kontrolcüsünün güncellenmesinde kullanılan fonksiyondur.
 * İçerisine PIDController yapısı alır.
 * Hedef değerini ve ölçüm değerini girdi olarak alır.
 * Kontrolcü çıktısını döndürür.
 */
float PIDController_Update(KSB_PIDCONTROLLER *pid, float setpoint, float measurement){

	/*
	 * Hedef ile ölçüm arasındaki fark hesaplanır.
	 */
    float error = setpoint - measurement;

	/*
	 * Hata ile Kp katsayısı çarpılarak P değeri bulunur.
	 */
    float proportional = pid->Kp * error;

	/*
	 * İntegral değerinin üzerine yeni integral değeri (zaman * hata) eklenir.
	 * Daha yavaş ve yumuşak değişim için önceki ve yeni hata kullanılır.
	 * Daha hızlı ve keskin tepkiler için sadece yeni hata kullanılır.
	 */
    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->deltaT * (error + pid->prevError);
//    pid->integrator = pid->integrator + pid->Ki * pid->deltaT * error ;

	/*
	 * İntegralin çok uzun süre toplanarak sonsuza ulaşması engellenir.
	 */
	if (pid->integrator > pid->limInt) {

		pid->integrator = pid->limInt;

	} else if (pid->integrator < -pid->limInt) {

		pid->integrator = -pid->limInt;

	}

	/*
	 * Türev alınırken hata değişimi yerine ölçüm değişimine bakılır.
	 * Bu sayede türev tekmesi (derivative kick) önlenmiş olur.
	 * Ölçüm değişimi zamana bölünür ve Kd katsayısı ile çarpılır.
	 * Hata değişimi yerine ölçüm değerine bakıldığı için işaret değişimi yapılır.
	 * Sonuçta elde edilen değer PID kontrolcüsünün D değeridir.
	 */
    pid->differentiator = -1.0f * pid->Kd * (measurement - pid->prevMeasurement)/ pid->deltaT;

	/*
	 * PID kontrolcüsünün P, I ve D değerleri toplanarak çıkış bulunur.
	 */
    pid->out = proportional + pid->integrator + pid->differentiator;


    /*
     * Çıkışın belirlenen limitleri aşmaması için limitleme yapılır.
     */
    if (pid->out > pid->limMax) {

        pid->out = pid->limMax;

    } else if (pid->out < pid->limMin) {

        pid->out = pid->limMin;

    }

	/*
	 * Sonraki döngüde kullanılmak üzere değerler kayıt edilir.
	 */
    pid->prevError       = error;
    pid->prevMeasurement = measurement;


	/*
	 * Çıkış fonksiyon üzerinden döndürülür.
	 */
    return pid->out;

}
