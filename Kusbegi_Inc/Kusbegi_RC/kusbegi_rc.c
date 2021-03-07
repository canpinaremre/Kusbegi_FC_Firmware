/*
 * Kusbegi RC
 *
 * Yazar: İsmail Emre CANPINAR
 * @canpinaremre
 *
 * Açıklama: Okunan SBUS verilerini
 * anlamlı PWM değerlerine dönüştürülmesi ve
 * ARM / DISARM durumlarının belirlenmesini içerir.
 *
 *
 */

#include <Kusbegi_Inc/Kusbegi_RC/kusbegi_rc.h>

/*
 * @Fonksiyon: KUSBEGI_RC_PWM_Map
 * @Açıklama: 16 Bitlik işaretsiz tam sayıyı ölçeklendirir.
 * @param0-girdi: (uint16_t)Ölçeklendirilecek sayı
 * @param1-girdi: (uint16_t)Girdinin minimum değeri
 * @param2-girdi: (uint16_t)Girdinin maksimum değeri
 * @param3-girdi: (uint16_t)Çıkışın minimum değeri
 * @param4-girdi: (uint16_t)Çıkışın maksimum değeri
 * @return: (uint16_t)Ölçeklendirmenin sonucu.
 */
uint16_t KUSBEGI_Map_Unsigned(uint16_t u16_IN, uint16_t u16_INmin,
		uint16_t u16_INmax, uint16_t u16_OUTmin, uint16_t u16_OUTmax) {
	return ((((u16_IN - u16_INmin) * (u16_OUTmax - u16_OUTmin))
			/ (u16_INmax - u16_INmin)) + u16_OUTmin);
}

/*
 * @Fonksiyon: KUSBEGI_RC_PWM_Map
 * @Açıklama: Kesirli sayıyı ölçeklendirir.
 * @param0-girdi: (float)Ölçeklendirilecek sayı
 * @param1-girdi: (float)Girdinin minimum değeri
 * @param2-girdi: (float)Girdinin maksimum değeri
 * @param3-girdi: (float)Çıkışın minimum değeri
 * @param4-girdi: (float)Çıkışın maksimum değeri
 * @return: (float)Ölçeklendirmenin sonucu.
 */
float KUSBEGI_Map_Float(float f_IN, float f_INmin,
		float f_INmax, float f_OUTmin, float f_OUTmax){
	return ((((f_IN - f_INmin) * (f_OUTmax - f_OUTmin))
				/ (f_INmax - f_INmin)) + f_OUTmin);
}

/*
 * @Fonksiyon: KUSBEGI_RC_RawToPWM
 * @Açıklama: Okunan SBUS değerlerini PWM değerine ölçekler.
 * @param0-girdi: (KSB_KUSBEGI)KSB_KUSBEGI yapısı
 * @return: (bool)Fonksiyon başarısı
 */
int8_t KUSBEGI_RC_RawToPWM(KSB_KUSBEGI *Kusbegi){


	for (uint8_t i = 0; i < RC_CHANNEL_MAX; i++) {
		Kusbegi->rc.rc_channels_PWM[i] = KUSBEGI_Map_Unsigned(Kusbegi->rc.rc_channels_raw[i], SBUS_IN_MIN, SBUS_IN_MAX, RC_PWM_MIN, RC_PWM_MAX);
	}

	return true;

}


/*
 * @Fonksiyon: KUSBEGI_RC_Update
 * @Açıklama: Alıcının kontrolleri yapıldıktan sonra
 * gerekli yapılandırmaları yapar
 *
 * @param0-girdi: (KSB_KUSBEGI)KSB_KUSBEGI yapısı
 * @return: (bool)Fonksiyon başarısı
 */
RC_Status KUSBEGI_RC_Update(KSB_KUSBEGI *Kusbegi){
	/*
	 * Önce bağlantı kopmuş mu kontrol edilir.
	 */
	if(Kusbegi->sbus.connection_err){
		return rc_conn_err;
	}

	/*
	 * Bağlantıda problem yoksa sonra failsafe kontrol edilir.
	 */
	if(Kusbegi->sbus.failsafe){
		return rc_failsafe;
	}

	/*
	 * Failsafe de yoksa en son framelost kontrol edilir.
	 */
	if(Kusbegi->sbus.frame_lost){
		return rc_frame_lost;
	}

	/*
	 * Okumada bir problem yok ise önce okunan değerler PWM
	 * sinyal aralığına çevrilir.
	 */
	KUSBEGI_RC_RawToPWM(Kusbegi);



	/*
	 * YAW ve THROTTLE ile arm/disarm işlemi yapılmasını
	 * sağlayan bölüm.
	 *
	 * THROTTLE minimum değerine ve YAW maksimum değerine geldiğinde = ARM
	 * THROTTLE minimum değerine ve YAW minimum değerine geldiğinde = DISARM
	 *
	 * Switch ile arm/disarm'dan bağımsız güvenlik amaçlı her zaman açık olacaktır.
	 */
	if (Kusbegi->rc.rc_channels_PWM[rc_throttle]
			<= (RC_PWM_MIN + RC_ARM_DISARM_TOLARANCE)) {

		if (Kusbegi->rc.rc_channels_PWM[rc_yaw]
				<= (RC_PWM_MIN + RC_ARM_DISARM_TOLARANCE)) {
			Kusbegi->rc.disarm_cnt++;
			if (Kusbegi->rc.disarm_cnt >= RC_STICK_DISARM_CNT) {
				Kusbegi->rc.state_arm = false;
			}
		} else {
			Kusbegi->rc.disarm_cnt = 0;
		}

		if (Kusbegi->rc.rc_channels_PWM[rc_yaw]
				>= (RC_PWM_MAX - RC_ARM_DISARM_TOLARANCE)) {
			Kusbegi->rc.arm_cnt++;
			if (Kusbegi->rc.arm_cnt >= RC_STICK_ARM_CNT) {
/*
 * Eğer kanal ile ARM/DISARM tanımlı ise bu kanalın kontrolü yapılması gerekir.
 * Eğer kanal DISARM pozisyonundaysa çubuk ile ARM edilme inaktif edilmelidir.
 */
#ifdef RC_CHANNEL_ARM
				if (Kusbegi->rc.rc_channels_PWM[rc_arm] <= RC_PWM_MID_VALUE) {
					Kusbegi->rc.state_arm = false;
				} else {
					Kusbegi->rc.state_arm = true;
				}
#endif
#ifndef RC_CHANNEL_ARM
				Kusbegi->rc.state_arm = true;
#endif
			}
		} else {
			Kusbegi->rc.arm_cnt = 0;
		}

	} else {
		Kusbegi->rc.disarm_cnt = 0;
		Kusbegi->rc.arm_cnt = 0;
	}


	/*
	 * Mod güncellemesi kumanda yapısına yazılır.
	 * Doğru modun seçilebilmesi için
	 * Flight_Modes enum içerisinde numaralandırılma yapılması gerekir.
	 */
	Flight_Modes fm_mode;
	if (Kusbegi->rc.rc_channels_PWM[rc_mode] < RC_MODE1_TH)
		fm_mode = 0; // Flight_Modes enum 0
	else if (Kusbegi->rc.rc_channels_PWM[rc_mode] < RC_MODE2_TH)
		fm_mode = 1; // Flight_Modes enum 1
	else if(Kusbegi->rc.rc_channels_PWM[rc_mode] >= RC_MODE2_TH)
		fm_mode = 2; // Flight_Modes enum 2

	Kusbegi->rc.state_mode = fm_mode;


#ifdef RC_CHANNEL_KILL
				if (Kusbegi->rc.rc_channels_PWM[rc_kill_s] <= RC_PWM_MID_VALUE) {
					Kusbegi->status.kill = false;
				} else {
					Kusbegi->status.kill = true;
				}
#endif

	return RC_OK;

}










