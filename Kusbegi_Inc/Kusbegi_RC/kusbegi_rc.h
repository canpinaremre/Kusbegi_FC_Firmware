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
#include <Kusbegi_Inc/kusbegi.h>
#include <Kusbegi_Inc/kusbegi_fc_parameters.h>


#ifndef __KUSBEGI_RC_H__
#define __KUSBEGI_RC_H__

#define SBUS_IN_MIN		172
#define	SBUS_IN_MAX		1811
#define RC_PWM_MIN 		1000
#define RC_PWM_MAX 		2000
#define RC_MODE1_TH		1350
#define RC_MODE2_TH		1700
#define RC_PWM_MID_VALUE ((RC_PWM_MIN + RC_PWM_MAX)/2)


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
		uint16_t u16_INmax, uint16_t u16_OUTmin, uint16_t u16_OUTmax);

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
		float f_INmax, float f_OUTmin, float f_OUTmax);

/*
 * @Fonksiyon: KUSBEGI_RC_RawToPWM
 * @Açıklama: Okunan SBUS değerlerini PWM değerine ölçekler.
 * @param0-girdi: (KSB_KUSBEGI)KSB_KUSBEGI yapısı
 * @return: (bool)Fonksiyon başarısı
 */
int8_t KUSBEGI_RC_RawToPWM(KSB_KUSBEGI *Kusbegi);



RC_Status KUSBEGI_RC_Update(KSB_KUSBEGI *Kusbegi);


#endif /* __KUSBEGI_RC_H__ */
