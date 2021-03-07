/*
 * Kusbegi SBUS
 *
 * Yazar: İsmail Emre CANPINAR
 * @canpinaremre
 *
 * Açıklama: Kumanda alıcısının gönderdiği
 * inverted UART mesajları olan SBUS mesajlarını
 * parçalayarak anlamlı verilere dönüştüren fonksiyonu içerir.
 *
 *
 */


#include <Kusbegi_Inc/Kusbegi_SBUS/kusbegi_sbus.h>


uint8_t sbus_error_cnt = 0;


/*
 * Okunan mesajın başlığı doğru ise SBUS protokole uygun
 * olarak parçalanıp kanallara atanır.
 *
 * The SBUS protocol uses an inverted serial logic with a baud rate of 100000, 8 data bits, even parity, and 2 stop bits. The SBUS packet is 25 bytes long consisting of:
 * SBUS protokolü ters çevirilmiş seri haberleşmedir.
 * UART ile okyabilmek için önce sinyali ters çevirmek gerekir. (NOT gate kullanılabilir)
 *
 * Mesajın anlaşılması için gerekenler:
 * BAUD RATE = 100000
 * Data bits = 8
 * Parity = Even
 * Stop bits = 2
 *
 * Mesaj içeriği 25 bitttir:
 *
 * Byte[0] = SBUS Header
 * Byte[1-22] = 16 kanalı temsil ediyor her biri 11 bit.
 * Byte[23]:
 * Bit 7 = kanal 17
 * Bit 6 = kanal 18
 * Bit 5 = frame lost
 * Bit 4 = failsafe activated
 * Byte[24] = SBUS footer
 *
 */
int8_t KUSBEGI_SBUS_ReadSBUS(KSB_KUSBEGI *Kusbegi){

	HAL_UART_Receive_DMA(Kusbegi->com.SBUS, Kusbegi->sbus.msg_raw, 25);

	/*
	 * Gelen SBUS mesajının doğruluğu kontrol edilir.
	 * Eğer yanlış başlık okunuyorsa fonksiyon yanlış döndürür.
	 */
	if (Kusbegi->sbus.msg_raw[0] != SBUS_HEADER_BYTE) {
		/*
		 * Sürekli hatalı mesaj okunması durumu için hata sayacı
		 */
		Kusbegi->sbus.header_check_cnt++;
		if (Kusbegi->sbus.header_check_cnt > SBUS_MAX_HEADER_ERR_CNT) {
			Kusbegi->sbus.connection_err = true;
		}
		return false;
	}
	Kusbegi->sbus.header_check_cnt = 0;
	Kusbegi->sbus.connection_err = false;
	/*
	 * Her bir kanal 11 bit olduğu için
	 * aşağıdaki gibi bir parçalama yapılması gereklidir.
	 * Daha detaylı bilgi: TODO: Github üzerinde tablo oluşturulacak.Linki Buraya konulacak
	 */
	Kusbegi->rc.rc_channels_raw[0] = (((uint16_t) Kusbegi->sbus.msg_raw[1])
			| ((uint16_t) Kusbegi->sbus.msg_raw[2] << 8)) & 0x07FF;
	Kusbegi->rc.rc_channels_raw[1] = (((uint16_t) Kusbegi->sbus.msg_raw[2] >> 3)
			| ((uint16_t) Kusbegi->sbus.msg_raw[3] << 5)) & 0x07FF;
	Kusbegi->rc.rc_channels_raw[2] = (((uint16_t) Kusbegi->sbus.msg_raw[3] >> 6)
			| ((uint16_t) Kusbegi->sbus.msg_raw[4] << 2)
			| ((uint16_t) Kusbegi->sbus.msg_raw[5] << 10)) & 0x07FF;
	Kusbegi->rc.rc_channels_raw[3] = (((uint16_t) Kusbegi->sbus.msg_raw[5] >> 1)
			| ((uint16_t) Kusbegi->sbus.msg_raw[6] << 7)) & 0x07FF;
	Kusbegi->rc.rc_channels_raw[4] = (((uint16_t) Kusbegi->sbus.msg_raw[6] >> 4)
			| ((uint16_t) Kusbegi->sbus.msg_raw[7] << 4)) & 0x07FF;
	Kusbegi->rc.rc_channels_raw[5] = (((uint16_t) Kusbegi->sbus.msg_raw[7] >> 7)
			| ((uint16_t) Kusbegi->sbus.msg_raw[8] << 1)
			| ((uint16_t) Kusbegi->sbus.msg_raw[9] << 9)) & 0x07FF;
	Kusbegi->rc.rc_channels_raw[6] = (((uint16_t) Kusbegi->sbus.msg_raw[9] >> 2)
			| ((uint16_t) Kusbegi->sbus.msg_raw[10] << 6)) & 0x07FF;
	Kusbegi->rc.rc_channels_raw[7] = (((uint16_t) Kusbegi->sbus.msg_raw[10] >> 5)
			| ((uint16_t) Kusbegi->sbus.msg_raw[11] << 3)) & 0x07FF;
	Kusbegi->rc.rc_channels_raw[8] = (((uint16_t) Kusbegi->sbus.msg_raw[12])
			| ((uint16_t) Kusbegi->sbus.msg_raw[13] << 8)) & 0x07FF;
	Kusbegi->rc.rc_channels_raw[9] = (((uint16_t) Kusbegi->sbus.msg_raw[13] >> 3)
			| ((uint16_t) Kusbegi->sbus.msg_raw[14] << 5)) & 0x07FF;
	Kusbegi->rc.rc_channels_raw[10] = (((uint16_t) Kusbegi->sbus.msg_raw[14] >> 6)
			| ((uint16_t) Kusbegi->sbus.msg_raw[15] << 2)
			| ((uint16_t) Kusbegi->sbus.msg_raw[16] << 10)) & 0x07FF;
	Kusbegi->rc.rc_channels_raw[11] = (((uint16_t) Kusbegi->sbus.msg_raw[16] >> 1)
			| ((uint16_t) Kusbegi->sbus.msg_raw[17] << 7)) & 0x07FF;
	Kusbegi->rc.rc_channels_raw[12] = (((uint16_t) Kusbegi->sbus.msg_raw[17] >> 4)
			| ((uint16_t) Kusbegi->sbus.msg_raw[18] << 4)) & 0x07FF;
	Kusbegi->rc.rc_channels_raw[13] = (((uint16_t) Kusbegi->sbus.msg_raw[18] >> 7)
			| ((uint16_t) Kusbegi->sbus.msg_raw[19] << 1)
			| ((uint16_t) Kusbegi->sbus.msg_raw[20] << 9)) & 0x07FF;
	Kusbegi->rc.rc_channels_raw[14] = (((uint16_t) Kusbegi->sbus.msg_raw[20] >> 2)
			| ((uint16_t) Kusbegi->sbus.msg_raw[21] << 6)) & 0x07FF;
	Kusbegi->rc.rc_channels_raw[15] = (((uint16_t) Kusbegi->sbus.msg_raw[21] >> 5)
			| ((uint16_t) Kusbegi->sbus.msg_raw[22] << 3)) & 0x07FF;

	if (Kusbegi->sbus.msg_raw[23] & (1 << 2)) {
		Kusbegi->sbus.frame_lost = true;
	}
	else{
		Kusbegi->sbus.frame_lost = false;
	}

	if (Kusbegi->sbus.msg_raw[23] & (1 << 3)) {
		Kusbegi->sbus.failsafe = true;
	}
	else{
		Kusbegi->sbus.failsafe = false;
	}


	return true;
}
