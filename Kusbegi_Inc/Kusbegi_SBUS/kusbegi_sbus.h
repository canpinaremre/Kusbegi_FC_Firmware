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
#include <Kusbegi_Inc/kusbegi.h>
#include <Kusbegi_Inc/kusbegi_fc_parameters.h>

#ifndef __KUSBEGI_SBUS_H__
#define __KUSBEGI_SBUS_H__

#define SBUS_MAX_HEADER_ERR_CNT 10
#define SBUS_HEADER_BYTE 0x0F


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
int8_t KUSBEGI_SBUS_ReadSBUS(KSB_KUSBEGI *Kusbegi);




#endif /* __KUSBEGI_SBUS_H__ */
