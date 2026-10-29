#ifndef HIL_BRIDGE_H
#define HIL_BRIDGE_H

#include <stdint.h>
#include "simulation/hil_protocol.h"   // HIL_Sim_Data_t, HIL_Fc_Data_t ve çerçeve formatı


/* Hat hızı; derlerken değiştirilebilir: make HIL_BAUD=115200 (simülatörde --baud 115200) */
#ifndef HIL_BAUD
#define HIL_BAUD 2000000
#endif

/* 1 ise LED görevi saniyede bir alım sayaçlarını yazar (make HIL_RX_DIAG=1) */
#ifndef HIL_RX_DIAG
#define HIL_RX_DIAG 0
#endif

/* USART1 (PA9 TX, PA10 RX, HIL_BAUD) ayarlar ve "Hello from STM32" gönderir */
void start_usb_hil_system(void);

/* Alım sayaçlarını tek satır metin olarak yazar:
   "RX bytes=.. fe=.. ne=.. ore=.. drop=.. crc=.. frames=.." (fe/ne/ore: USART çerçeve/gürültü/taşma
   hatası, drop: halka tampon doluydu) */
void hil_print_rx_stats(void);

/* Düz metin gönderir (çerçevesiz) */
void hil_send(const char *str);

/* Yeni bir simülasyon paketi gelene kadar bekler, 1 döndürür.
   Aynı seq ile tekrar gelen paketlere önbellekteki cevabı yeniden gönderir. */
uint8_t hil_receive_full_packet(HIL_Sim_Data_t *data_out);

/* Uçuş kontrolcüsü cevabını (motorlar + telemetri) çerçeveleyip gönderir */
void hil_send_fc_data(const HIL_Fc_Data_t *data);

#endif
