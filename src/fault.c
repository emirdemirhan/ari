/*
 * Hata raporlama: hard fault, FreeRTOS yığın taşması, assert ve bellek ayırma hatası.
 *
 * Kart takılırsa sebebini USART1 üzerinden (HIL hattı, 2 Mbps) yarım saniyede bir yazar ve
 * LED'i hızlı yakıp söndürür. Simülatör bu metni yakalayıp "LINK LOST" mesajında gösterir.
 * Örnek: "ARI HARDFAULT pc=0x08001234 lr=0x08004567 cfsr=0x00000400"
 */

#include <stdint.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/scb.h>
#include "FreeRTOS.h"
#include "task.h"
#include "peripherals.h"

static void fault_puts(const char *s)
{
    while (*s) {
        usart_send_blocking(USART1, (uint8_t)*s++);
    }
}

static void fault_hex(uint32_t v)
{
    char buf[11] = "0x00000000";
    for (int i = 0; i < 8; i++) {
        uint32_t nib = (v >> (28 - 4 * i)) & 0xF;
        buf[2 + i] = (char)(nib < 10 ? '0' + nib : 'a' + nib - 10);
    }
    fault_puts(buf);
}

static void fault_dec(uint32_t v)
{
    char buf[11];
    int i = 10;
    buf[i] = '\0';
    do {
        buf[--i] = (char)('0' + v % 10);
        v /= 10;
    } while (v && i > 0);
    fault_puts(&buf[i]);
}

/* Bir daha dönmez: mesajı tekrar tekrar gönderir, LED'i hızlı yakıp söndürür */
static void __attribute__((noreturn)) fault_halt(const char *what, const char *detail,
                                                 uint32_t pc, uint32_t lr, uint32_t cfsr, uint32_t line)
{
    for (;;) {
        fault_puts("\r\nARI ");
        fault_puts(what);
        if (detail) {
            fault_puts(" ");
            fault_puts(detail);
        }
        if (line) {
            fault_puts(":");
            fault_dec(line);
        }
        if (pc || lr || cfsr) {
            fault_puts(" pc=");
            fault_hex(pc);
            fault_puts(" lr=");
            fault_hex(lr);
            fault_puts(" cfsr=");
            fault_hex(cfsr);
        }
        fault_puts("\r\n");

        for (int k = 0; k < 10; k++) {
            gpio_toggle(LED_PORT, LED_PIN);
            for (volatile uint32_t d = 0; d < 500000; d++) { }   // ~50 ms, kesmeler kapalı olabilir
        }
    }
}

/* Hard fault: yığına kaydedilen çerçeveden hatanın olduğu adres (pc) ve dönüş adresi (lr) */
void hard_fault_report(uint32_t *frame);
void hard_fault_report(uint32_t *frame)
{
    fault_halt("HARDFAULT", 0, frame[6], frame[5], SCB_CFSR, 0);
}

/* libopencm3'teki zayıf (weak) hard_fault_handler yerine geçer */
void __attribute__((naked)) hard_fault_handler(void)
{
    __asm volatile(
        "tst lr, #4      \n"
        "ite eq          \n"
        "mrseq r0, msp   \n"
        "mrsne r0, psp   \n"
        "b hard_fault_report \n");
}

/* configCHECK_FOR_STACK_OVERFLOW */
void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    fault_halt("STACK OVERFLOW", name, 0, 0, 0, 0);
}

/* configUSE_MALLOC_FAILED_HOOK */
void vApplicationMallocFailedHook(void)
{
    fault_halt("MALLOC FAILED", 0, 0, 0, 0, 0);
}

/* configASSERT */
void ari_assert_failed(const char *file, unsigned long line)
{
    fault_halt("ASSERT", file, 0, 0, 0, line);
}
