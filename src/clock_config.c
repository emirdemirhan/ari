#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/f4/flash.h>
#include "clock_config.h"


clocksetup_err_t clock_config(void)
{
    /* rcc clock config struct */
    static const struct rcc_clock_scale rcc_hse_8mhz_to_100mhz = {
   
    .pllm = 8,    // 8 MHz / 8 = 1 MHz 
    .plln = 200,  // 1 MHz * 200 = 200 MHz (VCO)
    .pllp = 2,    // 200 MHz / 2 = 100 MHz (Sysclk)
    
    .pllq = 2,    // USB
    .pllr = 2,    // I2S
    
    .pll_source = RCC_CFGR_PLLSRC_HSE_CLK, // HSE as pll source
    
    // --- Bus Bölücüleri ---
    .hpre = RCC_CFGR_HPRE_DIV_NONE,  // AHB Prescaler: 1 -> AHB = 100 MHz
    .ppre1 = RCC_CFGR_PPRE_DIV_2,    // APB1 Prescaler: 2 -> APB1 = 50 MHz (Max  50 MHz!)
    .ppre2 = RCC_CFGR_PPRE_DIV_NONE, // APB2 Prescaler: 1 -> APB2 = 100 MHz (Max 100 MHz)
    
    .voltage_scale = PWR_SCALE1,  // Power scale 1 for 100mhz 
    
    // 3.3V 100 MHz 3 Wait State (3WS) is necessary.
    // Instruction cache and Data cache enabled
    .flash_config = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_3WS,

    .ahb_frequency  = 100000000,
    .apb1_frequency = 50000000,
    .apb2_frequency = 100000000,
    
};
    
    rcc_clock_setup_pll(&rcc_hse_8mhz_to_100mhz);  // System clock
    rcc_periph_clock_enable(RCC_GPIOA);     // NRF pin gpio port
    rcc_periph_clock_enable(RCC_GPIOB);     // LED blink gpio port
    rcc_periph_clock_enable(RCC_SPI1);      // SPI communication port for NRF
    rcc_periph_clock_enable(RCC_SPI2);      // SPI communication port for BMI160


    return clocksetup_success;
}