// Code to setup clocks on stm32h5 (initial bring-up target: STM32H503)
//
// The PLL/voltage/flash settings here mirror Prusa's *validated* config for this
// exact chip+board (xBuddy extension), read from the open-source puppy firmware
// (Prusa-Firmware-Buddy src/puppy/xbuddy_extension/hal_clock.cpp): HSE 24 MHz ->
// PLL1 (M12/N240/P2) = 240 MHz SYSCLK, VOS0, flash 5 WS, APB1/2/3 = /8. USB is
// clocked from PLL1Q = 48 MHz (HSE-derived, crystal-accurate, Prusa's validated
// choice); HSI48+CRS is set up below as the self-contained alternative (USBSEL=0).
//
// Copyright (C) 2026
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h"          // CONFIG_CLOCK_FREQ
#include "board/armcm_boot.h"  // armcm_main, VectorTable
#include "board/armcm_reset.h" // try_request_canboot
#include "board/irq.h"         // irq_disable
#include "board/misc.h"        // bootloader_request
#include "command.h"           // DECL_CONSTANT_STR
#include "internal.h"          // enable_pclock
#include "sched.h"             // sched_main

// PLL1: 24 MHz HSE / 12 * 240 / 2 = 240 MHz; /10 = 48 MHz (USB-capable PLL1Q)
#define PLL1M 12
#define PLL1N 240
#define PLL1P 2
#define PLL1Q 10

#define FREQ_PERIPH CONFIG_CLOCK_FREQ   // SYSCLK; APB peripherals are /8 (see TODO)

// Map a peripheral address to its enable bits
struct cline
lookup_clock_line(uint32_t periph_base)
{
    // GPIO ports live on AHB2 (GPIOAEN = bit 0, +1 per port)
    if (periph_base >= GPIOA_BASE && periph_base < GPIOA_BASE + 16 * 0x400) {
        uint32_t bit = 1 << ((periph_base - GPIOA_BASE) / 0x400);
        return (struct cline){ .en = &RCC->AHB2ENR, .rst = NULL, .bit = bit };
    }
    // APB1L peripherals
    if (periph_base == TIM2_BASE)
        return (struct cline){ .en=&RCC->APB1LENR, .bit=RCC_APB1LENR_TIM2EN };
    if (periph_base == TIM3_BASE)
        return (struct cline){ .en=&RCC->APB1LENR, .bit=RCC_APB1LENR_TIM3EN };
    if (periph_base == I2C2_BASE)
        return (struct cline){ .en=&RCC->APB1LENR, .bit=RCC_APB1LENR_I2C2EN };
    if (periph_base == USART2_BASE)
        return (struct cline){ .en=&RCC->APB1LENR, .bit=RCC_APB1LENR_USART2EN };
    if (periph_base == USART3_BASE)
        return (struct cline){ .en=&RCC->APB1LENR, .bit=RCC_APB1LENR_USART3EN };
    if (periph_base == CRS_BASE)
        return (struct cline){ .en=&RCC->APB1LENR, .bit=RCC_APB1LENR_CRSEN };
    // APB2 peripherals
    if (periph_base == TIM1_BASE)
        return (struct cline){ .en=&RCC->APB2ENR, .bit=RCC_APB2ENR_TIM1EN };
    if (periph_base == USART1_BASE)
        return (struct cline){ .en=&RCC->APB2ENR, .bit=RCC_APB2ENR_USART1EN };
    if (periph_base == USB_DRD_FS_BASE)
        return (struct cline){ .en=&RCC->APB2ENR, .bit=RCC_APB2ENR_USBEN };
    // AHB2 peripherals. NB: Klipper's ADC code (stm32h7_adc.c) enables the
    // clock via the ADC *common* base (ADC12_COMMON), not ADC1 -- match both,
    // or the ADC clock never turns on and the first ADC register access stalls
    // the AHB bus (gated peripheral) and hangs the core during config.
    if (periph_base == ADC1_BASE || periph_base == ADC12_COMMON_BASE)
        return (struct cline){ .en=&RCC->AHB2ENR, .bit=RCC_AHB2ENR_ADCEN };
    // unknown peripheral -> .bit=0 makes enable_pclock a no-op
    return (struct cline){ .en=&RCC->APB2ENR, .bit=0 };
}

// Return the frequency of the given peripheral clock
uint32_t
get_pclock_frequency(uint32_t periph_base)
{
    // AHB buses (GPIO, ADC) run at HCLK = SYSCLK (240 MHz); APB1/2/3 run at
    // SYSCLK/8 = 30 MHz. hard_pwm.c applies the APB-timer x2 itself, so timers
    // must report their APB clock here. (APB3 sits above AHB2 in the map but is
    // unused on this board.)
    if (periph_base >= AHB1PERIPH_BASE)
        return CONFIG_CLOCK_FREQ;
    return CONFIG_CLOCK_FREQ / 8;
}

// Enable a GPIO peripheral clock
void
gpio_clock_enable(GPIO_TypeDef *regs)
{
    uint32_t rcc_pos = ((uint32_t)regs - GPIOA_BASE) / 0x400;
    RCC->AHB2ENR |= 1 << rcc_pos;
    RCC->AHB2ENR;
}

#if !CONFIG_STM32_CLOCK_REF_INTERNAL
DECL_CONSTANT_STR("RESERVE_PINS_crystal", "PH0,PH1");
#endif

// Configure SYSCLK = 240 MHz (HSE->PLL1) and the USB 48 MHz clock (HSI48 + CRS)
static void
clock_setup(void)
{
    // Voltage scale 0 (required to sustain 240 MHz), wait until stable
    PWR->VOSCR = (0b11 << PWR_VOSCR_VOS_Pos);
    while (!(PWR->VOSSR & PWR_VOSSR_VOSRDY))
        ;

    // Enable the 24 MHz HSE crystal
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY))
        ;

    // PLL1: HSE source, 2-4 MHz input range, wide VCO; enable P and Q outputs
    RCC->PLL1CFGR = (0b11 << RCC_PLL1CFGR_PLL1SRC_Pos)
                    | (0b01 << RCC_PLL1CFGR_PLL1RGE_Pos)
                    | (PLL1M << RCC_PLL1CFGR_PLL1M_Pos)
                    | RCC_PLL1CFGR_PLL1PEN | RCC_PLL1CFGR_PLL1QEN;
    RCC->PLL1DIVR = ((PLL1N - 1) << RCC_PLL1DIVR_PLL1N_Pos)
                    | ((PLL1P - 1) << RCC_PLL1DIVR_PLL1P_Pos)
                    | ((PLL1Q - 1) << RCC_PLL1DIVR_PLL1Q_Pos);
    RCC->CR |= RCC_CR_PLL1ON;
    while (!(RCC->CR & RCC_CR_PLL1RDY))
        ;

    // APB1/2/3 = SYSCLK/8 (30 MHz), AHB = /1 (240 MHz)
    RCC->CFGR2 = (0b110 << RCC_CFGR2_PPRE1_Pos)
                 | (0b110 << RCC_CFGR2_PPRE2_Pos)
                 | (0b110 << RCC_CFGR2_PPRE3_Pos);

    // Switch SYSCLK to PLL1 (SW = 0b11)
    RCC->CFGR1 = (RCC->CFGR1 & ~RCC_CFGR1_SW_Msk) | (0b11 << RCC_CFGR1_SW_Pos);
    while ((RCC->CFGR1 & RCC_CFGR1_SWS_Msk) != (0b11 << RCC_CFGR1_SWS_Pos))
        ;

    // 48 MHz USB clock: HSI48 auto-trimmed by the CRS against the USB SOF
    RCC->CR |= RCC_CR_HSI48ON;
    while (!(RCC->CR & RCC_CR_HSI48RDY))
        ;
    enable_pclock(CRS_BASE);
    CRS->CR |= CRS_CR_AUTOTRIMEN | CRS_CR_CEN;
    // Select PLL1Q (48 MHz) as the USB kernel clock. USBSEL (RM0492 CCIPR4[5:4]):
    //   0 = HSI48 , 1 = PLL1Q , 2 = PLL2Q , 3 = HSE.
    // PLL1Q (1) = HSE 24MHz / M12 * N240 / Q10 = 48 MHz exactly -> crystal-accurate
    // (meets USB-FS +-0.25% with no CRS dependency) and is Prusa's validated choice.
    // HSI48 (0, trimmed by the CRS above) is the self-contained fallback. (The
    // original value 3 = HSE/24 MHz and could never enumerate.)
    RCC->CCIPR4 = (RCC->CCIPR4 & ~RCC_CCIPR4_USBSEL_Msk)
                  | (1 << RCC_CCIPR4_USBSEL_Pos);
}


/****************************************************************
 * Bootloader
 ****************************************************************/

// Handle USB reboot requests
void
bootloader_request(void)
{
    try_request_canboot();
    dfu_reboot();
}


/****************************************************************
 * Startup
 ****************************************************************/

// Main entry point - called from armcm_boot.c:ResetHandler()
void
armcm_main(void)
{
    SCB->VTOR = (uint32_t)VectorTable;

    // Make sure HSI is the clock source while we (re)configure the tree
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY))
        ;

    dfu_reboot_check();

    // Flash latency for 240 MHz at VOS0 (5 WS + prefetch + WRHIGHFREQ), per RM0492
    FLASH->ACR = FLASH_ACR_PRFTEN | (5 << FLASH_ACR_LATENCY_Pos)
                 | (0b10 << FLASH_ACR_WRHIGHFREQ_Pos);

    clock_setup();

    sched_main();
}
