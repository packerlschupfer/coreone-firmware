// Definitions for irq enable/disable on ARM Cortex-M processors
//
// Copyright (C) 2017-2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/internal.h" // __CORTEX_M
#include "irq.h" // irqstatus_t
#include "sched.h" // DECL_SHUTDOWN

#if __CORTEX_M >= 3 && !CONFIG_MACH_STM32H5
// BASEPRI-based critical sections (Cortex-M3/M4/M7). The H5/M33 has BASEPRI too but is
// EXCLUDED (!CONFIG_MACH_STM32H5): its IRQ priorities are unaudited for the prio-0 reserve
// and it never phase-steps (F427-only) -> H5 uses the #else PRIMASK branch (its validated
// behaviour).
//
// Klipper's critical sections historically used PRIMASK ("cpsid i"), which masks
// EVERY maskable IRQ regardless of NVIC priority. That makes it impossible to run
// a real-time peripheral IRQ during a critical section. Instead, mask only NVIC
// priorities >= 1 via BASEPRI, leaving priority 0 always-runnable and RESERVED for
// the phase-stepping real-time IRQ (TIM8 + SPI3-TX DMA TC; see src/stm32/phase_exec.c).
//
// SAFETY INVARIANT: every IRQ Klipper relies on for its critical-section data must
// sit at priority >= 1 so it STAYS masked here (identical to the old cpsid behavior).
// Audited on STM32F4: serial + CAN moved 0 -> 1, USB = 1, SysTick(scheduler) = 2.
// Nothing but the phase IRQ may use priority 0. (M0/M0+ have no BASEPRI -> see #else.)
#define IRQ_MASK_BASEPRI (1 << (8 - __NVIC_PRIO_BITS))

void
irq_disable(void)
{
    asm volatile("msr basepri, %0" :: "r"(IRQ_MASK_BASEPRI) : "memory");
}

void
irq_enable(void)
{
    asm volatile("msr basepri, %0" :: "r"(0) : "memory");
}

irqstatus_t
irq_save(void)
{
    irqstatus_t flag;
    asm volatile("mrs %0, basepri" : "=r" (flag) :: "memory");
    irq_disable();
    return flag;
}

void
irq_restore(irqstatus_t flag)
{
    asm volatile("msr basepri, %0" :: "r" (flag) : "memory");
}

void
irq_wait(void)
{
    // Briefly drop BASEPRI to 0 so a pending IRQ (scheduler/comms) is actually
    // taken, then re-mask. (cpsid/cpsie is not used here so PRIMASK stays clear.)
    if (__CORTEX_M == 7)
        // Cortex-m7 may disable cpu counter on wfi, so use nop
        asm volatile("msr basepri, %0\n    nop\n    msr basepri, %1\n"
                     :: "r"(0), "r"(IRQ_MASK_BASEPRI) : "memory");
    else
        asm volatile("msr basepri, %0\n    wfi\n    msr basepri, %1\n"
                     :: "r"(0), "r"(IRQ_MASK_BASEPRI) : "memory");
}

#else // Cortex-M0/M0+ : no BASEPRI register -> keep PRIMASK ("cpsid i")

void
irq_disable(void)
{
    asm volatile("cpsid i" ::: "memory");
}

void
irq_enable(void)
{
    asm volatile("cpsie i" ::: "memory");
}

irqstatus_t
irq_save(void)
{
    irqstatus_t flag;
    asm volatile("mrs %0, primask" : "=r" (flag) :: "memory");
    irq_disable();
    return flag;
}

void
irq_restore(irqstatus_t flag)
{
    asm volatile("msr primask, %0" :: "r" (flag) : "memory");
}

void
irq_wait(void)
{
    asm volatile("cpsie i\n    wfi\n    cpsid i\n" ::: "memory");
}

#endif

void
irq_poll(void)
{
}

// Clear the active irq if a shutdown happened in an irq handler
void
clear_active_irq(void)
{
    uint32_t psr;
    asm volatile("mrs %0, psr" : "=r" (psr));
    if (!(psr & 0x1ff))
        // Shutdown did not occur in an irq - nothing to do.
        return;
    // Clear active irq status
    psr = 1<<24; // T-bit
    uint32_t temp;
    asm volatile(
        "  push { %1 }\n"
        "  adr %0, 1f\n"
        "  push { %0 }\n"
        "  push { r0, r1, r2, r3, r4, lr }\n"
        "  bx %2\n"
        ".balign 4\n"
        "1:\n"
        : "=&r"(temp) : "r"(psr), "r"(0xfffffff9) : "r12", "cc");
}
DECL_SHUTDOWN(clear_active_irq);
