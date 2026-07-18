// Support for bit-banging commands to HX711 and HX717 ADC chips
//
// Copyright (C) 2024 Gareth Farrington <gareth@waves.ky>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdbool.h>
#include <stdint.h>
#include "autoconf.h" // CONFIG_MACH_AVR
#include "basecmd.h" // oid_alloc
#include "board/gpio.h" // gpio_out_write
#include "board/irq.h" // irq_poll
#include "board/misc.h" // timer_read_time
#include "command.h" // DECL_COMMAND
#include "sched.h" // sched_add_timer
#include "sensor_bulk.h" // sensor_bulk_report
#include "trigger_analog.h" // trigger_analog_update
#if CONFIG_MACH_STM32F4
// Optional DOUT data-ready EXTI mode (F427): replaces the ~3200/s polling timer
// with a GPIO interrupt that fires only when a sample is ready (~320/s), freeing
// the MCU main loop so high-rate phase-stepping can coexist with the load cell.
#include "board/internal.h" // EXTI, SYSCFG, RCC (CMSIS)
#include "board/armcm_boot.h" // armcm_enable_irq
#endif

struct hx71x_adc {
    struct timer timer;
    uint8_t gain_channel;   // the gain+channel selection (1-4)
    uint8_t flags;
    uint32_t rest_ticks;
    uint32_t last_error;
    struct gpio_in dout; // pin used to receive data from the hx71x
    struct gpio_out sclk; // pin used to generate clock for the hx71x
    struct sensor_bulk sb;
    struct trigger_analog *ta;
    uint32_t dout_pin;   // raw dout pin (for EXTI setup)
    uint8_t irq_line;    // EXTI line (= dout pin % 16); valid only when irq_mode
    uint8_t irq_mode;    // 1 = data-ready EXTI instead of the polling timer
    // --- channel interleave (Prusa HX717Mux 12:1 equivalent) -------------
    // When enabled, the MCU autonomously alternates the HX717 gain/channel so
    // BOTH channel A (loadcell + e-stall feed) and channel B (filament-presence)
    // stay live during a print: one chB sample per il_duty reads, the rest chA.
    // Each reported sample is TAGGED with its channel (HX_TAG_CHB) so the host
    // can demux a single periodic stream. Off by default -> the legacy single-
    // channel path (set by gain_channel) is byte-for-byte unchanged, so probing
    // (which pins channel A and never interleaves) carries no risk.
    uint8_t il_flags;       // HX_IL_ENABLE when interleaving
    uint8_t il_chA;         // gain_channel code for channel A (loadcell, =1)
    uint8_t il_chB;         // gain_channel code for channel B (fsensor, =4)
    uint8_t il_duty;        // emit a chB sample once per this many reads (=13)
    uint8_t il_counter;     // schedule counter
    uint8_t il_cur_ch;      // channel (0=A,1=B) of the sample about to be read
                            // -- pipeline-lagged: set by the PREVIOUS read's
                            // extra clock pulses (same delayed-select as TMC SPI)
    uint8_t il_stale;       // remaining chA samples to flag discard after a B->A
                            // return (Prusa UNDEFINED_SAMPLE_MAX_CNT settling)
};

enum {
    HX_PENDING = 1<<0, HX_OVERFLOW = 1<<1,
};

enum { HX_IL_ENABLE = 1<<0 };

#define BYTES_PER_SAMPLE 4
#define SAMPLE_ERROR_DESYNC 1L << 31
#define SAMPLE_ERROR_READ_TOO_LONG 1L << 30
// Interleave sample tags (top byte of the reported 32-bit word). Only set when
// il_flags & HX_IL_ENABLE; legacy (non-interleave) reports keep the old
// sign-extended-count packing. Errors still use bits 30/31 (above), which a
// tagged value never sets (the value is 24-bit, tags live in bits 24/25).
#define HX_TAG_CHB   (1L << 24)   // sample is channel B (else channel A)
#define HX_TAG_STALE (1L << 25)   // post-switch settling sample -> host discards

static struct task_wake wake_hx71x;


/****************************************************************
 * Low-level bit-banging
 ****************************************************************/

#define MIN_PULSE_TIME nsecs_to_ticks(200)

static uint32_t
nsecs_to_ticks(uint32_t ns)
{
    return timer_from_us(ns * 1000) / 1000000;
}

// Pause for 200ns
static void
hx71x_delay_noirq(void)
{
    if (CONFIG_MACH_AVR) {
        // Optimize avr, as calculating time takes longer than needed delay
        asm("nop\n    nop");
        return;
    }
    uint32_t end = timer_read_time() + MIN_PULSE_TIME;
    while (timer_is_before(timer_read_time(), end))
        ;
}

// Pause for a minimum of 200ns
static void
hx71x_delay(void)
{
    if (CONFIG_MACH_AVR)
        // Optimize avr, as calculating time takes longer than needed delay
        return;
    uint32_t end = timer_read_time() + MIN_PULSE_TIME;
    while (timer_is_before(timer_read_time(), end))
        irq_poll();
}

// Read 'num_bits' from the sensor
static uint32_t
hx71x_raw_read(struct gpio_in dout, struct gpio_out sclk, int num_bits)
{
    uint32_t bits_read = 0;
    while (num_bits--) {
        irq_disable();
        gpio_out_toggle_noirq(sclk);
        hx71x_delay_noirq();
        gpio_out_toggle_noirq(sclk);
        uint_fast8_t bit = gpio_in_read(dout);
        irq_enable();
        hx71x_delay();
        bits_read = (bits_read << 1) | bit;
    }
    return bits_read;
}


/****************************************************************
 * HX711 and HX717 Sensor Support
 ****************************************************************/

// Check if data is ready
static uint_fast8_t
hx71x_is_data_ready(struct hx71x_adc *hx71x)
{
    return !gpio_in_read(hx71x->dout);
}

// Event handler that wakes wake_hx71x() periodically
static uint_fast8_t
hx71x_event(struct timer *timer)
{
    struct hx71x_adc *hx71x = container_of(timer, struct hx71x_adc, timer);
    uint32_t rest_ticks = hx71x->rest_ticks;
    uint8_t flags = hx71x->flags;
    if (flags & HX_PENDING) {
        hx71x->sb.possible_overflows++;
        hx71x->flags = HX_PENDING | HX_OVERFLOW;
        rest_ticks *= 4;
    } else if (hx71x_is_data_ready(hx71x)) {
        // New sample pending
        hx71x->flags = HX_PENDING;
        sched_wake_task(&wake_hx71x);
        rest_ticks *= 8;
    }
    hx71x->timer.waketime += rest_ticks;
    return SF_RESCHEDULE;
}

static void
add_sample(struct hx71x_adc *hx71x, uint8_t oid, uint32_t counts,
                uint8_t force_flush) {
    // Add measurement to buffer
    hx71x->sb.data[hx71x->sb.data_count] = counts;
    hx71x->sb.data[hx71x->sb.data_count + 1] = counts >> 8;
    hx71x->sb.data[hx71x->sb.data_count + 2] = counts >> 16;
    hx71x->sb.data[hx71x->sb.data_count + 3] = counts >> 24;
    hx71x->sb.data_count += BYTES_PER_SAMPLE;

    if (hx71x->sb.data_count + BYTES_PER_SAMPLE > ARRAY_SIZE(hx71x->sb.data)
        || force_flush)
        sensor_bulk_report(&hx71x->sb, oid);
}

// hx71x ADC query
static void
hx71x_read_adc(struct hx71x_adc *hx71x, uint8_t oid)
{
    // Decide this read's gain/channel code (= number of extra clock pulses,
    // which selects the channel of the NEXT conversion -- the HX71x select is
    // pipelined one sample deep, exactly like the TMC SPI delayed read). In
    // interleave mode the schedule overrides the static gain_channel and tags
    // each sample with the channel its DATA actually came from.
    uint_fast8_t gain_channel;
    uint8_t il = hx71x->il_flags & HX_IL_ENABLE;
    uint8_t data_ch = 0;        // channel of THIS sample's data (0=A, 1=B)
    uint8_t stale = 0;
    if (il) {
        data_ch = hx71x->il_cur_ch;     // set by the previous read's pulses
        if (data_ch) {
            // this is the (single) chB sample -> the next chA samples need to
            // settle: discard the first il_duty-independent UNDEFINED count.
            hx71x->il_stale = 2;
        } else if (hx71x->il_stale) {
            stale = 1;
            hx71x->il_stale--;
        }
        // Schedule the NEXT sample: chB once per il_duty reads, else chA.
        uint8_t next_ch = 0;
        if (++hx71x->il_counter >= hx71x->il_duty) {
            hx71x->il_counter = 0;
            next_ch = 1;
        }
        gain_channel = next_ch ? hx71x->il_chB : hx71x->il_chA;
        hx71x->il_cur_ch = next_ch;
    } else {
        gain_channel = hx71x->gain_channel;
    }

    // Read from sensor (24 data bits + gain_channel extra pulses)
    uint32_t adc = hx71x_raw_read(hx71x->dout, hx71x->sclk, 24 + gain_channel);

    // Clear pending flag (and note if an overflow occurred)
    irq_disable();
    uint8_t flags = hx71x->flags;
    hx71x->flags = 0;
    irq_enable();

    // Extract the clean sign-extended count (used for the MCU-side probe
    // trigger and, in legacy mode, reported as-is).
    uint32_t counts = adc >> gain_channel;
    if (counts & 0x800000)
        counts |= 0xFF000000;

    // Check for errors
    uint_fast8_t extras_mask = (1 << gain_channel) - 1;
    if ((adc & extras_mask) != extras_mask) {
        // Transfer did not complete correctly
        hx71x->last_error = SAMPLE_ERROR_DESYNC;
    } else if (flags & HX_OVERFLOW) {
        // Transfer took too long
        hx71x->last_error = SAMPLE_ERROR_READ_TOO_LONG;
    }

    // Build the reported word. Errors override (sticky until reset). Otherwise
    // legacy mode reports the sign-extended count unchanged; interleave mode
    // packs a 24-bit value + channel/stale tags (see HX_TAG_*).
    uint32_t report;
    if (hx71x->last_error != 0) {
        report = hx71x->last_error;
    } else {
        // probe trigger only ever runs in single-channel mode (interleave is
        // disabled during a probe session); feed it chA data only, never chB.
        if (!il || !data_ch)
            trigger_analog_update(hx71x->ta, counts);
        if (il)
            report = (counts & 0xFFFFFF)
                     | (data_ch ? HX_TAG_CHB : 0)
                     | (stale ? HX_TAG_STALE : 0);
        else
            report = counts;
    }

    // Add measurement to buffer
    add_sample(hx71x, oid, report, false);
}

// Create a hx71x sensor
void
command_config_hx71x(uint32_t *args)
{
    struct hx71x_adc *hx71x = oid_alloc(args[0]
                , command_config_hx71x, sizeof(*hx71x));
    hx71x->timer.func = hx71x_event;
    uint8_t gain_channel = args[1];
    if (gain_channel < 1 || gain_channel > 4) {
        shutdown("HX71x gain/channel out of range 1-4");
    }
    hx71x->gain_channel = gain_channel;
    hx71x->dout_pin = args[2];
    hx71x->dout = gpio_in_setup(args[2], 1);
    hx71x->sclk = gpio_out_setup(args[3], 0);
    gpio_out_write(hx71x->sclk, 1); // put chip in power down state
}
DECL_COMMAND(command_config_hx71x, "config_hx71x oid=%c gain_channel=%c"
             " dout_pin=%u sclk_pin=%u");

#if CONFIG_MACH_STM32F4
// ---- Optional DOUT data-ready EXTI (F427) ---------------------------------
// The HX711/HX717 pulls DOUT low when a conversion is ready. Hook that as a
// falling-edge GPIO interrupt instead of polling it from a timer: the IRQ fires
// ~320/s (the sample rate) vs the timer's ~3200/s, removing scheduler load so
// high-rate phase-stepping no longer starves the MCU main loop (-> the load
// cell's host bulk-status query keeps getting answered). The read itself still
// runs in hx71x_capture_task; the IRQ only flags + wakes it. One device only.
static struct hx71x_adc *hx71x_irq_dev;

void
EXTI9_5_IRQHandler(void)
{
    struct hx71x_adc *hx = hx71x_irq_dev;
    if (!hx)
        return;
    uint32_t bit = 1 << hx->irq_line;
    if (EXTI->PR & bit) {
        EXTI->PR = bit;                    // clear pending (write-1-to-clear)
        if (hx->rest_ticks && !(hx->flags & HX_PENDING)) {
            hx->flags = HX_PENDING;
            sched_wake_task(&wake_hx71x);
        }
    }
}

void
command_config_hx71x_irq(uint32_t *args)
{
    struct hx71x_adc *hx71x = oid_lookup(args[0], command_config_hx71x);
    uint32_t pin = hx71x->dout_pin;
    uint32_t port = GPIO2PORT(pin);
    uint32_t line = pin % 16;
    if (line < 5 || line > 9)
        shutdown("hx71x data-ready IRQ needs a dout pin on EXTI line 5-9");
    hx71x->irq_line = line;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    RCC->APB2ENR;                          // ensure the clock is up
    uint32_t idx = line >> 2, sh = (line & 3) * 4;
    SYSCFG->EXTICR[idx] = (SYSCFG->EXTICR[idx] & ~(0xf << sh)) | (port << sh);
    uint32_t bit = 1 << line;
    EXTI->RTSR &= ~bit;                    // data-ready = DOUT falling edge only
    EXTI->FTSR |= bit;
    EXTI->PR = bit;                        // clear any stale pending
    EXTI->IMR &= ~bit;                     // stay masked until a query starts
    hx71x_irq_dev = hx71x;
    hx71x->irq_mode = 1;
    armcm_enable_irq(EXTI9_5_IRQHandler, EXTI9_5_IRQn, 2);
}
DECL_COMMAND(command_config_hx71x_irq, "config_hx71x_irq oid=%c");
#endif

void
hx71x_attach_trigger_analog(uint32_t *args) {
    uint8_t oid = args[0];
    struct hx71x_adc *hx71x = oid_lookup(oid, command_config_hx71x);
    hx71x->ta = trigger_analog_oid_lookup(args[1]);
}
#if CONFIG_WANT_TRIGGER_ANALOG
DECL_COMMAND(hx71x_attach_trigger_analog, "hx71x_attach_trigger_analog oid=%c"
    " trigger_analog_oid=%c");
#endif

// start/stop capturing ADC data
void
command_query_hx71x(uint32_t *args)
{
    uint8_t oid = args[0];
    struct hx71x_adc *hx71x = oid_lookup(oid, command_config_hx71x);
    if (!hx71x->irq_mode)
        sched_del_timer(&hx71x->timer);
    hx71x->flags = 0;
    hx71x->last_error = 0;
    hx71x->rest_ticks = args[1];
    // gain_channel selects channel A (loadcell) vs B (filament sensor) at runtime;
    // switching it here keeps it atomic with the stream stop/restart below.
    uint8_t gain_channel = args[2];
    if (gain_channel < 1 || gain_channel > 4)
        shutdown("HX71x gain/channel out of range 1-4");
    hx71x->gain_channel = gain_channel;
    if (!hx71x->rest_ticks) {
        // End measurements
#if CONFIG_MACH_STM32F4
        if (hx71x->irq_mode)
            EXTI->IMR &= ~(1 << hx71x->irq_line);   // disable data-ready IRQ
#endif
        gpio_out_write(hx71x->sclk, 1); // put chip in power down state
        return;
    }
    // Start new measurements
    gpio_out_write(hx71x->sclk, 0); // wake chip from power down
    sensor_bulk_reset(&hx71x->sb);
#if CONFIG_MACH_STM32F4
    if (hx71x->irq_mode) {
        // Data-ready EXTI mode: arm the line; catch a sample already pending (the
        // edge may have happened while masked -> DOUT is a level, not an edge).
        uint32_t bit = 1 << hx71x->irq_line;
        irq_disable();
        EXTI->PR = bit;
        EXTI->IMR |= bit;
        if (hx71x_is_data_ready(hx71x)) {
            hx71x->flags = HX_PENDING;
            sched_wake_task(&wake_hx71x);
        }
        irq_enable();
        return;
    }
#endif
    irq_disable();
    hx71x->timer.waketime = timer_read_time() + hx71x->rest_ticks;
    sched_add_timer(&hx71x->timer);
    irq_enable();
}
DECL_COMMAND(command_query_hx71x, "query_hx71x oid=%c rest_ticks=%u"
             " gain_channel=%c");

// Enable/disable autonomous channel interleave (Prusa HX717Mux equivalent).
// enable=0 -> revert to the static gain_channel set by query_hx71x. enable=1 ->
// alternate chA/chB on a duty:1 schedule, tagging each reported sample. The host
// must pin channel A (query_hx71x gain_channel=chA) BEFORE enabling so il_cur_ch
// starts coherent; the leading samples are flagged stale to cover the entry.
void
command_hx71x_interleave(uint32_t *args)
{
    struct hx71x_adc *hx71x = oid_lookup(args[0], command_config_hx71x);
    if (!args[1]) {                     // disable
        hx71x->il_flags = 0;
        return;
    }
    uint8_t chA = args[2], chB = args[3], duty = args[4];
    if (chA < 1 || chA > 4 || chB < 1 || chB > 4 || duty < 2)
        shutdown("HX71x interleave params out of range");
    hx71x->il_chA = chA;
    hx71x->il_chB = chB;
    hx71x->il_duty = duty;
    hx71x->il_counter = 0;
    hx71x->il_cur_ch = 0;               // host pinned chA before enabling
    hx71x->il_stale = 2;               // flush the transition into interleave
    hx71x->il_flags = HX_IL_ENABLE;
}
DECL_COMMAND(command_hx71x_interleave, "hx71x_interleave oid=%c enable=%c"
             " chA_gc=%c chB_gc=%c duty=%c");

void
command_query_hx71x_status(const uint32_t *args)
{
    uint8_t oid = args[0];
    struct hx71x_adc *hx71x = oid_lookup(oid, command_config_hx71x);
    irq_disable();
    const uint32_t start_t = timer_read_time();
    uint8_t is_data_ready = hx71x_is_data_ready(hx71x);
    irq_enable();
    uint8_t pending_bytes = is_data_ready ? BYTES_PER_SAMPLE : 0;
    sensor_bulk_status(&hx71x->sb, oid, start_t, 0, pending_bytes);
}
DECL_COMMAND(command_query_hx71x_status, "query_hx71x_status oid=%c");

// Background task that performs measurements
void
hx71x_capture_task(void)
{
    if (!sched_check_wake(&wake_hx71x))
        return;
    uint8_t oid;
    struct hx71x_adc *hx71x;
    foreach_oid(oid, hx71x, command_config_hx71x) {
        if (!hx71x->flags)
            continue;
#if CONFIG_MACH_STM32F4
        if (hx71x->irq_mode) {
            uint32_t bit = 1 << hx71x->irq_line;
            EXTI->IMR &= ~bit;           // mask: clocking SCK toggles DOUT
            hx71x_read_adc(hx71x, oid);
            EXTI->PR = bit;             // drop spurious pending from the read
            EXTI->IMR |= bit;
            if (hx71x->rest_ticks && hx71x_is_data_ready(hx71x)) {
                // next sample already low while we were masked -> level, no edge
                irq_disable();
                hx71x->flags = HX_PENDING;
                sched_wake_task(&wake_hx71x);
                irq_enable();
            }
            continue;
        }
#endif
        hx71x_read_adc(hx71x, oid);
    }
}
DECL_TASK(hx71x_capture_task);
