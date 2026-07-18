// PuppyBus master — RS-485 half-duplex (USART6 PC6/PC7) for the Prusa xBuddy extension
//
// Test-grade BLOCKING transceiver. The host (klippy/extras/puppybus.py) builds a
// fully-framed request (address + command + data + CRC16) and calls puppybus_txrx;
// the MCU raises DE (default PB7), transmits, lowers DE, then captures the reply
// into a buffer using a first-byte + inter-byte gap timeout and returns it. ALL
// protocol logic (framing, CRC, bootloader/Modbus state machine) lives host-side.
//
// NOTE: the txrx command handler BLOCKS for up to read_timeout (a few ms in the
// normal case). Keep the steppers idle during a transaction. This is deliberately
// simple to reach first contact; an async/IRQ version comes later if we proceed to
// continuous fan/chamber control.
//
// Copyright (C) 2026
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h"     // CONFIG_MACH_STM32F4
#include "board/misc.h"   // timer_read_time, timer_from_us
#include "command.h"      // DECL_COMMAND, sendf, command_decode_ptr, DIV_ROUND_CLOSEST
#include "gpio.h"         // gpio_out_setup, gpio_out_write
#include "internal.h"     // enable_pclock, get_pclock_frequency, gpio_peripheral, GPIO
#include "sched.h"        // DECL_SHUTDOWN, shutdown

#if !CONFIG_MACH_STM32F4
#error "puppybus.c currently targets STM32F4 (USART6) only"
#endif

#define PB_TX_PIN GPIO('C', 6)   // USART6_TX, AF8
#define PB_RX_PIN GPIO('C', 7)   // USART6_RX, AF8
#define PB_AF     8
// Capped to fit a single Klipper MCU message (payload max ~59 bytes). Ample for
// the bootloader version/hwinfo/start handshake and Modbus register files; larger
// transfers (flash chunks) will be split host-side into <= this many bytes.
#define PB_BUFSIZE 48

static struct gpio_out pb_de;    // RS-485 driver-enable
static struct gpio_out pb_pwr;   // ext_pwr_enable (PG2) — powers the extension port
static struct gpio_out pb_reset; // ext_reset (PG8) — H503 reset (polarity host-given)
static struct gpio_in pb_fault;  // ext_fault (PB6) — active-low port overcurrent fault
static uint8_t pb_de_tx = 1;     // DE level for transmit (1 = high=TX; 0 inverts it)
static uint8_t pb_ready;
static uint8_t pb_buf[PB_BUFSIZE];

static inline void
pb_udelay(uint32_t us)
{
    uint32_t end = timer_read_time() + timer_from_us(us);
    while ((int32_t)(timer_read_time() - end) < 0)
        ;
}

// TX a frame (half-duplex, DE-managed) then capture the reply into pb_buf using a
// first-byte + inter-byte-gap timeout. Returns the number of bytes received.
static uint8_t
pb_transceive(uint8_t *write, uint8_t write_len, uint32_t read_timeout_us,
              uint32_t gap_us)
{
    USART6->CR1 &= ~USART_CR1_RE;          // receiver off during TX
    gpio_out_write(pb_de, pb_de_tx);       // DE -> transmit
    pb_udelay(20);                         // RS-485 driver enable settle
    for (uint8_t i = 0; i < write_len; i++) {
        while (!(USART6->SR & USART_SR_TXE))
            ;
        USART6->DR = write[i];
    }
    while (!(USART6->SR & USART_SR_TC))     // last byte fully shifted out
        ;
    pb_udelay(10);                         // stop-bit settle before releasing the bus
    gpio_out_write(pb_de, !pb_de_tx);      // DE -> receive
    USART6->CR1 |= USART_CR1_RE;
    (void)USART6->SR; (void)USART6->DR;    // clear stale RXNE/ORE/FE

    uint8_t count = 0;
    uint32_t deadline = timer_read_time() + timer_from_us(read_timeout_us);
    for (;;) {
        if (USART6->SR & USART_SR_RXNE) {
            pb_buf[count++] = USART6->DR;
            break;
        }
        if ((int32_t)(timer_read_time() - deadline) >= 0)
            return 0;                      // no response at all
    }
    deadline = timer_read_time() + timer_from_us(gap_us);
    while (count < PB_BUFSIZE) {
        if (USART6->SR & USART_SR_RXNE) {
            pb_buf[count++] = USART6->DR;
            deadline = timer_read_time() + timer_from_us(gap_us);
        } else if ((int32_t)(timer_read_time() - deadline) >= 0) {
            break;
        }
    }
    return count;
}

void
command_config_puppybus(uint32_t *args)
{
    uint32_t de = args[0], baud = args[1], pwr = args[2], de_tx = args[3];
    uint32_t reset = args[4];

    pb_de_tx = de_tx ? 1 : 0;
    pb_de = gpio_out_setup(de, pb_de_tx ? 0 : 1);  // idle = receive = !tx_level
    pb_pwr = gpio_out_setup(pwr, 0); // extension port power off until precharge
    pb_reset = gpio_out_setup(reset, 0); // H503 reset; driven by puppybus_reset_probe
    pb_fault = gpio_in_setup(GPIO('B', 6), 1);  // ext_fault, pull-up (active low)

    enable_pclock((uint32_t)USART6);
    uint32_t pclk = get_pclock_frequency((uint32_t)USART6);
    uint32_t div = DIV_ROUND_CLOSEST(pclk, baud);
    USART6->BRR = (((div / 16) << USART_BRR_DIV_Mantissa_Pos)
                   | ((div % 16) << USART_BRR_DIV_Fraction_Pos));
    USART6->CR2 = 0;
    USART6->CR3 = 0;
    USART6->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;   // 8N1, polled

    gpio_peripheral(PB_RX_PIN, GPIO_FUNCTION(PB_AF), 1);
    gpio_peripheral(PB_TX_PIN, GPIO_FUNCTION(PB_AF), 0);

    pb_ready = 1;
}
DECL_COMMAND(command_config_puppybus,
             "config_puppybus de_pin=%u baud=%u pwr_pin=%u de_tx_level=%u"
             " reset_pin=%u");

// Power the extension port (PG2 / ext_pwr_enable) with a soft-start to limit
// capacitor inrush (mirrors Buddy mmu_port::mmu_soft_start: ~5us on / 70us off
// pulses for precharge_ms), then hold it on. Drive the H503 reset pin (host-side
// [output_pin]) asserted across this, release it after, then probe.
void
command_puppybus_precharge(uint32_t *args)
{
    uint32_t precharge_ms = args[0];
    if (!pb_ready)
        shutdown("puppybus not configured");
    uint32_t iters = (precharge_ms * 1000) / 75;
    for (uint32_t i = 0; i < iters; i++) {
        gpio_out_write(pb_pwr, 1);
        pb_udelay(5);
        gpio_out_write(pb_pwr, 0);
        pb_udelay(70);
    }
    gpio_out_write(pb_pwr, 1);   // hold port power on
}
DECL_COMMAND(command_puppybus_precharge, "puppybus_precharge precharge_ms=%u");

// Set the extension-port power (PG2) directly. on=0 cuts it so PUPPY_REFLASH can power-cycle a
// RUNNING H503 into its bootloader without a FIRMWARE_RESTART; on=1 forces it on (prefer
// puppybus_precharge, which soft-starts to limit inrush). Cutting power is instant (no inrush).
void
command_puppybus_power(uint32_t *args)
{
    if (!pb_ready)
        shutdown("puppybus not configured");
    gpio_out_write(pb_pwr, args[0] ? 1 : 0);
}
DECL_COMMAND(command_puppybus_power, "puppybus_power on=%u");

void
command_puppybus_txrx(uint32_t *args)
{
    uint8_t write_len = args[0];
    uint8_t *write = command_decode_ptr(args[1]);
    uint32_t read_timeout_us = args[2];   // wait this long for the first reply byte
    uint32_t gap_us = args[3];            // an inter-byte gap this long ends the reply

    if (!pb_ready)
        shutdown("puppybus not configured");
    if (write_len > PB_BUFSIZE)
        shutdown("puppybus write too large");

    uint8_t count = pb_transceive(write, write_len, read_timeout_us, gap_us);
    sendf("puppybus_response read=%*s", count, pb_buf);
}
DECL_COMMAND(command_puppybus_txrx,
             "puppybus_txrx write=%*s read_timeout=%u gap=%u");

// Atomic discovery: pulse the H503 reset, then probe IMMEDIATELY (Buddy's
// PuppyBootstrap timing — reset, ~1ms, release, then get_protocolversion with no
// host round-trip in between). reset_assert/reset_run let the host sweep polarity;
// boot_us is the settle after release before the H503 bootloader can receive.
void
command_puppybus_reset_probe(uint32_t *args)
{
    uint32_t reset_assert = args[0], reset_run = args[1];
    uint32_t reset_us = args[2], boot_us = args[3];
    uint8_t write_len = args[4];
    uint8_t *write = command_decode_ptr(args[5]);
    uint32_t read_timeout_us = args[6], gap_us = args[7];

    if (!pb_ready)
        shutdown("puppybus not configured");
    if (write_len > PB_BUFSIZE)
        shutdown("puppybus write too large");

    gpio_out_write(pb_reset, reset_assert);
    pb_udelay(reset_us);
    gpio_out_write(pb_reset, reset_run);
    pb_udelay(boot_us);

    uint8_t count = pb_transceive(write, write_len, read_timeout_us, gap_us);
    sendf("puppybus_reset_response read=%*s", count, pb_buf);
}
DECL_COMMAND(command_puppybus_reset_probe,
             "puppybus_reset_probe reset_assert=%u reset_run=%u reset_us=%u"
             " boot_us=%u write=%*s read_timeout=%u gap=%u");

// Self-test: report USART6 config registers + whether a test byte actually
// clocks out (TC asserts). pclk should be 42000000, brr ~182 (0xB6), cr1 has
// UE|TE|RE = 0x200C, txe=1, tc=1. Confirms the TX peripheral is live without a scope.
void
command_puppybus_debug(uint32_t *args)
{
    if (!pb_ready)
        shutdown("puppybus not configured");
    uint32_t pclk = get_pclock_frequency((uint32_t)USART6);
    uint32_t brr = USART6->BRR;
    uint32_t cr1 = USART6->CR1;
    uint32_t sr0 = USART6->SR;

    USART6->CR1 &= ~USART_CR1_RE;
    gpio_out_write(pb_de, pb_de_tx);
    pb_udelay(5);
    uint32_t txe = (USART6->SR & USART_SR_TXE) ? 1 : 0;
    USART6->DR = 0x55;                      // 0x55 = nice alternating pattern on a scope
    uint32_t tc = 0, t0 = timer_read_time();
    while ((int32_t)(timer_read_time() - (t0 + timer_from_us(2000))) < 0) {
        if (USART6->SR & USART_SR_TC) { tc = 1; break; }
    }
    gpio_out_write(pb_de, !pb_de_tx);
    USART6->CR1 |= USART_CR1_RE;

    // ext_fault (PB6): 1 = OK (pulled up), 0 = port overcurrent fault (power latched off)
    uint32_t fault_ok = gpio_in_read(pb_fault) ? 1 : 0;
    // raw output-driver states: PG2 power (bit2), PG8 reset (bit8), PC13 shutdown (bit13)
    uint32_t gpiog_odr = GPIOG->ODR, gpioc_odr = GPIOC->ODR;

    sendf("puppybus_debug_result pclk=%u brr=%u cr1=%u sr0=%u txe=%u tc=%u"
          " fault_ok=%u gpiog=%u gpioc=%u"
          , pclk, brr, cr1, sr0, txe, tc, fault_ok, gpiog_odr, gpioc_odr);
}
DECL_COMMAND(command_puppybus_debug, "puppybus_debug");

// Report PC6/PC7 GPIO routing: MODER (each pin should be 0b10 = alt-function) and
// AFRL (each should be 8 = AF8/USART6). Confirms the TX signal is actually on PC6.
void
command_puppybus_pininfo(uint32_t *args)
{
    sendf("puppybus_pininfo_result moder=%u afrl=%u", GPIOC->MODER, GPIOC->AFR[0]);
}
DECL_COMMAND(command_puppybus_pininfo, "puppybus_pininfo");

void
puppybus_shutdown(void)
{
    if (pb_ready)
        gpio_out_write(pb_de, !pb_de_tx);  // leave the bus in receive mode
}
DECL_SHUTDOWN(puppybus_shutdown);
