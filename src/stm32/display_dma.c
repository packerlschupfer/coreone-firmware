// Async SPI6 TX-DMA play-out engine for the ILI9488 LCD (Prusa Core One / F427).
//
// Klipper's generic spi_transfer busy-waits byte-by-byte with no scheduler
// yield, so the flood of small spi_send commands a colour-TFT flush produces
// keeps command_task busy long enough to delay the loadcell's sensor_bulk_status
// response -> MCU shutdown during a probe. Stock Prusa firmware avoids this by
// streaming the panel over SPI6 *DMA* (CPU free) while sampling the loadcell in
// a high-priority ISR.
//
// This module is the Klipper equivalent: a ring of "segments", each either a
// run of bytes to DMA out SPI6 or (when len==0) just a CS/DC level change. The
// host enqueues a whole blit (CASET/RASET/RAMWR + pixels + CS framing) as fast
// memcpys via display_dma_seg and returns immediately; a DMA transfer-complete
// IRQ chains through the segments (applying CS/DC, kicking the next DMA) entirely
// in interrupt context. command_task is freed almost instantly, so the loadcell
// query is answered on time and the display can update even during a probe.
//
// SPI6_TX maps to DMA2 Stream5 Channel1 on the STM32F4 (same stream Prusa uses).
// The ring lives in normal SRAM (DMA2 cannot reach CCM RAM on the F4).
//
// Copyright (C) 2026
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memcpy
#include "board/armcm_boot.h" // armcm_enable_irq
#include "board/irq.h" // irq_disable
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "gpio.h" // spi_setup, gpio_out_setup
#include "internal.h" // SPI6, DMA2_Stream5, GPIO defs (CMSIS)
#include "board/misc.h" // timer_read_time
#include "sched.h" // DECL_SHUTDOWN

#define SEG_MAX 48    // max bytes per segment (Klipper message payload cap)
#define NSEG    512   // ring depth -> 512*48 = 24KB data buffer (SRAM)

// cap on the two command-context spins that wait for the DMA IRQ to drain the
// ring. A single 48-byte segment DMAs in << 1ms and even a full 24KB ring drains in
// tens of ms; 500ms is far above any real flush, so exceeding it means the engine
// has WEDGED (a lost TC IRQ with running=1 -- no self-recovery path). Bound the spin
// and shutdown() with a name, so the failure is attributed instead of a bare watchdog
// reset (and the loadcell starvation this module prevents can't reappear as a hard hang).
#define DD_STALL_US 500000

struct dma_seg {
    uint8_t cs, dc, len;
};

struct display_dma {
    SPI_TypeDef *spi;
    struct gpio_out cs, dc;
    volatile uint16_t head, tail, count; // segment ring (head=producer)
    volatile uint8_t running;            // a DMA transfer is in flight
    struct dma_seg segs[NSEG];
    uint8_t data[NSEG][SEG_MAX];
};

// Single instance (one LCD); the DMA IRQ needs to reach it.
static struct display_dma *dd_active;

#define DD_STREAM DMA2_Stream5

// Begin a DMA transfer of len bytes from buf out the SPI TX register.
static void
dd_start_dma(struct display_dma *d, uint8_t *buf, uint32_t len)
{
    DMA_Stream_TypeDef *s = DD_STREAM;
    s->CR &= ~DMA_SxCR_EN;
    while (s->CR & DMA_SxCR_EN)
        ;
    DMA2->HIFCR = (DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5
                   | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5);
    s->M0AR = (uint32_t)buf;
    s->NDTR = len;
    d->spi->CR2 |= SPI_CR2_TXDMAEN;
    s->CR |= DMA_SxCR_EN;
}

// Advance through the ring: apply CS/DC for the next segment and, if it carries
// data, start its DMA (leaving running=1). Pure GPIO segments are consumed
// inline. Must be called with irq disabled or from the DMA irq.
static void
dd_process(struct display_dma *d)
{
    while (d->count) {
        struct dma_seg *seg = &d->segs[d->tail];
        gpio_out_write(d->cs, seg->cs);
        gpio_out_write(d->dc, seg->dc);
        if (seg->len) {
            d->running = 1;
            dd_start_dma(d, d->data[d->tail], seg->len);
            return;
        }
        d->tail = (d->tail + 1 < NSEG) ? d->tail + 1 : 0;
        d->count--;
    }
    d->running = 0;
}

// DMA transfer-complete: finish the current segment and chain to the next.
void
DMA2_Stream5_IRQHandler(void)
{
    struct display_dma *d = dd_active;
    DMA_Stream_TypeDef *s = DD_STREAM;
    if (DMA2->HISR & DMA_HISR_TCIF5) {
        DMA2->HIFCR = DMA_HIFCR_CTCIF5;
        // The DMA TC fires when the last byte is handed to the SPI, not when it
        // has shifted out. Wait for TXE (byte left DR) AND BSY clear (shift
        // done) BEFORE disabling the stream or letting any CS/DC change follow;
        // otherwise the final byte of a CASET/RASET word is truncated -> a
        // mis-addressed blit -> a garbage block that the cell-diff then locks in.
        while (!(d->spi->SR & SPI_SR_TXE))
            ;
        while (d->spi->SR & SPI_SR_BSY)
            ;
        s->CR &= ~DMA_SxCR_EN;
        while (s->CR & DMA_SxCR_EN)
            ;
        d->spi->CR2 &= ~SPI_CR2_TXDMAEN;
        d->tail = (d->tail + 1 < NSEG) ? d->tail + 1 : 0;
        d->count--;
        dd_process(d);
    } else {
        // Transfer/FIFO error: clear flags, drop this segment, keep going so a
        // back-pressured producer can never wedge.
        DMA2->HIFCR = (DMA_HIFCR_CTEIF5 | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5);
        s->CR &= ~DMA_SxCR_EN;
        while (s->CR & DMA_SxCR_EN)
            ;
        d->spi->CR2 &= ~SPI_CR2_TXDMAEN;
        if (d->count) {
            d->tail = (d->tail + 1 < NSEG) ? d->tail + 1 : 0;
            d->count--;
        }
        dd_process(d);
    }
}

void
command_config_display_dma(uint32_t *args)
{
    struct display_dma *d = oid_alloc(
        args[0], command_config_display_dma, sizeof(*d));
    struct spi_config sc = spi_setup(args[1], args[2], args[3]);
    d->spi = sc.spi;
    spi_prepare(sc);
    d->cs = gpio_out_setup(args[4], 1); // CS idle high
    d->dc = gpio_out_setup(args[5], 1);
    d->head = d->tail = d->count = 0;
    d->running = 0;
    dd_active = d;

    // DMA2 Stream5 Channel1 = SPI6_TX, memory-to-peripheral, byte, MINC, TCIE.
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    RCC->AHB1ENR; // dummy read to ensure the clock is up
    DMA_Stream_TypeDef *s = DD_STREAM;
    s->CR &= ~DMA_SxCR_EN;
    while (s->CR & DMA_SxCR_EN)
        ;
    s->CR = (DMA_SxCR_CHSEL_0 | DMA_SxCR_DIR_0 | DMA_SxCR_MINC
             | DMA_SxCR_PL_0 | DMA_SxCR_TCIE);
    s->FCR = 0; // direct mode (PSIZE==MSIZE==byte)
    s->PAR = (uint32_t)&d->spi->DR;
    armcm_enable_irq(DMA2_Stream5_IRQHandler, DMA2_Stream5_IRQn, 2);
}
DECL_COMMAND(command_config_display_dma,
             "config_display_dma oid=%c spi_bus=%u mode=%u rate=%u"
             " cs_pin=%u dc_pin=%u");

// Enqueue one segment. flags bit0 = CS level, bit1 = DC level (applied before
// the segment's data streams). Returns as soon as the bytes are copied in; the
// IRQ engine plays it out. Back-pressures only when the ring is full.
void
command_display_dma_seg(uint32_t *args)
{
    struct display_dma *d = oid_lookup(args[0], command_config_display_dma);
    uint8_t flags = args[1];
    uint8_t len = args[2];
    uint8_t *data = command_decode_ptr(args[3]);
    if (len > SEG_MAX)
        shutdown("display_dma segment too long");
    uint32_t deadline = timer_read_time() + timer_from_us(DD_STALL_US);
    while (d->count >= NSEG) // ring full: IRQ drains it
        if (!timer_is_before(timer_read_time(), deadline))
            shutdown("display_dma ring stall");
    uint16_t h = d->head;
    if (len)
        memcpy(d->data[h], data, len);
    d->segs[h].cs = flags & 1;
    d->segs[h].dc = (flags >> 1) & 1;
    d->segs[h].len = len;
    irq_disable();
    d->head = (h + 1 < NSEG) ? h + 1 : 0;
    d->count++;
    if (!d->running)
        dd_process(d);
    irq_enable();
}
DECL_COMMAND(command_display_dma_seg,
             "display_dma_seg oid=%c flags=%u data=%*s");

// --- glyph fast path -------------------------------------------------------
//
// A char cell rendered host-side is 8*scale * 16*scale * 3 bytes (3456 at
// scale 3). Shipping those expanded pixels cost 72 display_dma_seg MESSAGES per
// character -- and it is that command-link flood, not the SPI, that keeps
// command_task busy and delays the loadcell's sensor_bulk_status reply. So the
// host now sends the 16-byte MONO bitmap once and the MCU expands it here: one
// message per cell instead of 72.
//
// The expansion still lands in the existing segment ring in SEG_MAX chunks, and
// deliberately adds NO new buffer: a pool big enough to hold whole cells would
// just move the cap (buffering N cells costs N*3456 bytes however it is chunked),
// while costing ~20KB of SRAM. Ring depth stays the flush budget; the link is
// what this removes.
//
// The bitmap is sent per glyph rather than indexed out of an MCU font ROM ON
// PURPOSE: Klipper's display framework lets the host define glyphs at runtime
// (set_glyphs -- our heater/chamber icons), so a ROM would need syncing with the
// host font and would break custom icons.

#define ILI_CASET 0x2a
#define ILI_RASET 0x2b
#define ILI_RAMWR 0x2c
#define GLYPH_SCALE_MAX 3   // host falls back to expanded pixels above this

// Enqueue one segment, copying into the ring's own data slot. Mirrors the
// host-facing producer (same bounded ring-full spin, same irq discipline).
static void
dd_push(struct display_dma *d, uint8_t cs, uint8_t dc,
        const uint8_t *buf, uint8_t len)
{
    uint32_t deadline = timer_read_time() + timer_from_us(DD_STALL_US);
    while (d->count >= NSEG)
        if (!timer_is_before(timer_read_time(), deadline))
            shutdown("display_dma ring stall");
    uint16_t h = d->head;
    if (len)
        memcpy(d->data[h], buf, len);
    d->segs[h].cs = cs;
    d->segs[h].dc = dc;
    d->segs[h].len = len;
    irq_disable();
    d->head = (h + 1 < NSEG) ? h + 1 : 0;
    d->count++;
    if (!d->running)
        dd_process(d);
    irq_enable();
}

// Accumulate pixel bytes and emit full SEG_MAX segments, so a cell costs the
// minimum 3456/48 = 72 data segments rather than one per pixel row.
struct dd_acc {
    struct display_dma *d;
    uint8_t n;
    uint8_t buf[SEG_MAX];
};

static void
dd_acc_flush(struct dd_acc *a)
{
    if (a->n) {
        dd_push(a->d, 0, 1, a->buf, a->n);
        a->n = 0;
    }
}

static void
dd_acc_put(struct dd_acc *a, const uint8_t *src, uint32_t len)
{
    while (len) {
        uint32_t take = SEG_MAX - a->n;
        if (take > len)
            take = len;
        memcpy(a->buf + a->n, src, take);
        a->n += take;
        src += take;
        len -= take;
        if (a->n >= SEG_MAX)
            dd_acc_flush(a);
    }
}

// Expand a 16-byte 8x16 MONO glyph to scaled RGB666 and blit it. Mirrors the
// host's old _render_cell exactly: MSB = leftmost column, each pixel repeated
// `scale` across, each pixel row repeated `scale` down.
void
command_display_dma_glyph(uint32_t *args)
{
    struct display_dma *d = oid_lookup(args[0], command_config_display_dma);
    uint32_t x = args[1], y = args[2], scale = args[3];
    uint32_t fg = args[4], bg = args[5];
    uint8_t blen = args[6];
    uint8_t *bitmap = command_decode_ptr(args[7]);
    if (scale < 1 || scale > GLYPH_SCALE_MAX)
        shutdown("display_dma glyph scale out of range");
    if (blen < 16)
        shutdown("display_dma glyph bitmap too short");

    uint32_t w = 8 * scale, h = 16 * scale;
    uint32_t x1 = x + w - 1, y1 = y + h - 1;
    uint8_t cmd, win[4];

    cmd = ILI_CASET;
    dd_push(d, 0, 0, &cmd, 1);
    win[0] = x >> 8; win[1] = x; win[2] = x1 >> 8; win[3] = x1;
    dd_push(d, 0, 1, win, 4);
    cmd = ILI_RASET;
    dd_push(d, 0, 0, &cmd, 1);
    win[0] = y >> 8; win[1] = y; win[2] = y1 >> 8; win[3] = y1;
    dd_push(d, 0, 1, win, 4);
    cmd = ILI_RAMWR;
    dd_push(d, 0, 0, &cmd, 1);

    uint8_t fgb[3] = { fg >> 16, fg >> 8, fg };
    uint8_t bgb[3] = { bg >> 16, bg >> 8, bg };
    uint8_t row_px[8 * GLYPH_SCALE_MAX * 3];
    struct dd_acc acc = { .d = d, .n = 0 };
    uint32_t row, col, k, r;
    for (row = 0; row < 16; row++) {
        uint8_t rb = bitmap[row];
        uint32_t o = 0;
        for (col = 0; col < 8; col++) {
            uint8_t *c = ((rb >> (7 - col)) & 1) ? fgb : bgb;
            for (k = 0; k < scale; k++) {
                row_px[o++] = c[0];
                row_px[o++] = c[1];
                row_px[o++] = c[2];
            }
        }
        for (r = 0; r < scale; r++)
            dd_acc_put(&acc, row_px, o);
    }
    dd_acc_flush(&acc);
    dd_push(d, 1, 1, NULL, 0);   // CS high
}
DECL_COMMAND(command_display_dma_glyph,
             "display_dma_glyph oid=%c x=%u y=%u scale=%u fg=%u bg=%u"
             " bitmap=%*s");

// Block until the ring has fully drained (used before any non-engine SPI6 access
// e.g. a backlight command, so it never collides with an in-flight flush).
void
command_display_dma_wait(uint32_t *args)
{
    struct display_dma *d = oid_lookup(args[0], command_config_display_dma);
    uint32_t deadline = timer_read_time() + timer_from_us(DD_STALL_US);
    while (d->count)
        if (!timer_is_before(timer_read_time(), deadline))
            shutdown("display_dma drain stall");
}
DECL_COMMAND(command_display_dma_wait, "display_dma_wait oid=%c");

void
display_dma_shutdown(void)
{
    struct display_dma *d = dd_active;
    if (!d)
        return;
    DD_STREAM->CR &= ~DMA_SxCR_EN;
    d->spi->CR2 &= ~SPI_CR2_TXDMAEN;
    d->count = d->head = d->tail = 0;
    d->running = 0;
    gpio_out_write(d->cs, 1); // deselect the panel
}
DECL_SHUTDOWN(display_dma_shutdown);
