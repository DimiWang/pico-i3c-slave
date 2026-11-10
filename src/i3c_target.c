/*
 * i3c_target - protocol state machine of the I3C SDR target.
 *
 * How it works
 * ------------
 * The PIO bit engine (i3c_target.pio) executes "slots": a group of bits that
 * we either receive or drive (address byte, data byte + T-bit, ACK, ...).
 * This file decides, slot by slot, what the next slots are, and interprets
 * the bits that come back:
 *
 *   - Every slot that returns a word gets a tag pushed on `tags`, so each word
 *     read from the RX FIFO is interpreted with the tag of the slot that
 *     produced it (FIFO order == slot order).
 *   - Slots are queued in software (`pend`) and moved into the 4-deep PIO TX
 *     FIFO by pump(). Bytes we will *receive* are always queued in advance
 *     (two RX9 slots), so the PIO never waits for the CPU there.
 *   - The only places where the CPU is on the critical path are the
 *     ACK/NACK decisions after an address byte (decided between the 8th SCL
 *     rising edge and the controller's ACK sample point) and re-arming after
 *     a repeated START. See README "Speed".
 *   - START/Sr/STOP come from a separate PIO state machine. Before acting on
 *     a condition all words already received are processed, because they
 *     happened before it on the bus.
 *
 * I3C SDR rules implemented here (MIPI I3C Basic v1.1.1, section 5.1):
 *   - Address phase after START/Sr: 7 address bits + RnW, then ACK (target
 *     pulls SDA low, open drain) or NACK (leaves SDA high).
 *   - 0x7E/W is the broadcast address: every I3C target ACKs it. It is
 *     followed by a CCC code byte, or by Sr (I3CBlaster always sends 0x7E/W
 *     first, then Sr + target address, as an arbitration header).
 *   - Controller-written bytes are 8 bits + T-bit, T = odd parity.
 *   - Target-read bytes are 8 bits + T-bit, T = 1 "more data", T = 0 "end".
 *     T=1 is handed over to the controller on the SCL rising edge so it can
 *     abort with Sr (see the tbit slot).
 *   - Broadcast CCC ends at Sr/P. Direct CCC continues across Sr (one or
 *     more Sr + target address segments) until P or Sr + 0x7E/W.
 *   - ENTDAA: Sr + 0x7E/R, ACK, then 48-bit PID + BCR + DCR sent MSB first
 *     in open drain with bitwise arbitration (lowest value wins), then the
 *     controller sends 7-bit DA + odd parity bit, the winner ACKs.
 *
 * SPDX-License-Identifier: MIT
 */
#include "i3c_target.h"
#include "i3c_ccc.h"
#include "i3c_log.h"
#include "i3c_phy.h"

#include <string.h>

/* ------------------------------------------------------------------------- */
/* Tags for words returned by the bit engine                                 */
/* ------------------------------------------------------------------------- */
enum {
    TAG_NONE = 0,     /* slot returns no word (tbit)                          */
    TAG_IGNORE,       /* ACK / NACK / release slots                           */
    TAG_ADDR,         /* 8 bits: address + RnW after START/Sr                 */
    TAG_CCC,          /* 9 bits: CCC code + T after 0x7E/W                    */
    TAG_WDATA,        /* 9 bits: written data byte + T                        */
    TAG_TXBYTE,       /* 8 bits: a byte we transmitted (push-pull)            */
    TAG_PID,          /* 8 bits: an ENTDAA arbitration byte                   */
    TAG_DAA_ADDR,     /* 8 bits: ENTDAA assigned address + parity             */
    TAG_I2C_WDATA,    /* 9 bits: legacy I2C byte + our ACK                    */
    TAG_I2C_TXBYTE,   /* 8 bits: legacy I2C byte we sent (open drain)         */
    TAG_I2C_MACK,     /* 1 bit : controller ACK/NACK after an I2C read byte   */
};

/* What the current segment (START/Sr .. Sr/P) is about */
enum {
    SEG_NONE = 0,
    SEG_CCC_HDR,      /* 0x7E/W [+ CCC code + data]                           */
    SEG_DIRECT,       /* direct-CCC segment addressed to us                   */
    SEG_PRIV_W, SEG_PRIV_R,
    SEG_I2C_W, SEG_I2C_R,
    SEG_ENTDAA,       /* one ENTDAA round we take part in                     */
};

/* Interpretation of TAG_WDATA bytes */
enum { CTX_IGNORE = 0, CTX_CCC_BC, CTX_CCC_DIR_W, CTX_PRIV_W };

/* Response source for reads */
enum { RD_NONE = 0, RD_MEM, RD_BUF };

#define PEND_SIZE 32u   /* power of 2 */
#define TAG_SIZE  64u   /* power of 2 */

volatile uint8_t i3c_mem[I3C_MEM_SIZE];

static struct {
    /* device state */
    uint8_t  da;
    uint8_t  events;
    uint8_t  ptr;
    uint16_t mwl, mrl;
    bool     proto_err;           /* reported (and cleared) by GETSTATUS     */

    /* frame state */
    bool     armed_start;         /* engine pre-armed for the next START      */
    bool     first_after_start;   /* next address follows START, not Sr       */
    bool     ccc_active;          /* a CCC is in progress                     */
    uint8_t  ccc;
    bool     entdaa;              /* ENTDAA seen in this frame                */
    bool     i2c_frame;           /* frame started as legacy I2C              */
    bool     hdr;                 /* ignoring an HDR mode transfer            */
    uint32_t hdr_idle_t0;
    bool     hdr_idle;

    /* segment state */
    uint8_t  seg;
    uint8_t  ctx;
    uint8_t  seg_addr;
    bool     wr_first;            /* first private write byte = pointer       */
    uint8_t  rd_start;
    uint16_t seg_len;             /* bytes on the bus in this segment         */
    uint8_t  logbuf[I3C_LOG_MAX_DATA];
    bool     halt_seen;
    bool     entdaa_lost;
    uint8_t  pid_bytes;

    /* read producer */
    uint8_t  rd_src;
    uint16_t rd_idx, rd_len;
    uint8_t  rd_buf[8];

    /* slot queue (software) + expected-word tags */
    uint32_t pend[PEND_SIZE];
    uint8_t  pend_tag[PEND_SIZE];
    uint32_t pend_head, pend_tail;
    uint8_t  tags[TAG_SIZE];
    uint32_t tag_head, tag_tail;

    /* pending "dynamic address changed" log line */
    uint8_t  da_log_reason, da_log_old;
    uint16_t da_log_len;

    /* statistics */
    uint32_t frames, naks, parity_errors, entdaa_lost_cnt;
} t;

static uint8_t identity[8];   /* PID[47:40] .. PID[7:0], BCR, DCR */

/* Slot commands are pre-encoded at init so that the time-critical paths
 * (ACK/NACK after an address) only copy a word into the PIO FIFO.          */
static uint32_t s_ack, s_nack, s_rx1, s_rx8, s_rx9, s_i2c_rx_ack;
static uint32_t s_tbit_more, s_tbit_last;
static uint32_t s_arb_id[8];
static uint32_t s_tx8[256];

/* ------------------------------------------------------------------------- */
/* Small helpers                                                             */
/* ------------------------------------------------------------------------- */

static inline bool odd_parity_ok(uint8_t byte, uint32_t t_bit)
{
    /* the 8 data bits plus T must contain an odd number of ones */
    return ((uint32_t)__builtin_parity(byte) ^ (t_bit & 1u)) == 1u;
}

static void log_rec(uint8_t type, uint8_t addr, uint8_t ccc, uint8_t arg,
                    const uint8_t *data, uint16_t len)
{
    i3c_log_rec_t r;
    r.t_us = phy_time_us();
    r.type = type;
    r.addr = addr;
    r.ccc  = ccc;
    r.arg  = arg;
    r.len  = len;
    uint16_t n = len < I3C_LOG_MAX_DATA ? len : I3C_LOG_MAX_DATA;
    if (data && n) memcpy(r.data, data, n);
    i3c_log_push(&r);
}

static void seg_append(uint8_t b)
{
    if (t.seg_len < I3C_LOG_MAX_DATA) t.logbuf[t.seg_len] = b;
    t.seg_len++;
}

/* Change the dynamic address now; the log line is written at the end of the
 * segment so it appears after the CCC that caused it.                      */
static void set_da(uint8_t new_da, uint8_t reason, const uint8_t *data, uint16_t len)
{
    if (!t.da_log_reason) t.da_log_old = t.da;
    t.da = new_da;
    t.da_log_reason = reason;
    t.da_log_len = data ? len : 0;
}

static void flush_da_log(void)
{
    if (!t.da_log_reason) return;
    log_rec(EV_DA, t.da, t.da_log_old, t.da_log_reason, t.da_log_len ? identity : NULL, t.da_log_len);
    t.da_log_reason = 0;
}

static bool da_valid(uint8_t a)
{
    /* 0x7E is broadcast; 0x00..0x07 and 0x78..0x7F are reserved by I2C/I3C;
     * the single-bit-error neighbours of 0x7E must never be used either.   */
    if (a < 0x08 || a > 0x77) return false;
    switch (a) {
    case 0x3E: case 0x5E: case 0x6E: case 0x76: return false;
    default: return true;
    }
}

/* queue a slot; tag != TAG_NONE means the slot returns one word */
static void q(uint32_t slot, uint8_t tag)
{
    uint32_t i = t.pend_head & (PEND_SIZE - 1u);
    t.pend[i] = slot;
    t.pend_tag[i] = tag;
    t.pend_head++;
}

static inline uint32_t pend_count(void) { return t.pend_head - t.pend_tail; }

/* Move queued slots into the PIO FIFO and produce read data on demand. */
static void pump(void)
{
    for (;;) {
        if (pend_count() == 0) {
            if (t.rd_src == RD_NONE || t.rd_idx >= t.rd_len || t.halt_seen)
                return;
            /* next byte of the current read + its T-bit */
            uint8_t b = (t.rd_src == RD_MEM)
                        ? i3c_mem[(uint8_t)(t.rd_start + t.rd_idx)]
                        : t.rd_buf[t.rd_idx];
            bool more = (t.rd_idx + 1u) < t.rd_len;
            if (t.seg == SEG_I2C_R) {
                /* legacy I2C: open-drain byte, then the controller's ACK.
                 * The next byte is only produced after an ACK.            */
                q(slot_arb(8, b), TAG_I2C_TXBYTE);
                q(s_rx1, TAG_I2C_MACK);
                t.rd_src = RD_NONE;
            } else {
                q(s_tx8[b], TAG_TXBYTE);
                q((more ? s_tbit_more : s_tbit_last), TAG_NONE);
            }
            t.rd_idx++;
        }
        if (phy_tx_full())
            return;
        uint32_t i = t.pend_tail & (PEND_SIZE - 1u);
        if (t.pend_tag[i] != TAG_NONE) {
            t.tags[t.tag_head & (TAG_SIZE - 1u)] = t.pend_tag[i];
            t.tag_head++;
        }
        phy_tx_put(t.pend[i]);
        t.pend_tail++;
    }
}

static void start_read(uint8_t src, uint8_t start, uint16_t len)
{
    t.rd_src = src;
    t.rd_start = start;
    t.rd_idx = 0;
    t.rd_len = len;
}

/* Restart the bit engine with nothing queued. */
static void engine_reset(void)
{
    phy_reset();
    t.pend_head = t.pend_tail = 0;
    t.tag_head = t.tag_tail = 0;
    t.rd_src = RD_NONE;
    t.halt_seen = false;
}

static void seg_begin(uint8_t seg, uint8_t addr)
{
    t.seg = seg;
    t.seg_addr = addr;
    t.seg_len = 0;
    t.entdaa_lost = false;
    t.pid_bytes = 0;
}

/* ------------------------------------------------------------------------- */
/* End of a segment (Sr or P): apply delayed effects and write the log       */
/* ------------------------------------------------------------------------- */
static void seg_end(void)
{
    uint16_t n = t.seg_len;
    switch (t.seg) {
    case SEG_CCC_HDR:
        if (!t.ccc_active) break;                    /* only an arb header   */
        if (t.ccc < 0x80) log_rec(EV_CCC_BC, I3C_BCAST_ADDR, t.ccc, 0, t.logbuf, n);
        else              log_rec(EV_CCC_DIRECT, I3C_BCAST_ADDR, t.ccc, 0, t.logbuf, n);
        break;
    case SEG_DIRECT:
        log_rec(t.ctx == CTX_CCC_DIR_W || t.rd_len == 0 ? EV_CCC_DIR_W : EV_CCC_DIR_R,
                t.seg_addr, t.ccc, 0, t.logbuf, n);
        break;
    case SEG_PRIV_W:
        log_rec(EV_PRIV_W, t.seg_addr, 0, 0, t.logbuf, n);
        break;
    case SEG_PRIV_R:
        log_rec(EV_PRIV_R, t.seg_addr, 0, t.rd_start, t.logbuf, n);
        t.ptr = (uint8_t)(t.rd_start + n);
        break;
    case SEG_I2C_W:
        log_rec(EV_I2C_W, t.seg_addr, 0, 0, t.logbuf, n);
        break;
    case SEG_I2C_R:
        log_rec(EV_I2C_R, t.seg_addr, 0, t.rd_start, t.logbuf, n);
        t.ptr = (uint8_t)(t.rd_start + n);
        break;
    case SEG_ENTDAA:
        if (t.entdaa_lost) {
            t.entdaa_lost_cnt++;
            log_rec(EV_ENTDAA_LOST, 0, 0, t.pid_bytes, NULL, 0);
        }
        break;
    default:
        break;
    }
    flush_da_log();
    /* A broadcast CCC ends at Sr; a direct CCC lasts until P or a new 0x7E/W. */
    if (t.ccc_active && t.ccc < 0x80)
        t.ccc_active = false;
    t.seg = SEG_NONE;
    t.ctx = CTX_IGNORE;
    t.rd_src = RD_NONE;
    t.rd_len = 0;
}

/* ------------------------------------------------------------------------- */
/* Address phase                                                             */
/* ------------------------------------------------------------------------- */

static void nack(void)
{
    q(s_nack, TAG_IGNORE);
    t.naks++;
}

/* arg bits: 0 = RnW, 1 = ACK, 2 = followed a START (not a repeated START) */
static void log_addr(uint8_t a, bool rnw, bool ack, bool start)
{
    uint8_t flags = (uint8_t)((rnw ? 1u : 0u) | (ack ? 2u : 0u) | (start ? 4u : 0u));
    log_rec(EV_ADDR, a, 0, flags, NULL, 0);
}

static void nack_addr(uint8_t a, bool rnw, bool start)
{
    log_addr(a, rnw, false, start);
    nack();
}

static void ack_then_receive(uint8_t ctx)
{
    q(s_ack, TAG_IGNORE);
    q(s_rx9, TAG_WDATA);    /* two bytes always queued ahead */
    q(s_rx9, TAG_WDATA);
    t.ctx = ctx;
}

static void be16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }

/* Sr + address while a direct CCC is active */
static void on_direct_addr(uint8_t a, bool rnw, bool start)
{
    bool match = (t.ccc == CCC_SETDASA) ? (t.da == 0 && a == I3C_STATIC_ADDR)
                                        : (t.da != 0 && a == t.da);
    if (!match) { nack_addr(a, rnw, start); return; }

    uint8_t len = 0;
    if (!rnw) {
        switch (t.ccc) {
        case CCC_SETDASA: case CCC_SETNEWDA:
        case CCC_ENEC_D:  case CCC_DISEC_D:
        case CCC_SETMWL_D: case CCC_SETMRL_D:
            log_addr(a, rnw, true, start);
            seg_begin(SEG_DIRECT, a);
            ack_then_receive(CTX_CCC_DIR_W);
            return;
        case CCC_RSTDAA_D:
            log_addr(a, rnw, true, start);
            seg_begin(SEG_DIRECT, a);
            q(s_ack, TAG_IGNORE);
            q(s_rx1, TAG_IGNORE);           /* release SDA after ACK */
            set_da(0, DA_RSTDAA_DIRECT, NULL, 0);
            return;
        default:
            nack_addr(a, rnw, start);
            return;
        }
    }

    switch (t.ccc) {
    case CCC_GETPID:    memcpy(t.rd_buf, identity, 6); len = 6; break;
    case CCC_GETBCR:    t.rd_buf[0] = identity[6]; len = 1; break;
    case CCC_GETDCR:    t.rd_buf[0] = identity[7]; len = 1; break;
    case CCC_GETSTATUS:
        /* [15:8] vendor = 0, [7:6] activity mode 0, [5] protocol error,
         * [3:0] pending interrupt = 0                                       */
        t.rd_buf[0] = 0x00;
        t.rd_buf[1] = t.proto_err ? 0x20 : 0x00;
        t.proto_err = false;
        len = 2;
        break;
    case CCC_GETMWL:    be16(t.rd_buf, t.mwl); len = 2; break;
    case CCC_GETMRL:    be16(t.rd_buf, t.mrl); len = 2; break;   /* BCR[2]=0: no IBI byte */
    case CCC_GETMXDS:   t.rd_buf[0] = I3C_MXDS_WR; t.rd_buf[1] = I3C_MXDS_RD; len = 2; break;
    default:
        nack_addr(a, rnw, start);
        return;
    }
    log_addr(a, rnw, true, start);
    seg_begin(SEG_DIRECT, a);
    t.ctx = CTX_IGNORE;
    q(s_ack, TAG_IGNORE);
    start_read(RD_BUF, 0, len);
}

static void on_addr(uint32_t w)
{
    uint8_t a = (uint8_t)((w >> 1) & 0x7Fu);
    bool rnw = (w & 1u) != 0;
    bool first = t.first_after_start;
    t.first_after_start = false;

    if (a == I3C_BCAST_ADDR) {
        if (!rnw) {
            /* Broadcast header: always ACK. A CCC code may follow, or Sr.   */
            log_addr(a, rnw, true, first);
            t.ccc_active = false;
            t.entdaa = false;
            seg_begin(SEG_CCC_HDR, a);
            q(s_ack, TAG_IGNORE);
            q(s_rx9, TAG_CCC);
            q(s_rx9, TAG_WDATA);
            t.ctx = CTX_IGNORE;       /* set properly once the code is known */
            return;
        }
        if (t.entdaa && t.da == 0) {
            /* ENTDAA round: ACK, then arbitrate with PID/BCR/DCR. The address
             * byte slot is queued right behind; we only get there if we win. */
            log_addr(a, rnw, true, first);
            seg_begin(SEG_ENTDAA, a);
            q(s_ack, TAG_IGNORE);
            for (int i = 0; i < 8; i++)
                q(s_arb_id[i], TAG_PID);
            q(s_rx8, TAG_DAA_ADDR);
            return;
        }
        nack_addr(a, rnw, first);
        return;
    }

    if (t.ccc_active && t.ccc >= 0x80) {
        on_direct_addr(a, rnw, first);
        return;
    }

    bool on_da = (t.da != 0 && a == t.da);
    bool on_sa = (t.da == 0 && a == I3C_STATIC_ADDR);

#if I3C_LEGACY_I2C
    if (on_sa && (first || t.i2c_frame)) {
        t.i2c_frame = true;
        /* START + static address without 0x7E header: legacy I2C framing */
        log_addr(a, rnw, true, first);
        seg_begin(rnw ? SEG_I2C_R : SEG_I2C_W, a);
        q(s_ack, TAG_IGNORE);
        if (rnw) {
            start_read(RD_MEM, t.ptr, 0xFFFF);
        } else {
            t.wr_first = true;
            q(s_i2c_rx_ack, TAG_I2C_WDATA);
            q(s_i2c_rx_ack, TAG_I2C_WDATA);
        }
        return;
    }
#endif
    (void)first;

#if !I3C_RESPOND_ON_STATIC_ADDR
    on_sa = false;
#endif
    if (!on_da && !on_sa) { nack_addr(a, rnw, first); return; }

    /* I3C SDR private transfer */
    log_addr(a, rnw, true, first);
    if (rnw) {
        seg_begin(SEG_PRIV_R, a);
        q(s_ack, TAG_IGNORE);
        start_read(RD_MEM, t.ptr, t.mrl ? t.mrl : 0xFFFF);
    } else {
        seg_begin(SEG_PRIV_W, a);
        t.wr_first = true;
        ack_then_receive(CTX_PRIV_W);
    }
}

/* ------------------------------------------------------------------------- */
/* CCC code and data bytes                                                   */
/* ------------------------------------------------------------------------- */

static void on_ccc(uint32_t w)
{
    uint8_t c = (uint8_t)(w >> 1);
    if (!odd_parity_ok(c, w)) {
        t.parity_errors++;
        t.proto_err = true;
        log_rec(EV_ERR, I3C_BCAST_ADDR, c, ERR_PARITY_CCC, NULL, 0);
        t.ctx = CTX_IGNORE;
        return;
    }
    t.ccc = c;
    t.ccc_active = true;
    t.ctx = (c < 0x80) ? CTX_CCC_BC : CTX_IGNORE;   /* direct: defining bytes */

    switch (c) {
    case CCC_RSTDAA_B:
        set_da(0, DA_RSTDAA_BC, NULL, 0);
        break;
    case CCC_ENTDAA:
        t.entdaa = true;
        break;
    case CCC_SETAASA:
        if (t.da == 0) set_da(I3C_STATIC_ADDR, DA_SETAASA, NULL, 0);
        break;
    default:
        if (c >= CCC_ENTHDR0 && c <= CCC_ENTHDR7) {
            /* We cannot follow HDR: stop listening until the bus is idle. */
            log_rec(EV_HDR, I3C_BCAST_ADDR, c, 0, NULL, 0);
            t.hdr = true;
            t.hdr_idle = false;
            t.ccc_active = false;
            t.seg = SEG_NONE;
            engine_reset();
        }
        break;
    }
}

static void apply_bc_data(void)
{
    uint8_t *d = t.logbuf;
    switch (t.ccc) {
    case CCC_ENEC_B:   if (t.seg_len == 1) t.events |= d[0];  break;
    case CCC_DISEC_B:  if (t.seg_len == 1) t.events &= (uint8_t)~d[0]; break;
    case CCC_SETMWL_B: if (t.seg_len == 2) t.mwl = (uint16_t)(d[0] << 8 | d[1]); break;
    case CCC_SETMRL_B: if (t.seg_len == 2) t.mrl = (uint16_t)(d[0] << 8 | d[1]); break;
    default: break;
    }
}

static void apply_dir_data(void)
{
    uint8_t *d = t.logbuf;
    switch (t.ccc) {
    case CCC_SETDASA:
    case CCC_SETNEWDA:
        if (t.seg_len == 1) {
            uint8_t nda = d[0] >> 1;              /* payload = DA << 1     */
            if (da_valid(nda))
                set_da(nda, t.ccc == CCC_SETDASA ? DA_SETDASA : DA_SETNEWDA, NULL, 0);
            else
                log_rec(EV_ERR, nda, t.ccc, ERR_BAD_DA, NULL, 0);
        }
        break;
    case CCC_ENEC_D:   if (t.seg_len == 1) t.events |= d[0]; break;
    case CCC_DISEC_D:  if (t.seg_len == 1) t.events &= (uint8_t)~d[0]; break;
    case CCC_SETMWL_D: if (t.seg_len == 2) t.mwl = (uint16_t)(d[0] << 8 | d[1]); break;
    case CCC_SETMRL_D: if (t.seg_len == 2) t.mrl = (uint16_t)(d[0] << 8 | d[1]); break;
    default: break;
    }
}

static void on_wdata(uint32_t w)
{
    q(s_rx9, TAG_WDATA);                 /* keep two bytes queued ahead */
    uint8_t b = (uint8_t)(w >> 1);

    if (t.ctx == CTX_IGNORE && !(t.seg == SEG_CCC_HDR && t.ccc_active))
        return;
    if (!odd_parity_ok(b, w)) {
        t.parity_errors++;
        t.proto_err = true;
        log_rec(EV_ERR, t.seg_addr, b, ERR_PARITY_DATA, NULL, 0);
        t.ctx = CTX_IGNORE;                   /* ignore the rest of the segment */
        t.ccc_active = (t.seg == SEG_CCC_HDR) ? false : t.ccc_active;
        return;
    }
    switch (t.ctx) {
    case CTX_CCC_BC:
        seg_append(b);
        apply_bc_data();
        break;
    case CTX_CCC_DIR_W:
        seg_append(b);
        apply_dir_data();
        break;
    case CTX_PRIV_W:
        seg_append(b);
        if (t.wr_first) { t.ptr = b; t.wr_first = false; }
        else            { i3c_mem[t.ptr] = b; t.ptr++; }
        break;
    default:                                  /* direct CCC defining byte */
        seg_append(b);
        break;
    }
}

/* ------------------------------------------------------------------------- */
/* Words from the bit engine                                                 */
/* ------------------------------------------------------------------------- */

static void on_word(uint32_t w)
{
    if (t.tag_tail == t.tag_head)
        return;                               /* nothing expected: stale   */
    uint8_t tag = t.tags[t.tag_tail & (TAG_SIZE - 1u)];
    t.tag_tail++;

    switch (tag) {
    case TAG_ADDR:   on_addr(w & 0xFFu); break;
    case TAG_CCC:    on_ccc(w & 0x1FFu); break;
    case TAG_WDATA:  on_wdata(w & 0x1FFu); break;
    case TAG_TXBYTE: seg_append((uint8_t)w); break;
    case TAG_PID:
        t.pid_bytes++;
        break;
    case TAG_DAA_ADDR: {
        uint8_t byte = (uint8_t)w;
        uint8_t nda = byte >> 1;
        if (__builtin_parity(byte) == 1 && da_valid(nda)) {
            q(s_ack, TAG_IGNORE);
            q(s_rx1, TAG_IGNORE);        /* release SDA after the ACK */
            set_da(nda, DA_ENTDAA, identity, 8);
        } else {
            q(s_nack, TAG_IGNORE);
            t.parity_errors++;
            log_rec(EV_ERR, nda, byte, __builtin_parity(byte) == 1 ? ERR_BAD_DA : ERR_PARITY_DAA,
                    NULL, 0);
        }
        break;
    }
    case TAG_I2C_WDATA: {
        q(s_i2c_rx_ack, TAG_I2C_WDATA);
        uint8_t b = (uint8_t)(w >> 1);
        seg_append(b);
        if (t.wr_first) { t.ptr = b; t.wr_first = false; }
        else            { i3c_mem[t.ptr] = b; t.ptr++; }
        break;
    }
    case TAG_I2C_TXBYTE:
        seg_append((uint8_t)w);
        break;
    case TAG_I2C_MACK:
        if ((w & 1u) == 0)                    /* ACK: controller wants more */
            t.rd_src = RD_MEM;                /* pump() produces next byte  */
        break;
    default:
        break;
    }
}

static void drain_rx(void)
{
    while (!phy_rx_empty()) {
        on_word(phy_rx_get());
        pump();
    }
}

/* Idle bus (after STOP / at boot): pre-arm the engine for the next START so
 * that the first address bits never depend on CPU latency.                  */
static void arm_idle(void)
{
    engine_reset();
    q(s_rx8, TAG_ADDR);
    pump();
    t.armed_start = true;
    t.first_after_start = true;
}

static void on_condition(uint32_t c)
{
    if (c == PHY_COND_START) {
        if (t.armed_start) {
            /* START from idle: the engine is already waiting for the address */
            t.armed_start = false;
            t.frames++;
            log_rec(EV_BUS, 0, 0, 1, NULL, 0);
            return;
        }
        /* repeated START */
        log_rec(EV_BUS, 0, 0, 2, NULL, 0);
        seg_end();
        engine_reset();
        q(s_rx8, TAG_ADDR);
        pump();
        t.first_after_start = false;
    } else {
        log_rec(EV_BUS, 0, 0, 3, NULL, 0);
        seg_end();
        t.ccc_active = false;
        t.entdaa = false;
        t.i2c_frame = false;
        arm_idle();
    }
}

/* ------------------------------------------------------------------------- */
/* Public API                                                                */
/* ------------------------------------------------------------------------- */

void i3c_target_init(void)
{
    memset(&t, 0, sizeof t);
    for (int i = 0; i < 6; i++)
        identity[i] = (uint8_t)(I3C_PID >> (40 - 8 * i));
    identity[6] = I3C_BCR;
    identity[7] = I3C_DCR;
    for (unsigned i = 0; i < I3C_MEM_SIZE; i++)
        i3c_mem[i] = (uint8_t)i;              /* readable test pattern */
    s_ack = slot_ack();
    s_nack = slot_nack();
    s_rx1 = slot_rx(1);
    s_rx8 = slot_rx(8);
    s_rx9 = slot_rx(9);
    s_i2c_rx_ack = slot_i2c_rx_ack();
    s_tbit_more = slot_tbit(true);
    s_tbit_last = slot_tbit(false);
    for (int i = 0; i < 8; i++) s_arb_id[i] = slot_arb(8, identity[i]);
    for (int i = 0; i < 256; i++) s_tx8[i] = slot_tx8((uint8_t)i);
    t.mwl = I3C_MWL;
    t.mrl = I3C_MRL;
    arm_idle();
}

void i3c_target_poll(void)
{
    if (t.hdr) {
        /* Ignore everything until SCL and SDA have been high for 50 us
         * (HDR exit pattern + STOP, then idle).                            */
        while (!phy_cond_empty()) (void)phy_cond_get();
        while (!phy_rx_empty()) (void)phy_rx_get();
        if (phy_bus_idle_levels()) {
            uint32_t now = phy_time_us();
            if (!t.hdr_idle) { t.hdr_idle = true; t.hdr_idle_t0 = now; }
            else if (now - t.hdr_idle_t0 > 50u) {
                t.hdr = false;
                t.entdaa = false;
                arm_idle();
            }
        } else {
            t.hdr_idle = false;
        }
        return;
    }

    while (!phy_cond_empty()) {
        uint32_t c = phy_cond_get();
        drain_rx();                           /* these bits came first */
        if (t.hdr) return;
        on_condition(c);
    }
    drain_rx();
    if (!t.halt_seen && phy_halted()) {
        drain_rx();
        t.halt_seen = true;                   /* read finished / aborted, or */
        if (t.seg == SEG_ENTDAA)              /* lost ENTDAA arbitration     */
            t.entdaa_lost = true;
    }
    pump();
}

void i3c_target_get_status(i3c_target_status_t *s)
{
    s->dyn_addr = t.da;
    s->static_addr = I3C_STATIC_ADDR;
    s->events = t.events;
    s->mem_ptr = t.ptr;
    s->mwl = t.mwl;
    s->mrl = t.mrl;
    s->frames = t.frames;
    s->naks = t.naks;
    s->parity_errors = t.parity_errors;
    s->entdaa_lost = t.entdaa_lost_cnt;
}
