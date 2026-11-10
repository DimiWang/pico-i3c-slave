/*
 * Host simulation of pico-i3c-slave.
 *
 * Runs the real protocol layer (src/i3c_target.c) and the real PIO programs
 * (assembled by pioasm, executed by a small PIO interpreter) against a
 * bit-level model of an I3C controller that issues the same transfer
 * sequences as I3CBlaster (0x7E arbitration header, Sr, ...).
 *
 *   1 tick = 1 RP2040 PIO cycle (5 ns at 200 MHz)
 *
 * SPDX-License-Identifier: MIT
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pio_sim.h"
#include "i3c_phy.h"
#include "i3c_target.h"
#include "i3c_log.h"
#include "config.h"

#define PICO_NO_HARDWARE 1
#include "i3c_target.pio.h"

/* ------------------------------------------------------------------------- */
/* Simulation parameters                                                     */
/* ------------------------------------------------------------------------- */
static int H = 100;          /* push-pull half period in ticks (100 = 1 MHz) */
static int HOD = 200;        /* open-drain low phase                         */
static int CPU_K = 100;      /* protocol layer polled every K ticks          */
static int RISE = 20;        /* pull-up rise time in ticks                   */
static int verbose = 1;

/* ------------------------------------------------------------------------- */
/* Bus + target model                                                        */
/* ------------------------------------------------------------------------- */
static pio_sm_sim_t bits, cond;
static uint64_t now;
static int c_scl = 1, c_oe = 0, c_val = 1;  /* controller drivers           */
static int comp_low = 0;                    /* competitor device pulls low  */
static int sda = 1, rel_cnt = 0;
static uint32_t hist[4];
static long contention_ticks, contention_run, contention_max;
static long fail_count;

static int bus_sda(void) { return sda; }

static void tick(void)
{
    int t_low  = bits.pindir && !bits.pinval;
    int t_high = bits.pindir && bits.pinval;
    int low  = t_low || (c_oe && !c_val) || comp_low;
    int high = t_high || (c_oe && c_val);
    if (low && high) {
        contention_ticks++;
        if (++contention_run > contention_max) contention_max = contention_run;
        if (contention_run == 1 && getenv("SIM_DEBUG") && atoi(getenv("SIM_DEBUG")) > 1)
            printf("    !! contention at tick %llu: target pc=%u c_oe=%d c_val=%d scl=%d\n",
                   (unsigned long long)now, bits.pc, c_oe, c_val, c_scl);
    } else {
        if (contention_run > 8 && getenv("SIM_DEBUG"))
            printf("    !! long contention ended at tick %llu: %ld ticks\n", (unsigned long long)now, contention_run);
        contention_run = 0;
    }

    if (low) { sda = 0; rel_cnt = 0; }
    else if (high) { sda = 1; }
    else if (!sda && ++rel_cnt >= RISE) sda = 1;    /* pull-up */

    uint32_t pins = (uint32_t)sda | ((uint32_t)c_scl << 1);
    /* 2-stage input synchroniser */
    uint32_t synced = hist[1];
    hist[1] = hist[0];
    hist[0] = pins;

    pio_sim_step(&bits, synced);
    pio_sim_step(&cond, synced);

    if (now % (uint64_t)CPU_K == 0)
        i3c_target_poll();
    now++;

    i3c_log_rec_t r;
    char buf[256];
    while (i3c_log_pop(&r)) {
        i3c_log_format(&r, buf, sizeof buf);
        if (verbose) printf("    target| %s\n", buf);
    }
}

static void run(int n) { while (n-- > 0) tick(); }

/* ------------------------------------------------------------------------- */
/* i3c_phy implementation for the simulator                                  */
/* ------------------------------------------------------------------------- */
uint8_t phy_entry_xfer, phy_entry_arb, phy_entry_tbit;

void phy_init(void)
{
    phy_entry_xfer = i3c_tgt_bits_offset_xfer;
    phy_entry_arb  = i3c_tgt_bits_offset_arb;
    phy_entry_tbit = i3c_tgt_bits_offset_tbit;

    memset(&bits, 0, sizeof bits);
    bits.prog = i3c_tgt_bits_program_instructions;
    bits.wrap_bottom = i3c_tgt_bits_wrap_target;
    bits.wrap_top = i3c_tgt_bits_wrap;
    bits.in_base = 0;
    bits.jmp_pin = 0;          /* SDA */
    bits.out_right = true;
    bits.in_left = true;
    pio_sim_reset(&bits, i3c_tgt_bits_offset_slot);

    memset(&cond, 0, sizeof cond);
    cond.prog = i3c_tgt_cond_program_instructions;
    cond.wrap_bottom = i3c_tgt_cond_wrap_target;
    cond.wrap_top = i3c_tgt_cond_wrap;
    cond.in_base = 0;
    cond.jmp_pin = 1;          /* SCL */
    cond.in_left = true;
    pio_sim_reset(&cond, 0);
}

void phy_reset(void)
{
    pio_sim_reset(&bits, i3c_tgt_bits_offset_slot);
    bits.pindir = 0;
    bits.pinval = 0;
}
bool phy_tx_full(void) { return bits.tx_n >= 4; }
void phy_tx_put(uint32_t w) { pio_sim_tx_put(&bits, w); }
bool phy_rx_empty(void) { return bits.rx_n == 0; }
uint32_t phy_rx_get(void) { uint32_t w = 0; pio_sim_rx_get(&bits, &w); return w; }
bool phy_cond_empty(void) { return cond.rx_n == 0; }
uint32_t phy_cond_get(void) { uint32_t w = 0; pio_sim_rx_get(&cond, &w); return w; }
bool phy_halted(void) { return bits.irq0; }
bool phy_bus_idle_levels(void) { return sda && c_scl; }
uint32_t phy_time_us(void) { return (uint32_t)(now / 200u); }

/* ------------------------------------------------------------------------- */
/* Controller model (same sequences as I3CBlaster's i3c_hl.c)                */
/* ------------------------------------------------------------------------- */
enum { REL = 0, PP = 1, OD = 2 };

static void c_drive(int mode, int v)
{
    if (mode == REL) { c_oe = 0; }
    else if (mode == PP) { c_oe = 1; c_val = v; }
    else { c_oe = !v; c_val = 0; }
}

/* one SCL clock; SDA set shortly after the falling edge, sampled just before
 * the rising edge (like I3CBlaster). Returns the sampled SDA level.        */
static int c_clock(int mode, int v, int low)
{
    c_scl = 0;
    run(2);
    c_drive(mode, v);
    run(low - 3);
    int s = bus_sda();
    run(1);
    c_scl = 1;
    run(H);
    return s;
}

static void c_start(void)
{
    c_drive(PP, 0);
    run(H);
    c_scl = 0;
}

static void c_restart(void)
{
    c_scl = 0;
    run(2);
    c_drive(PP, 1);
    run(H);
    c_scl = 1;
    run(H / 2);
    c_drive(PP, 0);
    run(H / 2);
    c_scl = 0;
}

static void c_stop(void)
{
    c_scl = 0;
    run(2);
    c_drive(PP, 0);
    run(H);
    c_scl = 1;
    run(H / 2);
    c_drive(REL, 0);
    run(4 * H);
}

static int c_byte(uint8_t b, int mode, int low)
{
    for (int i = 7; i >= 0; i--) c_clock(mode, (b >> i) & 1, low);
    return 0;
}

static bool c_ack(void) { return c_clock(REL, 0, HOD) == 0; }

/* address after START: open drain (arbitration header) */
static bool c_addr_od(uint8_t byte) { c_byte(byte, OD, HOD); return c_ack(); }
/* address after Sr: push-pull bits, open-drain ACK */
static bool c_addr_pp(uint8_t byte) { c_byte(byte, PP, H); return c_ack(); }

static void c_wbyte(uint8_t b, bool bad_parity)
{
    c_byte(b, PP, H);
    int t = (__builtin_parity(b) ^ 1) ^ (bad_parity ? 1 : 0);
    c_clock(PP, t, H);
}

/* read 8 bits + handle the T-bit. Returns T. */
static int c_rbyte(uint8_t *out, bool want_more)
{
    uint8_t b = 0;
    for (int i = 0; i < 8; i++) b = (uint8_t)(b << 1 | c_clock(REL, 0, H));
    *out = b;
    c_scl = 0;
    run(2);
    c_drive(REL, 0);
    run(H - 3);
    int T = bus_sda();
    run(1);
    c_scl = 1;
    if (T) {
        if (want_more) run(H);
        else { run(H / 2); c_drive(PP, 0); run(H / 2); }     /* abort: Sr */
    } else {
        run(2); c_drive(PP, 0); run(H - 2);                  /* take over */
    }
    return T;
}

static bool c_arbhdr(void) { c_start(); return c_addr_od(0xFC); }

static bool m_rstdaa(void)
{
    bool a = c_arbhdr();
    if (a) c_wbyte(0x06, false);
    c_stop();
    return a;
}

static bool m_bc_ccc(const uint8_t *d, int n)
{
    bool a = c_arbhdr();
    if (a) for (int i = 0; i < n; i++) c_wbyte(d[i], false);
    c_stop();
    return a;
}

/* competitor device for ENTDAA arbitration tests */
static bool comp_present;
static uint64_t comp_id;     /* 64 bits: PID, BCR, DCR */
static uint8_t comp_da;

static bool m_entdaa(uint8_t addr, uint8_t *id)
{
    bool ok = false;
    if (!c_arbhdr()) { c_stop(); return false; }
    c_wbyte(0x07, false);
    c_restart();
    /* 7E/R: competitor ACKs too if it has no address */
    c_byte(0xFD, PP, H);
    bool comp_in = comp_present && comp_da == 0;
    c_scl = 0; run(2); c_drive(REL, 0); comp_low = comp_in; run(HOD - 3);
    bool ack = bus_sda() == 0; run(1); c_scl = 1; run(H);
    if (ack) {
        uint64_t got = 0;
        for (int i = 63; i >= 0; i--) {
            int cb = comp_in ? (int)((comp_id >> i) & 1) : 1;
            c_scl = 0; run(2); c_drive(REL, 0); comp_low = comp_in && cb == 0; run(HOD - 3);
            int s = bus_sda(); run(1); c_scl = 1; run(H);
            if (comp_in && cb == 1 && s == 0) comp_in = false;   /* competitor lost */
            got = got << 1 | (uint64_t)s;
        }
        for (int i = 0; i < 8; i++) id[i] = (uint8_t)(got >> (56 - 8 * i));
        uint8_t ab = (uint8_t)(addr << 1 | (__builtin_parity(addr) ^ 1));
        c_scl = 0; run(2); comp_low = 0;      /* competitor releases at SCL fall */
        c_byte(ab, PP, H);
        c_scl = 0; run(2); c_drive(REL, 0); comp_low = comp_in; run(HOD - 3);
        ok = bus_sda() == 0; run(1); c_scl = 1; run(H);
        if (comp_in && ok) comp_da = addr;
        comp_low = 0;
    }
    c_stop();
    comp_low = 0;
    return ok;
}

static bool m_write(uint8_t addr, const uint8_t *d, int n, int bad_index)
{
    bool a = c_arbhdr();
    if (a) {
        c_restart();
        a = c_addr_pp((uint8_t)(addr << 1));
        if (a) for (int i = 0; i < n; i++) c_wbyte(d[i], i == bad_index);
    }
    c_stop();
    return a;
}

static int c_read_n(uint8_t *out, int n)
{
    int got = 0;
    for (int i = 0; i < n; i++) {
        int T = c_rbyte(&out[i], i < n - 1);
        got++;
        if (!T) break;
    }
    return got;
}

static int m_read(uint8_t addr, uint8_t *out, int n)
{
    int got = -1;
    if (c_arbhdr()) {
        c_restart();
        if (c_addr_pp((uint8_t)(addr << 1 | 1))) got = c_read_n(out, n);
    }
    c_stop();
    return got;
}

static int m_writeread(uint8_t addr, const uint8_t *w, int wn, uint8_t *out, int n)
{
    int got = -1;
    if (c_arbhdr()) {
        c_restart();
        if (c_addr_pp((uint8_t)(addr << 1))) {
            for (int i = 0; i < wn; i++) c_wbyte(w[i], false);
            c_restart();
            if (c_addr_pp((uint8_t)(addr << 1 | 1))) got = c_read_n(out, n);
        }
    }
    c_stop();
    return got;
}

static bool m_dir_write(uint8_t ccc, uint8_t addr, const uint8_t *d, int n)
{
    bool a = c_arbhdr();
    if (a) {
        c_wbyte(ccc, false);
        c_restart();
        a = c_addr_pp((uint8_t)(addr << 1));
        if (a) for (int i = 0; i < n; i++) c_wbyte(d[i], false);
    }
    c_stop();
    return a;
}

static int m_dir_read(uint8_t ccc, uint8_t addr, uint8_t *out, int n)
{
    int got = -1;
    if (c_arbhdr()) {
        c_wbyte(ccc, false);
        c_restart();
        if (c_addr_pp((uint8_t)(addr << 1 | 1))) got = c_read_n(out, n);
    }
    c_stop();
    return got;
}

static bool m_scan(uint8_t addr)
{
    bool a = c_arbhdr();
    if (a) { c_restart(); a = c_addr_pp((uint8_t)(addr << 1)); }
    c_stop();
    return a;
}

/* legacy I2C at ~400 kHz (all open drain, controller ACKs read bytes) */
static bool m_i2c_write(uint8_t addr, const uint8_t *d, int n)
{
    int sh = H; H = 250;
    c_start();
    bool a = c_addr_od((uint8_t)(addr << 1));
    if (a) for (int i = 0; i < n; i++) { c_byte(d[i], OD, 250); if (!c_ack()) { a = false; break; } }
    c_stop();
    H = sh;
    return a;
}

static int m_i2c_read(uint8_t addr, uint8_t *out, int n)
{
    int sh = H; H = 250;
    int got = -1;
    c_start();
    if (c_addr_od((uint8_t)(addr << 1 | 1))) {
        got = 0;
        for (int i = 0; i < n; i++) {
            uint8_t b = 0;
            for (int k = 0; k < 8; k++) b = (uint8_t)(b << 1 | c_clock(REL, 0, 250));
            out[got++] = b;
            c_clock(OD, i < n - 1 ? 0 : 1, 250);   /* ACK all but the last */
        }
    }
    c_stop();
    H = sh;
    return got;
}

/* I2C write then repeated START + read (like i2c_writeread) */
static int m_i2c_writeread(uint8_t addr, const uint8_t *d, int wn, uint8_t *out, int n)
{
    int sh = H; H = 250;
    int got = -1;
    c_start();
    if (c_addr_od((uint8_t)(addr << 1))) {
        for (int i = 0; i < wn; i++) { c_byte(d[i], OD, 250); c_ack(); }
        c_restart();
        if (c_addr_od((uint8_t)(addr << 1 | 1))) {
            got = 0;
            for (int i = 0; i < n; i++) {
                uint8_t b = 0;
                for (int k = 0; k < 8; k++) b = (uint8_t)(b << 1 | c_clock(REL, 0, 250));
                out[got++] = b;
                c_clock(OD, i < n - 1 ? 0 : 1, 250);
            }
        }
    }
    c_stop();
    H = sh;
    return got;
}

/* ------------------------------------------------------------------------- */
/* Tests                                                                     */
/* ------------------------------------------------------------------------- */
#define CHECK(c, ...) do { if (!(c)) { fail_count++; if (verbose) { printf("  FAIL %s:%d: ", __FILE__, __LINE__); \
                           printf(__VA_ARGS__); printf("\n"); } } else if (verbose) { printf("  ok   "); \
                           printf(__VA_ARGS__); printf("\n"); } } while (0)

static uint8_t da(void) { i3c_target_status_t s; i3c_target_get_status(&s); return s.dyn_addr; }

static void hex(const char *p, const uint8_t *d, int n)
{
    if (!verbose) return;
    printf("    ctrl  | %s", p);
    for (int i = 0; i < n; i++) printf(" %02X", d[i]);
    printf("\n");
}

static void run_suite(void)
{
    uint8_t id[8], buf[300];
    int n;

    if (verbose) printf("-- RSTDAA, scan before any dynamic address\n");
    CHECK(m_rstdaa(), "RSTDAA: 0x7E ACKed");
    CHECK(da() == 0, "no dynamic address");
    CHECK(m_scan(0x42), "scan: static address 0x42 ACKs (I3C_RESPOND_ON_STATIC_ADDR)");
    CHECK(!m_scan(0x30), "scan: 0x30 NACKs");

    if (verbose) printf("-- ENTDAA\n");
    memset(id, 0, sizeof id);
    CHECK(m_entdaa(0x30, id), "ENTDAA 0x30 ACKed");
    hex("ENTDAA PID+BCR+DCR:", id, 8);
    static const uint8_t exp_id[8] = { 0xFF, 0xFE, 0x3C, 0x01, 0x00, 0x01, 0x01, 0x00 };
    CHECK(!memcmp(id, exp_id, 8), "ENTDAA returned FF FE 3C 01 00 01 / BCR 01 / DCR 00");
    CHECK(da() == 0x30, "dynamic address is 0x30");
    CHECK(!m_entdaa(0x31, id), "second ENTDAA: nobody left (7E/R NACK)");
    CHECK(m_scan(0x30) && !m_scan(0x42), "scan: 0x30 ACKs, static 0x42 no longer");

    if (verbose) printf("-- GET CCCs\n");
    n = m_dir_read(0x8D, 0x30, buf, 6);
    hex("GETPID:", buf, n > 0 ? n : 0);
    CHECK(n == 6 && !memcmp(buf, exp_id, 6), "GETPID");
    n = m_dir_read(0x8E, 0x30, buf, 1);
    CHECK(n == 1 && buf[0] == 0x01, "GETBCR = 0x01");
    n = m_dir_read(0x8F, 0x30, buf, 1);
    CHECK(n == 1 && buf[0] == 0x00, "GETDCR = 0x00");
    n = m_dir_read(0x90, 0x30, buf, 2);
    CHECK(n == 2 && buf[0] == 0 && buf[1] == 0, "GETSTATUS = 00 00");
    n = m_dir_read(0x8C, 0x30, buf, 2);
    CHECK(n == 2 && buf[0] == 0x01 && buf[1] == 0x00, "GETMRL = 256");
    n = m_dir_read(0x94, 0x30, buf, 2);
    CHECK(n == 2 && buf[0] == 0x04, "GETMXDS");
    n = m_dir_read(0x8D, 0x31, buf, 6);
    CHECK(n < 0, "GETPID to 0x31 NACKed");
    n = m_dir_read(0x95, 0x30, buf, 2);
    CHECK(n < 0, "unsupported GETCAPS NACKed");

    if (verbose) printf("-- SDR private write / read\n");
    static const uint8_t wr[] = { 0x10, 0xAA, 0xBB, 0xCC, 0xDD };
    CHECK(m_write(0x30, wr, 5, -1), "write ptr 0x10 + AA BB CC DD");
    n = m_writeread(0x30, wr, 1, buf, 4);
    hex("writeread 0x10, 4:", buf, n > 0 ? n : 0);
    CHECK(n == 4 && !memcmp(buf, wr + 1, 4), "read back AA BB CC DD");
    n = m_read(0x30, buf, 3);
    hex("read 3:", buf, n > 0 ? n : 0);
    CHECK(n == 3 && buf[0] == 0x14 && buf[1] == 0x15 && buf[2] == 0x16, "plain read continues at 0x14");
    static const uint8_t p0[] = { 0x00 };
    n = m_writeread(0x30, p0, 1, buf, 300);
    CHECK(n == 256, "read stops itself after MRL=256 bytes (T=0), got %d", n);
    CHECK(i3c_mem[0x12] == 0xCC, "target memory updated");

    if (verbose) printf("-- parity error\n");
    static const uint8_t bad[] = { 0x20, 0x55, 0x66 };
    CHECK(m_write(0x30, bad, 3, 1), "write with a wrong T-bit on byte 1");
    CHECK(i3c_mem[0x20] == 0x20, "bad byte not stored");
    n = m_dir_read(0x90, 0x30, buf, 2);
    CHECK(n == 2 && buf[1] == 0x20, "GETSTATUS reports protocol error (0x0020)");

    if (verbose) printf("-- SETNEWDA\n");
    static const uint8_t nd[] = { 0x34 << 1 };
    CHECK(m_dir_write(0x88, 0x30, nd, 1), "SETNEWDA 0x30 -> 0x34");
    CHECK(da() == 0x34, "dynamic address is 0x34");

    if (verbose) printf("-- RSTDAA + SETDASA\n");
    CHECK(m_rstdaa() && da() == 0, "RSTDAA clears the address");
    static const uint8_t sd[] = { 0x31 << 1 };
    CHECK(!m_dir_write(0x87, 0x43, sd, 1), "SETDASA to wrong static address NACKed");
    CHECK(m_dir_write(0x87, 0x42, sd, 1), "SETDASA 0x42 -> 0x31");
    CHECK(da() == 0x31, "dynamic address is 0x31");
    CHECK(!m_dir_write(0x87, 0x42, sd, 1), "SETDASA again: static address ignored now");

    if (verbose) printf("-- direct RSTDAA, SETAASA, ENEC/DISEC\n");
    CHECK(m_dir_write(0x86, 0x31, NULL, 0) && da() == 0, "direct RSTDAA");
    static const uint8_t aasa[] = { 0x29 };
    CHECK(m_bc_ccc(aasa, 1) && da() == 0x42, "SETAASA: DA = static 0x42");
    static const uint8_t enec[] = { 0x00, 0x01 }, disec[] = { 0x01, 0x01 };
    m_bc_ccc(enec, 2);
    i3c_target_status_t st; i3c_target_get_status(&st);
    CHECK(st.events == 0x01, "ENEC INT stored");
    m_bc_ccc(disec, 2);
    i3c_target_get_status(&st);
    CHECK(st.events == 0x00, "DISEC INT cleared");

    if (verbose) printf("-- ENTDAA with a competing device that has a lower PID\n");
    m_rstdaa();
    comp_present = true; comp_da = 0;
    comp_id = 0x0123456789AB0000ull | 0x0000;   /* PID 0123456789AB, BCR 00, DCR 00 */
    CHECK(m_entdaa(0x30, id), "round 1 assigns 0x30");
    hex("round 1 winner:", id, 8);
    CHECK(comp_da == 0x30 && da() == 0, "competitor won round 1, we did not take 0x30");
    CHECK(m_entdaa(0x31, id), "round 2 assigns 0x31");
    hex("round 2 winner:", id, 8);
    CHECK(da() == 0x31 && !memcmp(id, exp_id, 8), "we won round 2 and got 0x31");
    comp_present = false;

    if (verbose) printf("-- legacy I2C on the static address\n");
    m_rstdaa();
    static const uint8_t i2w[] = { 0x40, 0x11, 0x22, 0x33 };
    CHECK(m_i2c_write(0x42, i2w, 4), "I2C write 40: 11 22 33");
    static const uint8_t i2p[] = { 0x40 };
    CHECK(m_i2c_write(0x42, i2p, 1), "I2C write pointer 0x40");
    n = m_i2c_read(0x42, buf, 3);
    hex("I2C read 3:", buf, n > 0 ? n : 0);
    CHECK(n == 3 && buf[0] == 0x11 && buf[1] == 0x22 && buf[2] == 0x33, "I2C read back");
    CHECK(!m_i2c_write(0x43, i2p, 1), "I2C to other address NACKed");
    static const uint8_t i2p2[] = { 0x41 };
    n = m_i2c_writeread(0x42, i2p2, 1, buf, 2);
    hex("I2C writeread 0x41, 2:", buf, n > 0 ? n : 0);
    CHECK(n == 2 && buf[0] == 0x22 && buf[1] == 0x33, "I2C write + Sr + read");
}

int main(int argc, char **argv)
{
    bool sweep = argc > 1 && !strcmp(argv[1], "--sweep");
    if (getenv("SIM_H"))    { H = atoi(getenv("SIM_H")); HOD = 2 * H; }
    if (getenv("SIM_K"))    CPU_K = atoi(getenv("SIM_K"));
    if (getenv("SIM_RISE")) RISE = atoi(getenv("SIM_RISE"));
    if (getenv("SIM_QUIET")) verbose = 0;
    phy_init();
    i3c_target_init();
    run(1000);

    if (!sweep) {
        printf("I3C target simulation: SCL %.2f MHz, CPU poll every %d ns, pull-up rise %d ns\n",
               200.0 / (2 * H), CPU_K * 5, RISE * 5);
        run_suite();
        printf("\ncontention: %ld ticks total, longest %ld ticks (%d ns)\n",
               contention_ticks, contention_max, (int)contention_max * 5);
        printf("%s: %ld failure(s)\n", fail_count ? "FAILED" : "PASSED", fail_count);
        return fail_count ? 1 : 0;
    }

    verbose = 0;
    printf("SCL(MHz)  cpu-latency(ns)  result\n");
    static const int hs[] = { 500, 250, 200, 100, 67, 50, 40, 25 };
    static const int ks[] = { 40, 100, 200 };
    for (unsigned i = 0; i < sizeof hs / sizeof hs[0]; i++)
        for (unsigned j = 0; j < sizeof ks / sizeof ks[0]; j++) {
            H = hs[i]; HOD = 2 * hs[i]; CPU_K = ks[j];
            fail_count = 0;
            phy_init(); i3c_target_init(); run(1000);
            run_suite();
            printf("%7.2f   %6d           %s\n", 200.0 / (2 * H), CPU_K * 5,
                   fail_count ? "FAIL" : "pass");
        }
    return 0;
}
