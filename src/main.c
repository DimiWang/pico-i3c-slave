/*
 * pico-i3c-slave - Raspberry Pi Pico (RP2040) as an I3C SDR target
 *
 *   core 1 : I3C protocol state machine (i3c_target.c) + PIO bit layer
 *   core 0 : USB CDC serial: prints every bus event and offers a tiny console
 *
 * SPDX-License-Identifier: MIT
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "tusb.h"

#include "config.h"
#include "i3c_log.h"
#include "i3c_phy.h"
#include "i3c_target.h"

#define FW_VERSION "1.0.0"

static void core1_main(void)
{
    phy_init();
    i3c_target_init();
    for (;;)
        i3c_target_poll();
}

static void print_banner(void)
{
    i3c_target_status_t s;
    i3c_target_get_status(&s);
    printf("\r\n=== pico-i3c-slave v" FW_VERSION " : RP2040 I3C SDR target ===\r\n");
    printf("SDA=GPIO%d SCL=GPIO%d  sys_clk=%lu kHz\r\n", I3C_SDA_PIN, I3C_SCL_PIN,
           (unsigned long)(clock_get_hz(clk_sys) / 1000u));
    printf("PID=%012llX BCR=0x%02X DCR=0x%02X static=0x%02X dynamic=",
           (unsigned long long)I3C_PID, I3C_BCR, I3C_DCR, I3C_STATIC_ADDR);
    if (s.dyn_addr) printf("0x%02X\r\n", s.dyn_addr); else printf("none\r\n");
    printf("Type 'help' for console commands. Bus events follow:\r\n");
}

static void print_status(void)
{
    i3c_target_status_t s;
    i3c_target_get_status(&s);
    printf("dynamic address : ");
    if (s.dyn_addr) printf("0x%02X\r\n", s.dyn_addr); else printf("none\r\n");
    printf("static address  : 0x%02X\r\n", s.static_addr);
    printf("PID/BCR/DCR     : %012llX / 0x%02X / 0x%02X\r\n",
           (unsigned long long)I3C_PID, I3C_BCR, I3C_DCR);
    printf("events (ENEC)   : 0x%02X  (IBI/Hot-Join not implemented)\r\n", s.events);
    printf("MWL / MRL       : %u / %u\r\n", s.mwl, s.mrl);
    printf("register pointer: 0x%02X\r\n", s.mem_ptr);
    printf("frames %lu, NACKed addresses %lu, parity errors %lu, ENTDAA lost %lu, log dropped %lu\r\n",
           (unsigned long)s.frames, (unsigned long)s.naks, (unsigned long)s.parity_errors,
           (unsigned long)s.entdaa_lost, (unsigned long)i3c_log_dropped());
}

static void dump_mem(unsigned off, unsigned len)
{
    if (len == 0) len = 256;
    for (unsigned i = 0; i < len; i++) {
        unsigned a = (off + i) & 0xFF;
        if (i % 16 == 0) printf("%s%02X:", i ? "\r\n" : "", a);
        printf(" %02X", i3c_mem[a]);
    }
    printf("\r\n");
}

static void handle_line(char *line)
{
    char *cmd = strtok(line, " \t");
    if (!cmd) return;
    char *a1 = strtok(NULL, " \t");
    char *a2 = strtok(NULL, " \t");

    if (!strcmp(cmd, "help") || !strcmp(cmd, "?")) {
        printf("help                 this text\r\n"
               "status               addresses, identity, counters\r\n"
               "dump [off] [len]     hex dump of the 256-byte register memory\r\n"
               "fill <byte>          fill memory with one value\r\n"
               "pattern              fill memory with 00 01 02 .. FF (boot default)\r\n");
    } else if (!strcmp(cmd, "status")) {
        print_status();
    } else if (!strcmp(cmd, "dump")) {
        dump_mem(a1 ? (unsigned)strtoul(a1, NULL, 0) : 0, a2 ? (unsigned)strtoul(a2, NULL, 0) : 256);
    } else if (!strcmp(cmd, "fill") && a1) {
        uint8_t v = (uint8_t)strtoul(a1, NULL, 0);
        for (unsigned i = 0; i < I3C_MEM_SIZE; i++) i3c_mem[i] = v;
        printf("OK\r\n");
    } else if (!strcmp(cmd, "pattern")) {
        for (unsigned i = 0; i < I3C_MEM_SIZE; i++) i3c_mem[i] = (uint8_t)i;
        printf("OK\r\n");
    } else {
        printf("unknown command '%s' (try 'help')\r\n", cmd);
    }
}

int main(void)
{
#if I3C_SYS_CLK_KHZ > 133000
    vreg_set_voltage(VREG_VOLTAGE_1_15);
    sleep_ms(2);
#endif
    set_sys_clock_khz(I3C_SYS_CLK_KHZ, true);
    stdio_init_all();

    multicore_launch_core1(core1_main);

    char line[80];
    unsigned len = 0;
    bool was_connected = false;
    i3c_log_rec_t rec;
    char out[256];

    for (;;) {
        bool conn = tud_cdc_connected();
        if (conn && !was_connected) {
            sleep_ms(50);
            print_banner();
        }
        was_connected = conn;

        while (i3c_log_pop(&rec)) {
            i3c_log_format(&rec, out, sizeof out);
            printf("%s\r\n", out);
        }

        int ch = getchar_timeout_us(0);
        if (ch >= 0) {
            if (ch == '\r' || ch == '\n') {
                if (len) {
                    printf("\r\n");
                    line[len] = 0;
                    handle_line(line);
                    len = 0;
                }
            } else if ((ch == 8 || ch == 127) && len) {
                len--;
                printf("\b \b");
            } else if (ch >= 32 && ch < 127 && len < sizeof line - 1) {
                line[len++] = (char)ch;
                putchar(ch);
            }
        }
        sleep_us(200);
    }
}
