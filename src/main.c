/*
 * pico-i3c-slave - Raspberry Pi Pico (RP2040) as an I3C SDR target
 *
 *   core 1 : I3C protocol state machine (i3c_target.c) + PIO bit layer
 *   core 0 : USB CDC + UART0 serial: prints every bus event and a heartbeat
 *
 * SPDX-License-Identifier: MIT
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/vreg.h"
#include "tusb.h"

#include "config.h"
#include "i3c_log.h"
#include "i3c_phy.h"
#include "i3c_target.h"

#define FW_VERSION "1.1.0"
#define HEARTBEAT_US 2000000u
#define LED_TOGGLE_US 500000u

static bool heartbeat_on = true;

static void core1_main(void)
{
    phy_init();
    i3c_target_init();
    for (;;)
        i3c_target_poll();
}

static void led_init(void)
{
#ifdef PICO_DEFAULT_LED_PIN
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
#endif
}

static void led_toggle(void)
{
#ifdef PICO_DEFAULT_LED_PIN
    gpio_xor_mask(1u << PICO_DEFAULT_LED_PIN);
#endif
}

static const char *usb_state(void)
{
    if (tud_cdc_connected()) return "DTR";
    if (tud_mounted())       return "mounted, no DTR";
    if (tud_ready())         return "ready";
    return "not enumerated";
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
    printf("USB=%s  debug UART=GPIO0 TX / GPIO1 RX 115200 8N1\r\n", usb_state());
    printf("Onboard LED blinks if this firmware is running.\r\n");
    printf("Type 'help' for console commands. Heartbeat every 2s; bus events follow:\r\n");
    stdio_flush();
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
    printf("SDA=%d SCL=%d (idle is both 1, pulled up)\r\n",
           gpio_get(I3C_SDA_PIN), gpio_get(I3C_SCL_PIN));
    printf("USB             : %s\r\n", usb_state());
    printf("frames %lu, NACKed addresses %lu, parity errors %lu, ENTDAA lost %lu, log dropped %lu\r\n",
           (unsigned long)s.frames, (unsigned long)s.naks, (unsigned long)s.parity_errors,
           (unsigned long)s.entdaa_lost, (unsigned long)i3c_log_dropped());
    stdio_flush();
}

static void print_heartbeat(void)
{
    i3c_target_status_t s;
    i3c_target_get_status(&s);
    uint32_t us = time_us_32();
    printf("[HB  %lu.%06lu] usb=%s  SDA=%d SCL=%d  static=0x%02X dynamic=",
           (unsigned long)(us / 1000000u), (unsigned long)(us % 1000000u),
           usb_state(), gpio_get(I3C_SDA_PIN), gpio_get(I3C_SCL_PIN), I3C_STATIC_ADDR);
    if (s.dyn_addr) printf("0x%02X", s.dyn_addr); else printf("none");
    printf("  frames=%lu naks=%lu\r\n",
           (unsigned long)s.frames, (unsigned long)s.naks);
    stdio_flush();
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
    stdio_flush();
}

static void handle_line(char *line)
{
    char *cmd = strtok(line, " \t");
    if (!cmd) return;
    char *a1 = strtok(NULL, " \t");
    char *a2 = strtok(NULL, " \t");

    if (!strcmp(cmd, "help") || !strcmp(cmd, "?")) {
        printf("help                 this text\r\n"
               "status               addresses, identity, pin levels, counters\r\n"
               "dump [off] [len]     hex dump of the 256-byte register memory\r\n"
               "fill <byte>          fill memory with one value\r\n"
               "pattern              fill memory with 00 01 02 .. FF (boot default)\r\n"
               "heartbeat on|off     periodic alive message (default on)\r\n");
    } else if (!strcmp(cmd, "status")) {
        print_status();
        return;
    } else if (!strcmp(cmd, "dump")) {
        dump_mem(a1 ? (unsigned)strtoul(a1, NULL, 0) : 0, a2 ? (unsigned)strtoul(a2, NULL, 0) : 256);
        return;
    } else if (!strcmp(cmd, "fill") && a1) {
        uint8_t v = (uint8_t)strtoul(a1, NULL, 0);
        for (unsigned i = 0; i < I3C_MEM_SIZE; i++) i3c_mem[i] = v;
        printf("OK\r\n");
    } else if (!strcmp(cmd, "pattern")) {
        for (unsigned i = 0; i < I3C_MEM_SIZE; i++) i3c_mem[i] = (uint8_t)i;
        printf("OK\r\n");
    } else if (!strcmp(cmd, "heartbeat")) {
        if (a1 && (!strcmp(a1, "on") || !strcmp(a1, "1"))) heartbeat_on = true;
        else if (a1 && (!strcmp(a1, "off") || !strcmp(a1, "0"))) heartbeat_on = false;
        printf("heartbeat %s\r\n", heartbeat_on ? "on" : "off");
    } else {
        printf("unknown command '%s' (try 'help')\r\n", cmd);
    }
    stdio_flush();
}

int main(void)
{
#if I3C_SYS_CLK_KHZ > 133000
    vreg_set_voltage(VREG_VOLTAGE_1_15);
    sleep_ms(2);
#endif
    set_sys_clock_khz(I3C_SYS_CLK_KHZ, true);
    stdio_init_all();
    led_init();

    /* Banner goes out on UART immediately. USB may still be enumerating. */
    print_banner();

    multicore_launch_core1(core1_main);

    char line[80];
    unsigned len = 0;
    bool was_connected = tud_cdc_connected();
    bool announced_usb = false;
    i3c_log_rec_t rec;
    char out[256];
    uint32_t last_hb = time_us_32();
    uint32_t last_led = last_hb;

    for (;;) {
        bool conn = tud_cdc_connected();
        if (conn && !was_connected) {
            sleep_ms(50);
            print_banner();
        }
        was_connected = conn;
        if (tud_mounted() && !announced_usb) {
            announced_usb = true;
            printf("USB enumerated. Product string is 'pico-i3c-slave'.\r\n");
            stdio_flush();
        }

        while (i3c_log_pop(&rec)) {
            i3c_log_format(&rec, out, sizeof out);
            printf("%s\r\n", out);
            stdio_flush();
        }

        uint32_t now = time_us_32();
        if ((uint32_t)(now - last_led) >= LED_TOGGLE_US) {
            last_led = now;
            led_toggle();
        }
        if (heartbeat_on && (uint32_t)(now - last_hb) >= HEARTBEAT_US) {
            last_hb = now;
            print_heartbeat();
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
