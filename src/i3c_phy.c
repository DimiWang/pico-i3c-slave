/*
 * i3c_phy (RP2040) - PIO set-up and FIFO access for the I3C target.
 *
 *   PIO0 SM0 : i3c_tgt_bits  (bit engine, owns SDA)
 *   PIO1 SM0 : i3c_tgt_cond  (START/STOP detector, input only)
 *
 * SPDX-License-Identifier: MIT
 */
#include "i3c_phy.h"
#include "config.h"

#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/structs/sio.h"
#include "i3c_target.pio.h"

#define BITS_PIO pio0
#define BITS_SM  0u
#define COND_PIO pio1
#define COND_SM  0u

uint8_t phy_entry_xfer;
uint8_t phy_entry_arb;
uint8_t phy_entry_tbit;

static uint bits_offset;

void phy_init(void)
{
    /* The bit engine dispatches with "out pc, 5": it needs absolute
     * addresses, so it is loaded at offset 0 and uses all 32 words.        */
    pio_add_program_at_offset(BITS_PIO, &i3c_tgt_bits_program, 0);
    bits_offset = 0;
    uint cond_offset = pio_add_program(COND_PIO, &i3c_tgt_cond_program);

    phy_entry_xfer = (uint8_t)(bits_offset + i3c_tgt_bits_offset_xfer);
    phy_entry_arb  = (uint8_t)(bits_offset + i3c_tgt_bits_offset_arb);
    phy_entry_tbit = (uint8_t)(bits_offset + i3c_tgt_bits_offset_tbit);

    /* No internal pulls: the I3CBlaster already has 2.2k pull-ups to 3V3.   */
    gpio_init(I3C_SCL_PIN);
    gpio_set_dir(I3C_SCL_PIN, false);
    gpio_disable_pulls(I3C_SCL_PIN);
    gpio_disable_pulls(I3C_SDA_PIN);
    pio_gpio_init(BITS_PIO, I3C_SDA_PIN);           /* SDA output via PIO0   */
    gpio_set_drive_strength(I3C_SDA_PIN, GPIO_DRIVE_STRENGTH_8MA);
    gpio_set_slew_rate(I3C_SDA_PIN, GPIO_SLEW_RATE_FAST);
    /* Input synchronisers stay enabled on both pins (2 cycles, identical on
     * SDA and SCL, so their relative timing is preserved).                  */

    /* ---- bit engine ---- */
    pio_sm_config c = i3c_tgt_bits_program_get_default_config(bits_offset);
    sm_config_set_out_pins(&c, I3C_SDA_PIN, 1);
    sm_config_set_set_pins(&c, I3C_SDA_PIN, 1);
    sm_config_set_in_pins(&c, I3C_SDA_PIN);
    sm_config_set_jmp_pin(&c, I3C_SDA_PIN);
    sm_config_set_out_shift(&c, true /* right */, false, 32);
    sm_config_set_in_shift(&c, false /* left */, false, 32);
    sm_config_set_clkdiv(&c, 1.0f);
    pio_sm_set_pins_with_mask(BITS_PIO, BITS_SM, 0, 1u << I3C_SDA_PIN);
    pio_sm_set_pindirs_with_mask(BITS_PIO, BITS_SM, 0, 1u << I3C_SDA_PIN);
    pio_sm_init(BITS_PIO, BITS_SM, bits_offset + i3c_tgt_bits_offset_slot, &c);

    /* ---- START/STOP detector ---- */
    pio_sm_config d = i3c_tgt_cond_program_get_default_config(cond_offset);
    sm_config_set_in_pins(&d, I3C_SDA_PIN);
    sm_config_set_jmp_pin(&d, I3C_SCL_PIN);
    sm_config_set_in_shift(&d, false, false, 32);
    sm_config_set_clkdiv(&d, 1.0f);
    pio_sm_init(COND_PIO, COND_SM, cond_offset, &d);

    pio_interrupt_clear(BITS_PIO, 0);
    pio_sm_set_enabled(BITS_PIO, BITS_SM, true);
    pio_sm_set_enabled(COND_PIO, COND_SM, true);
}

void __not_in_flash_func(phy_reset)(void)
{
    PIO pio = BITS_PIO;
    pio_sm_set_enabled(pio, BITS_SM, false);
    pio_sm_clear_fifos(pio, BITS_SM);
    pio_sm_restart(pio, BITS_SM);           /* also drops a stalled wait/irq */
    pio_sm_exec(pio, BITS_SM, pio_encode_set(pio_pindirs, 0));   /* release */
    pio_sm_exec(pio, BITS_SM, pio_encode_set(pio_pins, 0));      /* latch 0 */
    pio_sm_exec(pio, BITS_SM, pio_encode_mov(pio_isr, pio_null));
    pio_sm_exec(pio, BITS_SM, pio_encode_jmp(bits_offset + i3c_tgt_bits_offset_slot));
    pio_interrupt_clear(pio, 0);
    pio_sm_set_enabled(pio, BITS_SM, true);
}

bool __not_in_flash_func(phy_tx_full)(void)   { return pio_sm_is_tx_fifo_full(BITS_PIO, BITS_SM); }
void __not_in_flash_func(phy_tx_put)(uint32_t w) { pio_sm_put(BITS_PIO, BITS_SM, w); }
bool __not_in_flash_func(phy_rx_empty)(void)  { return pio_sm_is_rx_fifo_empty(BITS_PIO, BITS_SM); }
uint32_t __not_in_flash_func(phy_rx_get)(void) { return pio_sm_get(BITS_PIO, BITS_SM); }
bool __not_in_flash_func(phy_cond_empty)(void) { return pio_sm_is_rx_fifo_empty(COND_PIO, COND_SM); }
uint32_t __not_in_flash_func(phy_cond_get)(void) { return pio_sm_get(COND_PIO, COND_SM); }
bool __not_in_flash_func(phy_halted)(void)    { return pio_interrupt_get(BITS_PIO, 0); }

bool __not_in_flash_func(phy_bus_idle_levels)(void)
{
    uint32_t m = (1u << I3C_SDA_PIN) | (1u << I3C_SCL_PIN);
    return (sio_hw->gpio_in & m) == m;
}

uint32_t __not_in_flash_func(phy_time_us)(void) { return time_us_32(); }
