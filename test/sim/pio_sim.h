/* Minimal cycle-level model of one RP2040 PIO state machine (subset of the
 * instruction set that the target programs use: JMP, WAIT pin, IN pins,
 * OUT pins/pindirs/x/y/pc, PUSH, PULL, MOV, IRQ, SET). No side-set.
 * SPDX-License-Identifier: MIT */
#ifndef PIO_SIM_H
#define PIO_SIM_H
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    const uint16_t *prog;
    unsigned wrap_bottom, wrap_top;
    unsigned in_base;        /* index into the pin vector (0 = SDA, 1 = SCL) */
    unsigned jmp_pin;
    bool out_right, in_left;

    unsigned pc;
    uint32_t x, y, isr, osr;
    unsigned isr_cnt, osr_cnt;
    unsigned delay;
    uint32_t tx[4], rx[4];
    unsigned tx_n, rx_n;
    bool irq0;
    bool irq_waiting;
    /* outputs (SDA only) */
    int pindir, pinval;
} pio_sm_sim_t;

void pio_sim_reset(pio_sm_sim_t *sm, unsigned pc);
/* Execute one clock. pins: bit0 = SDA, bit1 = SCL (already synchronised). */
void pio_sim_step(pio_sm_sim_t *sm, uint32_t pins);

bool pio_sim_tx_put(pio_sm_sim_t *sm, uint32_t w);
bool pio_sim_rx_get(pio_sm_sim_t *sm, uint32_t *w);
#endif
