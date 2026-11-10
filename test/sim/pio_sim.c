/* SPDX-License-Identifier: MIT */
#include "pio_sim.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void pio_sim_reset(pio_sm_sim_t *sm, unsigned pc)
{
    sm->pc = pc;
    sm->x = sm->y = sm->isr = sm->osr = 0;
    sm->isr_cnt = 0;
    sm->osr_cnt = 32;           /* empty */
    sm->delay = 0;
    sm->tx_n = sm->rx_n = 0;
    sm->irq0 = false;
    sm->irq_waiting = false;
}

bool pio_sim_tx_put(pio_sm_sim_t *sm, uint32_t w)
{
    if (sm->tx_n >= 4) return false;
    sm->tx[sm->tx_n++] = w;
    return true;
}

bool pio_sim_rx_get(pio_sm_sim_t *sm, uint32_t *w)
{
    if (!sm->rx_n) return false;
    *w = sm->rx[0];
    memmove(sm->rx, sm->rx + 1, (--sm->rx_n) * sizeof(uint32_t));
    return true;
}

static uint32_t out_bits(pio_sm_sim_t *sm, unsigned n)
{
    uint32_t m = n == 32 ? 0xFFFFFFFFu : ((1u << n) - 1u);
    uint32_t v;
    if (sm->out_right) { v = sm->osr & m; sm->osr = n == 32 ? 0 : sm->osr >> n; }
    else { v = (sm->osr >> (32 - n)) & m; sm->osr = n == 32 ? 0 : sm->osr << n; }
    sm->osr_cnt += n;
    return v;
}

static void advance(pio_sm_sim_t *sm)
{
    if (sm->pc == sm->wrap_top) sm->pc = sm->wrap_bottom;
    else sm->pc = (sm->pc + 1) & 31;
}

void pio_sim_step(pio_sm_sim_t *sm, uint32_t pins)
{
    if (sm->delay) { sm->delay--; return; }
    uint16_t ins = sm->prog[sm->pc];
    unsigned op = ins >> 13;
    unsigned dly = (ins >> 8) & 0x1F;
    unsigned arg = ins & 0xFF;
    bool stall = false, jumped = false;

    switch (op) {
    case 0: { /* JMP */
        unsigned cond = (arg >> 5) & 7, addr = arg & 31;
        bool take = false;
        switch (cond) {
        case 0: take = true; break;
        case 1: take = sm->x == 0; break;
        case 2: take = sm->x != 0; sm->x--; break;
        case 3: take = sm->y == 0; break;
        case 4: take = sm->y != 0; sm->y--; break;
        case 5: take = sm->x != sm->y; break;
        case 6: take = (pins >> sm->jmp_pin) & 1; break;
        case 7: take = sm->osr_cnt < 32; break;
        }
        if (take) { sm->pc = addr; jumped = true; }
        break;
    }
    case 1: { /* WAIT */
        unsigned pol = (arg >> 7) & 1, src = (arg >> 5) & 3, idx = arg & 31;
        unsigned v;
        if (src == 1) v = (pins >> (sm->in_base + idx)) & 1;
        else if (src == 0) v = (pins >> idx) & 1;
        else { fprintf(stderr, "pio_sim: wait irq unsupported\n"); exit(2); }
        if (v != pol) stall = true;
        break;
    }
    case 2: { /* IN */
        unsigned src = (arg >> 5) & 7, n = arg & 31; if (!n) n = 32;
        uint32_t v;
        if (src == 0) v = pins >> sm->in_base;
        else if (src == 3) v = 0;
        else if (src == 1) v = sm->x;
        else if (src == 2) v = sm->y;
        else { fprintf(stderr, "pio_sim: in src %u unsupported\n", src); exit(2); }
        v &= n == 32 ? 0xFFFFFFFFu : ((1u << n) - 1u);
        if (sm->in_left) sm->isr = (n == 32 ? 0 : sm->isr << n) | v;
        else sm->isr = (n == 32 ? 0 : sm->isr >> n) | (v << (32 - n));
        sm->isr_cnt += n;
        break;
    }
    case 3: { /* OUT */
        unsigned dst = (arg >> 5) & 7, n = arg & 31; if (!n) n = 32;
        uint32_t v = out_bits(sm, n);
        switch (dst) {
        case 0: sm->pinval = v & 1; break;
        case 1: sm->x = v; break;
        case 2: sm->y = v; break;
        case 3: break;
        case 4: sm->pindir = v & 1; break;
        case 5: sm->pc = v & 31; jumped = true; break;
        default: fprintf(stderr, "pio_sim: out dst %u unsupported\n", dst); exit(2);
        }
        break;
    }
    case 4: { /* PUSH / PULL */
        bool pull = (arg >> 7) & 1, block = (arg >> 5) & 1;
        if (pull) {
            if (sm->tx_n) {
                sm->osr = sm->tx[0];
                memmove(sm->tx, sm->tx + 1, (--sm->tx_n) * sizeof(uint32_t));
                sm->osr_cnt = 0;
            } else if (block) stall = true;
            else { sm->osr = sm->x; sm->osr_cnt = 0; }
        } else {
            if (sm->rx_n < 4) { sm->rx[sm->rx_n++] = sm->isr; sm->isr = 0; sm->isr_cnt = 0; }
            else if (block) stall = true;
            else { sm->isr = 0; sm->isr_cnt = 0; }   /* dropped */
        }
        break;
    }
    case 5: { /* MOV */
        unsigned dst = (arg >> 5) & 7, opn = (arg >> 3) & 3, src = arg & 7;
        uint32_t v;
        switch (src) {
        case 0: v = pins >> sm->in_base; break;
        case 1: v = sm->x; break;
        case 2: v = sm->y; break;
        case 3: v = 0; break;
        case 6: v = sm->isr; break;
        case 7: v = sm->osr; break;
        default: fprintf(stderr, "pio_sim: mov src %u unsupported\n", src); exit(2);
        }
        if (opn == 1) v = ~v;
        switch (dst) {
        case 1: sm->x = v; break;
        case 2: sm->y = v; break;
        case 6: sm->isr = v; sm->isr_cnt = 0; break;
        case 7: sm->osr = v; sm->osr_cnt = 0; break;
        default: fprintf(stderr, "pio_sim: mov dst %u unsupported\n", dst); exit(2);
        }
        break;
    }
    case 6: { /* IRQ */
        bool clr = (arg >> 6) & 1, wait = (arg >> 5) & 1;
        if ((arg & 7) != 0) { fprintf(stderr, "pio_sim: only irq 0\n"); exit(2); }
        if (clr) sm->irq0 = false;
        else {
            /* "irq wait": set the flag once, then stall until it is cleared */
            if (!sm->irq_waiting) {
                sm->irq0 = true;
                if (wait) { sm->irq_waiting = true; stall = true; }
            } else if (sm->irq0) {
                stall = true;
            } else {
                sm->irq_waiting = false;
            }
        }
        break;
    }
    case 7: { /* SET */
        unsigned dst = (arg >> 5) & 7, v = arg & 31;
        switch (dst) {
        case 0: sm->pinval = v & 1; break;
        case 1: sm->x = v; break;
        case 2: sm->y = v; break;
        case 4: sm->pindir = v & 1; break;
        default: fprintf(stderr, "pio_sim: set dst %u unsupported\n", dst); exit(2);
        }
        break;
    }
    }
    if (stall) return;              /* re-execute next cycle, no delay */
    if (!jumped) advance(sm);
    sm->delay = dly;
}
