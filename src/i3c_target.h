/*
 * i3c_target - protocol state machine of the I3C SDR target.
 *
 * Runs on core 1 in a tight loop: i3c_target_init() once, then call
 * i3c_target_poll() forever. Talks to the bus only through i3c_phy.h and
 * reports what happened through i3c_log.h.
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef I3C_TARGET_H
#define I3C_TARGET_H

#include <stdbool.h>
#include <stdint.h>
#include "config.h"

typedef struct {
    uint8_t  dyn_addr;        /* 0 = no dynamic address                      */
    uint8_t  static_addr;
    uint8_t  events;          /* ENEC/DISEC mask (stored only, no IBI/HJ)     */
    uint8_t  mem_ptr;         /* register pointer for private transfers      */
    uint16_t mwl, mrl;
    uint32_t frames;          /* START..STOP frames seen                     */
    uint32_t naks;            /* addresses not for us                        */
    uint32_t parity_errors;
    uint32_t entdaa_lost;
} i3c_target_status_t;

void i3c_target_init(void);
void i3c_target_poll(void);

/* Snapshot for the USB console (racy but harmless). */
void i3c_target_get_status(i3c_target_status_t *s);

/* The 256-byte register/memory buffer (read/write by private transfers). */
extern volatile uint8_t i3c_mem[I3C_MEM_SIZE];

#endif /* I3C_TARGET_H */
