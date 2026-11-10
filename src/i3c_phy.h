/*
 * i3c_phy - thin layer around the two PIO state machines.
 *
 * The protocol layer (i3c_target.c) only talks to the bus through these
 * functions, so it can run unchanged in the host simulator (test/sim).
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef I3C_PHY_H
#define I3C_PHY_H

#include <stdbool.h>
#include <stdint.h>

/* Bus condition tokens produced by the START/STOP detector. */
#define PHY_COND_START 1u   /* START or repeated START */
#define PHY_COND_STOP  2u

/* Entry addresses of the bit-engine sub-programs (set by phy_init). */
extern uint8_t phy_entry_xfer;
extern uint8_t phy_entry_arb;
extern uint8_t phy_entry_tbit;

void phy_init(void);

/* Abort whatever the bit engine is doing: release SDA, drop queued slots and
 * partially received bits, clear the halt flag, and wait for a new slot.   */
void phy_reset(void);

bool     phy_tx_full(void);
void     phy_tx_put(uint32_t slot);   /* caller checks phy_tx_full() first */
bool     phy_rx_empty(void);
uint32_t phy_rx_get(void);
bool     phy_cond_empty(void);
uint32_t phy_cond_get(void);
bool     phy_halted(void);            /* bit engine executed 'halt'        */
bool     phy_bus_idle_levels(void);   /* SCL and SDA both high right now   */
uint32_t phy_time_us(void);

/* ------------------------------------------------------------------------- */
/* Slot command encoders (see i3c_target.pio for the formats)                */
/* ------------------------------------------------------------------------- */

/* Generic transfer of n (1..11) bits. drive_mask / level_mask: bit (n-1-i)
 * describes the i-th bit on the wire, i.e. MSB-first like the data itself. */
static inline uint32_t slot_xfer(unsigned n, uint32_t drive_mask, uint32_t level_mask)
{
    uint32_t w = phy_entry_xfer | ((uint32_t)(n - 1u) << 5);
    for (unsigned i = 0; i < n; i++) {
        unsigned b = n - 1u - i;
        w |= ((level_mask >> b) & 1u) << (9u + 2u * i);
        w |= ((drive_mask >> b) & 1u) << (10u + 2u * i);
    }
    return w;
}

/* Open-drain arbitration of n (1..11) bits, MSB first. A '0' bit pulls SDA
 * low, a '1' bit releases it.                                               */
static inline uint32_t slot_arb(unsigned n, uint32_t value)
{
    uint32_t w = phy_entry_arb | ((uint32_t)(n - 1u) << 5);
    for (unsigned i = 0; i < n; i++) {
        uint32_t low = ((value >> (n - 1u - i)) & 1u) ^ 1u;
        w |= low << (9u + 2u * i);
        w |= low << (10u + 2u * i);
    }
    return w;
}

/* T-bit after a byte we transmit. more=true: T=1, more data follows.       */
static inline uint32_t slot_tbit(bool more)
{
    uint32_t t = more ? 1u : 0u;
    return phy_entry_tbit | (t << 5) | (t << 6);
}

/* Common slots */
static inline uint32_t slot_rx(unsigned n)      { return slot_xfer(n, 0, 0); }  /* receive only   */
static inline uint32_t slot_ack(void)           { return slot_xfer(1, 1, 0); }  /* drive SDA low  */
static inline uint32_t slot_nack(void)          { return slot_xfer(1, 0, 0); }  /* leave SDA high */
static inline uint32_t slot_tx8(uint8_t b)      { return slot_xfer(8, 0xFF, b); } /* push-pull byte */
/* Legacy I2C write byte: receive 8 bits, then pull SDA low for the ACK.    */
static inline uint32_t slot_i2c_rx_ack(void)    { return slot_xfer(9, 0x001, 0x000); }

#endif /* I3C_PHY_H */
