/*
 * i3c_log - event records from the bus core (core 1) to the USB core (core 0)
 *
 * Single-producer / single-consumer ring buffer, no locks. Core 1 never
 * blocks: if the ring is full the record is dropped and counted.
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef I3C_LOG_H
#define I3C_LOG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "config.h"

typedef enum {
    EV_CCC_BC = 1,     /* broadcast CCC: ccc, data = payload bytes          */
    EV_CCC_DIRECT,     /* direct CCC header: ccc, data = defining bytes      */
    EV_CCC_DIR_W,      /* direct CCC write to us: addr, ccc, data            */
    EV_CCC_DIR_R,      /* direct CCC read from us: addr, ccc, data we sent   */
    EV_PRIV_W,         /* SDR private write: addr, data                     */
    EV_PRIV_R,         /* SDR private read: addr, arg = start ptr, data sent */
    EV_I2C_W,          /* legacy I2C write                                  */
    EV_I2C_R,          /* legacy I2C read                                   */
    EV_DA,             /* dynamic address change: arg = da_reason_t,
                          addr = new DA (0 = none), ccc = old DA            */
    EV_ENTDAA_LOST,    /* lost ENTDAA arbitration: data = bytes we got out  */
    EV_ERR,            /* arg = err_t, addr/ccc = context                   */
    EV_HDR,            /* HDR entered (ccc = ENTHDRx), ignored until idle    */
    EV_BUS,            /* arg: 1=START, 2=Sr, 3=STOP                         */
    EV_ADDR,           /* address phase: addr, arg bits: 0=RnW, 1=ACK, 2=after START (not Sr) */
} ev_type_t;

typedef enum {
    DA_ENTDAA = 1, DA_SETDASA, DA_SETNEWDA, DA_SETAASA, DA_RSTDAA_BC, DA_RSTDAA_DIRECT,
} da_reason_t;

typedef enum {
    ERR_PARITY_CCC = 1,   /* T-bit (parity) wrong on the CCC code byte        */
    ERR_PARITY_DATA,      /* T-bit wrong on a written data byte                */
    ERR_PARITY_DAA,       /* parity wrong on the address assigned by ENTDAA   */
    ERR_BAD_DA,           /* controller tried to assign an invalid address    */
} err_t;

typedef struct {
    uint32_t t_us;
    uint8_t  type;
    uint8_t  addr;
    uint8_t  ccc;
    uint8_t  arg;
    uint16_t len;         /* full length on the bus                          */
    uint8_t  data[I3C_LOG_MAX_DATA]; /* first min(len, MAX) bytes           */
} i3c_log_rec_t;

void   i3c_log_push(const i3c_log_rec_t *r);   /* producer (core 1)         */
bool   i3c_log_pop(i3c_log_rec_t *r);          /* consumer (core 0)         */
uint32_t i3c_log_dropped(void);

/* Render one record as a single text line (no trailing newline).          */
size_t i3c_log_format(const i3c_log_rec_t *r, char *buf, size_t size);

const char *i3c_ccc_name(uint8_t ccc);

#endif /* I3C_LOG_H */
