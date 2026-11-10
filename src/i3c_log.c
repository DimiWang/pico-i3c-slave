/*
 * i3c_log - SPSC ring buffer + text formatting of bus events.
 *
 * SPDX-License-Identifier: MIT
 */
#include "i3c_log.h"
#include "i3c_ccc.h"

#include <stdio.h>
#include <string.h>

#ifdef PICO_BUILD
#include "pico.h"
#define LOG_RAM_FUNC(f) __not_in_flash_func(f)
#else
#define LOG_RAM_FUNC(f) f
#endif

static i3c_log_rec_t ring[I3C_LOG_DEPTH];
static volatile uint32_t head;   /* written by producer */
static volatile uint32_t tail;   /* written by consumer */
static volatile uint32_t dropped;

void LOG_RAM_FUNC(i3c_log_push)(const i3c_log_rec_t *r)
{
    uint32_t h = head;
    if (h - __atomic_load_n(&tail, __ATOMIC_ACQUIRE) >= I3C_LOG_DEPTH) {
        dropped++;
        return;
    }
    ring[h % I3C_LOG_DEPTH] = *r;
    __atomic_store_n(&head, h + 1, __ATOMIC_RELEASE);
}

bool i3c_log_pop(i3c_log_rec_t *r)
{
    uint32_t t = tail;
    if (__atomic_load_n(&head, __ATOMIC_ACQUIRE) == t)
        return false;
    *r = ring[t % I3C_LOG_DEPTH];
    __atomic_store_n(&tail, t + 1, __ATOMIC_RELEASE);
    return true;
}

uint32_t i3c_log_dropped(void) { return dropped; }

const char *i3c_ccc_name(uint8_t c)
{
    if (c >= CCC_ENTHDR0 && c <= CCC_ENTHDR7) return "ENTHDRx";
    switch (c) {
    case CCC_ENEC_B:     return "ENEC";
    case CCC_DISEC_B:    return "DISEC";
    case CCC_ENTAS0_B:   return "ENTAS0";
    case CCC_ENTAS1_B:   return "ENTAS1";
    case CCC_ENTAS2_B:   return "ENTAS2";
    case CCC_ENTAS3_B:   return "ENTAS3";
    case CCC_RSTDAA_B:   return "RSTDAA";
    case CCC_ENTDAA:     return "ENTDAA";
    case CCC_DEFTGTS:    return "DEFTGTS";
    case CCC_SETMWL_B:   return "SETMWL";
    case CCC_SETMRL_B:   return "SETMRL";
    case CCC_ENTTM:      return "ENTTM";
    case CCC_SETBUSCON:  return "SETBUSCON";
    case CCC_ENDXFER_B:  return "ENDXFER";
    case CCC_SETXTIME_B: return "SETXTIME";
    case CCC_SETAASA:    return "SETAASA";
    case CCC_RSTACT_B:   return "RSTACT";
    case CCC_DEFGRPA:    return "DEFGRPA";
    case CCC_RSTGRPA_B:  return "RSTGRPA";
    case CCC_ENEC_D:     return "ENEC(direct)";
    case CCC_DISEC_D:    return "DISEC(direct)";
    case CCC_RSTDAA_D:   return "RSTDAA(direct)";
    case CCC_SETDASA:    return "SETDASA";
    case CCC_SETNEWDA:   return "SETNEWDA";
    case CCC_SETMWL_D:   return "SETMWL(direct)";
    case CCC_SETMRL_D:   return "SETMRL(direct)";
    case CCC_GETMWL:     return "GETMWL";
    case CCC_GETMRL:     return "GETMRL";
    case CCC_GETPID:     return "GETPID";
    case CCC_GETBCR:     return "GETBCR";
    case CCC_GETDCR:     return "GETDCR";
    case CCC_GETSTATUS:  return "GETSTATUS";
    case CCC_GETACCCR:   return "GETACCCR";
    case CCC_ENDXFER_D:  return "ENDXFER(direct)";
    case CCC_GETMXDS:    return "GETMXDS";
    case CCC_GETCAPS:    return "GETCAPS";
    case CCC_GETXTIME:   return "GETXTIME";
    case CCC_RSTACT_D:   return "RSTACT(direct)";
    default:             return "?";
    }
}

static const char *da_reason(uint8_t r)
{
    switch (r) {
    case DA_ENTDAA:        return "ENTDAA";
    case DA_SETDASA:       return "SETDASA";
    case DA_SETNEWDA:      return "SETNEWDA";
    case DA_SETAASA:       return "SETAASA";
    case DA_RSTDAA_BC:     return "RSTDAA";
    case DA_RSTDAA_DIRECT: return "RSTDAA(direct)";
    default:               return "?";
    }
}

static size_t put_hex(char *p, size_t room, const i3c_log_rec_t *r)
{
    size_t n = 0;
    unsigned shown = r->len < I3C_LOG_MAX_DATA ? r->len : I3C_LOG_MAX_DATA;
    for (unsigned i = 0; i < shown && n + 4 < room; i++)
        n += (size_t)snprintf(p + n, room - n, " %02X", r->data[i]);
    if (shown < r->len && n + 16 < room)
        n += (size_t)snprintf(p + n, room - n, " ...(+%u)", (unsigned)(r->len - shown));
    return n;
}

size_t i3c_log_format(const i3c_log_rec_t *r, char *buf, size_t size)
{
    size_t n = (size_t)snprintf(buf, size, "[%5lu.%06lu] ",
                                (unsigned long)(r->t_us / 1000000u),
                                (unsigned long)(r->t_us % 1000000u));
#define ADD(...) do { if (n < size) n += (size_t)snprintf(buf + n, size - n, __VA_ARGS__); } while (0)
    switch (r->type) {
    case EV_CCC_BC:
        ADD("CCC  bcast  0x%02X %-9s", r->ccc, i3c_ccc_name(r->ccc));
        if (r->len) { ADD(" data:"); n += put_hex(buf + n, size - n, r); }
        break;
    case EV_CCC_DIRECT:
        ADD("CCC  direct 0x%02X %-9s", r->ccc, i3c_ccc_name(r->ccc));
        if (r->len) { ADD(" defbyte:"); n += put_hex(buf + n, size - n, r); }
        break;
    case EV_CCC_DIR_W:
        ADD("CCC  direct 0x%02X %-9s -> addr 0x%02X W len %u data:", r->ccc,
            i3c_ccc_name(r->ccc), r->addr, r->len);
        n += put_hex(buf + n, size - n, r);
        break;
    case EV_CCC_DIR_R:
        ADD("CCC  direct 0x%02X %-9s <- addr 0x%02X R len %u data:", r->ccc,
            i3c_ccc_name(r->ccc), r->addr, r->len);
        n += put_hex(buf + n, size - n, r);
        break;
    case EV_PRIV_W:
        if (r->len == 0) {
            ADD("SDR  write  addr 0x%02X len 0 (address probe, e.g. i3c_scan)", r->addr);
            break;
        }
        ADD("SDR  write  addr 0x%02X len %u (ptr=0x%02X) data:", r->addr, r->len, r->data[0]);
        n += put_hex(buf + n, size - n, r);
        break;
    case EV_PRIV_R:
        ADD("SDR  read   addr 0x%02X len %u (mem[0x%02X..]) data:", r->addr, r->len, r->arg);
        n += put_hex(buf + n, size - n, r);
        break;
    case EV_I2C_W:
        ADD("I2C  write  addr 0x%02X len %u data:", r->addr, r->len);
        n += put_hex(buf + n, size - n, r);
        break;
    case EV_I2C_R:
        ADD("I2C  read   addr 0x%02X len %u (mem[0x%02X..]) data:", r->addr, r->len, r->arg);
        n += put_hex(buf + n, size - n, r);
        break;
    case EV_DA:
        if (r->addr)
            ADD("DAA  %s: dynamic address 0x%02X assigned", da_reason(r->arg), r->addr);
        else
            ADD("DAA  %s: dynamic address cleared", da_reason(r->arg));
        if (r->ccc) ADD(" (was 0x%02X)", r->ccc);
        else        ADD(" (was none)");
        if (r->len) { ADD(" PID+BCR+DCR:"); n += put_hex(buf + n, size - n, r); }
        break;
    case EV_ENTDAA_LOST:
        ADD("DAA  ENTDAA: lost arbitration in PID/BCR/DCR byte %u (lower PID won), "
            "will take part in the next round", (unsigned)r->arg + 1u);
        break;
    case EV_ERR:
        switch (r->arg) {
        case ERR_PARITY_CCC:  ADD("ERR  parity error on CCC byte 0x%02X", r->ccc); break;
        case ERR_PARITY_DATA: ADD("ERR  parity error on data byte 0x%02X (addr 0x%02X)", r->ccc, r->addr); break;
        case ERR_PARITY_DAA:  ADD("ERR  ENTDAA address byte 0x%02X has bad parity -> NACK", r->ccc); break;
        case ERR_BAD_DA:      ADD("ERR  refused invalid dynamic address 0x%02X", r->addr); break;
        default:              ADD("ERR  code %u", r->arg); break;
        }
        break;
    case EV_HDR:
        ADD("HDR  0x%02X %s: HDR is not supported, ignoring bus until it is idle", r->ccc,
            i3c_ccc_name(r->ccc));
        break;
    default:
        ADD("??? event %u", r->type);
        break;
    }
#undef ADD
    return n < size ? n : size - 1;
}
