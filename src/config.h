/*
 * pico-i3c-slave - compile-time configuration
 *
 * Every value here can be overridden from CMake, e.g.
 *   cmake -DI3C_STATIC_ADDR=0x50 -DI3C_SDA_PIN=16 ..
 * (CMakeLists.txt forwards the cache variables as compile definitions).
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef I3C_CONFIG_H
#define I3C_CONFIG_H

/* ------------------------------------------------------------------------- */
/* Pins                                                                      */
/* ------------------------------------------------------------------------- */

/* SDA pin. Default GPIO16, the same pin the I3CBlaster uses for SDA.        */
#ifndef I3C_SDA_PIN
#define I3C_SDA_PIN 16
#endif

/* SCL pin. Default GPIO17. The PIO programs address SCL relative to SDA,
 * so SCL MUST be SDA + 1.                                                    */
#ifndef I3C_SCL_PIN
#define I3C_SCL_PIN 17
#endif

#if (I3C_SCL_PIN) != (I3C_SDA_PIN) + 1
#error "I3C_SCL_PIN must be I3C_SDA_PIN + 1 (the PIO programs read SCL as 'pin 1')"
#endif

/* ------------------------------------------------------------------------- */
/* Device identity                                                           */
/* ------------------------------------------------------------------------- */

/*
 * 48-bit Provisional ID, as sent MSB first during ENTDAA and GETPID:
 *
 *   [47:33] MIPI manufacturer ID   0x7FFF  (placeholder, NOT an assigned ID)
 *   [32]    PID type selector      0       (vendor-fixed value, not random)
 *   [31:16] part ID                0x3C01
 *   [15:12] instance ID            0x0
 *   [11:0]  vendor-defined         0x001
 *
 *   => 0xFFFE_3C01_0001  (bytes on the wire: FF FE 3C 01 00 01)
 *
 * Because the manufacturer ID is all ones the PID is numerically large, so
 * this test target LOSES ENTDAA arbitration against virtually every real
 * device on the same bus and gets its address last.
 */
#ifndef I3C_PID
#define I3C_PID 0xFFFE3C010001ULL
#endif

/*
 * Bus Characteristics Register
 *   [7:6] 00 = I3C target          [5] 0 = no advanced capabilities
 *   [4]   0  = not a virtual target [3] 0 = not offline capable
 *   [2]   0  = no IBI payload       [1] 0 = NOT IBI request capable
 *   [0]   1  = has a max data speed limitation (answer GETMXDS)
 * Hot-Join is not supported either.
 */
#ifndef I3C_BCR
#define I3C_BCR 0x01
#endif

/* Device Characteristics Register: 0x00 = generic device                   */
#ifndef I3C_DCR
#define I3C_DCR 0x00
#endif

/* 7-bit static address, used for SETDASA / SETAASA and (before a dynamic
 * address exists) for private transfers and legacy I2C transfers.          */
#ifndef I3C_STATIC_ADDR
#define I3C_STATIC_ADDR 0x42
#endif

/* Answer private SDR transfers on the static address while no dynamic
 * address is assigned (handy for first bring-up). 0 = strict: SA is only
 * used by SETDASA/SETAASA.                                                  */
#ifndef I3C_RESPOND_ON_STATIC_ADDR
#define I3C_RESPOND_ON_STATIC_ADDR 1
#endif

/* Answer legacy I2C transfers (START + static address, no 0x7E header)
 * while no dynamic address is assigned.                                    */
#ifndef I3C_LEGACY_I2C
#define I3C_LEGACY_I2C 1
#endif

/* Size of the register/memory buffer. Must be 256 (8-bit pointer).          */
#define I3C_MEM_SIZE 256

/* Max write/read length reported by GETMWL / GETMRL (bytes).               */
#ifndef I3C_MWL
#define I3C_MWL 256
#endif
#ifndef I3C_MRL
#define I3C_MRL 256
#endif

/* GETMXDS answer: max write / max read data speed byte.
 * 0x04 = "2 MHz max sustained data rate", the slowest value the spec can
 * encode. See README "Speed" for what this firmware really sustains.        */
#ifndef I3C_MXDS_WR
#define I3C_MXDS_WR 0x04
#endif
#ifndef I3C_MXDS_RD
#define I3C_MXDS_RD 0x04
#endif

/* ------------------------------------------------------------------------- */
/* System                                                                    */
/* ------------------------------------------------------------------------- */

/* System clock in kHz. Faster = lower reaction latency on the bus.
 * 200 MHz is supported by pico-sdk >= 2.1 on RP2040 (core voltage is raised
 * to 1.15 V for it). Use 125000 to stay at the stock clock.                */
#ifndef I3C_SYS_CLK_KHZ
#define I3C_SYS_CLK_KHZ 200000
#endif

/* Number of log records buffered between the bus core and the USB core.   */
#define I3C_LOG_DEPTH 64
/* Max payload bytes stored per log record (longer transfers are truncated
 * in the log only, never on the bus).                                       */
#define I3C_LOG_MAX_DATA 48

#endif /* I3C_CONFIG_H */
