/*
 * MIPI I3C Common Command Codes used by this target.
 * Broadcast CCCs are 0x00..0x7F, direct CCCs are 0x80..0xFE.
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef I3C_CCC_H
#define I3C_CCC_H

#define I3C_BCAST_ADDR      0x7E

/* broadcast */
#define CCC_ENEC_B          0x00
#define CCC_DISEC_B         0x01
#define CCC_ENTAS0_B        0x02
#define CCC_ENTAS1_B        0x03
#define CCC_ENTAS2_B        0x04
#define CCC_ENTAS3_B        0x05
#define CCC_RSTDAA_B        0x06
#define CCC_ENTDAA          0x07
#define CCC_DEFTGTS         0x08
#define CCC_SETMWL_B        0x09
#define CCC_SETMRL_B        0x0A
#define CCC_ENTTM           0x0B
#define CCC_SETBUSCON       0x0C
#define CCC_ENDXFER_B       0x12
#define CCC_ENTHDR0         0x20
#define CCC_ENTHDR7         0x27
#define CCC_SETXTIME_B      0x28
#define CCC_SETAASA         0x29
#define CCC_RSTACT_B        0x2A
#define CCC_DEFGRPA         0x2B
#define CCC_RSTGRPA_B       0x2C

/* direct */
#define CCC_ENEC_D          0x80
#define CCC_DISEC_D         0x81
#define CCC_ENTAS0_D        0x82
#define CCC_ENTAS3_D        0x85
#define CCC_RSTDAA_D        0x86   /* deprecated in I3C v1.1, still handled */
#define CCC_SETDASA         0x87
#define CCC_SETNEWDA        0x88
#define CCC_SETMWL_D        0x89
#define CCC_SETMRL_D        0x8A
#define CCC_GETMWL          0x8B
#define CCC_GETMRL          0x8C
#define CCC_GETPID          0x8D
#define CCC_GETBCR          0x8E
#define CCC_GETDCR          0x8F
#define CCC_GETSTATUS       0x90
#define CCC_GETACCCR        0x91
#define CCC_ENDXFER_D       0x92
#define CCC_SETBRGTGT       0x93
#define CCC_GETMXDS         0x94
#define CCC_GETCAPS         0x95
#define CCC_SETROUTE        0x96
#define CCC_SETXTIME_D      0x98
#define CCC_GETXTIME        0x99
#define CCC_RSTACT_D        0x9A

/* ENEC / DISEC event bits */
#define CCC_EVT_INT         0x01
#define CCC_EVT_CR          0x02
#define CCC_EVT_HJ          0x08

#endif /* I3C_CCC_H */
