/*------------------------------------------------------------------------------
* cssr.h : header file of cssr functions
*
* Copyright (c) 2024-2026 Cabinet Office, Japan, All rights reserved.
* Copyright (c) 2022-2026, Lighthouse Technology & Consulting Co. Ltd., All rights reserved.
*
* author  : LHTC
* history : 2020/02/03 1.0  new
*           2025/03/18 1.1  change CSSR_MAXSYS
*                           add CSSR_MAXNSIG
*           2026/10/30 1.2  add sig_bit, sig_mask_bd3 to cssr_mask_t
*                           (BDS-3 satellites in GNSS ID 3)
*
*-----------------------------------------------------------------------------*/
#ifndef CSSR_H
#define CSSR_H

#include "rtklib.h"

/* constants -----------------------------------------------------------------*/

#define RTCM3PREAMB     0xD3        /* rtcm ver.3 frame preamble */
#define CSSR_MAXSYS     16          /* max number of gnss */
#define CSSR_MSGTYPE    4073        /* CSSR Messages type defined by IS-QZSS-L6 */
#define CSSR_MAXNSIG    16          /* max number of signals */

/* type definitions ----------------------------------------------------------*/

typedef struct {                        /* cssr mask type */
    gtime_t epoch;                      /* gps epoch time in sub-type 1 */
    int nsys;                           /* number of gnss */
    int sys [CSSR_MAXSYS];              /* satellite system*/
    int nsat[CSSR_MAXSYS];              /* number of satellites */
    int nsig[CSSR_MAXSYS];              /* number of signals */
    int ncell[CSSR_MAXSYS];             /* number of cells */
    int sat_mask[CSSR_MAXSYS][MAXSAT];  /* satellite mask */
    int sig_mask[CSSR_MAXSYS][CSSR_MAXNSIG]; /* signal mask */
    int sig_bit[CSSR_MAXSYS][CSSR_MAXNSIG]; /* signal mask bit (1:set) */
    int sig_mask_bd3[CSSR_MAXNSIG];     /* signal mask of BDS-3 in GNSS ID 3 */
    int cell_mask[CSSR_MAXSYS][CSSR_MAXNSIG*MAXSAT]; /* cell mask */
    int nbit;                           /* bit length of subtype 1 */
    int iod;                            /* IOD SSR of subtype 1 */
} cssr_mask_t;

typedef struct {                           /* cssr paramter type */
    cssr_mask_t cssr_mask;                 /* cssr mask */
} cssr_t;

/* function prototypes -------------------------------------------------------*/

extern void cssr_dump_set(int type);
extern void cssr_dump_unset();
extern int cssr_init(cssr_t *cssr);
extern int decode_cssr(rtcm_t *rtcm, cssr_t *cssr);
#endif /* CSSR_H */

