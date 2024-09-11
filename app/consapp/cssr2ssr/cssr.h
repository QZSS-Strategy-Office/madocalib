/*------------------------------------------------------------------------------
* cssr.h : header file of cssr functions
*
* Copyright (c) 2024 Cabinet Office, Japan, All rights reserved.
* Copyright (c) 2022-2024, Lighthouse Technology & Consulting Co. Ltd., All rights reserved.
*
* author  : LHTC
* history : 2020/02/03 1.0  new
*
*-----------------------------------------------------------------------------*/
#ifndef CSSR_H
#define CSSR_H

#include "rtklib.h"

/* constants -----------------------------------------------------------------*/

#define RTCM3PREAMB     0xD3        /* rtcm ver.3 frame preamble */
#define CSSR_MAXSYS     10          /* max number of gnss */
#define CSSR_MSGTYPE    4073        /* CSSR Messages type defined by IS-QZSS-L6 */

/* type definitions ----------------------------------------------------------*/

typedef struct {                        /* cssr mask type */
    gtime_t epoch;                      /* gps epoch time in sub-type 1 */
    int nsys;                           /* number of gnss */
    int sys [CSSR_MAXSYS];              /* satellite system*/
    int nsat[CSSR_MAXSYS];              /* number of satellites */
    int nsig[CSSR_MAXSYS];              /* number of signals */
    int ncell[CSSR_MAXSYS];             /* number of cells */
    int sat_mask[CSSR_MAXSYS][MAXSAT];  /* satellite mask */
    int sig_mask[CSSR_MAXSYS][MAXCODE]; /* signal mask */
    int cell_mask[CSSR_MAXSYS][MAXCODE*MAXSAT]; /* cell mask */
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

