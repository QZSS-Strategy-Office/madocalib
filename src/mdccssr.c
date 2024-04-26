/*------------------------------------------------------------------------------
* mdccssr.c : QZSS L6 Signal MADOCA-PPP Compact SSR message decode functions
*
* Copyright (C) 2023 TOSHIBA ELECTRONIC TECHNOLOGIES CORPORATION. All Rights Reserved.
*
* references :
*     [1]  CAO IS-QZSS-MDC-001, February, 2022
*
* version : $Revision:$ $Date:$
* history : 2023/02/01 1.0  new, for MADOCALIB from TETC original tools.
*-----------------------------------------------------------------------------*/
#include "rtklib.h"

/* constants -----------------------------------------------------------------*/

#define L6PREAMB          0x1ACFFC1Du   /* QZSS L6 Message Preamble */
#define L6BYTELEN                 218   /* QZSS L6 Message Byte Length w/o R-S CODE */
#define L6HEAD_BITLEN              49   /* L6 Message Header Bit Length */
#define L6DATA_BITLEN            1695   /* L6 Message Data Part Bit Length */
#define L6DATA_BYTELEN            212   /* L6 Message Data Part Byte Length */

#define RTCM_MN_CSSR             4073   /* Compact SSR Message Number */
                                    
#define MCSSR_BYTELEN            1060   /* MADOCA-PPP Message 5 frames Byte Length */

#define MCSSR_ST_MASK               1   /* Compact SSR Mask */
#define MCSSR_ST_OC                 2   /* Compact SSR GNSS Orbit Correction */
#define MCSSR_ST_CC                 3   /* Compact SSR GNSS Clock Correction */
#define MCSSR_ST_CB                 4   /* Compact SSR GNSS Satellite Code Bias */
#define MCSSR_ST_PB                 5   /* Compact SSR GNSS Satellite Phase Bias */
#define MCSSR_ST_URA                7   /* Compact SSR GNSS URA */

#define MCSSR_VALID_OC             60   /* Nominal Validity Period[s] (Table 4.2.2-3) */
#define MCSSR_VALID_CC             10
#define MCSSR_VALID_CB             60
#define MCSSR_VALID_PB             60
#define MCSSR_VALID_URA            60

#define MCSSR_MAX_SYS               5   /* GNSS ID (Table 4.2.2-7) w/o SBS */
#define MCSSR_SYS_GPS               0
#define MCSSR_SYS_GLO               1
#define MCSSR_SYS_GAL               2
#define MCSSR_SYS_BDS               3
#define MCSSR_SYS_QZS               4

#define MCSSR_MAX_SATMASK          40   /* Satellite Mask (Table 4.2.2-8) */
#define MCSSR_SATMASK_GPS_U  0xFFFFFFFFu /* Note, U and L for -Wlong-long Avoidance */
#define MCSSR_SATMASK_GPS_L  0xFF
#define MCSSR_SATMASK_GLO_U  0xFFFFFF00u
#define MCSSR_SATMASK_GLO_L  0x00
#define MCSSR_SATMASK_GAL_U  0xFFFFFFFFu
#define MCSSR_SATMASK_GAL_L  0xF8
#define MCSSR_SATMASK_BDS_U  0xFFFFFFFFu
#define MCSSR_SATMASK_BDS_L  0xFF
#define MCSSR_SATMASK_QZS_U  0xFFC00000u
#define MCSSR_SATMASK_QZS_L  0x00

#define MCSSR_MAX_SIGMASK          16   /* Signal Mask (Table 4.2.2-9) */

#define MCSSR_INVALID_15BIT    -16384   /* Invalid Value (Table. 4.2.2-11, etc.) */
#define MCSSR_INVALID_13BIT     -4096
#define MCSSR_INVALID_11BIT     -1024

#define MCSSR_MAX_PRN_CHECK         7   /* Max track MADOCA-PPP signal for PRN auto selection */

/* SSR update intervals (Table 4.2.2-6) --------------------------------------*/
static const double ssrudint[16]={
    1,2,5,10,15,30,60,120,240,300,600,900,1800,3600,7200,10800
};

/* Compact SSR Signal mask (Table 4.2.2-9) w/o SBS ---------------------------*/
const uint8_t cssr_sig_gps[16]={
    CODE_L1C,CODE_L1P,CODE_L1W,CODE_L1S,CODE_L1L,CODE_L1X,CODE_L2S,CODE_L2L,
    CODE_L2X,CODE_L2P,CODE_L2W,CODE_L5I,CODE_L5Q,CODE_L5X,       0,       0
};
const uint8_t cssr_sig_glo[16]={
    CODE_L1C,CODE_L1P,CODE_L2C,CODE_L2P,CODE_L4A,CODE_L4B,CODE_L4X,CODE_L6A,
    CODE_L6B,CODE_L6X,CODE_L3I,CODE_L3Q,CODE_L3X,       0,       0,       0
};
const uint8_t cssr_sig_gal[16]={
    CODE_L1B,CODE_L1C,CODE_L1X,CODE_L5I,CODE_L5Q,CODE_L5X,CODE_L7I,CODE_L7Q,
    CODE_L7X,CODE_L8I,CODE_L8Q,CODE_L8X,       0,       0,       0,       0
};
const uint8_t cssr_sig_cmp[16]={
    CODE_L2I,CODE_L2Q,CODE_L2X,CODE_L6I,CODE_L6Q,CODE_L6X,CODE_L7I,CODE_L7Q,
    CODE_L7X,       0,       0,       0,       0,       0,       0,       0
};
const uint8_t cssr_sig_qzs[16]={
    CODE_L1C,CODE_L1S,CODE_L1L,CODE_L1X,CODE_L2S,CODE_L2L,CODE_L2X,CODE_L5I,
    CODE_L5Q,CODE_L5X,       0,       0,       0,       0,       0,       0
};

/* MADOCA-PPP Sub Type Transmission Pattern (Figure 4.2.2-1) -----------------*/
/* note : Change MCSSR_BYTELEN when defining 6 or more. */
static const int framelen[]={       /* frame length(-1 : invalid) */
    -1, 5,-1, 3, 5, 5,-1, 3,        /* ST 0, 1, 2, 3, 4, 5, 6, 7 */
    -1,-1,-1,-1,-1,-1,-1,-1         /* ST 8, 9,10,11,12,13,14,15 */
};

typedef struct {
    gtime_t gt;                     /* current Epoch Time 1s */
    int mgfid;                      /* current Msg. Gen. Facility ID */
    int prn;                        /* current Msg. PRN */
    int prn_chk_cnt;                /* check counter for PRN auto selection */
    uint8_t buff[MCSSR_BYTELEN];    /* Compact-SSR Message */
    int ibuff;                      /* Indicate the end of buff */
    int satlist[MAXSAT];            /* 0 : invalid, 1... : satno(G01...) */
    int gidlist[MAXSAT];            /* GNSS ID */
    int cellmask[MAXSAT];           /* Cellmask */
    int ncell[MAXSAT];              /* No. of cellmask flags */
    int siglist[MCSSR_MAX_SYS][MCSSR_MAX_SIGMASK];/* 0:Terminate,1...:CODE_xxx */
    int nsat[MCSSR_MAX_SYS];        /* No. of sat */
    int nsig[MCSSR_MAX_SYS];        /* No. of signal */
    int ns;                         /* total No. of */
    int maxframe;                   /* set framelen[first subtype] */
    int frame;                      /* frame counter */
    int tow0;                       /* for Hourly Epoch Time 1s */
    int ep0;                        /* for Hourly Epoch Time 1s regresses check */
    int iod;                        /* IOD SSR */
} mcssr_t;

mcssr_t _mcssr;

/* count 16 bit flags --------------------------------------------------------*/
static int flgcnt16(uint16_t flg)
{
    int i,cnt=0;

    for (i=0;i<16;i++) if((flg>>i) & 0x1) cnt++;
    return cnt;
}
/* ---------------------------------------------------------------------------*/
static int gnssid2sys(int gnssid)
{
    switch(gnssid){
        case MCSSR_SYS_GPS : return SYS_GPS;
        case MCSSR_SYS_GLO : return SYS_GLO;
        case MCSSR_SYS_GAL : return SYS_GAL;
        case MCSSR_SYS_BDS : return SYS_CMP;
        case MCSSR_SYS_QZS : return SYS_QZS;
    }
    return -1;
}
/* ---------------------------------------------------------------------------*/
static int svmask2list(uint64_t mask, int gnssid, int *satlist, int *gidlist)
{
    int i,offp,ns=0;
    uint64_t svbit = (uint64_t)1 << (MCSSR_MAX_SATMASK - 1);

    for(i = 0; i < MCSSR_MAX_SATMASK; i++){
        if(mask & svbit) {
            offp = (gnssid == MCSSR_SYS_QZS)? 193 : 1;
            satlist[ns] = satno(gnssid2sys(gnssid),offp+i);
            gidlist[ns] = gnssid;
            ns++;
        }
        svbit >>= 1;
    }
    return ns;
}

/* ---------------------------------------------------------------------------*/
static int sigmask2list(uint16_t mask, int gnssid, int *siglist)
{
    int i,nsig=0;
    const uint8_t *sigs;
    uint16_t sigbit = (uint16_t)1 << (MCSSR_MAX_SIGMASK - 1);

    switch(gnssid){
        case MCSSR_SYS_GPS : sigs = cssr_sig_gps; break;
        case MCSSR_SYS_GLO : sigs = cssr_sig_glo; break;
        case MCSSR_SYS_GAL : sigs = cssr_sig_gal; break;
        case MCSSR_SYS_BDS : sigs = cssr_sig_cmp; break;
        case MCSSR_SYS_QZS : sigs = cssr_sig_qzs; break;
        default: return 0;
    }

    for(i = 0; i < MCSSR_MAX_SIGMASK; i++){
        if(mask & sigbit) {
            siglist[nsig] = sigs[i];
            nsig++;
        }
        sigbit >>= 1;
    }
    return nsig;
}

/* adjust weekly rollover of gps time ----------------------------------------*/
static void adjweek(gtime_t *gt, double tow)
{
    double tow_p;
    int week;
    
    /* if no time, get cpu time */
    if (gt->time == 0) *gt = utc2gpst(timeget());
    tow_p = time2gpst(*gt, &week);
    if      (tow < tow_p - 302400.0) tow+=604800.0;
    else if (tow > tow_p + 302400.0) tow-=604800.0;
    *gt = gpst2time(week, tow);
}

/* decode MADOCA-PPP L6 CSSR header ------------------------------------------*/
static int decode_mcssr_head(mcssr_t *mc, int *sync, int *iod, int *ep,
                             double *udint, int i, int st)
{
    int n, udi;

    n = (st == MCSSR_ST_MASK)? 20: 12;
    *ep    = getbitu(mc->buff, i, n); i+=n; /* GPS or GNSS Hourly Epoch Time 1s */
    udi    = getbitu(mc->buff, i, 4); i+=4; /* SSR Update Interval */
    *sync  = getbitu(mc->buff, i, 1); i+=1; /* Multiple Message Indicator */
    *iod   = getbitu(mc->buff, i, 4); i+=4; /* IOD SSR */
    *udint = ssrudint[udi];

    trace(4,"decode_mcssr_head: st=%d, ep=%d, udi=%d, sync=%d, iod=%d\n",
        st, *ep, udi, *sync, *iod);
    return i;
}

/* decode MADOCA-PPP L6 CSSR ST1 mask message --------------------------------*/
static int decode_mcssr_mask(mcssr_t *mc, int i)
{
    int j, k, sync, iod, ep, ngnss, gnssid, cmaflg, nsat, nsig;
    uint64_t svmask,gnssmask;
    uint16_t sigmask,cellmask;
    double udint;

    i = decode_mcssr_head(mc, &sync, &iod, &ep, &udint, i, MCSSR_ST_MASK);

    mc->tow0 = ep / 3600 * 3600;
    mc->ep0  = ep % 3600;
    adjweek(&mc->gt, ep);
    mc->iod  = iod;
    mc->ns = 0;
    for (j = 0; j < MCSSR_MAX_SYS; j++) {
        mc->nsig[j] = 0;
        mc->nsat[j] = 0;
    }
    for (j = 0; j < MAXSAT; j++) {
        mc->cellmask[j] = 0;
    }

    ngnss = getbitu(mc->buff, i, 4); i+=4;              /* No. of GNSS */
    for (j = 0; j < ngnss; j++) {
        gnssid = getbitu(mc->buff, i, 4); i+= 4;        /* GNSS ID */
        switch(gnssid) {
            case MCSSR_SYS_GPS : gnssmask = ((uint64_t)MCSSR_SATMASK_GPS_U) << 8 | MCSSR_SATMASK_GPS_L; break;
            case MCSSR_SYS_GLO : gnssmask = ((uint64_t)MCSSR_SATMASK_GLO_U) << 8 | MCSSR_SATMASK_GLO_L; break;
            case MCSSR_SYS_GAL : gnssmask = ((uint64_t)MCSSR_SATMASK_GAL_U) << 8 | MCSSR_SATMASK_GAL_L; break;
            case MCSSR_SYS_BDS : gnssmask = ((uint64_t)MCSSR_SATMASK_BDS_U) << 8 | MCSSR_SATMASK_BDS_L; break;
            case MCSSR_SYS_QZS : gnssmask = ((uint64_t)MCSSR_SATMASK_QZS_U) << 8 | MCSSR_SATMASK_QZS_L; break;
            default :
                trace(2,"decode_mcssr_mask: invalid gnssid=%d\n",gnssid);
                return -1;
        }
        svmask   = (uint64_t)getbitu(mc->buff, i, 8)<<32; i+=8;
        svmask  |= getbitu(mc->buff, i, 32); i+=32;     /* Sat mask */
        svmask  &= gnssmask;
        sigmask  = getbitu(mc->buff, i, 16); i+=16;     /* Signal mask */
        cmaflg   = getbitu(mc->buff, i,  1); i+= 1;     /* Cell-mask Avail.Flag */
        nsat = svmask2list (svmask,  gnssid,&mc->satlist[mc->ns], &mc->gidlist[mc->ns]);
        nsig = sigmask2list(sigmask, gnssid, mc->siglist[gnssid]);

        trace(4,"decode_mcssr_mask: gnssid=%d, svmask=0x%02X%08X, sigmask=0x%X, cmaflg=%d, nsat=%d, nsig=%d\n",
            gnssid, (uint8_t)(svmask>>32), (uint32_t)(svmask), sigmask, cmaflg, nsat, nsig);

        if(cmaflg) {
            for(k = 0; k < nsat; k++) {
                cellmask = getbitu(mc->buff, i, nsig); i+=nsig; /* Cell-mask */
                mc->cellmask[mc->ns + k] = cellmask;
                mc->ncell[mc->ns + k] = flgcnt16(cellmask);
                trace(4,"decode_mcssr_mask: k=%d, cellmask=0x%X, ncell=%2d\n",
                    k,cellmask,mc->ncell[mc->ns + k]);
            }
        }
        else {
            for(k = 0; k < nsat; k++) {
                mc->cellmask[mc->ns + k] = (1 << nsig) - 1; /* ex) 4=0xF,5=0x1F */
                mc->ncell[mc->ns + k] = nsig;
            }
        }
        mc->nsat[gnssid] = nsat;
        mc->nsig[gnssid] = nsig;
        mc->ns += nsat;
    }
    return i;
}

/* decode MADOCA-PPP L6 CSSR ST2 orbit correction ----------------------------*/
static int decode_mcssr_oc(mcssr_t *mc, ssr_t *ssr, int i, int apply)
{
    int j, k, n, sync, iod, ep, sat, iode, deph[3];
    double udint;
    char satid[8];

    i = decode_mcssr_head(mc, &sync, &iod, &ep, &udint, i, MCSSR_ST_OC);
    if(_mcssr.tow0 < 0) {
        trace(2,"decode_mcssr_oc :%s,unprocessed mask\n",
            time_str(_mcssr.gt,3));
        apply=0;
    }
    else {
        if(mc->ep0 > ep) ep += 3600;
        adjweek(&mc->gt, mc->tow0 + ep);
        if(_mcssr.iod != iod) {
            trace(2,"decode_mcssr_oc :%s,iod mismatch %d != %d\n",
                time_str(_mcssr.gt,3),_mcssr.iod,iod);
            apply=0;
        }
    }
    for (j = 0; j < mc->ns; j++) {
        n = (mc->gidlist[j]==MCSSR_SYS_GAL)? 10: 8;
        iode    = getbitu(mc->buff, i,  n); i+= n; /* IODE */
        deph[0] = getbits(mc->buff, i, 15); i+=15; /* Delta Radial */
        deph[1] = getbits(mc->buff, i, 13); i+=13; /* Delta Along-Track */
        deph[2] = getbits(mc->buff, i, 13); i+=13; /* Delta Cross-Track */

        sat=mc->satlist[j];
        satno2id(sat, satid);
        trace(4,"decode_mcssr_oc :%s,i=%4d,j=%3d,sat=%s,iode=%4d,deph=%6d,%6d,%6d,%8.4f,%8.4f,%8.4f\n",
            time_str(_mcssr.gt,3),i,j,satid,iode,deph[0],deph[1],deph[2],
            deph[0]*0.0016,deph[1]*0.0064,deph[2]*0.0064);

        if(sat == 0) continue; /* unsupprted sat */
        if((deph[0] == MCSSR_INVALID_15BIT) ||
           (deph[1] == MCSSR_INVALID_13BIT) ||
           (deph[2] == MCSSR_INVALID_13BIT)) continue; /* invalid value */

        if(apply) {
            ssr[sat-1].t0[0]   = mc->gt;
            ssr[sat-1].udi[0]  = udint;
            ssr[sat-1].iod[0]  = iod;
            ssr[sat-1].iode    = iode;
            ssr[sat-1].deph[0] = deph[0] * 0.0016;
            ssr[sat-1].deph[1] = deph[1] * 0.0064;
            ssr[sat-1].deph[2] = deph[2] * 0.0064;
            for(k = 0; k < 3; k++) ssr[sat-1].ddeph[k] = 0.0;
            ssr[sat-1].update=1;
        }
    }
    return i;
}

/* decode MADOCA-PPP L6 CSSR ST3 clock correction ----------------------------*/
static int decode_mcssr_cc(mcssr_t *mc, ssr_t *ssr, int i, int apply)
{
    int j, sync, iod, ep, sat, dclk;
    double udint;
    char satid[8];

    i = decode_mcssr_head(mc, &sync, &iod, &ep, &udint, i, MCSSR_ST_CC);
    if(_mcssr.tow0 < 0) {
        trace(2,"decode_mcssr_cc :%s,unprocessed mask\n",
            time_str(_mcssr.gt,3));
        apply=0;
    }
    else {
        if(mc->ep0 > ep) ep += 3600;
        adjweek(&mc->gt, mc->tow0 + ep);
        if(_mcssr.iod != iod) {
            trace(2,"decode_mcssr_cc :%s,iod mismatch %d != %d\n",
                time_str(_mcssr.gt,3),_mcssr.iod,iod);
            apply=0;
        }
    }
    for (j = 0; j < mc->ns; j++) {
        dclk = getbits(mc->buff, i, 15); i+=15; /* Delta Clock C0 */

        sat=mc->satlist[j];
        satno2id(sat, satid);

        trace(4,"decode_mcssr_cc :%s,i=%4d,j=%3d,sat=%s,dclk=%6d,%8.4f\n",
            time_str(_mcssr.gt,3),i,j,satid,dclk,dclk*0.0016);

        if(sat == 0) continue; /* unsupprted sat */
        if(dclk == MCSSR_INVALID_15BIT) continue; /* invalid value */

        if(apply) {
            ssr[sat-1].t0[1]   = mc->gt;
            ssr[sat-1].udi[1]  = udint;
            ssr[sat-1].iod[1]  = iod;
            ssr[sat-1].dclk[0] = dclk * 0.0016;
            ssr[sat-1].dclk[1] = 0.0;
            ssr[sat-1].dclk[2] = 0.0;
            ssr[sat-1].update=1;
        }
    }
    return i;
}

/* decode MADOCA-PPP L6 CSSR ST4 code bias -----------------------------------*/
static int decode_mcssr_cb(mcssr_t *mc, ssr_t *ssr, int i, int apply)
{
    int j, k, sync, iod, ep, sat, nsig, mask, cb, flg;
    double udint,cbias[MAXCODE];
    char satid[8];

    i = decode_mcssr_head(mc, &sync, &iod, &ep, &udint, i, MCSSR_ST_CB);
    if(_mcssr.tow0 < 0) {
        trace(2,"decode_mcssr_cb :%s,unprocessed mask\n",
            time_str(_mcssr.gt,3));
        apply=0;
    }
    else {
        if(mc->ep0 > ep) ep += 3600;
        adjweek(&mc->gt, mc->tow0 + ep);
        if(_mcssr.iod != iod) {
            trace(2,"decode_mcssr_cb :%s,iod mismatch %d != %d%d\n",
                time_str(_mcssr.gt,3),_mcssr.iod,iod);
            apply=0;
        }
    }
    for (j = 0; j < mc->ns; j++) {
        for (k=0;k<MAXCODE;k++) cbias[k]=0.0;
        sat = mc->satlist[j];
        satno2id(sat, satid);
        nsig = mc->nsig[mc->gidlist[j]];
        mask = 1 << (nsig - 1);
        for (k = 0; k < nsig; k++) {
            if((flg = (mc->cellmask[j] & mask))) {
                cb = getbits(mc->buff, i, 11); i+=11; /* Code Bias */
                trace(4,"decode_mcssr_cb :%s,i=%4d,j=%3d,k=%2d,sat=%s,flg=0x%2X,sig=%2d,cb=%5d,%6.2f\n",
                    time_str(_mcssr.gt,3),i,j,k,satid,flg,mc->siglist[mc->gidlist[j]][k],cb,cb*0.02);

                if(sat == 0) continue; /* unsupprted sat */
                if(cb == MCSSR_INVALID_11BIT) continue; /* invalid value */

                cbias[mc->siglist[mc->gidlist[j]][k]-1] = cb * 0.02;
            }
            mask >>= 1;
        }
        if(apply) {
            ssr[sat-1].t0[4]   = mc->gt;
            ssr[sat-1].udi[4]  = udint;
            ssr[sat-1].iod[4]  = iod;
            for (k=0;k<MAXCODE;k++) ssr[sat-1].cbias[k]=(float)cbias[k];
            ssr[sat-1].update=1;
        }
    }
    return i;
}

/* decode MADOCA-PPP L6 CSSR ST5 phase bias ----------------------------------*/
static int decode_mcssr_pb(mcssr_t *mc, ssr_t *ssr, int i, int apply)
{
    int j, k, sync, iod, ep, sat, nsig, mask, pb, di, flg;
    double udint,pbias[MAXCODE];
    char satid[8];

    i = decode_mcssr_head(mc, &sync, &iod, &ep, &udint, i, MCSSR_ST_PB);
    if(_mcssr.tow0 < 0) {
        trace(2,"decode_mcssr_pb :%s,unprocessed mask\n",
            time_str(_mcssr.gt,3));
        apply=0;
    }
    else {
        if(mc->ep0 > ep) ep += 3600;
        adjweek(&mc->gt, mc->tow0 + ep);
        if(_mcssr.iod != iod) {
            trace(2,"decode_mcssr_pb :%s,iod mismatch %d != %d%d\n",
                time_str(_mcssr.gt,3),_mcssr.iod,iod);
            apply=0;
        }
    }
    for (j = 0; j < mc->ns; j++) {
        for (k=0;k<MAXCODE;k++) pbias[k]=0.0;
        sat = mc->satlist[j];
        satno2id(sat, satid);
        nsig = mc->nsig[mc->gidlist[j]];
        mask = 1 << (nsig - 1);
        for (k = 0; k < nsig; k++) {
            if((flg = (mc->cellmask[j] & mask))) {
                pb = getbits(mc->buff, i, 15); i+=15; /* Phase Bias */
                di = getbitu(mc->buff, i,  2); i+= 2; /* Discontinuity Indicator */

                trace(4,"decode_mcssr_pb :%s,i=%4d,j=%3d,k=%2d,sat=%s,flg=0x%2X,sig=%2d,di=%d,pb=%5d,%7.3f\n",
                    time_str(_mcssr.gt,3),i,j,k,satid,flg,mc->siglist[mc->gidlist[j]][k],di,pb,pb*0.001);

                if(sat == 0) continue; /* unsupprted sat */
                if(pb == MCSSR_INVALID_15BIT) continue; /* invalid value */

                pbias[mc->siglist[mc->gidlist[j]][k]-1] = pb * 0.001;
            }
            mask >>= 1;
        }
        if(apply) {
            ssr[sat-1].t0[5]    = mc->gt;
            ssr[sat-1].udi[5]   = udint;
            ssr[sat-1].iod[5]   = iod;
            ssr[sat-1].yaw_ang  = 0.0;
            ssr[sat-1].yaw_rate = 0.0;
            /* Note. Invert the value based on the process in corr_phase_bias_ssr() */
            for (k=0;k<MAXCODE;k++) ssr[sat-1].pbias[k] = -(float)pbias[k];
            ssr[sat-1].update=1;
        }
    }
    return i;
}

/* decode MADOCA-PPP L6 CSSR ST7 URA -----------------------------------------*/
static int decode_mcssr_ura(mcssr_t *mc, ssr_t *ssr, int i, int apply)
{
    int j, sync, iod, ep, sat, ura;
    double udint;
    char satid[8];

    i = decode_mcssr_head(mc, &sync, &iod, &ep, &udint, i, MCSSR_ST_URA);
    if(_mcssr.tow0 < 0) {
        trace(2,"decode_mcssr_ura :%s,unprocessed mask\n",
            time_str(_mcssr.gt,3));
        apply=0;
    }
    else {
        if(mc->ep0 > ep) ep += 3600;
        adjweek(&mc->gt, mc->tow0 + ep);
        if(_mcssr.iod != iod) {
            trace(2,"decode_mcssr_ura :%s,iod mismatch %d != %d\n",
                time_str(_mcssr.gt,3),_mcssr.iod,iod);
            apply=0;
        }
    }
    for (j = 0; j < mc->ns; j++) {
        ura = getbitu(mc->buff, i, 6); i+=6;

        sat=mc->satlist[j];
        satno2id(sat, satid);

        trace(4,"decode_mcssr_ura :%s,i=%4d,j=%3d,sat=%s,ura=0x%02X\n",
            time_str(_mcssr.gt,3),i,j,satid,ura);

        if(sat == 0) continue; /* unsupprted sat */
        if(apply) {
            ssr[sat-1].t0[3]   = mc->gt;
            ssr[sat-1].udi[3]  = udint;
            ssr[sat-1].iod[3]  = iod;
            ssr[sat-1].ura     = ura;
            ssr[sat-1].update=1;
        }
    }
    return i;
}

/* decode QZSS L6 message ----------------------------------------------------
* decode QZSS L6 message and convert to RTCM SSR
* args   : rtcm_t *rtcm IO   rtcm control struct
* return : status (-1: error message, 0: no message, 10: input ssr messages)
* note   : before calling the function, init_mcssr()
*-----------------------------------------------------------------------------*/
extern int decode_qzss_l6msg(rtcm_t *rtcm)
{
    unsigned int preamb,prn,type,alert,vid,mgfid,csid,anme,si;
    int i=0,n,ibuff,mn,st,apply,ret,optprn;
    char *str;
    uint8_t mask;

    /* decode L6 message header (49 bits) */
    preamb=getbitu(rtcm->buff,i,32); i+=32; /* Preamble */
    prn   =getbitu(rtcm->buff,i, 8); i+= 8; /* PRN */
    type  =getbitu(rtcm->buff,i, 8);        /* L6 Msg. Type ID */
    vid   =getbitu(rtcm->buff,i, 3); i+= 3; /* Vendor ID(2:MADOCA-PPP) */
    mgfid =getbitu(rtcm->buff,i, 2); i+= 2; /* Meg. Gen. Facility ID(0-1:Hitachi-Ota,2-3:Kobe) */
    csid  =getbitu(rtcm->buff,i, 1); i+= 1; /* Correction Service ID(0:Clock/Ephemeris) */
    anme  =getbitu(rtcm->buff,i, 1); i+= 1; /* Applicable Nav. Msg. Ext.(0:Default,1:CNAV/CNAV2) */
    si    =getbitu(rtcm->buff,i, 1); i+= 1; /* Subframe Indicator(0:others,1:First Data) */
    alert =getbitu(rtcm->buff,i, 1); i+= 1; /* Alert Flag(1:Alert) */
    
    trace(3,"decode_qzss_l6msg : %s,preamb=0x%08X,prn=%d,type=0x%X(vid=%d,mgfid=%d,csid=%d,anme=%d,si=%d),alert=%d\n",
        time_str(_mcssr.gt,3),preamb,prn,type,vid,mgfid,csid,anme,si,alert);

    if (preamb != L6PREAMB) {
        trace(2,"decode_qzss_l6msg : %s,invalid preamb=0x%08X\n",
            time_str(_mcssr.gt,3),preamb);
        return -1;
    }

    if (alert) {
        if (strstr(rtcm->opt,"-IGNORE_MCSSR_ALERT")) {
            trace(3,"decode_qzss_l6msg : ignore alert\n",time_str(_mcssr.gt,3));
        }
        else {
            trace(2,"decode_qzss_l6msg : alert\n",time_str(_mcssr.gt,3));
            return -1;
        }
    }

    if(vid!=2) return 0;           /* unsupport vendor ID */

    if ((str = strstr(rtcm->opt,"-MCSSR_PRN"))) {
        optprn = atoi(strstr(str,"=")+1);
        if(optprn != prn) {
            trace(3,"decode_qzss_l6msg : -MCSSR_PRN=%d\n",optprn);
            return -1;
        }
    }
    else {
        if(_mcssr.prn == -1) {
            _mcssr.prn         = prn;
        }
        else {
            if(_mcssr.prn != prn) {
                _mcssr.prn_chk_cnt++;
                trace(3,"decode_qzss_l6msg : prn mismatch _mcssr.prn=%d,prn=%d,cnt=%d\n",
                    _mcssr.prn,prn,_mcssr.prn_chk_cnt);
                if(_mcssr.prn_chk_cnt < MCSSR_MAX_PRN_CHECK) {
                    return -1;
                }
                else {
                    trace(2,"decode_qzss_l6msg :%s,prn switched %d -> %d\n",
                        time_str(_mcssr.gt,3),_mcssr.prn,prn);
                    _mcssr.prn = prn;
                }
            }
        }
        _mcssr.prn_chk_cnt = 0;
    }

    if(si==1) _mcssr.maxframe = 0; /* Subframe Indicator : First Data */

    /* frame recognition */
    if(_mcssr.maxframe <= 0) {
        mn = getbitu(rtcm->buff,   i, 12); /* Message Number(note:not i++) */
        st = getbitu(rtcm->buff,i+12,  4); /* Message Sub Type ID */
        if(mn != RTCM_MN_CSSR) {
            trace(2,"decode_qzss_l6msg :%s,invalid mn=%d\n",
                time_str(_mcssr.gt,3),mn);
            return -1;
        }
        if((_mcssr.maxframe = framelen[st]) < 1) {
            trace(2,"decode_qzss_l6msg :%s,invalid frame start st=%d\n",
                time_str(_mcssr.gt,3),st);
            return -1;
        }
        trace(3,"decode_qzss_l6msg : %s,mn=%d,st=%d,maxframe=%d\n",
            time_str(_mcssr.gt,3),mn,st,_mcssr.maxframe);
        _mcssr.frame=0;
        _mcssr.ibuff=0;
    }

    /* save data part (1695 bits) */
    _mcssr.frame++;
    trace(3,"decode_qzss_l6msg : %s,maxframe=%d,frame=%d\n",
        time_str(_mcssr.gt,3),_mcssr.maxframe,_mcssr.frame);
    switch(_mcssr.frame){
        case 1: ibuff=  0; n = 8; mask = 0xFE; break;
        case 2: ibuff=211; n = 1; mask = 0xFC; break; 
        case 3: ibuff=423; n = 2; mask = 0xF8; break;
        case 4: ibuff=635; n = 3; mask = 0xF0; break;
        case 5: ibuff=847; n = 4; mask = 0xE0; break;
        default:trace(3,"decode_qzss_l6msg : invalid frame\n"); return -1;
    }
    _mcssr.buff[ibuff++] |= (unsigned char)getbitu(rtcm->buff, i, n); i += n;
    while (i < L6HEAD_BITLEN + L6DATA_BITLEN) {
        _mcssr.buff[ibuff++] = (unsigned char)getbitu(rtcm->buff, i, 8); i += 8;
    }
    _mcssr.buff[ibuff - 1] &= mask;

    if(_mcssr.frame != _mcssr.maxframe) return 0; /* no message */

    ret=10; /* input ssr messages */
    i=0;
    while(i < L6DATA_BITLEN * _mcssr.maxframe) {
        mn = getbitu(_mcssr.buff,i,12); i+=12; /* Message Number */
        if(mn != RTCM_MN_CSSR) continue;
        st = getbitu(_mcssr.buff,i, 4); i+= 4; /* Message Sub Type ID */
        if((st != MCSSR_ST_MASK) && (st != MCSSR_ST_OC) && 
           (st != MCSSR_ST_CC)   && (st != MCSSR_ST_CB) &&
           (st != MCSSR_ST_PB)   && (st != MCSSR_ST_URA)) continue;
        if (st == MCSSR_ST_MASK) {
            if(_mcssr.mgfid != mgfid) {
                trace(2,"decode_qzss_l6msg :%s,mgfid switched %d -> %d\n",
                    time_str(_mcssr.gt,3),_mcssr.mgfid,mgfid);
                _mcssr.mgfid = mgfid;
            }
            i = decode_mcssr_mask(&_mcssr, i);
        }
        else {
            apply = 1;
            if(_mcssr.mgfid != mgfid) {
                trace(2,"decode_qzss_l6msg :%s,st=%d,mgfid mismatch %d != %d\n",
                    time_str(_mcssr.gt,3),st,_mcssr.mgfid,mgfid);
                apply = 0;
            }
            switch(st){
                case MCSSR_ST_OC  :
                    i = decode_mcssr_oc (&_mcssr, rtcm->ssr, i, apply);
                    break;
                case MCSSR_ST_CC  :
                    i = decode_mcssr_cc (&_mcssr, rtcm->ssr, i, apply);
                    break;
                case MCSSR_ST_CB  :
                    if (strstr(rtcm->opt,"-DIS_MCSSR_CB")) apply = 0;
                    i = decode_mcssr_cb (&_mcssr, rtcm->ssr, i, apply);
                    break;
                case MCSSR_ST_PB  :
                    i = decode_mcssr_pb (&_mcssr, rtcm->ssr, i, apply);
                    break;
                case MCSSR_ST_URA :
                    i = decode_mcssr_ura(&_mcssr, rtcm->ssr, i, apply);
                    break;
            default :
                trace(2,"decode_qzss_l6msg : invalid st=%d\n",st);
                ret = -1;
                break;
            }
        }
        if(i<0) {
            trace(2,"decode_qzss_l6msg : decode error st=%d\n",st);
            ret = -1;
            break;
        }
        rtcm->time = _mcssr.gt;
    }
    _mcssr.prn = -1;
    _mcssr.maxframe = 0;
    trace(4,"decode_qzss_l6msg : %s,frames=%d\n",
        time_str(_mcssr.gt,3),_mcssr.frame);
    return ret; 
}

/* initialize MADOCA-PPP CSSR control ----------------------------------------
* initialize MADOCA-PPP CSSR control struct
* args   : gtime_t *gt   I    GPST for week number determination
* return : none
*-----------------------------------------------------------------------------*/
extern void init_mcssr(gtime_t gt)
{
    _mcssr.gt    = gt; /* for week number determination */
    _mcssr.mgfid = -1; /* invalid value */
    _mcssr.prn   = -1; /* invalid value */
    _mcssr.tow0  = -1; /* invalid value */
    _mcssr.frame =  0; /* initial value */
}

/* input QZSS L6 message from file -------------------------------------------
* fetch next QZSS L6 message and input a messsage from file
* args   : rtcm_t *rtcm IO   rtcm control struct
*          FILE  *fp    I    file pointer
* return : status (-2: end of file, -1: error message, 0: no message,
*                  10: input CSSR messages)
* note   : before calling the function, init_mcssr()
*-----------------------------------------------------------------------------*/
extern int input_qzssl6(rtcm_t *rtcm, uint8_t data)
{
    trace(5,"input_qzssl6: data=%02x\n",data);
    
    rtcm->buff[rtcm->nbyte++]=data;

    /* synchronize frame with L6 preamble */
    if(rtcm->nbyte==4) {
        if((rtcm->buff[0]!=0x1A)||(rtcm->buff[1]!=0xCF)||
           (rtcm->buff[2]!=0xFC)||(rtcm->buff[3]!=0x1D)) {
            rtcm->nbyte=0;
            return 0;
        }
    }
    if (rtcm->nbyte<L6BYTELEN) return 0;

    rtcm->nbyte=0;
    return decode_qzss_l6msg(rtcm);
}

/* input QZSS L6 message from file -------------------------------------------
* fetch next QZSS L6 message and input a messsage from file
* args   : rtcm_t *rtcm IO   rtcm control struct
*          FILE  *fp    I    file pointer
* return : status (-2: end of file, -1...10: same as input_qzssl6())
* note   : same as input_qzssl6()
*-----------------------------------------------------------------------------*/
extern int input_qzssl6f(rtcm_t *rtcm, FILE *fp)
{
    int i,data,ret;
    
    trace(4,"input_qzssl6f\n");
    
    for (i=0;i<4096;i++) {
        if ((data=fgetc(fp))==EOF) return -2;
        if ((ret=input_qzssl6(rtcm,(uint8_t)data))) return ret;
    }
    return 0; /* return at every 4k bytes */
}

/* MADOCA-PPP bias code selection  -------------------------------------------
* select MADOCA-PPP code bias, phase bias code from observation code
* args   : int sys  I    navigation system
*          int code I    observation code
* return : MADOCA-PPP CB,PB code
* note   : ref [1] section 5.5.3.1 Applicable Signals of Code/Phase Bias
*-----------------------------------------------------------------------------*/
extern int mcssr_sel_biascode(int sys, int code)
{
    switch (sys) {
        case SYS_GPS:
        switch (code) {
            case CODE_L1C: return CODE_L1C; /* L1C/A */
            case CODE_L1P:                  /* L1P */
            case CODE_L1W: return CODE_L1W; /* L1 Z-tracking */
            case CODE_L1S:                  /* L1C(D) */
            case CODE_L1L:                  /* L1C(P) */
            case CODE_L1X: return CODE_L1X; /* L1C(D+P) */
            case CODE_L2S:                  /* L2C(M) */
            case CODE_L2L:                  /* L2C(L) */
            case CODE_L2X: return CODE_L2X; /* L2C(M+L) */
            case CODE_L2P:                  /* L2P */
            case CODE_L2W: return CODE_L2W; /* L2 Z-tracking */
            case CODE_L5I:                  /* L5I */
            case CODE_L5Q:                  /* L5Q */
            case CODE_L5X: return CODE_L5X; /* L5X */
        }
        break;
    case SYS_GLO:
        switch (code) {
            case CODE_L1C: return CODE_L1C; /* G1C/A */
            case CODE_L1P: return CODE_L1P; /* G1P */
            case CODE_L2C: return CODE_L2C; /* G2C/A */
            case CODE_L2P: return CODE_L2P; /* G2P */
        }
        break;
    case SYS_GAL:
        switch (code) {
            case CODE_L1C:                  /* E1C */
            case CODE_L1B:                  /* E1B */
            case CODE_L1X: return CODE_L1X; /* E1B+C */
            case CODE_L5I:                  /* E5aI */
            case CODE_L5Q:                  /* E5aQ */
            case CODE_L5X: return CODE_L5X; /* E5aI+Q */
        }
        break;
    case SYS_QZS:
        switch (code) {
            case CODE_L1C: return CODE_L1C; /* L1C/A */
            case CODE_L1S:                  /* L1C(D) */
            case CODE_L1L:                  /* L1C(P) */
            case CODE_L1X: return CODE_L1X; /* L1C(D+P) */
            case CODE_L2S:                  /* L2C(M) */
            case CODE_L2L:                  /* L2C(L) */
            case CODE_L2X: return CODE_L2X; /* L2C(M+L) */
            case CODE_L5I:                  /* L5I */
            case CODE_L5Q:                  /* L5Q */
            case CODE_L5X: return CODE_L5X; /* L5X */
        }
        break;
    }
    return CODE_NONE;
}