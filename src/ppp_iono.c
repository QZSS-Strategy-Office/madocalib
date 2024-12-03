/*------------------------------------------------------------------------------
* ppp_iono.c : ppp ionospheric correction functions
*
*          Copyright (C) 2024 Cabinet Office, Japan, All rights reserved.
*          Copyright (C) 2024 Lighthouse Technology & Consulting Co. Ltd., All rights reserved.
*
* references :
*     [1]  CAO IS-QZSS-MDC-002, November, 2023
*
* version : $Revision:$ $Date:$
* history : 2024/01/10 1.0  new, for MADOCALIB
*           2024/05/24 1.1  algorithm correction for const_iono_corr().
*                           change convergence threshold (H,V) from 
*                           0.3,0.5 to 2.0,3.0.
*           2024/09/27 1.2  delete NM
*-----------------------------------------------------------------------------*/
#include "rtklib.h"

#define IONO_REJ_STD     1.0 /* reject   STEC corr. std (m) */
#define IONO_COV_RATIO   1.0 /* ratio of STEC corr. cov (m) */
#define IONO_THRE_H      2.0 /* Horizontal convergence threshold (m) */
#define IONO_THRE_V      3.0 /* Vertical   convergence threshold (m) */

#define SQR(x)      ((x)*(x))
#define SQRT(x)     ((x)<=0.0||(x)!=(x)?0.0:sqrt(x))

/* number and index of states */
#define NP(opt)     ((opt)->dynamics?9:3)
#define NC(opt)     (NSYS)
#define NT(opt)     ((opt)->tropopt<TROPOPT_EST?0:((opt)->tropopt==TROPOPT_EST?1:3))
#define IM(s,opt)   (NP(opt)+NC(opt)+NT(opt)+(s))
#define II(s,opt)   (NP(opt)+NC(opt)+NT(opt)+(s)-1)

/* read stat correction data -------------------------------------------------
* read ppp ionosphere correction data from stat file
* args   : pppiono_corr_t *corr IO  ppp ionosphere correction data
*          FILE           *fp   I   file pointer
* return : status (-2: end of file, 0: no data, 1: input correction data)
*-----------------------------------------------------------------------------*/
extern int input_statcorr(pppiono_corr_t *corr, FILE *fp)
{
    gtime_t time;
    int week,stat,sat,ret=0;
    double tow,pos[3],std[3],azel[2],dly,dlystd;
    char buff[1024],satid[64],*p;

    /* Note, File format :
       $POS,GPS Week,TOW,status(1:FIX,6:PPP),X(m),Y(m),Z(m),X STD(m), Y STD(m),Z STD(m)
       $ION,GPS Week,TOW,status(1:FIX,6:PPP),Sat,Az(deg),El(deg),L1 Slant Delay(m),L1 Slant Delay STD(m)
    */

    while (fgets(buff,sizeof(buff),fp)) {

        for (p=buff;*p;p++) if (*p==',') *p=' ';

        /* $POS record */
        if (sscanf(buff,"$POS%d%lf%d%lf%lf%lf%lf%lf%lf",
                &week,&tow,&stat,pos,pos+1,pos+2,std,std+1,std+2)>=9) {
            time=gpst2time(week,tow);
            if((corr->time.time != 0)&&(corr->time.time != time.time)) return ret;
        }

        /* $ION record */
        if (sscanf(buff,"$ION%d%lf%d%s%lf%lf%lf%lf",
                &week,&tow,&stat,satid,azel,azel+1,&dly,&dlystd)>=8) {
            time=gpst2time(week,tow);
            if((sat=satid2no(satid))!=0) {
                corr->t0[sat-1] =corr->time=time;
                corr->dly[sat-1]=dly;
                corr->std[sat-1]=dlystd;
                ret=1;
            }
            trace(4,"input_statcorr : %s,%s,az=%7.2f,el=%5.2f,dly=%8.4f,std=%8.4f\n",
                time_str(time,0),satid,azel[0],azel[1],dly,dlystd);
        }
    }
    return -2;
}

/* constraint to ionospheric correction --------------------------------------*/
extern int const_iono_corr(rtk_t *rtk, const obsd_t *obs, const nav_t *nav,
                           const int n, const double *azel, const int *exc,
                           const double *rr, const double *x, double *v,
                           double *H, double *var)
{
    gtime_t time=obs[0].time;
    pppiono_corr_t corr;
    int i,j,k,m,sat,nv=0,s,ns;
    double tt,std,sd[2],pos[3],P[9],Q[9];
    double rejstd=IONO_REJ_STD,covratio=IONO_COV_RATIO,thre[2]={IONO_THRE_H,IONO_THRE_V};
    double ave,bsys[NSYS]={0.0},f;
    char *p,*tstr,satid[8];
    const int sys[NSYS]={SYS_GPS,SYS_GLO,SYS_GAL,SYS_QZS,0};

    if ((p=strstr(rtk->opt.pppopt,"-IONOCORR_THRE_H="))) {
        sscanf(p,"-IONOCORR_THRE_H=%lf",&thre[0]);
    }
    if ((p=strstr(rtk->opt.pppopt,"-IONOCORR_THRE_V="))) {
        sscanf(p,"-IONOCORR_THRE_V=%lf",&thre[1]);
    }
    if ((p=strstr(rtk->opt.pppopt,"-IONOCORR_REJ_STD="))) {
        sscanf(p,"-IONOCORR_REJ_STD=%lf",&rejstd);
    }
    if ((p=strstr(rtk->opt.pppopt,"-IONOCORR_COV_RATIO="))) {
        sscanf(p,"-IONOCORR_COV_RATIO=%lf",&covratio);
    }

    ecef2pos(rr,pos);
    P[0]     =rtk->prev_qr[0]; /* xx */
    P[4]     =rtk->prev_qr[1]; /* yy */
    P[8]     =rtk->prev_qr[2]; /* zz */
    P[1]=P[3]=rtk->prev_qr[3]; /* xy */
    P[5]=P[7]=rtk->prev_qr[4]; /* yz */
    P[2]=P[6]=rtk->prev_qr[5]; /* zx */
    covenu(pos,P,Q);
    sd[0] = SQRT(Q[0]+Q[4]);   /* horizontal std */
    sd[1] = SQRT(Q[8]);        /* vertical   std */

    trace(2,"const_iono_corr : sd=%8.4f,%8.4f,thre=%8.4f,%8.4f,rejstd=%.3f,covratio=%.9f\n",
        sd[0],sd[1],thre[0],thre[1],rejstd,covratio);

    if((sd[0]!=0.0)&&(sd[1]!=0.0)&&(sd[0]<thre[0])&&(sd[1]<thre[1])) return 0;

    m=IM(0,&rtk->opt);
    corr = nav->pppiono.corr;
    
    /* estimate system bias */
    for (s=0;sys[s];s++) { /* for each system */
        ave=0.0;ns=0;
        for (i=0;i<n;i++) {
            sat=obs[i].sat;
            if (satsys(sat,NULL)!=sys[s]) continue;
            if(corr.t0[sat-1].time == 0.0) continue;
            tt = fabs(timediff(time,corr.t0[sat-1]));
            tstr = time_str(corr.t0[sat-1],0);
            
            satno2id(sat, satid);
            if (exc[i]) continue;
            if (tt > MIONO_MAX_AGE) continue;
            if (corr.std[sat-1] > rejstd) continue;
            
            f=ionmapf(pos,azel+i*2);
            j=II(sat,&rtk->opt);
            ave+=(corr.dly[sat-1]-f*x[j]);
            ns++;
        }
        if (ns>0) bsys[s]=ave/(double)ns;
    }
    
    /* constraint to external ionosphere correction */
    for (i=0;i<n;i++) {
        sat=obs[i].sat;
        if(corr.t0[sat-1].time == 0.0) continue;
        tt = fabs(timediff(time,corr.t0[sat-1]));
        tstr = time_str(corr.t0[sat-1],0);

        satno2id(sat, satid);

        switch (satsys(sat,NULL)) {
            case SYS_GPS: s=0; break;
            case SYS_GLO: s=1; break;
            case SYS_GAL: s=2; break;
            case SYS_QZS: s=3; break;
            default:      continue;
        }

        if (exc[i]) continue;
        if (tt > MIONO_MAX_AGE) continue;
        if (corr.std[sat-1] > rejstd) continue;

        std = corr.std[sat-1];
        f=ionmapf(pos,azel+i*2);
        j=II(sat,&rtk->opt);
        v[nv]=corr.dly[sat-1]-f*x[j]-bsys[s];

        trace(2,"const_iono_corr %s,%s,tt=%11d,az=%5.1f,el=%4.1f,sdly=%8.4f,std=%8.4f,v=%8.4f,f*x=%8.4f,bsys=%8.4f\n",
            satid,tstr,(int)tt,(azel+i*2)[0]*R2D,(azel+i*2)[1]*R2D,corr.dly[sat-1],corr.std[sat-1],v[nv],f*x[j],bsys[s]);

        for (k=0;k<rtk->nx;k++) H[k+nv*rtk->nx]=k==j?f:0.0;
        var[nv++]=SQR(std)*covratio;
    }
    return nv;
}
