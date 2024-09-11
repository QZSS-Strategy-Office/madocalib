/*------------------------------------------------------------------------------
* cssr.c : cssr functions
*
* Copyright (c) 2024 Cabinet Office, Japan, All rights reserved.
* Copyright (c) 2022-2024, Lighthouse Technology & Consulting Co. Ltd., All rights reserved.
*
* author  : LHTC
* history : 2022/02/03 1.0  new
*
*-----------------------------------------------------------------------------*/
#include "cssr.h"

static FILE *fp_dump;

/* dump handling functions ---------------------------------------------------*/
static int dump_flag=0;
static int dump_type=0; /* 0:all types */
extern void cssr_dump_set(int type)
{
    dump_flag=1;
    dump_type=type;
}
extern void cssr_dump_unset()
{
    dump_flag=0;
}
static int dump(int type, const char *format, ...)
{
    va_list ap;
    char buff[2048],*p=buff;
    
    if (!dump_flag) return 0;
    if (dump_type>0 && type>0 && type!=dump_type) return 0;
    
    fp_dump=stdout;
    if (!fp_dump) return 0;
    
    va_start(ap,format); p+=vsprintf(p,format,ap); va_end(ap);
    fprintf(fp_dump,"%s",buff);
    fflush(fp_dump);
    return 0;
}
/* initialize cssr control ---------------------------------------------------*/
extern int cssr_init(cssr_t *cssr)
{
    cssr_t cssr0={{{0}}};
    *cssr=cssr0;
    return 1;
}
/* adjust weekly rollover of gps time ----------------------------------------*/
static void adjweek(rtcm_t *rtcm, double tow)
{
    double tow_p;
    int week;
    
    /* if no time, get cpu time */
    if (rtcm->time.time==0) rtcm->time=utc2gpst(timeget());
    tow_p=time2gpst(rtcm->time,&week);
    if      (tow<tow_p-302400.0) tow+=604800.0;
    else if (tow>tow_p+302400.0) tow-=604800.0;
    rtcm->time=gpst2time(week,tow);
}
/* ssr update intervals ------------------------------------------------------*/
static const double ssrudint[16]={
    1,2,5,10,15,30,60,120,240,300,600,900,1800,3600,7200,10800
};
/* dump ssr 1,4 message header -----------------------------------------------*/
static int decode_cssr_head(rtcm_t *rtcm, cssr_t *cssr, int *sync, int *iod,
                          double *udint, int *refd, int *hsize)
{
    gtime_t time_p;
    double tow,dt=0.0;
    int i=0,mt,type,udi;
    
    mt  =getbitu(rtcm->buff,i,12); i+=12; /* message type */
    type=getbitu(rtcm->buff,i, 4); i+= 4; /* sub type id */
    
    if (type==1) { /* gps epoch time */
        tow=getbitu(rtcm->buff,i,20); i+=20;
        adjweek(rtcm,tow);
        cssr->cssr_mask.epoch=rtcm->time;
    }
    else { /* gps hourly epoch time */
        time_p=rtcm->time;
        tow=getbitu(rtcm->buff,i,12); i+=12;
        rtcm->time.time=(rtcm->time.time/3600)*3600+(int)tow;
        dt=timediff(rtcm->time,time_p);
        if (dt> 1800) rtcm->time=timeadd(rtcm->time,-3600);
        if (dt<-1800) rtcm->time=timeadd(rtcm->time, 3600);
    }
    
    udi   =getbitu(rtcm->buff,i, 4); i+= 4; /* update interval */
    *sync =getbitu(rtcm->buff,i, 1); i+= 1; /* multiple message indicator */
    *iod  =getbitu(rtcm->buff,i, 4); i+= 4; /* iod ssr */
    *udint=ssrudint[udi];
    
    dump(0,"%4d %4d: %s  TOW=%6.0f  UDI=%3.0f  IOD=%2d  SYNC=%d i=%d",
             mt,type,time_str(rtcm->time,0),tow,*udint,*iod,*sync,i);
    
    *hsize=i;
    return i;
}
/* dump cssr 1: cssr mask ----------------------------------------------------*/
static int decode_st1(rtcm_t *rtcm, cssr_t *cssr)
{
    cssr_mask_t *cssr_mask=&cssr->cssr_mask;
    double udint;
    int i,j,sat,sync,iod,refd=0,minprn;
    int b,gnssid,nsys,isys,cell_ava,nsat,nsig,isat,isig;
    int k,idx,sys;
    /* signal and tracking mode id (ref []) */
    const unsigned char codes_gps[16]={
        CODE_L1C,CODE_L1P,CODE_L1W,CODE_L1S,CODE_L1L,CODE_L1X,
        CODE_L2S,CODE_L2L,CODE_L2X,CODE_L2P,CODE_L2W,
        CODE_L5I,CODE_L5Q,CODE_L5X,0
    };
    const unsigned char codes_glo[16]={
        CODE_L1C,CODE_L1P,CODE_L2C,CODE_L2P,
        CODE_L4A,CODE_L4B,CODE_L4X,
        CODE_L6A,CODE_L6B,CODE_L6X,
        CODE_L3I,CODE_L3Q,CODE_L3X,0
    };
    const unsigned char codes_gal[16]={
        CODE_L1B,CODE_L1C,CODE_L1X,
        CODE_L5I,CODE_L5Q,CODE_L5X,
        CODE_L7I,CODE_L7Q,CODE_L7X,
        CODE_L8I,CODE_L8Q,CODE_L8X,
        CODE_L6A,CODE_L6B,0
    };
    const unsigned char codes_bds[16]={
        CODE_L2I,CODE_L2Q,CODE_L2X,
        CODE_L6I,CODE_L6Q,CODE_L6X,
        CODE_L7I,CODE_L7Q,CODE_L7X,0
    };
    const unsigned char codes_qzs[16]={
        CODE_L1C,CODE_L1S,CODE_L1L,CODE_L1X,
        CODE_L2S,CODE_L2L,CODE_L2X,
        CODE_L5I,CODE_L5Q,CODE_L5X,
        CODE_L6I,CODE_L6Q,CODE_L6X,CODE_L1A,0
    };
    const unsigned char codes_sbs[16]={
        CODE_L1C,
        CODE_L5I,CODE_L5Q,CODE_L5X,0
    };
    const unsigned char *codes;
    cssr_mask_t cssr_mask0={{0}};
    
    /* initialize compact ssr mask */
    *cssr_mask=cssr_mask0;
    
    /* decode compact ssr header */
    i=decode_cssr_head(rtcm,cssr,&sync,&iod,&udint,&refd,&i);
    
    /* number of GNSS */
    nsys=getbitu(rtcm->buff,i,4); i+=4;
    dump(1," NGNSS=%d\n",nsys);
    cssr_mask->nsys=nsys;
    
    for (j=0;j<nsys;j++) {
        
        gnssid=getbitu(rtcm->buff,i,4); i+=4; /* gnss id */
        dump(1,"[%d/%d]GNSSID =%d\n",j,nsys,gnssid);
        
        switch (gnssid) {
            case 0: sys=SYS_GPS; codes=codes_gps; isys=0; minprn=MINPRNGPS; break;
            case 1: sys=SYS_GLO; codes=codes_glo; isys=1; minprn=MINPRNGLO; break;
            case 2: sys=SYS_GAL; codes=codes_gal; isys=2; minprn=MINPRNGAL; break;
            case 3: sys=SYS_CMP; codes=codes_bds; isys=3; minprn=MINPRNCMP; break;
            case 4: sys=SYS_QZS; codes=codes_qzs; isys=4; minprn=MINPRNQZS; break;
            case 5: sys=SYS_SBS; codes=codes_sbs; isys=5; minprn=MINPRNSBS; break;
            default: return sync?0:20;
        }
        
        cssr_mask->sys[isys]=sys;
        
        dump(1,"SATMASK=");
        for (k=0;k<40;k++) {
            b=getbitu(rtcm->buff,i,1); i+=1; /* satellite mask */
            
            sat=satno(sys,k+minprn);
            if (sat<1||sat>=MAXSAT) continue;
            
            cssr_mask->sat_mask[isys][sat-1]=b;
            if (b) cssr_mask->nsat[isys]++;
            dump(1,"%d",b);
        }
        dump(1,"\n");
        
        dump(1,"SIGMASK=");
        for (k=0;k<16;k++) {
            b=getbitu(rtcm->buff,i,1); i+=1; /* signal mask */
            if (b) cssr_mask->nsig[isys]++;
            dump(1,"%d",b);
            
            idx=codes[k]-1;
            if (idx<0) continue;
            cssr_mask->sig_mask[isys][idx]=b;
            
        }
        dump(1,"\n");
        
        cell_ava=getbitu(rtcm->buff,i,1); i+=1; /* cell mask availability flag */
        dump(1,"CELLAVA=%d\n",cell_ava);
        
        cssr_mask->ncell[isys]=cssr_mask->nsat[isys]*cssr_mask->nsig[isys];
        dump(1,"DEBUG: isys=%d nsat=%d nsig=%d ncell=%d\n",
            isys,cssr_mask->nsat[isys],cssr_mask->nsig[isys],cssr_mask->ncell[isys]);
        
        for (k=0;k<cssr_mask->ncell[isys];k++) {
            if (cell_ava) {
                b=getbitu(rtcm->buff,i,1); i+=1; /* cell mask */
            }
            else {
                b=1;
            }
            cssr_mask->cell_mask[isys][k]=b;
        }
        
        dump(1,"CELLMASK=\n");
        nsat=cssr_mask->nsat[isys];
        nsig=cssr_mask->nsig[isys];
        for (isig=0;isig<nsig;isig++) {
            for (isat=0;isat<nsat;isat++) {
                dump(1,"%d",cssr_mask->cell_mask[isys][isig+nsig*isat]);
            }
            dump(1,"\n");
        }
        
        dump(1,"\n");
    }
    cssr_mask->nbit=i;
    cssr_mask->iod=iod;
    return sync?0:1;
}
/* dump cssr 2: orbit correction ---------------------------------------------*/
static int decode_st2(rtcm_t *rtcm, cssr_t *cssr)
{
    cssr_mask_t *cssr_mask=&cssr->cssr_mask;
    double udint,deph[3];
    int i,j,sync,iode,iod,refd=0;
    int k,l,ni;
    char str[4];
    
    /* decode compact ssr header */
    i=decode_cssr_head(rtcm,cssr,&sync,&iod,&udint,&refd,&i);
    if (cssr_mask->iod != iod) {
        trace(2,"decode_st2: %s,iod mismatch %d != %d\n",
                        time_str(cssr_mask->epoch,3),cssr_mask->iod,iod);
        return 0;
    }
    dump(2,"\n");
    
    /* satellite specific part of cssr gnss orbit correction */
    for (j=0;j<CSSR_MAXSYS;j++) {
        for (k=0;k<MAXSAT;k++) {
            if (!cssr_mask->sat_mask[j][k]) continue;
            
            /* number of bits for iode */
            if (satsys(k+1,NULL)==SYS_GAL) ni=10;
            else                           ni= 8;
            
            iode    =getbitu(rtcm->buff,i,ni);       i+=ni;
            deph [0]=getbits(rtcm->buff,i,15)*16E-4; i+=15;
            deph [1]=getbits(rtcm->buff,i,13)*64E-4; i+=13;
            deph [2]=getbits(rtcm->buff,i,13)*64E-4; i+=13;
            
            satno2id(k+1,str);
            
            dump(2,"[%s] IODE=%4d DEPH=%10.4f %10.4f %10.4f\n",
                str,iode,deph[0],deph[1],deph[2]);
            
            rtcm->ssr[k].t0 [0]=rtcm->time;
            rtcm->ssr[k].udi[0]=udint;
            rtcm->ssr[k].iod[0]=iod;
            rtcm->ssr[k].iode=iode;   
            rtcm->ssr[k].iodcrc=iode; 
            rtcm->ssr[k].refd=refd;
            
            for (l=0;l<3;l++) {
                rtcm->ssr[k].deph [l]=deph [l];
                rtcm->ssr[k].ddeph[l]=0.0;
            }
            rtcm->ssr[k].update=1;
            
        }
    }
    return sync?0:2;
}
/* dump cssr 3: clock correction ---------------------------------------------*/
static int decode_st3(rtcm_t *rtcm, cssr_t *cssr)
{
    cssr_mask_t *cssr_mask=&cssr->cssr_mask;
    double udint,dclk;
    int i,j,sync,iod,refd=0;
    int k;
    char str[4];
    
    /* decode compact ssr header */
    i=decode_cssr_head(rtcm,cssr,&sync,&iod,&udint,&refd,&i);
    if (cssr_mask->iod != iod) {
        trace(2,"decode_st3: %s,iod mismatch %d != %d\n",
                        time_str(cssr_mask->epoch,3),cssr_mask->iod,iod);
        return 0;
    }
    dump(3,"\n");
    
    /* satellite specific part of cssr gnss clock correction */
    for (j=0;j<CSSR_MAXSYS;j++) {
        for (k=0;k<MAXSAT;k++) {
            if (!cssr_mask->sat_mask[j][k]) continue;
            
            dclk=getbits(rtcm->buff,i,15)*16E-4; i+=15;
            satno2id(k+1,str);
            
            dump(3,"[%s] DCLK=%12.4f\n",str,dclk);
            
            rtcm->ssr[k].t0 [1]=rtcm->time;
            rtcm->ssr[k].udi[1]=udint;
            rtcm->ssr[k].iod[1]=iod;
            rtcm->ssr[k].dclk[0]=dclk;
            rtcm->ssr[k].dclk[1]=0.0;
            rtcm->ssr[k].dclk[2]=0.0;
            rtcm->ssr[k].update=1;
        }
    }
    return sync?0:3;
}
/* dump cssr 4: code bias correction -----------------------------------------*/
static int decode_st4(rtcm_t *rtcm, cssr_t *cssr)
{
    cssr_mask_t *cssr_mask=&cssr->cssr_mask;
    double udint,bias,cbias[MAXCODE]={0.0};
    int i,j,sync,iod,refd=0,vcbias[MAXCODE]={0};
    int isat,isig;
    int k,l;
    char str[4];
    
    /* decode compact ssr header */
    i=decode_cssr_head(rtcm,cssr,&sync,&iod,&udint,&refd,&i);
    if (cssr_mask->iod != iod) {
        trace(2,"decode_st4: %s,iod mismatch %d != %d\n",
                        time_str(cssr_mask->epoch,3),cssr_mask->iod,iod);
        return 0;
    }
    dump(4,"\n");
    
    /* satellite specific part of cssr gnss code bias correction */
    for (j=0;j<CSSR_MAXSYS;j++) {
        isat=0;
        for (k=0;k<MAXSAT;k++) {
            if (!cssr_mask->sat_mask[j][k]) continue;
            satno2id(k+1,str);
            dump(4,"[%s]",str);
            
            isig=-1;
            for (l=0;l<MAXCODE;l++) { 
                cbias[l]=0.0;
                vcbias[l]=0;
                if (!cssr_mask->sig_mask[j][l]) continue;
                isig++;
                
                if (!cssr_mask->cell_mask[j][isig+cssr_mask->nsig[j]*isat]) continue;
                
                bias=getbits(rtcm->buff,i,11)*0.02; i+=11;
                cbias[l]=bias;
                vcbias[l]=1; /* 1:output */
                dump(4," CODE=%s(%2d)",code2obs(l+1),l+1);
                dump(4," BIAS=%10.2f",bias);
                
            }
            dump(4,"\n");
            isat++;
            
            rtcm->ssr[k].t0 [4]=rtcm->time;
            rtcm->ssr[k].udi[4]=udint;
            rtcm->ssr[k].iod[4]=iod;
            for (l=0;l<MAXCODE;l++) {
                rtcm->ssr[k].cbias[l]=(float)cbias[l];
                rtcm->ssr[k].vcbias[l]=vcbias[l]; /* 1:output */
            }
            rtcm->ssr[k].update=1;
        }
    }
    return sync?0:4;
}
/* dump cssr 5: phase bias correction ----------------------------------------*/
static int decode_st5(rtcm_t *rtcm, cssr_t *cssr)
{
    cssr_mask_t *cssr_mask=&cssr->cssr_mask;
    double udint,bias,pbias[MAXCODE]={0.0};
    int i,j,sync,iod,refd=0,vpbias[MAXCODE]={0},discnt[MAXCODE]={0};
    int isat,isig;
    int k,l,di;
    char str[4];
    
    /* decode compact ssr header */
    i=decode_cssr_head(rtcm,cssr,&sync,&iod,&udint,&refd,&i);
    if (cssr_mask->iod != iod) {
        trace(2,"decode_st5: %s,iod mismatch %d != %d\n",
                        time_str(cssr_mask->epoch,3),cssr_mask->iod,iod);
        return 0;
    }
    dump(5,"\n");
    
    /* satellite specific part of cssr gnss phase bias correction */
    for (j=0;j<CSSR_MAXSYS;j++) {
        isat=0;
        for (k=0;k<MAXSAT;k++) {
            if (!cssr_mask->sat_mask[j][k]) continue;
            satno2id(k+1,str);
            dump(5,"[%s]",str);
            
            isig=-1;
            for (l=0;l<MAXCODE;l++) { 
                pbias[l]=0.0;
                vpbias[l]=0;
                discnt[l]=0;
                if (!cssr_mask->sig_mask[j][l]) continue;
                isig++;
                
                if (!cssr_mask->cell_mask[j][isig+cssr_mask->nsig[j]*isat]) continue;
                
                bias=getbits(rtcm->buff,i,15)*0.001; i+=15;
                di  =getbitu(rtcm->buff,i, 2);       i+= 2; /* discontinuity indicator */
                pbias [l]=bias;
                discnt[l]=di;
                vpbias[l]=1; /* 1:output */
                dump(5," PHASE=%s(%2d)",code2obs(l+1),l+1);
                dump(5," BIAS=%9.3f DI=%d",bias,di);
                
            }
            dump(5,"\n");
            isat++;
            
            
            rtcm->ssr[k].t0 [5]=rtcm->time;
            rtcm->ssr[k].udi[5]=udint;
            rtcm->ssr[k].iod[5]=iod;
            rtcm->ssr[k].yaw_ang =0.0; /* (deg) */
            rtcm->ssr[k].yaw_rate=0.0; /* (deg/s) */
            
            for (l=0;l<MAXCODE;l++) {
                rtcm->ssr[k].pbias[l]=pbias[l];
                rtcm->ssr[k].stdpb[l]=0.0;
                rtcm->ssr[k].vpbias[l]=vpbias[l]; /* 1:output */
                rtcm->ssr[k].discnt[l]=discnt[l];
            }
        }
    }
    return sync?0:5;
}
/* dump cssr 7: ura --------------------------------------------------------*/
static int decode_st7(rtcm_t *rtcm, cssr_t *cssr)
{
    cssr_mask_t *cssr_mask=&cssr->cssr_mask;
    double udint;
    int i,j,sync,iod,refd=0;
    int k,cla,val,ura;
    char str[4];
    double ura_mm;
    /* decode compact ssr header */
    i=decode_cssr_head(rtcm,cssr,&sync,&iod,&udint,&refd,&i);
    if (cssr_mask->iod != iod) {
        trace(2,"decode_st7: %s,iod mismatch %d != %d\n",
                        time_str(cssr_mask->epoch,3),cssr_mask->iod,iod);
        return 0;
    }
    dump(6,"\n");
    
    /* satellite specific part of cssr gnss clock correction */
    for (j=0;j<CSSR_MAXSYS;j++) {
        for (k=0;k<MAXSAT;k++) {
            if (!cssr_mask->sat_mask[j][k]) continue;
            
            ura=getbitu(rtcm->buff,i,6); i+=6;
            cla=(ura>>3);
            val=(ura&0x07);
            satno2id(k+1,str);
            ura_mm=pow(3.0,(double)cla)*(1.0+(double)val/4.0)-1.0; /* (mm) */
            
            dump(6,"[%s] URA=%2d CLASS=%d VAL=%d URA[mm]=%8.3f\n",str,ura,cla,val,ura_mm);
            
            rtcm->ssr[k].t0 [3]=rtcm->time;
            rtcm->ssr[k].udi[3]=udint;
            rtcm->ssr[k].iod[3]=iod;
            rtcm->ssr[k].ura=ura;
            rtcm->ssr[k].update=1;
        }
    }
    return sync?0:7;
}
/* dump rtcm 3 message -------------------------------------------------------*/
extern int decode_cssr(rtcm_t *rtcm, cssr_t *cssr)
{
    double tow;
    int week,ret=0;
    int mt  =getbitu(rtcm->buff, 0,12);
    int type=getbitu(rtcm->buff,12, 4);
    
    trace(3,"%-10s(%4d): len=%3d type=%d\n","decode_cssr",__LINE__,rtcm->len,type);
    
    if (mt!=CSSR_MSGTYPE) {
        trace(3,"%-10s(%4d): no compact ssr message type %d\n","decode_cssr",__LINE__,mt);
        return ret;
    }
    if (rtcm->outtype) {
        sprintf(rtcm->msgtype,"RTCM %4d (%4d):",type,rtcm->len);
    }
    /* real-time input option */
    if (strstr(rtcm->opt,"-RT_INP")) {
        tow=time2gpst(utc2gpst(timeget()),&week);
        rtcm->time=gpst2time(week,floor(tow));
    }
    switch (type) {
        case  1: ret=decode_st1 (rtcm,cssr); break;
        case  2: ret=decode_st2 (rtcm,cssr); break;
        case  3: ret=decode_st3 (rtcm,cssr); break;
        case  4: ret=decode_st4 (rtcm,cssr); break;
        case  5: ret=decode_st5 (rtcm,cssr); break;
        case  7: ret=decode_st7 (rtcm,cssr); break;
    }
    if (ret>=0) {
        if      (1001<=type&&type<=1299) rtcm->nmsg3[type-1000]++; /* 1001-1299 */
        else if (   1<=type&&type<=  99) rtcm->nmsg3[type+ 300]++; /*    1-  99 */
        else rtcm->nmsg3[0]++; /* others */
    }
    return ret;
}
