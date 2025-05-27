/*------------------------------------------------------------------------------
* cssr2ssr : convert cssr message file to ssr message file
*
* Copyright (c) 2024-2025 Cabinet Office, Japan, All rights reserved.
* Copyright (c) 2022-2025, Lighthouse Technology & Consulting Co. Ltd., All rights reserved.
*
* author  : LHTC
* history : 2022/02/03 1.0  new
*           2025/03/18 1.1 use CSSR_MAXNSIG instead of MAXCODE
*
*-----------------------------------------------------------------------------*/
#include "rtklib.h"
#include "cssr.h"

#define L6MSGLEN       2000            /* L6 message length (bits) */
#define L6HDRLEN       49              /* L6 message header length (bits) */
#define L6DATLEN       1695            /* L6 message length (bits) */
#define L6RSLEN        256             /* L6 message reed solomon length (bits) */
#define L6FMSGREAMB    0x1ACFFC1Du     /* L6 message preamble */
#define MAX_SUBTYPE1_LEN 7059          /* max length of subtype 1 (bits) */
                                       /*  = 49+(61+Ncell)*Nsys */
                                       /*  = 49+(61+16*40)*10 */

#define NUM_MSG_FRAME        30        /* number of messages per fram */
#define NUM_MSG_SUBFRAME      5        /* number of messages per subfram */

#define VENDORID_CLAS       0x5        /* vendor id :CLAS   101b */
#define VENDORID_MADOCA     0x2        /* vendor id :MADOCA 010b */
#define VENDORID_QZNMA      0x3        /* vendor id :QZNMA  011b */

#define SERVICE_CEC         0          /* correction service id : clock/ephemeris */
#define SERVICE_ION         1          /* correction service id : ionosphere */

typedef struct {                       /* L6 data stream type */
    uint32_t preamble;                 /* message preamble */
    int sync_subframe;                 /* flag for subframe synchronization */
    int sync_frame;                    /* flag for frame synchronization */
    int prn;                           /* PRN number of the satellite transmitting that message */
    int facility_id;                   /* Message Generation Facility ID */
    int msgnum;                        /* message number */
    int subtype;                       /* sub type id */
    int n_msg;                         /* number of messages in frame (1-30) */
    int nbyte;                         /* number of bytes in message buffer */
    int nbit;                          /* number of bis in subframe buffer */ 
    uint8_t buff[L6MSGLEN/8];          /* message buffer */
    uint8_t subframe[L6MSGLEN/8*5];    /* subframe buffer */
} l6str_t;

static rtcm_t gl_rtcm,gl_out;


/* calculate size of subtype 1 (bits) ----------------------------------------*/
static int get_sizeof_subtype1(const cssr_t *cssr)
{
    return cssr->cssr_mask.nbit;
}
/* calculate size of subtype 2 (bits) ----------------------------------------*/
static int get_sizeof_subtype2(cssr_t *cssr)
{
    cssr_mask_t *cssr_mask=&cssr->cssr_mask;
    int i,nb,nbit;
    
    nbit=37;
    for (i=0;i<CSSR_MAXSYS;i++) {
        if (cssr_mask->sys[i]==0) continue;
        nb=cssr_mask->sys[i]==SYS_GAL?51:49;
        nbit+=nb*cssr_mask->nsat[i];
    }
    return nbit;
}
/* calculate size of subtype 3, 7 (bits) --------------------------------------*/
static int get_sizeof_subtype_nsat(cssr_t *cssr, int nb)
{
    cssr_mask_t *cssr_mask=&cssr->cssr_mask;
    int i;
    int nbit;
    
    nbit=37;
    for (i=0;i<CSSR_MAXSYS;i++) {
        if (cssr_mask->sys[i]==0) continue;
        nbit+=nb*cssr_mask->nsat[i];
    }
    return nbit;
}
/* calculate size of subtype 4, 5 (bits) --------------------------------------*/
static int get_sizeof_subtype_ncell(cssr_t *cssr, int nb)
{
    cssr_mask_t *cssr_mask=&cssr->cssr_mask;
    int i,k,l,isat,isig;
    int nbit;
    
    nbit=37;
    for (i=0;i<CSSR_MAXSYS;i++) {
        isat=0;
        for (k=0;k<MAXSAT;k++) {
            if (!cssr_mask->sat_mask[i][k]) continue;
            isig=-1;
            for (l=0;l<CSSR_MAXNSIG;l++) {
                if (!cssr_mask->sig_mask[i][l]) continue;
                isig++;
                if (!cssr_mask->cell_mask[i][isig+cssr_mask->nsig[i]*isat]) continue;
                nbit+=nb;
            }
            isat++;
        }
    }
    return nbit;
}
/* calculate size of subtypes (bits) ------------------------------------------*/
static int get_sizeof_subtype(cssr_t *cssr, int subtype)
{
    int nbit=0;
    
    /* no ssr mask information */
    if (!get_sizeof_subtype1(cssr)) return 0;
    
    switch (subtype) {
        case 1:  nbit=get_sizeof_subtype1     (cssr   ); break;
        case 2:  nbit=get_sizeof_subtype2     (cssr   ); break;
        case 3:  nbit=get_sizeof_subtype_nsat (cssr,15); break;
        case 4:  nbit=get_sizeof_subtype_ncell(cssr,11); break;
        case 5:  nbit=get_sizeof_subtype_ncell(cssr,17); break;
        case 7:  nbit=get_sizeof_subtype_nsat (cssr, 6); break;
        default: nbit=0;
    }
    trace(3,"%-10s(%04d): nbit=%d subtype=%d\n","get_sizeof_subtype",__LINE__,nbit,subtype);
    return nbit;
}
/* clear L6 message buffer ---------------------------------------------------*/
static void clear_buff(l6str_t *l6str)
{
    int i;
    l6str->nbyte=0;
    for (i=0;i<L6MSGLEN/8;i++) l6str->buff[i]=0;
}
/* synchronize L6 message (1s) -----------------------------------------------*/
static int sync_l6message(l6str_t *l6str, int data)
{
    
    if (l6str->nbyte==0) {
        
        /* detect preamble */
        l6str->preamble = (l6str->preamble << 8) | data;
        if (l6str->preamble != L6FMSGREAMB) {
            return 0;
        }
        
        l6str->buff[0]=(L6FMSGREAMB>>24) & 0xff;
        l6str->buff[1]=(L6FMSGREAMB>>16) & 0xff;
        l6str->buff[2]=(L6FMSGREAMB>> 8) & 0xff;
        l6str->buff[3]=(L6FMSGREAMB    ) & 0xff;
        l6str->nbyte=4;
        return 0;
    }
    
    return 1; /* synchronized */
}
/* synchronize subframe (5s) --------------------------------------------------*/
static int sync_subframe(l6str_t *l6str, int sf_id, const uint8_t *buff)
{
    l6str->msgnum=0;
    l6str->subtype=0;
    
    if (sf_id==1) {
        l6str->sync_subframe=1; /* synchronized */
        l6str->nbit=0;
        
        l6str->msgnum =getbitu(buff,   49,12);
        l6str->subtype=getbitu(buff,49+12, 4);
        
        return 1;
    }
    else {
        if (l6str->sync_subframe) return 1;  /* already synchronized */
    }
    
    return 0;
}
/* synchronize frame (30s) ---------------------------------------------------*/
static int sync_frame(l6str_t *l6str, int prn, int facility_id)
{
    /* synchronize Compact SSR Mask (subtype 1) <- subtype1 header */
    if (l6str->msgnum==CSSR_MSGTYPE && l6str->subtype==1) {
        l6str->sync_frame=1; /* synchronized */
        l6str->n_msg=1;      /* start from "1" (1-30) */
        l6str->prn=prn;
        l6str->facility_id=facility_id;
        return 1;
    }
    else {
        if (l6str->sync_frame) return 1; /* already synchronized */
    }
    
    return 0;
}
/* check end of subframe (5s) -------------------------------------------------*/
static int is_end_subframe(const l6str_t *l6str)
{
    if (l6str->n_msg%NUM_MSG_SUBFRAME==0) return 1;
    return 0;
}
/* copy data part to subframe buffer ------------------------------------------*/
static int set_subframe(l6str_t *l6str, const uint8_t *buff)
{
    uint32_t bit;
    int i;
    
    /* only data part */
    for (i=L6HDRLEN;i<L6HDRLEN+L6DATLEN;i++) {
        bit=getbitu(buff,i,1);
        if (l6str->nbit>=L6DATLEN*5) return 0;
        setbitu(l6str->subframe,l6str->nbit++,1,bit);
    }
    return 1;
}
/* copy subframe buffer to rtcm message buffer --------------------------------*/
static int copy_subframe2rtcm(rtcm_t *rtcm, const l6str_t *l6str, int pos, int len)
{
    uint32_t bit;
    int i;
    
    if (pos+len>MAX_SUBTYPE1_LEN) {
        return 0;
    }
    
    for (i=0;i<len;i++) {
        bit=getbitu(l6str->subframe,pos+i,1);
        setbitu(rtcm->buff,i,1,bit);
    }
    rtcm->len=len/8+1; /* (byte) */
    
    return 1;
}
/* initialize ssr -------------------------------------------------------------*/
static void init_ssr(rtcm_t *rtcm)
{
    ssr_t ssr0={{{0}}};
    int i;
    
    for (i=0;i<MAXSAT;i++) {
        rtcm->ssr[i]=ssr0;
    }
}
/* write ssr to file ----------------------------------------------------------*/
static void write_ssr(FILE *ofp, rtcm_t *out, int subtype, cssr_t *cssr)
{
    int i,isys,sys;
    const int ssr_types[5][7]={
    /*  { ST1, ST2,  ST3,   ST4,  ST5,  ST6,  ST7} */
    /*  {MASK, OBT,  CLK, CBIAS,PBIAS,  N/A,  URA} */
        {   0, 1057, 1058, 1059, 1265,    0, 1061}, /* GPS */
        {   0, 1063, 1064, 1065, 1266,    0, 1067}, /* GLONASS */
        {   0, 1240, 1241, 1242, 1267,    0, 1244}, /* Galileo */
        {   0, 1258, 1259, 1260, 1270,    0, 1262}, /* BeiDou */
        {   0, 1246, 1247, 1248, 1268,    0, 1250}  /* QZSS */
    };
    
    for (i=0;i<CSSR_MAXSYS;i++) {
        sys=cssr->cssr_mask.sys[i];
        switch (sys) {
            case SYS_GPS: isys=0; break;
            case SYS_GLO: isys=1; break;
            case SYS_GAL: isys=2; break;
            case SYS_CMP: isys=3; break;
            case SYS_QZS: isys=4; break;
            default: continue;
        }
        
        gen_rtcm3(out,ssr_types[isys][subtype-1],0,0);
        fwrite(out->buff,1,out->nbyte,ofp);

        trace(2,"%-10s(%04d): %s isys=%d subtype=%d:%4d nbyte=%3d nbit=%4d\n","write_ssr",__LINE__,
            time_str(out->time,0),isys,subtype,ssr_types[isys][subtype-1],out->nbyte,out->nbit);
        
    }
}
/* encode ssr orbit, code bias, phase bias, ura ------------------------------*/
static int encode_ssr(FILE *ofp, rtcm_t *inp, cssr_t *cssr, int subtype)
{
    int i,type;
    
    const int types[]={
    /*  {ST1, ST2,  ST3,   ST4,  ST5, ST6, ST7} */
    /*  {MASK, OBT,  CLK, CBIAS,PBIAS, N/A, URA} */
           -1,   0,    1,     4,    5,  -1,   3 /*{0:eph,1:clk,2:hrclk,3:ura,4:bias,5:pbias}*/
    };
    
    trace(3,"%-10s(%04d): %s subtype=%d\n","encode_ssr",
        __LINE__,time_str(inp->time,0),subtype);
    
    if (subtype<1 || subtype>7) {
        trace(1,"%-10s(%04d): error, %s subtype=%d\n","encode_ssr",
            __LINE__,time_str(inp->time,0),subtype);
        return 0;
    }
    type=types[subtype-1];
    if (type<0) {
        trace(1,"%-10s(%04d): error, %s type=%d subtype=%d\n","encode_ssr",
            __LINE__,time_str(inp->time,0),type,subtype);
        return 0;
    }
    
    init_rtcm(&gl_out);
    
    for (i=0;i<MAXSAT;i++) {
        gl_out.ssr[i]=inp->ssr[i];
        if (!inp->ssr[i].update) continue;
        gl_out.time=inp->ssr[i].t0[type];
    }
    write_ssr(ofp,&gl_out,subtype,cssr);
    
    free_rtcm(&gl_out);
    return 1;
}

/* decode cssr messages in the QZS L6 subframe --------------------------------*/
static int decode_subframe(FILE *ofp, rtcm_t *rtcm, cssr_t *cssr, const l6str_t *l6str)
{
    int msgnum =getbitu(l6str->subframe, 0,12);
    int subtype=getbitu(l6str->subframe,12, 4);
    int nbit;
    int pos=0;
    int i,ret;
    
    trace(3,"%-10s(%04d): msgnum=%d subtype=%d MAX_SUBTYPE1_LEN/8=%d\n",
        "decode_subframe",__LINE__,msgnum,subtype,MAX_SUBTYPE1_LEN/8);
    
    /* decode subtype 1 and seek next subtype */
    if (subtype==1) {
        for (i=0;i<MAX_SUBTYPE1_LEN/8;i++) rtcm->buff[i]=l6str->subframe[i];
        rtcm->len=MAX_SUBTYPE1_LEN/8;
        
        init_ssr(rtcm);
        
        /* get cssr mask information to calculate the size of each subtype */
        decode_cssr(rtcm,cssr);
        
        /* get size of subtype 1 (bits) */
        pos=get_sizeof_subtype(cssr,subtype);
    }
    
    /* fetch subtype from subframe */
    while (1) {
        if (pos>=L6DATLEN*5-12-4) break; /* exceed subframe length (bits) */
        msgnum =getbitu(l6str->subframe, pos   ,12);
        subtype=getbitu(l6str->subframe, pos+12, 4);
        
        nbit=get_sizeof_subtype(cssr,subtype);
        if (msgnum!=CSSR_MSGTYPE) break;
        if (nbit==0) break;
        
        trace(3,"%-10s(%04d): msgnum=%d subtype=%d nbit=%d\n",
            "decode_subframe",__LINE__,msgnum,subtype,nbit);
        
        if (!copy_subframe2rtcm(rtcm,l6str,pos,nbit)) break;
        pos+=nbit;
        ret=decode_cssr(rtcm,cssr);
        
        if (ret>1) {
            encode_ssr(ofp,rtcm,cssr,ret);
        }
    }
    return 1;
}
/* fetch next L6 subframe and frame from byte stream -------------------------*/
extern int input_cssr(l6str_t *l6str, int data)
{
    uint32_t type;
    int prn,alert;
    int vendor_id,facility_id,service_id,navmsg_ext,sf_id;
    
    /* synchronize L6 message (preamble) */
    if (!sync_l6message(l6str,data)) return 0;
    
    /* set data to the buffer */
    l6str->buff[l6str->nbyte++]=(uint8_t)data;
    
    if (l6str->nbyte<L6MSGLEN/8) return 0;
    l6str->n_msg++;
    
    /* message header */
    prn  =getbitu(l6str->buff,32, 8);
    type =getbitu(l6str->buff,40, 8);
    alert=getbitu(l6str->buff,48, 1);
    
    /* message type ID */
    vendor_id  =(type&0xE0)>>5;
    facility_id=(type&0x18)>>3; /* Message Generation Facility ID */
    service_id =(type&0x04)>>2; /* Correction Service ID */
    navmsg_ext =(type&0x02)>>1; /* Applicable Navigation Message Extention */
    sf_id      =(type&0x01);    /* Subframe Indicator */
    
    trace(3,"%-10s(%04d): prn=%3d type=0x%02X alert=%d vendor_id=%2d facility_id=%2d service_id=%d navmsg_ext=%d sf_id=%d\n","input_cssr",__LINE__,
        prn,type,alert,vendor_id,facility_id,service_id,navmsg_ext,sf_id);
    
    /* read only MADOCA-PPP */
    if (vendor_id!=VENDORID_MADOCA) {
        trace(2,"%-10s(%04d): skip unreading message vendor-id prn=%3d type=0x%02X alert=%d vendor_id=%2d facility_id=%2d\n",
              "input_cssr",__LINE__,prn,type,alert,vendor_id,sf_id);
        clear_buff(l6str);
        return 0;
    }
    
    /* read only clock/ephemeris correction */
    if (service_id!=SERVICE_CEC) {
        trace(2,"%-10s(%04d): skip ionospheric corrections prn=%3d type=0x%02X alert=%d vendor_id=%2d facility_id=%2d\n",
              "input_cssr",__LINE__,prn,type,alert,vendor_id,sf_id);
        clear_buff(l6str);
        return 0;
    }

    /* Alert Flag check */
    if (alert) {
        trace(1,"%-10s(%04d): prn=%3d type=0x%02X alert=%d Alert Flag NG.\n","input_cssr",__LINE__, prn,type,alert);
        clear_buff(l6str);
        return 0;
    }

    /* Applicable Navigation Message Extention 1:CNAV/CNAV2 */
    if (navmsg_ext) {
        trace(1,"%-10s(%04d): prn=%3d type=0x%02X navmsg_ext=%d Applicable Navigation Message Extention CNAV/CNAV2\n",
            "input_cssr",__LINE__, prn,type,alert,facility_id,navmsg_ext);
    }

    /* synchronize subframe (subframe indicator) <- 1subframe */
    if (!sync_subframe(l6str,sf_id,l6str->buff)) {
        clear_buff(l6str);
        return 0;
    }
    
    /* synchronize Compact SSR Mask (subtype 1) <- subtype1 header */
    if (!sync_frame(l6str,prn,facility_id)) {
        clear_buff(l6str);
        return 0;
    }
    
    /* skip unmatched facility ID */
    if ((facility_id&0x1)!=(l6str->facility_id&&0x1)) {
        trace(2,"%-10s(%04d): skip unmatched facility ID prn=%3d type=0x%02X alert=%d vendor_id=%2d facility_id=%2d / %2d\n",
              "input_cssr",__LINE__,prn,type,alert,vendor_id,sf_id,l6str->facility_id);
        clear_buff(l6str);
        return 0;
    }
    
    /* copy data part to subframe buffer */
    if (!set_subframe(l6str,l6str->buff)) {
        clear_buff(l6str);
        return 0;
    }
    trace(3,"%-10s(%04d): n_mng=%2d subtype=%2d\n","input_cssr",__LINE__,
        l6str->n_msg,l6str->subtype);
    
    /* end of frame (30s) */
#if 0    
    if (is_end_frame(l6str)) {
        clear_buff(l6str);
        return 2;
    }
#endif    
    /* end of subframe (5s) */
    if (is_end_subframe(l6str)) {
        clear_buff(l6str);
        return 1;
    }
    
    /* initialize sync status */
    if (l6str->n_msg>NUM_MSG_FRAME) {
        trace(2,"%-10s(%04d): invalid message n_mng=%2d.\n","input_cssr",__LINE__,
            l6str->n_msg);
        clear_buff(l6str);
        l6str->sync_subframe=0;
        l6str->sync_frame=0;
        return 0;
    }
    
    clear_buff(l6str);
    return 0;
}
/* convert cssr to ssr -------------------------------------------------------*/
static int cssr2ssr(const char *ifile, const char *ofile, gtime_t time)
{
    FILE *ifp,*ofp;
    l6str_t l6str={0};
    cssr_t cssr;
    int data;
    int ret;
    
    fprintf(stderr,"%-10s(%04d):\n","cssr2ssr",__LINE__);
    
    init_rtcm(&gl_rtcm);
    cssr_init(&cssr);
    gl_rtcm.time=time;
    
    /* open input file */
    if (!(ifp=fopen(ifile,"rb"))) {
        fprintf(stderr,"file open error: %s\n",ifile);
        return 0;
    }
    fprintf(stderr,"reading... %s\n",ifile);
    
    /* open input file */
    if (!(ofp=fopen(ofile,"wb"))) {
        fprintf(stderr,"file open error: %s\n",ofile);
        return 0;
    }
    
    while ((data=fgetc(ifp))>=0) {
        ret=input_cssr(&l6str,data);
        if (ret>0) decode_subframe(ofp,&gl_rtcm,&cssr,&l6str);
    }
    fclose(ifp);
    fclose(ofp);
    
    fprintf(stderr,"finished\n");
    
    free_rtcm(&gl_rtcm);
    return 1;
}
/*------------------------------------------------------------------------------
*  name:
*    cssr2ssr - cssr2ssr : convert Compact SSR (cssr) message file to SSR message file
*
*  synopsis:
*    cssr2ssr [options] file
*
*  description:
*    Convert cssr message file to ssr message file. Specify the cssr file as file in
*    command line options. The results are output to output file specified by -o options.
*    supported Compact SSR messages (cssr) and SSR messages (ssr) are as bellow.
*
*    cssr: subtype 1(mask),2(orbit),3(clock),4(code bias),5(phase bias),7(ura)
*    ssr : orbit,   clock, code bias, phase bias,    ura
*           1057,    1058,      1059,       1265,   1061 : GPS
*           1063,    1064,      1065,       1266,   1067 : GLONASS
*           1240,    1241,      1242,       1267,   1244 : Galileo
*           1246,    1247,      1248,       1268,   1250 : QZSS
*           1258,    1259,      1260,       1270,   1262 : BeiDou
*
*  options:
*    -td y/m/d      date for message time (y=year,m=month,d=day)
*    -o file        output ssr file
*    -t level       log level of tracefile. 0:no output, 1-4 [default : 0]
*    -d             dump debug log on decoding cssr          [default : no dump]
*    file           input cssr file
*
*-----------------------------------------------------------------------------*/
int main(int argc, char **argv)
{
    gtime_t time=utc2gpst(timeget());
    int i,level=0;
    char *ifile="",*ofile="";
    char *tfile="cssr2ssr.trace";
    double ep[6]={0};
    
    for (i=1;i<argc;i++) {
        if      (!strcmp(argv[i],"-o")&&i+1<argc) ofile=argv[++i];
        else if (!strcmp(argv[i],"-t")&&i+1<argc) level=atoi(argv[++i]);
        else if (!strcmp(argv[i],"-d"))           cssr_dump_set(0);
        else if (!strcmp(argv[i],"-td")&&i+1<argc) {
            sscanf(argv[++i],"%lf/%lf/%lf",ep,ep+1,ep+2);
        }
        else ifile=argv[i];
    }
    if (ep[0]>0.0) time=epoch2time(ep);
    
    if (level>0 && level<5) {
        traceopen(tfile);
        tracelevel(level);
        trace(level,"trace open level=%d\n",level);
    }
    
    if (!*ifile) {
        fprintf(stderr,"Error : specify input file\n");
        trace(2,"Error : specify input file\n");
        traceclose();
        return -1;
    }
    if (!*ofile) {
        fprintf(stderr,"Error : specify output file\n");
        trace(2,"Error : specify output file\n");
        traceclose();
        return -1;
    }
    
    trace(3,"ifile = %s\n",ifile);
    trace(3,"ofile = %s\n",ofile);
    
    if (!cssr2ssr(ifile,ofile,time)) {
        fprintf(stderr,"Error: conversion error\n");
        trace(2,"Error: conversion error\n");
        traceclose();
        return -1;
    }
    
    traceclose();
    
    return 0;
}
