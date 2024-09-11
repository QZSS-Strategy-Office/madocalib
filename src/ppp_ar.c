/*------------------------------------------------------------------------------
* ppp_ar.c : ppp ambiguity resolution
*
*          Copyright (C) 2023-2024 Cabinet Office, Japan, All rights reserved.
*          Copyright (C) 2023-2024 Japan Aerospace Exploration Agency. All Rights Reserved.
*          Copyright (C) 2012-2015 by T.TAKASU, All rights reserved.
*
* reference :
*    [1] H.Okumura, C-gengo niyoru saishin algorithm jiten (in Japanese),
*        Software Technology, 1991
*    [2] CAO IS-QZSS-MDC-002, November, 2023
*
* version : $Revision:$ $Date:$
* history : 2013/03/11  1.0  new
*           2015/05/15  1.1  refine complete algorithms
*           2015/05/31  1.2  delete WL-ambiguity resolution by ILS
*                            add PAR (partial ambiguity resolution)
*           2015/11/26  1.3  support option opt->pppopt=-TRACE_AR
*           2023/02/01  1.4  branch from MALIB PP for MADOCALIB
*           2023/03/08  1.5  change sys to GPS only
*           2024/01/10  1.6  add references [2]
*                            support MADOCA-PPP ionospheric corrections, add NM
*                            delete L5-receiver-dcb estimation, ND
*                            support option opt->arsys in gen_sat_sd()
*-----------------------------------------------------------------------------*/
#include "rtklib.h"

#define MIN_AMB_RES 4         /* min number of ambiguities for ILS-AR */
#define MIN_ARC_GAP 300.0     /* min arc gap (s) */
#define CONST_AMB   0.003     /* constraint to fixed ambiguity */

#define LOG_PI      1.14472988584940017 /* log(pi) */
#define SQRT2       1.41421356237309510 /* sqrt(2) */

#define MIN(x,y)    ((x)<(y)?(x):(y))
#define MAX(x,y)    ((x)>(y)?(x):(y))
#define SQR(x)      ((x)*(x))
#define ROUND(x)    (int)floor((x)+0.5)
#define SWAP_I(x,y) do {int    _tmp=x; x=y; y=_tmp;} while (0)
#define SWAP_D(x,y) do {double _tmp=x; x=y; y=_tmp;} while (0)

/* number and index of ekf states */
#define NF(opt)     ((opt)->ionoopt==IONOOPT_IFLC?1:(opt)->nf)
#define NP(opt)     ((opt)->dynamics?9:3)
#define NC(opt)     (NSYS)
#define NT(opt)     ((opt)->tropopt<TROPOPT_EST?0:((opt)->tropopt==TROPOPT_EST?1:3))
#define NM(opt)     (4)
#define NI(opt)     ((opt)->ionoopt==IONOOPT_EST?MAXSAT:0)
#define NR(opt)     (NP(opt)+NC(opt)+NT(opt)+NM(opt)+NI(opt))
#define IB(s,f,opt) (NR(opt)+MAXSAT*(f)+(s)-1)

/* generate satellite SD (single-difference) ---------------------------------*/
static int gen_sat_sd(rtk_t *rtk, const obsd_t *obs, int n, const int *exc,
                      const double *azel, int f, int *sat1, int *sat2, int *frq)
{
    int sys[4],s=0;
    double elmask,el[MAXOBS];
    int i,j,k,m,ns=0,sat[MAXOBS];

    if(rtk->opt.arsys & SYS_GPS) sys[s++]=SYS_GPS;
    if(rtk->opt.arsys & SYS_GAL) sys[s++]=SYS_GAL;
    if(rtk->opt.arsys & SYS_QZS) sys[s++]=SYS_QZS;
    sys[s]=0;

    elmask=MAX(rtk->opt.elmaskar,rtk->opt.elmin);
    
    for (i=0;sys[i];i++) { /* for each system */
        
        /* sort by elevation angle */
        for (j=m=0;j<n;j++) {
            if (exc[j]||!(satsys(obs[j].sat,NULL)&sys[i])||
                !rtk->ssat[obs[j].sat-1].vsat[f]||azel[1+j*2]<elmask) continue;
            if (obs[j].L[f]==0.0) continue;
            sat[m]=obs[j].sat;
            el[m++]=azel[1+j*2];
        }
        for (j=0;j<m-1;j++) for (k=j+1;k<m;k++) {
            if (el[j]>=el[k]) continue;
            SWAP_I(sat[j],sat[k]); SWAP_D(el[j],el[k]);
        }
        /* generate SD referenced to max elevation angle */
        for (j=1;j<m;j++) {
            sat1[ns]=sat[j];
            sat2[ns]=sat[0];
            frq[ns++]=f;
        }
    }
    return ns; /* # of SD */
}
/* write debug trace for PAR -------------------------------------------------*/
static void write_trace1(rtk_t *rtk, const double *Z, const double *a,
                         const double *Q, int na, const int *sat1,
                         const int *sat2, const int *frq)
{
    const char freq[]="125678";
    char buff[1024],s[32],*p=buff;
    int i,j;
    
    trace(2,"EPOCH=%s NFIX=%d\n",time_str(rtk->sol.time,0),rtk->nfix);
    
    for (i=0,p=buff;i<na;i++) {
        satno2id(sat1[i],s); p+=sprintf(p,"%s ",s);
    }
    trace(2,"     %s          Z*a     STD\n",buff);
    for (i=0,p=buff;i<na;i++) {
        satno2id(sat2[i],s); p+=sprintf(p,"%s ",s);
    }
    trace(2,"     %s         (cyc)   (cyc)\n",buff);
    for (i=0,p=buff;i<na;i++) {
        p+=sprintf(p,"L%c  ",freq[frq[i]]);
    }
    trace(2,"      %s\n",buff);
    for (i=na-1;i>=0;i--) {
        p=buff;
        p+=sprintf(p,"%3d: ",na-i);
        for (j=0;j<na;j++) p+=sprintf(p,"%3.0f ",Z[j+i*na]);
        p+=sprintf(p,"%14.3f %7.3f",a[i],sqrt(Q[i+i*na]));
        trace(2,"%s\n",buff);
    }
    trace(2,"%3s: %7s %9s (%9s/%9s) [ N1 N2 ... NN ]\n","FIX","STD-POS","RATIO",
          "S1","S2");
}
static void write_trace2(rtk_t *rtk, const double *x, const double *P,
                         const double *a, const double *N, const double *D,
                         int na, const double *s)
{
    double *xp,*Pp,b[256],R[256*256]={0},std[3];
    char buff[1024],*p=buff;
    int i;
    
    xp=mat(rtk->nx,1); Pp=mat(rtk->nx,rtk->nx);
    matcpy(xp,x,rtk->nx,1);
    matcpy(Pp,P,rtk->nx,rtk->nx);
    for (i=0;i<na;i++) {
        b[i]=N[i]-a[i];
        R[i+i*na]=SQR(CONST_AMB);
    }
    for (i=na-1;i>=0;i--) {
        p+=sprintf(p,"%s%d",i==na-1?"":" ",(int)N[i]);
    }
    if (!filter(xp,Pp,D,b,R,rtk->nx,na)) {
        for (i=0;i<3;i++) std[i]=sqrt(Pp[i+i*rtk->nx]);
        trace(2,"%3d: %7.3f %9.3f (%9.3f/%9.3f) [%s]\n",na,norm(std,3),
              MIN(99999.999,s[1]/s[0]),s[0],s[1],buff);
    }
    free(xp); free(Pp);
}
/* decorrelate search space --------------------------------------------------*/
static int decorr_space(rtk_t *rtk, double *a, double *Q, double *D, int na,
                        const int *sat1, const int *sat2, const int *frq)
{
    double *W=mat(na,rtk->nx),*Z=eye(na);
    
    /* lambda reduction */
    if (lambda_reduction(na,Q,Z)) {
        free(W); free(Z);
        return 0;
    }
    /* a=Z'*a, Q=Z'*Q*Z, D'=Z'*D' */
    matmul("TN",na,1,na,1.0,Z,a,0.0,W);
    matcpy(a,W,na,1);
    matmul("TN",na,na,na,1.0,Z,Q,0.0,W);
    matmul("NN",na,na,na,1.0,W,Z,0.0,Q);
    matmul("NN",rtk->nx,na,na,1.0,D,Z,0.0,W);
    matcpy(D,W,rtk->nx,na);
    
    if (strstr(rtk->opt.pppopt,"-TRACE_AR")) {
        write_trace1(rtk,Z,a,Q,na,sat1,sat2,frq);
    }
    free(W); free(Z);
    return 1;
}
/* shrink search space -------------------------------------------------------*/
static int shrink_space(double *a, double *Q, double *H, int is, int na, int nx)
{
    int i,j,n=0,*index=imat(na,1);
    
    for (i=is;i<na;i++) {
        index[n++]=i;
    }
    for (i=0;i<n;i++) {
        a[i]=a[index[i]];
        for (j=0;j<n;j++) Q[j+i*n]=Q[index[j]+index[i]*na];
        matcpy(H+i*nx,H+index[i]*nx,nx,1);
    }
    free(index);
    return n;
}
/* freq-ambiguity resolution by ILS ------------------------------------------*/
static int ppp_amb_ILS_FRQ(rtk_t *rtk, const obsd_t *obs, int n, const int *exc,
                           const nav_t *nav, const double *azel,
                           const double *x, const double *P, double *D,
                           double *a, double *N)
{
    const double thres_fact[]={3.0,2.5,2.0,1.5,1.2};
    double *W,*Q,s[2],thres=0.0;
    int i,j,k,l,na=0,sat1[MAXOBS*NFREQ],sat2[MAXOBS*NFREQ],frq[MAXOBS*NFREQ];
    int trace_AR=0;
    
    if (strstr(rtk->opt.pppopt,"-TRACE_AR")) trace_AR=1;
    
    /* generate satellite SD */
    for (i=0;i<rtk->opt.nf;i++) {
        if (rtk->opt.freqopt==1&&i==1) continue;
        na+=gen_sat_sd(rtk,obs,n,exc,azel,i,sat1+na,sat2+na,frq+na);
    }
    if (na<=0) return 0;
    
    /* freq-ambiguity SD-matrix */
    for (i=0;i<na;i++) {
        for(l=0;l<n;l++)if(sat1[i]==obs[l].sat)break;

        j=IB(sat1[i],frq[i],&rtk->opt);
        k=IB(sat2[i],frq[i],&rtk->opt);
        D[j+i*rtk->nx]= 1.0/(CLIGHT/sat2freq(obs[l].sat,obs[l].code[frq[i]],nav));
        D[k+i*rtk->nx]=-1.0/(CLIGHT/sat2freq(obs[l].sat,obs[l].code[frq[i]],nav));
    }
    /* a=D'*x, Q=D'*P*D */
    W=mat(rtk->nx,na); Q=mat(na,na);
    matmul("TN",na,1,rtk->nx,1.0,D,x,0.0,a);
    matmul("TN",na,rtk->nx,rtk->nx,1.0,D,P,0.0,W);
    matmul("NN",na,na,rtk->nx,1.0,W,D,0.0,Q);
    
    /* decorrelate search space */
    if (!decorr_space(rtk,a,Q,D,na,sat1,sat2,frq)) {
        free(W); free(Q);
        return 0;
    }
    for (i=0;i<rtk->opt.armaxiter&&na>=MIN_AMB_RES;i++) {
        
        /* integer least-square (a->N) */
        if (lambda_search(na,2,a,Q,N,s)||s[0]<=0.0) {
            free(W); free(Q);
            return 0;
        }
        if (trace_AR) {
            write_trace2(rtk,x,P,a,N,D,na,s);
        }
        thres=rtk->opt.thresar[0];
        if (na-MIN_AMB_RES<5) thres*=thres_fact[na-MIN_AMB_RES];
        
        /* validation by ratio-test */
        if (s[1]/s[0]>=thres) break;
        
        /* shrink search space */
        na=shrink_space(a,Q,D,1,na,rtk->nx);
    }
    free(W); free(Q);
    
    rtk->sol.ratio=(float)MIN(s[1]/s[0],999.9);
    rtk->sol.thres=(float)thres;
    
    if (i>=rtk->opt.armaxiter||na<MIN_AMB_RES) return 0;
    
    /* update fix flags */
    for (i=0;i<na;i++) {
        rtk->ssat[sat1[i]-1].fix[frq[i]]=rtk->ssat[sat2[i]-1].fix[frq[i]]=2;
    }
    return na;
}
/* ambiguity resolution by ILS (integer-least-square) ------------------------*/
static int ppp_amb_ILS(rtk_t *rtk, const obsd_t *obs, int n, int *exc,
                       const nav_t *nav, const double *azel, double *x,
                       double *P)
{
    double *D,*R,a[MAXOBS*NFREQ],N[MAXOBS*NFREQ*2];
    int i,na,info;
    
    D=zeros(rtk->nx,n*NF(&rtk->opt));
    
    /* freq-ambiguity resolution */
    if (!(na=ppp_amb_ILS_FRQ(rtk,obs,n,exc,nav,azel,x,P,D,a,N))) {
        free(D);
        return 0;
    }
    R=zeros(na,na);
    
    /* update states with integer ambiguity constraints */
    for (i=0;i<na;i++) {
        a[i]=N[i]-a[i];
        R[i+i*na]=SQR(CONST_AMB);
    }
    if ((info=filter(x,P,D,a,R,rtk->nx,na))) {
        trace(1,"filter error (info=%d)\n",info);
    }
    free(D); free(R);
    return info?0:1;
}
/* ambiguity resolution in ppp -----------------------------------------------*/
extern int ppp_ar(rtk_t *rtk, const obsd_t *obs, int n, int *exc,
                  const nav_t *nav, const double *azel, double *x, double *P)
{
    if (n<=0||rtk->opt.modear<ARMODE_CONT) return 0;
    
    if (rtk->opt.ionoopt!=IONOOPT_EST) return 0;
    return ppp_amb_ILS(rtk,obs,n,exc,nav,azel,x,P);
}
