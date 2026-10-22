#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rtklib.h"

typedef struct {
    obs_t obs;
    nav_t nav;
    sta_t sta;
} legsa_nav_context_t;

static const eph_t *legsa_select_eph(gtime_t time, int sat, const nav_t *nav)
{
    double age,tmax,tmin;
    int i,index=-1,sys=satsys(sat,NULL),selection;

    switch (sys) {
        case SYS_GPS: tmax=MAXDTOE+1.0; break;
        case SYS_GAL: tmax=MAXDTOE_GAL; break;
        case SYS_QZS: tmax=MAXDTOE_QZS+1.0; break;
        case SYS_CMP: tmax=MAXDTOE_CMP+1.0; break;
        case SYS_IRN: tmax=MAXDTOE_IRN+1.0; break;
        default: return NULL;
    }
    tmin=tmax+1.0;
    selection=getseleph(sys);
    for (i=0;i<nav->n;i++) {
        if (nav->eph[i].sat!=sat) continue;
        if (sys==SYS_GAL) {
            if (selection==0&&!(nav->eph[i].code&(1<<9))) continue;
            if (selection==1&&!(nav->eph[i].code&(1<<8))) continue;
            if (timediff(nav->eph[i].toe,time)>=0.0) continue;
        }
        age=fabs(timediff(nav->eph[i].toe,time));
        if (age>tmax) continue;
        if (age<=tmin) {index=i; tmin=age;}
    }
    return index>=0?nav->eph+index:NULL;
}

static const geph_t *legsa_select_geph(gtime_t time, int sat,
                                        const nav_t *nav)
{
    double age,tmin=MAXDTOE_GLO+1.0;
    int i,index=-1;

    for (i=0;i<nav->ng;i++) {
        if (nav->geph[i].sat!=sat) continue;
        age=fabs(timediff(nav->geph[i].toe,time));
        if (age>MAXDTOE_GLO) continue;
        if (age<=tmin) {index=i; tmin=age;}
    }
    return index>=0?nav->geph+index:NULL;
}

void *legsa_nav_load_rinex_paths(const char *const *paths, int count)
{
    legsa_nav_context_t *ctx;
    int i;

    if (!paths||count<=0) return NULL;
    ctx=(legsa_nav_context_t *)calloc(1,sizeof(*ctx));
    if (!ctx) return NULL;
    for (i=0;i<count;i++) {
        if (!paths[i]||!*paths[i]||
            readrnx(paths[i],i+1,"",&ctx->obs,&ctx->nav,&ctx->sta)<=0) {
            freeobs(&ctx->obs);
            freenav(&ctx->nav,0xFF);
            free(ctx);
            return NULL;
        }
    }
    uniqnav(&ctx->nav);
    if (ctx->nav.n<=0&&ctx->nav.ng<=0&&ctx->nav.ns<=0) {
        freeobs(&ctx->obs);
        freenav(&ctx->nav,0xFF);
        free(ctx);
        return NULL;
    }
    return ctx;
}

void *legsa_nav_load_rinex(const char *path)
{
    return legsa_nav_load_rinex_paths(&path,1);
}

void *legsa_nav_load_rinex_list(const char *semicolon_paths)
{
    const char **paths;
    char *copy,*p,*begin;
    int count=1,index=0;
    void *ctx;

    if (!semicolon_paths||!*semicolon_paths) return NULL;
    for (p=(char *)semicolon_paths;*p;p++) if (*p==';') count++;
    copy=(char *)malloc(strlen(semicolon_paths)+1);
    paths=(const char **)calloc((size_t)count,sizeof(*paths));
    if (!copy||!paths) {
        free(copy); free(paths);
        return NULL;
    }
    strcpy(copy,semicolon_paths);
    begin=copy;
    for (p=copy;;p++) {
        if (*p==';'||!*p) {
            char *end=p;
            int last=!*p;
            while (begin<end&&(*begin==' '||*begin=='\t')) begin++;
            while (end>begin&&(end[-1]==' '||end[-1]=='\t')) end--;
            *end='\0';
            if (!*begin) {
                free(copy); free(paths);
                return NULL;
            }
            paths[index++]=begin;
            if (last) break;
            begin=p+1;
        }
    }
    ctx=legsa_nav_load_rinex_paths(paths,index);
    free(copy); free(paths);
    return ctx;
}

void legsa_nav_free(void *handle)
{
    legsa_nav_context_t *ctx=(legsa_nav_context_t *)handle;

    if (!ctx) return;
    freeobs(&ctx->obs);
    freenav(&ctx->nav,0xFF);
    free(ctx);
}

int legsa_nav_counts(void *handle, int *gps_gal_bds_qzs, int *glo, int *sbas)
{
    legsa_nav_context_t *ctx=(legsa_nav_context_t *)handle;

    if (!ctx) return 0;
    if (gps_gal_bds_qzs) *gps_gal_bds_qzs=ctx->nav.n;
    if (glo) *glo=ctx->nav.ng;
    if (sbas) *sbas=ctx->nav.ns;
    return 1;
}

int legsa_sat_id_to_no(const char *sat_id)
{
    return sat_id?satid2no(sat_id):0;
}

int legsa_satpos_broadcast(void *handle, int gps_week, double tow, int sat,
                           double *rs6, double *dts2, double *variance,
                           int *health)
{
    legsa_nav_context_t *ctx=(legsa_nav_context_t *)handle;
    gtime_t time;

    if (!ctx||!rs6||!dts2||!variance||!health||sat<=0) return 0;
    time=gpst2time(gps_week,tow);
    return satpos(time,time,sat,EPHOPT_BRDC,&ctx->nav,rs6,dts2,variance,health);
}

int legsa_satpos_transmit_broadcast(void *handle, int receive_gps_week,
                                    double receive_tow, int sat,
                                    double pseudorange_m, double *rs6,
                                    double *dts2, double *variance, int *health)
{
    legsa_nav_context_t *ctx=(legsa_nav_context_t *)handle;
    obsd_t obs;

    if (!ctx||sat<=0||!isfinite(pseudorange_m)||pseudorange_m<=0.0||
        !rs6||!dts2||!variance||!health) return 0;
    memset(&obs,0,sizeof(obs));
    obs.time=gpst2time(receive_gps_week,receive_tow);
    obs.sat=(uint8_t)sat;
    obs.rcv=1;
    obs.P[0]=pseudorange_m;
    satposs(obs.time,&obs,1,&ctx->nav,EPHOPT_BRDC,rs6,dts2,variance,health);
    return rs6[0]!=0.0||rs6[1]!=0.0||rs6[2]!=0.0;
}

int legsa_broadcast_ephemeris_audit(void *handle, int receive_gps_week,
                                    double receive_tow, int sat,
                                    double *signed_age_seconds,
                                    int *toe_gps_week, double *toe_tow,
                                    int *toc_gps_week, double *toc_tow,
                                    int *health, int *iode)
{
    legsa_nav_context_t *ctx=(legsa_nav_context_t *)handle;
    const eph_t *eph;
    const geph_t *geph;
    gtime_t receive;
    int sys;

    if (!ctx||sat<=0||!isfinite(receive_tow)||!signed_age_seconds||
        !toe_gps_week||!toe_tow||!toc_gps_week||!toc_tow||!health||!iode) {
        return 0;
    }
    receive=gpst2time(receive_gps_week,receive_tow);
    sys=satsys(sat,NULL);
    if (sys==SYS_GLO) {
        if (!(geph=legsa_select_geph(receive,sat,&ctx->nav))) return 0;
        *signed_age_seconds=timediff(receive,geph->toe);
        *toe_tow=time2gpst(geph->toe,toe_gps_week);
        *toc_tow=time2gpst(geph->toe,toc_gps_week);
        *health=geph->svh;
        *iode=geph->iode;
        return 1;
    }
    if (!(eph=legsa_select_eph(receive,sat,&ctx->nav))) return 0;
    *signed_age_seconds=timediff(receive,eph->toe);
    *toe_tow=time2gpst(eph->toe,toe_gps_week);
    *toc_tow=time2gpst(eph->toc,toc_gps_week);
    *health=eph->svh;
    *iode=eph->iode;
    return 1;
}

double legsa_sat_frequency_hz(void *handle, int sat, const char *rinex_code)
{
    legsa_nav_context_t *ctx=(legsa_nav_context_t *)handle;
    uint8_t code;

    if (!ctx||sat<=0||!rinex_code) return 0.0;
    code=obs2code(rinex_code);
    return code?sat2freq(sat,code,&ctx->nav):0.0;
}

int legsa_lambda(int n, int candidates, const double *float_ambiguities,
                 const double *covariance, double *integer_candidates,
                 double *squared_norms)
{
    return lambda(n,candidates,float_ambiguities,covariance,
                  integer_candidates,squared_norms);
}

/* In-memory raw-code single-point positioning.  The caller supplies explicit
 * UBX prValid values: numeric pseudorange alone never establishes validity.
 * Return 1 for an accepted pntpos solution, 0 for a normal pntpos rejection,
 * and a negative value for a bridge input/mapping failure. */
int legsa_pntpos_rawx_epoch_v1(
    void *nav_handle, int gps_week, double gps_tow_seconds, int navsys_mask,
    int measurement_count, const int *gnss_id, const int *sv_id,
    const int *sig_id, const int *freq_id, const double *pr_mes_m,
    const int *pr_valid, const double *cp_mes_cycles, const double *do_mes_hz,
    const int *cno_dbhz, const int *cp_std_code, const int *tracking_status,
    const double *initial_rr3, double *rr6, double *qr6, double *dtr6,
    int *solution_status, int *valid_satellite_count,
    int *constructed_observation_count, char *message, int message_capacity)
{
    legsa_nav_context_t *ctx=(legsa_nav_context_t *)nav_handle;
    prcopt_t opt=prcopt_default;
    sol_t sol;
    obsd_t *obs;
    gtime_t time;
    int *sat_index;
    int i,j,nobs=0,status,sys,sat,slot,code,snr;

    if (rr6) memset(rr6,0,6*sizeof(*rr6));
    if (qr6) memset(qr6,0,6*sizeof(*qr6));
    if (dtr6) memset(dtr6,0,6*sizeof(*dtr6));
    if (solution_status) *solution_status=SOLQ_NONE;
    if (valid_satellite_count) *valid_satellite_count=0;
    if (constructed_observation_count) *constructed_observation_count=0;
    if (message&&message_capacity>0) message[0]='\0';
    if (!ctx||measurement_count<=0||measurement_count>4096||
        !gnss_id||!sv_id||!sig_id||!freq_id||!pr_mes_m||!pr_valid||
        !cp_mes_cycles||!do_mes_hz||!cno_dbhz||!cp_std_code||
        !tracking_status||!initial_rr3||!rr6||!qr6||!dtr6||
        !solution_status||!valid_satellite_count||
        !constructed_observation_count||!message||message_capacity<128||
        gps_week<0||!isfinite(gps_tow_seconds)||
        (navsys_mask!=SYS_GPS&&navsys_mask!=SYS_CMP&&
         navsys_mask!=(SYS_GPS|SYS_CMP))) return -1;
    for (i=0;i<3;i++) if (!isfinite(initial_rr3[i])) return -1;

    obs=(obsd_t *)calloc((size_t)measurement_count,sizeof(*obs));
    sat_index=(int *)malloc((MAXSAT+1)*sizeof(*sat_index));
    if (!obs||!sat_index) {
        free(obs); free(sat_index);
        return -5;
    }
    for (i=0;i<=MAXSAT;i++) sat_index[i]=-1;
    time=gpst2time(gps_week,gps_tow_seconds);
    for (i=0;i<measurement_count;i++) {
        if (pr_valid[i]!=0&&pr_valid[i]!=1) {status=-1; goto cleanup;}
        if (!isfinite(pr_mes_m[i])||!isfinite(cp_mes_cycles[i])||
            !isfinite(do_mes_hz[i])||cno_dbhz[i]<0||cno_dbhz[i]>255||
            cp_std_code[i]<0||cp_std_code[i]>15||tracking_status[i]<0||
            tracking_status[i]>255) {status=-1; goto cleanup;}
        slot=code=0;
        if (gnss_id[i]==0) {
            sys=SYS_GPS;
            if (sig_id[i]==0&&freq_id[i]==0) {slot=0; code=CODE_L1C;}
            else if (sig_id[i]==3&&freq_id[i]==0) {slot=1; code=CODE_L2L;}
            else if (sig_id[i]==4&&freq_id[i]==0) {slot=5; code=CODE_L2S;}
            else {status=-2; goto cleanup;}
        }
        else if (gnss_id[i]==3) {
            sys=SYS_CMP;
            if ((sig_id[i]==0||sig_id[i]==1)&&freq_id[i]==0) {
                slot=0; code=CODE_L2I;
            }
            else if ((sig_id[i]==2||sig_id[i]==3)&&freq_id[i]==0) {
                slot=1; code=CODE_L7I;
            }
            else {status=-2; goto cleanup;}
        }
        else {status=-2; goto cleanup;}
        if (!(navsys_mask&sys)||(sat=satno(sys,sv_id[i]))<=0) {
            status=-2; goto cleanup;
        }
        j=sat_index[sat];
        if (j<0) {
            if (nobs>=MAXOBS) {status=-4; goto cleanup;}
            j=nobs++;
            sat_index[sat]=j;
            obs[j].time=time;
            obs[j].sat=(uint8_t)sat;
            obs[j].rcv=1;
        }
        if (obs[j].code[slot]!=CODE_NONE) {status=-3; goto cleanup;}
        obs[j].code[slot]=(uint8_t)code;
        snr=(int)floor(cno_dbhz[i]/SNR_UNIT+0.5);
        obs[j].SNR[slot]=(uint16_t)(snr>65535?65535:snr);
        /* Strict raw-code path: carrier and Doppler are intentionally absent.
         * The corresponding arrays remain ABI inputs for future audited
         * extension but are never copied into obsd_t here. */
        obs[j].L[slot]=0.0;
        obs[j].D[slot]=0.0f;
        if (pr_valid[i]&&pr_mes_m[i]>0.0) obs[j].P[slot]=pr_mes_m[i];
    }
    /* BRDC single-frequency pntpos() consumes P[0] only (pntpos.c:prange).
     * Drop L2-only records so constructed_observation_count describes the
     * observations that can actually influence the solution. */
    for (i=j=0;i<nobs;i++) {
        if (obs[i].P[0]>0.0) {
            if (j!=i) obs[j]=obs[i];
            j++;
        }
    }
    nobs=j;
    /* RAWX row order is not a numerical input.  RTKLIB's merged epoch must be
     * deterministic and ordered by its canonical satellite number. */
    for (i=1;i<nobs;i++) {
        obsd_t value=obs[i];
        for (j=i;j>0&&obs[j-1].sat>value.sat;j--) obs[j]=obs[j-1];
        obs[j]=value;
    }
    *constructed_observation_count=nobs;
    if (nobs<=0) {
        snprintf(message,(size_t)message_capacity,"no explicitly prValid observations");
        status=0;
        goto cleanup;
    }
    memset(&sol,0,sizeof(sol));
    for (i=0;i<3;i++) sol.rr[i]=initial_rr3[i];
    opt.mode=PMODE_SINGLE;
    opt.navsys=navsys_mask;
    opt.sateph=EPHOPT_BRDC;
    opt.ionoopt=IONOOPT_BRDC;
    opt.tropopt=TROPOPT_SAAS;
    status=pntpos(obs,nobs,&ctx->nav,&opt,&sol,NULL,NULL,message);
    *solution_status=sol.stat;
    *valid_satellite_count=sol.ns;
    if (status) {
        for (i=0;i<6;i++) {
            rr6[i]=sol.rr[i];
            qr6[i]=sol.qr[i];
            dtr6[i]=sol.dtr[i];
        }
    }
cleanup:
    free(sat_index);
    free(obs);
    return status;
}
