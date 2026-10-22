#ifndef LEGSA_RTKLIB_BRIDGE_H
#define LEGSA_RTKLIB_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

void *legsa_nav_load_rinex(const char *path);
void *legsa_nav_load_rinex_list(const char *semicolon_paths);
void *legsa_nav_load_rinex_paths(const char *const *paths, int count);
void legsa_nav_free(void *handle);
int legsa_nav_counts(void *handle, int *gps_gal_bds_qzs, int *glo, int *sbas);
int legsa_sat_id_to_no(const char *sat_id);
int legsa_satpos_broadcast(void *handle, int gps_week, double tow, int sat,
                           double *rs6, double *dts2, double *variance,
                           int *health);
int legsa_satpos_transmit_broadcast(void *handle, int receive_gps_week,
                                    double receive_tow, int sat,
                                    double pseudorange_m, double *rs6,
                                    double *dts2, double *variance,
                                    int *health);
int legsa_broadcast_ephemeris_audit(void *handle, int receive_gps_week,
                                    double receive_tow, int sat,
                                    double *signed_age_seconds,
                                    int *toe_gps_week, double *toe_tow,
                                    int *toc_gps_week, double *toc_tow,
                                    int *health, int *iode);
double legsa_sat_frequency_hz(void *handle, int sat, const char *rinex_code);
int legsa_lambda(int n, int candidates, const double *float_ambiguities,
                 const double *covariance, double *integer_candidates,
                 double *squared_norms);
int legsa_pntpos_rawx_epoch_v1(
    void *nav_handle, int gps_week, double gps_tow_seconds, int navsys_mask,
    int measurement_count, const int *gnss_id, const int *sv_id,
    const int *sig_id, const int *freq_id, const double *pr_mes_m,
    const int *pr_valid, const double *cp_mes_cycles, const double *do_mes_hz,
    const int *cno_dbhz, const int *cp_std_code, const int *tracking_status,
    const double *initial_rr3, double *rr6, double *qr6, double *dtr6,
    int *solution_status, int *valid_satellite_count,
    int *constructed_observation_count, char *message, int message_capacity);

#ifdef __cplusplus
}
#endif
#endif
