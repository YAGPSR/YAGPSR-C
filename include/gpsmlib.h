#pragma once

extern int calculate_mean_variance_par(double *const, double *const, vdouble_t *const, const vdouble_t &);
extern int max_val_idx(double *const, int *const, const vdouble_t &);

extern double max(const double &, const double &);
extern int    sign(const double &);
extern void   binary_invert(vint_t *const, const vint_t &);