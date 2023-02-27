#ifndef TYPEDEFSH
#define TYPEDEFSH

#include <vector>
#include <complex>

typedef std::vector<int32_t> vint_t;
typedef std::vector<double>  vdouble_t;

typedef std::vector<std::complex<double> >  vcomplex_t;

// Input data types
typedef enum { DT_8_BIT_REAL, DT_8_BIT_COMPLEX, DT_16_BIT_REAL, DT_16_BIT_COMPLEX } tdStreamDataFormat;

typedef enum { SV_MODE_MULTI_PHASE_CORRELATION, SV_MODE_FFT_BASED_CORRELATION, SV_MODE_FULL_CORRELATION } tdSVCorrelationMode;

#endif
