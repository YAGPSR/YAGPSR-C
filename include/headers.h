#ifndef HEADERSH
#define HEADERSH

#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#define NOMINMAX
#include <windows.h>
#else
#include <signal.h>
#endif

#include <ctime>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <fstream>
#include <cmath>
#include <limits>
#include <vector>
#include <complex>

#include "fftw3.h"
#include "fft_interface.h"

#include "defns.h"
#include "typedefs.h"
#include "param.h"
#include "yagpsr_parallel.h"

#include "gpsmlib.h"
#include "misc.h"
#include "matrix_operations.h"

#include "SV.h"
#include "parse.h"
#include "farrow_interpolation.h"
#include "resample.h"
#include "filter.h"
#include "predefined_filters.h"
#include "frequency_shifter.h"
#include "pn_correlation.h"
#include "satellite_scan.h"
#include "svtrack.h"
#include "data_decode.h"
#include "position_solver.h"

#endif
