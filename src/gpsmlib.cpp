#include "headers.h"

#include <numeric>

int calculate_mean_variance_par(double *const meanval, double *const varval, vdouble_t *const PAR, const vdouble_t &x)
{
	vdouble_t y;

	(*meanval) = std::accumulate(x.begin(), x.end(), 0.0) / x.size();

	y.resize(0);
	y.reserve(x.size());
	for (unsigned n = 0; n < x.size(); n++)
	{
		y.push_back((x[n] - (*meanval)) * (x[n] - (*meanval)));
	}

	(*varval) = std::accumulate(y.begin(), y.end(), 0.0) / y.size();

	PAR->resize(0);
	PAR->reserve(y.size());
	for (unsigned n = 0; n < y.size(); n++)
	{
		PAR->push_back(y[n] / (*varval));
	}

	return(0);
}


int max_val_idx(double *const maxval, int *const maxidx, const vdouble_t &PAR)
{
	double running_max;
	int    running_idx;

	running_idx = 0;
	running_max = PAR[0];

	for (unsigned n = 1; n < PAR.size(); n++)
	{
		if (PAR[n] > running_max)
		{
			running_max = PAR[n];
			running_idx = n;
		}
	}

	(*maxval) = running_max;
	(*maxidx) = running_idx;

	return(0);
}

double max(const double &x, const double &y)
{
	return(x > y ? x : y);
}

int sign(const double &x)
{
	return(x > 0 ? +1 : -1);
}


void binary_invert(vint_t *const y, const vint_t &x)
{
	y->resize(0);
	y->reserve(x.size());

	for (size_t n = 0; n < x.size(); n++)
	{
		if (x[n] == 0)
		{
			y->push_back(1);
		}
		else
		{
			y->push_back(0);
		}
	}
}