#include "headers.h"

ClFilter::ClFilter()
{
	c.resize(0);
	c.push_back(1.0);
	reset_state();
}

ClFilter::ClFilter(const vdouble_t &coefficients)
{
	initialize(coefficients);
}

void ClFilter::initialize(const vdouble_t &coefficients)
{
	set_coefficients_and_reset(coefficients);
}

void ClFilter::set_coefficients_and_reset(const vdouble_t &coefficients)
{
	c = coefficients;
	reset_state();
}

void ClFilter::reset_state()
{
	state.resize(0);
	state.resize(c.size() - 1, 0.0);

	state_j.resize(0);
	state_j.resize(c.size() - 1, 0.0);

	cmplx_state.resize(0);
	cmplx_state.resize(c.size() - 1, std::complex <double>(0.0, 0.0));
}

/*
* The self-test only tests that multiple calls to this function with the data in small chunks produces the same output as a single call containing all the data
* Results are also written out to a file to allow independent verification in MATLAB
*/
int ClFilter::self_test()
{
	// Generate a short data sequence with known output
	vdouble_t coeff, ipdata, opdata, datasegment, opsegment, concatsegment;
	unsigned Nc, Ndata, Nseg, didx;
	
	Nc    = 11;
	Ndata = 5000;
	Nseg  = 237;

	// Random coefficients
	coeff.resize(0);
	coeff.reserve(Nc);

	for (unsigned n = 0; n < Nc; n++)
	{
		coeff.push_back((rand() / ((double)(RAND_MAX))) - 0.5);
	}

	this->set_coefficients_and_reset(coeff);

	// Random input data
	ipdata.resize(0);
	ipdata.reserve(Ndata);

	for (unsigned n = 0; n < Ndata; n++)
	{
		ipdata.push_back((rand() / ((double)(RAND_MAX))) - 0.5);
	}

	// Single-step filtering
	filter(&opdata, ipdata);

	// Reset the filter state before the second stage
	this->reset_state();

	// Non-overlapping segments and concatenate
	didx = 0;

	while (didx < ipdata.size())
	{
		datasegment.resize(0);
		datasegment.reserve(Nseg);
		for (unsigned n = 0; n < Nseg; n++)
		{
			datasegment.push_back(ipdata[didx]);
			didx++;
			if (didx == ipdata.size())
				break;
		}
		filter(&opsegment, datasegment);
		concatsegment.insert(concatsegment.end(), opsegment.begin(), opsegment.end());
	}

	FILE *ofp;
	ofp = fopen("SelfTest_ClFilter_taps.txt", "w");
	for (unsigned n = 0; n < coeff.size(); n++)
		fprintf(ofp, "%lf\n", coeff[n]);
	fclose(ofp);

	ofp = fopen("SelfTest_ClFilter_input.txt", "w");
	for (unsigned n = 0; n < ipdata.size(); n++)
		fprintf(ofp, "%lf\n", ipdata[n]);
	fclose(ofp);

	ofp = fopen("SelfTest_ClFilter_output.txt", "w");
	for (unsigned n = 0; n < opdata.size(); n++)
		fprintf(ofp, "%lf\n", opdata[n]);
	fclose(ofp);

	ofp = fopen("SelfTest_ClFilter_concatenated_output.txt", "w");
	for (unsigned n = 0; n < concatsegment.size(); n++)
		fprintf(ofp, "%lf\n", concatsegment[n]);
	fclose(ofp);


	// Compare the original and concatenated outputs
	// Identical outputs - Test PASS (return SELF_TEST_PASS)
	// Different outputs - Test FAIL (return error code SELF_TEST_FAIL)
	if (opdata.size() != concatsegment.size())
	{
		this->reset_state();
		return(SELF_TEST_FAIL);
	}

	for (unsigned n = 0; n < opdata.size(); n++)
	{
		// The values may not be completely identical due to floating-point effects (specifically with the realignment of the resampling index)
		// Thus we check that the difference is below a given threshold
		if (fabs(opdata[n] - concatsegment[n]) > 1e-6)
		{
			this->reset_state();
			return(SELF_TEST_FAIL);
		}
	}

	this->reset_state();
	return(SELF_TEST_PASS);
}

/*
*	Real input, real output, real coefficients
*/

void ClFilter::filter(vdouble_t *const y, const vdouble_t &x)
{
	vdouble_t z;

	y->resize(0);
	y->reserve(x.size());

	z.resize(0);
	z.insert(z.end(), state.begin(), state.end());
	z.insert(z.end(),     x.begin(),     x.end());

	double acc;
	for (unsigned n = 0; n < x.size(); n++)
	{
		acc = 0.0;
		for (unsigned cdx = 0; cdx < c.size(); cdx++)
		{
			acc += c[cdx] * z[n + c.size() - cdx - 1];
		}
		y->push_back(acc);
	}

	state.resize(c.size() - 1);

	for (unsigned cdx = 0; cdx < c.size() - 1; cdx++)
	{
		state[state.size() - 1 - cdx] = z[z.size() - 1 - cdx];
	}
}

/*
 *	Complex input, complex output, real coefficients
*/
//void ClFilter::filter(vcomplex_t *const y, const vcomplex_t &x)
//{
//	vcomplex_t z;
//
//	y->resize(0);
//	y->reserve(x.size());
//
//	z.resize(0);
//	z.insert(z.end(), cmplx_state.begin(), cmplx_state.end());
//	z.insert(z.end(), x.begin(), x.end());
//
//	std::complex <double> acc;
//	for (unsigned n = 0; n < x.size(); n++)
//	{
//		acc = std::complex <double>(0.0, 0.0);
//		for (unsigned cdx = 0; cdx < c.size(); cdx++)
//		{
//			acc += c[cdx] * z[n + c.size() - cdx - 1];
//		}
//		y->push_back(acc);
//	}
//
//	cmplx_state.resize(c.size() - 1);
//
//	for (unsigned cdx = 0; cdx < c.size() - 1; cdx++)
//	{
//		cmplx_state[state.size() - 1 - cdx] = z[z.size() - 1 - cdx];
//	}
//}

void ClFilter::filter(vcomplex_t *const y, const vcomplex_t &x)
{
	vcomplex_t z;
	vcomplex_t cstate;

	y->resize(0);
	y->reserve(x.size());

	cstate.resize(0);
	cstate.reserve(state.size());
	for (unsigned n = 0; n < state.size(); n++)
	{
		cstate.push_back(std::complex<double>(state[n], state_j[n]));
	}

	z.resize(0);
	z.insert(z.end(), cstate.begin(), cstate.end());
	z.insert(z.end(), x.begin(), x.end());

	std::complex <double> acc;
	for (unsigned n = 0; n < x.size(); n++)
	{
		acc = std::complex <double>(0.0, 0.0);
		for (unsigned cdx = 0; cdx < c.size(); cdx++)
		{
			acc += c[cdx] * z[n + c.size() - cdx - 1];
		}
		y->push_back(acc);
	}

	state.resize(c.size() - 1);

	for (unsigned cdx = 0; cdx < c.size() - 1; cdx++)
	{
		state  [state.size()   - 1 - cdx] = z[z.size() - 1 - cdx].real();
		state_j[state_j.size() - 1 - cdx] = z[z.size() - 1 - cdx].imag();
	}
}