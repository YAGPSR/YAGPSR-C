/*
 * Resampler
 * Note that this works as a streaming resampler only, i.e. each input must be concatenated to the previous input (which has all its non-required data removed during the resample operation)
 * To use the resampler on multiple different streams requires a reset and is not recommended; instead it is recommended to instantiate multiple resamplers and used them as streaming devices
*/

#include "headers.h"

ClResampler::ClResampler()
{
	FsTarget = 1;
	FsData   = 1;
	Delta_mu = 1;

	K        = 3;
	FarrowFilter.initialize(K);

	current_rsidx = K - (K / 2);

	data.resize(0);
}

ClResampler::ClResampler(const double &Fd, const double &Ft)
{
	FsData   = Fd;
	FsTarget = Ft;
	Delta_mu = FsData / FsTarget;           // Derived parameter used for resampling

	K        = 3;
	FarrowFilter.initialize(K);

	current_rsidx = K - (K / 2);
	
	data.resize(0);
}

void ClResampler::initialize(const double &Fd, const double &Ft)
{
	FsData   = Fd;
	FsTarget = Ft;
	Delta_mu = FsData / FsTarget;           // Derived parameter used for resampling

	K        = 3;
	FarrowFilter.initialize(K);

	current_rsidx = K - (K / 2);

	data.resize(0);
}

void ClResampler::reset()
{
	current_rsidx = K - (K / 2);

	data.resize(0);
}

///*
//* The self-test only tests that multiple calls to this function with the data in small chunks produces the same output as a single call containing all the data 
//* Results are also written out to a file to allow independent verification in MATLAB
//* It is not recommended to run the self-test in production code since it affects the internal state of the resampler
//* The resampler is reset after the self-test is complete
//*/
//int ClResampler::self_test()
//{
//	// Perform a test to ensure that resampling of multiple blocks produces the same result as resampling a single block
//	vdouble_t ipdata, ipcopy, opdata, datasegment, opsegment, concatsegment;
//	unsigned Ndata, Nseg, didx;
//
//	Ndata = 3007;
//	Nseg  = 917;
//
//	ipdata.resize(0);
//	ipdata.reserve(Ndata);
//
//	datasegment.resize(0);
//	datasegment.reserve(Ndata);			// Should not need this much space, but it's a small amount of memory which is only taken up during the self test
//
//	concatsegment.resize(0);
//	concatsegment.reserve(Ndata);
//
//	// Generate random input data
//	srand((unsigned)time(NULL));   // Time-dependent random seed
//
//	for (unsigned n = 0; n < Ndata; n++)
//	{
//		// Random floating-point data between -128 and +128
//		//ipdata.push_back(((rand() / ((double)(RAND_MAX))) - 0.5) * 256.0);
//
//		ipdata.push_back(cos(2 * 3.141592653589793238462643383279502884197169 * n / 256.0));
//	}
//
//	// Make a copy of the input data and process this since the resampler is destructive
//	ipcopy = ipdata;
//
//	// Ensure the resampler is in its initial state before performing the first resampling operation 
//	this->reset();
//
//	// Resample the complete input test data
//	resample(&opdata, &ipcopy);
//
//	// Reset the resampler to the initial state before the previous resample
//	this->reset();
//
//	// Resample non-overlapping segments and concatenate
//	didx = 0;
//
//	while (didx < ipdata.size())
//	{
//		datasegment.resize(0);
//		datasegment.reserve(Nseg);
//		for (unsigned n = 0; n < Nseg; n++)
//		{
//			datasegment.push_back(ipdata[didx]);
//			didx++;
//			if (didx == ipdata.size())
//				break;
//		}
//		resample(&opsegment, &datasegment);
//
//		concatsegment.insert(concatsegment.end(), opsegment.begin(), opsegment.end());
//	}
//
//	
//	FILE *ofp;
//	ofp = fopen("SelfTest_ClResampler_ouput.txt", "w");
//	for (unsigned n = 0; n < opdata.size(); n++)
//		fprintf(ofp, "%lf\n", opdata[n]);
//	fclose(ofp);
//
//	ofp = fopen("SelfTest_ClResampler_concatenated_output.txt", "w");
//	for (unsigned n = 0; n < concatsegment.size(); n++)
//		fprintf(ofp, "%lf\n", concatsegment[n]);
//	fclose(ofp);
//	
//
//	// Compare the original and concatenated outputs
//	// Identical outputs - Test PASS (return SELF_TEST_PASS)
//	// Different outputs - Test FAIL (return error code SELF_TEST_FAIL)
//	if (opdata.size() != concatsegment.size())
//	{
//		this->reset();
//		return(SELF_TEST_FAIL);
//	}
//
//	for (unsigned n = 0; n < opdata.size(); n++)
//	{
//		// The values may not be completely identical due to floating-point effects (specifically with the realignment of the resampling index)
//		// Thus we check that the difference is below a given threshold
//		// The input is a sinusoid from -1 to +1, thus we can choose the threshold appropriately
//		if (fabs(opdata[n] - concatsegment[n]) > 1e-6)
//		{
//			this->reset();
//			return(SELF_TEST_FAIL);
//		}
//	}
//
//	// Reset to allow the resampler to be used after the self-test has been completed
//	this->reset();
//	return(SELF_TEST_PASS);
//}


//template int ClResampler::resample(vdouble_t *const y, vdouble_t *const x);
//template int ClResampler::resample(vcomplex_t *const y, vcomplex_t *const x);
//template<typename T>
//int ClResampler::resample(T *const y, T *const x)
int ClResampler::resample(vcomplex_t *const y, vcomplex_t *const x)
{
	vdouble_t resample_idx;

	// Ensure that the output is empty before starting resampling
	y->resize(0);

	// Prepend the input data to the residual data from the previous resampling operation, needed to perform resampling on contiguous inputs vectors
	data.insert(data.end(), x->begin(), x->end());

	// Generate the set of target indices for samples to have after resampling
	resample_idx.reserve((int)ceil(data.size() / Delta_mu));

	while (current_rsidx < (data.size() - K + (K / 2)))			// Note: K is an integer, thus K/2 is floor(K/2.0)
	{
		resample_idx.push_back(current_rsidx);
		current_rsidx += Delta_mu;
	}

	// Perform the interpolation and write the result into y
	FarrowFilter.interpolate(y, data, resample_idx);

	// Realign the resample index and store the input data required for the next call to the resampler
	if (data.size() > (size_t)(K - (K / 2))) {
		
		int Nx        = 0;										// The number of data samples at the end of x which need to be retained for subsequent calls
		current_rsidx = current_rsidx - data.size();

		while (current_rsidx < K - (K / 2)) {
			current_rsidx++;
			Nx++;
		}

		for (int n = 0; n < Nx; n++) 
		{
			data[n] = data[data.size() - Nx + n];
		}

		data.resize(Nx);
	}

	return(0);
}
