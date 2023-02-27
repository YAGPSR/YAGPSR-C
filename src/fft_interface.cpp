//
// Wrapper for accessing the FFTW routines without initialization, memory allocation, destruction issues
//

#include "headers.h"

ClFFT::ClFFT(const int &N)
{
	Ntfm     = N;

	fft_in   = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * N);
	fft_out  = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * N);

	ifft_in  = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * N);
	ifft_out = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * N);

	if (fft_in == NULL || fft_out == NULL || ifft_in == NULL || ifft_out == NULL)
	{
		fprintf(stderr, "Error allocating memory for ClFFT object. Exiting safely.\n");
		fflush(stderr);
		exit(1);
	}

	fftw_plan_fft  = fftw_plan_dft_1d(N,  fft_in,  fft_out,  FFTW_FORWARD, FFTW_MEASURE);
	fftw_plan_ifft = fftw_plan_dft_1d(N, ifft_in, ifft_out, FFTW_BACKWARD, FFTW_MEASURE);

	if (fftw_plan_fft == NULL || fftw_plan_ifft == NULL)
	{
		fprintf(stderr, "Error creating ClFFT plan. Exiting safely.\n");
		fflush(stderr);
		exit(1);
	}
}

ClFFT::~ClFFT()
{
	fftw_destroy_plan(fftw_plan_fft);
	fftw_destroy_plan(fftw_plan_ifft);

	fftw_free(fft_in);
	fftw_free(fft_out);
	fftw_free(ifft_in);
	fftw_free(ifft_out);
}

void ClFFT::fft()
{
	fftw_execute(fftw_plan_fft);
}

void ClFFT::ifft()
{
	fftw_execute(fftw_plan_ifft);
}

int ClFFT::getN()
{
	return(Ntfm);
}