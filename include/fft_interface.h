#pragma once

class ClFFT {
public:
	fftw_complex *fft_in;
	fftw_complex *fft_out;
	fftw_complex *ifft_in;
	fftw_complex *ifft_out;

	ClFFT(const int &);
	~ClFFT();

	void fft();
	void ifft();

	int getN();

protected:

private:
	int Ntfm;

	fftw_plan fftw_plan_fft;
	fftw_plan fftw_plan_ifft;
};