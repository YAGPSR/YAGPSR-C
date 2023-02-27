#pragma once

class ClSVScan
{
public:
	ClSVScan();
	ClSVScan(const int &, const int &, const int &, const double &);
	~ClSVScan();

	void setDopplerRange(const int &, const int &, const int &, const double &);
	void setDopplerList(const vdouble_t &, const int &);
	void setParameters(const unsigned &, const unsigned &, const double &);

	void get_SV_code_fft(vcomplex_t *, const size_t &, const int &);
	void scanDoppler(t_SpaceVehicle *const, const vint_t &, const vcomplex_t &, const ClGPSParameters &);

	int reset();

private:
	ClFrequencyShifter DopplerShifter;

	vdouble_t          DopplerFreq;
	vdouble_t          NormalizedDoppler;
	
	vcomplex_t         PreviousCorrelatorState;

	// Parameterization
	void initialize_SV_codes();
	void setDefaultParameters();

	int      NPhases;
	int      OSR;
	double   DetectionThreshold;

	// Persistent storage for the FFT 
	// Note that each satellite has its own FFT (input/output/plan storage) to allow multi-threading
	void allocateMultithreadFFT();
	void deallocateMultithreadFFT();
	std::vector<ClFFT *>  pmultithreadtransform;

	// Persistent storage for the FFT result of the correlation filter
	std::vector<vcomplex_t > fft_SV_correlation;
	int                      fft_SV_Current_OSR;

protected:
};
