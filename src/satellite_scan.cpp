/*
* Scan over a range of Doppler frequencies to detect the presence of SVs
* and the appoximate Doppler frequency of that SV
* Further refinement of Doppler is performed using a tracking loop
*/

#include "headers.h"

#ifdef _MSC_VER
#include <ppl.h>
#endif


void ClSVScan::get_SV_code_fft(vcomplex_t *code, const size_t &SVIndex, const int &OSR)
{
	if (OSR == 0)
	{
		fprintf(stderr, "Error: Invalid OSR specified in SV code generation. Exiting safely.\n");
		fflush(stderr);
		exit(1);
	}

	// Regenerate only where the OSR has changed
	if (OSR != fft_SV_Current_OSR)
	{
		fft_SV_Current_OSR = OSR;
		fft_SV_correlation.resize(0);		
	}

	// Generate all the matched FFTs on the first call to this function, and store them for future use
	if (fft_SV_correlation.size() == 0)
	{
		ClFFT transform(1023 * OSR);

		fft_SV_correlation.resize(TOTAL_NUMBER_OF_SATELLITES + 1);

		for (int CurrentSVIndex = 1; CurrentSVIndex < TOTAL_NUMBER_OF_SATELLITES + 1; CurrentSVIndex++)
		{
			vint_t extractionInput;
			extractionInput.resize(0);
			extractionInput.resize(1023, 0);

			for (int n = 0; n < 1023; n++)
			{
				if (n == 0)
				{
					extractionInput[1023 - n - 1] = 1;
				}
				else
				{
					extractionInput[1023 - n - 1] = 1;
					extractionInput[1023 - n]     = 0;
				}

				for (int m = 0; m < OSR; m++)
				{
					transform.fft_in[n*OSR + m][0] = correlate_SV(extractionInput.data(), 0, CurrentSVIndex);
					transform.fft_in[n*OSR + m][1] = 0.0;
				}
			}

			transform.fft();

			fft_SV_correlation[CurrentSVIndex].reserve(1023 * OSR);
			for (int n = 0; n < 1023 * OSR; n++)
			{
				fft_SV_correlation[CurrentSVIndex].push_back(std::complex<double>(transform.fft_out[n][0], transform.fft_out[n][1]));
			}
		}
	}

	*code = fft_SV_correlation[SVIndex];
}


void ClSVScan::deallocateMultithreadFFT()
{
	if (pmultithreadtransform.size() != 0)
	{
		for (int n = 1; n < TOTAL_NUMBER_OF_SATELLITES + 1; n++)
		{
			delete pmultithreadtransform[n];
		}
	}

	pmultithreadtransform.resize(0);
}

void ClSVScan::allocateMultithreadFFT()
{
	// Do not attempt to allocate invalid FFT
	if (OSR == 0)
		return;

	// Only recreate if the size of the FFT has changed
	if (pmultithreadtransform.size() != 0)
	{
		if (OSR == pmultithreadtransform[1]->getN())
			return;
	}

	deallocateMultithreadFFT();

	pmultithreadtransform.resize(TOTAL_NUMBER_OF_SATELLITES + 1);
	for (int n = 1; n < TOTAL_NUMBER_OF_SATELLITES + 1; n++)
	{
		pmultithreadtransform[n] = new ClFFT(1023 * OSR);
	}
}

ClSVScan::ClSVScan()
{
	reset();
}


ClSVScan::ClSVScan(const int &DopplerLowHz, const int &DopplerHighHz, const int &DopplerStepHz, const double &Fs)
{
	reset();

	setDopplerRange(DopplerLowHz, DopplerHighHz, DopplerStepHz, Fs);
}

ClSVScan::~ClSVScan()
{
	deallocateMultithreadFFT();
}


void ClSVScan::initialize_SV_codes()
{
	vcomplex_t discard;
	get_SV_code_fft(&discard, 1, OSR);
}

int ClSVScan::reset()
{
	DopplerFreq.      resize(0);
	NormalizedDoppler.resize(0);

	DopplerShifter.reset_phase();

	PreviousCorrelatorState.resize(0);

	fft_SV_Current_OSR = 0;
	fft_SV_correlation.resize(0);

	setDefaultParameters();

	initialize_SV_codes();

	return(0);
}

void ClSVScan::setDefaultParameters()
{
	setParameters(4, 4, 200);
}

void ClSVScan::setParameters(const unsigned &x, const unsigned &y, const double &z)
{
	NPhases            = x;
	OSR                = y;
	DetectionThreshold = z;

	allocateMultithreadFFT();
}

void ClSVScan::setDopplerRange(const int &DopplerLowHz, const int &DopplerHighHz, const int &DopplerStepHz, const double &Fs)
{
	DopplerFreq.resize(0);
	DopplerFreq.reserve((int)max((DopplerHighHz - DopplerLowHz) / DopplerStepHz + 1, 0));

	NormalizedDoppler.resize(0);
	NormalizedDoppler.reserve((int)max((DopplerHighHz - DopplerLowHz) / DopplerStepHz + 1, 0));

	for (int f = DopplerLowHz; f < DopplerHighHz + 1; f += DopplerStepHz)
	{
		DopplerFreq.push_back(f);
		NormalizedDoppler.push_back((double)(f) / (double)Fs);
	}

	DopplerShifter.reset_phase();
}

void ClSVScan::setDopplerList(const vdouble_t &DopplerHz, const int &Fs)
{
	DopplerFreq.resize(0);
	DopplerFreq.reserve(DopplerHz.size());
	NormalizedDoppler.resize(0);
	NormalizedDoppler.reserve(DopplerHz.size());

	for (unsigned n = 0; n < DopplerHz.size(); n++)
	{
		DopplerFreq.push_back(DopplerHz[n]);
		NormalizedDoppler.push_back(DopplerHz[n] / (double)Fs);
	}

	DopplerShifter.reset_phase();
}


void ClSVScan::scanDoppler(t_SpaceVehicle *const SV, const vint_t &SVIndex, const vcomplex_t &x, const ClGPSParameters &YAGPSRParam)
{
	vcomplex_t          DopplerShiftedOutput;
	vcomplex_t          y;

	// Not thread-safe
	ClFFT                 transform(1023 * OSR);

	// Debug message //
	static double TimeElapsed_ms = 0;
	TimeElapsed_ms += (double)x.size() / (double)(1023 * OSR);
	//fprintf(stdout, "\nTotal data read / time elapsed: %8.2lf ms\n", TimeElapsed_ms); fflush(stdout);
	// End Debug message //

	// Process however many samples are passed in, however, the result will not be correct unless
	// the input has a size an integer multiple of the correlation window (i.e. 1023 samples at the code rate. 1023 * OSR at the input rate)
	// The correct behaviour of the storage of the correlator state is also dependent on the size being an exact multiple of the code rate
	if ((x.size() % (1023 * OSR)) != 0)
	{
		fprintf(stderr, "Input must be an exact multiple of 1023 x OSR. Exiting safely.\n"); fflush(stderr);
		exit(1);
	}

	if ((x.size() < (size_t)(2 * 1023 * OSR)))
	{
		fprintf(stderr, "Insufficient data to generate a single correlation window. This is an error as this function processes all input samples and thus should never be called without an appropriate number of input data. Exiting safely.\n"); fflush(stderr);
		exit(1);
	}

	// Prepend the previous state of the correlator taps to the input data
	y.reserve(x.size() + PreviousCorrelatorState.size());
	y.insert(y.end(), PreviousCorrelatorState.begin(), PreviousCorrelatorState.end());
	y.insert(y.end(), x.begin(), x.end());

	// Store the final internal state of the correlator so that samples are used on the next call to this function
	PreviousCorrelatorState.resize(0);
	PreviousCorrelatorState.reserve(1023 * OSR);

	for (int n = 0; n < (1023 * OSR); n++)
	{
		PreviousCorrelatorState.push_back(y[y.size() - (1023 * OSR) + n]);
	}

	// If no SVs are queued for detection, return and wait for more data/cooldown/loss of contact/different SV list/etc
	// This also discards the data corresponding to current correlation window calculation
	if (SVIndex.size() == 0)
	{
		return;
	}

	switch (YAGPSRParam.SVDetectionCorrelationMode)
	{
	case SV_MODE_MULTI_PHASE_CORRELATION:

		////
		// Scan for SVs (over a range of Doppler, code alignment subphase, and code phase values)
		for (unsigned iDoppler = 0; iDoppler < NormalizedDoppler.size(); iDoppler++)
		{
			DopplerShifter.set_shift_frequency(NormalizedDoppler[iDoppler]);
			DopplerShifter.shift(&DopplerShiftedOutput, y);

			// Alignment subphase (TsCodeRate / NPhase spacing between estimates)
			for (int iphase = 0; iphase < NPhases; iphase++)
			{
				int sphase;

#ifdef D_YAGPSR_SAT_FAST_SCAN
				vint_t symbol_sample_I, symbol_sample_Q;
#else
				vdouble_t symbol_sample_I, symbol_sample_Q;
#endif

				sphase = iphase * (OSR / NPhases);											// Note: integer division

				symbol_sample_I.resize(0); symbol_sample_I.reserve(DopplerShiftedOutput.size() / OSR);
				symbol_sample_Q.resize(0); symbol_sample_Q.reserve(DopplerShiftedOutput.size() / OSR);

				for (unsigned n = sphase; n < DopplerShiftedOutput.size(); n += OSR)
				{
#ifdef D_YAGPSR_SAT_FAST_SCAN
#if D_YAGPSR_SAT_FAST_SCAN_MODE == 0
					symbol_sample_I.push_back((int32_t)(128.0 * DopplerShiftedOutput[n].real()));
					symbol_sample_Q.push_back((int32_t)(128.0 * DopplerShiftedOutput[n].imag()));
#elif
					symbol_sample_I.push_back(sign(DopplerShiftedOutput[n].real()));
					symbol_sample_Q.push_back(sign(DopplerShiftedOutput[n].imag()));
#endif
#else
					symbol_sample_I.push_back(DopplerShiftedOutput[n].real());
					symbol_sample_Q.push_back(DopplerShiftedOutput[n].imag());
#endif
				}


				begin_yagpsr_parallel_for_each(SVIndex, size_t CurrentSVIndex)
				{
					vdouble_t           cn, corwin, PAR;
					double              cormean, corvar, maxval;
					int                 maxidx;


					// Ensure that this SV is not on cooldown due to non-detection over a number of samples (period of time)
					if (SV[CurrentSVIndex].SearchCooldown == 0)
					{
						//continue;			// Move to the next iteration of the loop without processing the current SV
					//}

					// Correlation output vector
						cn.resize(0);
						cn.reserve(symbol_sample_I.size() - 1023);

						// The final sample of symbol_sample_I is unused since it corresponds to an additional sample at code phase 0 compared to other code phases
						// Nc code windows, yields (Nc-1) correlation windows
						for (size_t n = 0; n < symbol_sample_I.size() - 1023; n++)
						{
							double cn_I, cn_Q;
							cn_I = correlate_SV(symbol_sample_I.data(), n, CurrentSVIndex);
							cn_Q = correlate_SV(symbol_sample_Q.data(), n, CurrentSVIndex);

							cn.push_back(fabs(cn_I + cn_Q));
						}

						// Correlation outputs averaged in a correlation window of 1023 (the number of possible code phases)
						corwin.resize(0);
						corwin.resize(1023, 0.0);
						int cnidx = 0;
						for (unsigned n = 0; n < cn.size(); n++)
						{
							corwin[cnidx++] += cn[n];
							if (cnidx == 1023)
							{
								cnidx = 0;
							}
						}

						// Calculate the metric to use to determine detection
						calculate_mean_variance_par(&cormean, &corvar, &PAR, corwin);
						max_val_idx(&maxval, &maxidx, PAR);

						if (maxval > DetectionThreshold) {
							if (maxval > SV[CurrentSVIndex].DetectionMetric)
							{
								SV[CurrentSVIndex].DetectionMetric = maxval;
								SV[CurrentSVIndex].NormalizedDoppler = NormalizedDoppler[iDoppler];
								SV[CurrentSVIndex].DopplerPhase = 0;
								SV[CurrentSVIndex].f_Detected = true;

								double CodePhase;
								CodePhase = maxidx * OSR + sphase;
								SV[CurrentSVIndex].Tracking.SampleTiming.i_CodePhase = (int)floor(CodePhase);
								SV[CurrentSVIndex].Tracking.SampleTiming.f_CodePhase = CodePhase - floor(CodePhase);
							}
						}
					}
				} // . iSV
				end_yagpsr_parallel_for_each;
			} // ..... iphase
		} // ......... iDoppler
		break;

	case SV_MODE_FFT_BASED_CORRELATION:

		/***********************************************************************/

		// Initialize the code FFT values
		// These are only calculated once, on the first call to this function, so the first call must not be put in a multi-threaded block
		// Very low overhead, since for all but the first call it just returns precalculated values
		// Ugly hack added at the same time as multi-threading implemented
		// Fix by moving the initialization somewhere else (be careful though, changes to OSR need trigger an update before the 
		// scanning block)
		{
			vcomplex_t discard;
			get_SV_code_fft(&discard, 1, OSR);
		}

		////
		// Scan for SVs (over a range of Doppler, and code phase values)
		for (unsigned iDoppler = 0; iDoppler < NormalizedDoppler.size(); iDoppler++)
		{
			// This could also be performed (to lower frequency resolution) by simple shift in the frequency domain after the fft
			// However, this is very low overhead compared to the total processing and provides better frequency resolution so we make the trade-off
			DopplerShifter.set_shift_frequency(NormalizedDoppler[iDoppler]);
			DopplerShifter.shift(&DopplerShiftedOutput, y);

			begin_yagpsr_parallel_for_each(SVIndex, size_t CurrentSVIndex)
			{
				vdouble_t           cn;
				vdouble_t			corwin;
				vdouble_t           PAR;
				double              cormean;
				double              corvar;
				double              maxval;
				int                 maxidx;

				vdouble_t  fwind;

				vcomplex_t fpn;
				int        count;
				int        zidx;
				vcomplex_t facc;

				ClFFT *tfm;

				tfm = pmultithreadtransform[CurrentSVIndex];

				zidx = 0;

				facc.resize(0);
				facc.resize(1023 * OSR, 0.0);
				fwind.resize(0);
				fwind.resize(1023 * OSR, 0.0);

				get_SV_code_fft(&fpn, CurrentSVIndex, OSR);

				count = 0;
				while (count < (int)(DopplerShiftedOutput.size() / (1023 * OSR)) - 1)
				{
					count++;

					for (int fidx = 0; fidx < 1023 * OSR; fidx++)
					{
						tfm->fft_in[fidx][0] = DopplerShiftedOutput[fidx + 1023 * OSR * count].real();
						tfm->fft_in[fidx][1] = DopplerShiftedOutput[fidx + 1023 * OSR * count].imag();
					}

					tfm->fft();

					for (int fidx = 0; fidx < 1023 * OSR; fidx++)
					{
						// Complex multiplication (fpn X fft_out)
						// split into real and imaginary parts for readability - re and im will most likely be optimized out by the compiler
						double re, im;

						re = std::real(fpn[(fidx + zidx) % (1023 * OSR)]) * tfm->fft_out[fidx][0] - std::imag(fpn[(fidx + zidx) % (1023 * OSR)]) * tfm->fft_out[fidx][1];
						im = std::real(fpn[(fidx + zidx) % (1023 * OSR)]) * tfm->fft_out[fidx][1] + std::imag(fpn[(fidx + zidx) % (1023 * OSR)]) * tfm->fft_out[fidx][0];

						facc[fidx] = std::complex<double>(re, im);
					}

					for (int fidx = 0; fidx < 1023 * OSR; fidx++)
					{
						tfm->ifft_in[fidx][0] = std::real(facc[(fidx + zidx) % (1023 * OSR)]);
						tfm->ifft_in[fidx][1] = std::imag(facc[(fidx + zidx) % (1023 * OSR)]);
					}

					tfm->ifft();

					// Normalize the IFFT output
					for (int fidx = 0; fidx < 1023 * OSR; fidx++)
					{
						tfm->ifft_out[fidx][0] = tfm->ifft_out[fidx][0] / (double)(1023 * OSR);
						tfm->ifft_out[fidx][1] = tfm->ifft_out[fidx][1] / (double)(1023 * OSR);
					}

					for (int fidx = 0; fidx < 1023 * OSR; fidx++)
					{
						fwind[fidx] += sqrt(tfm->ifft_out[fidx][0] * tfm->ifft_out[fidx][0] + tfm->ifft_out[fidx][1] * tfm->ifft_out[fidx][1]);
					}
				}

				corwin = fwind;

				// Calculate the metric to use to determine detection
				calculate_mean_variance_par(&cormean, &corvar, &PAR, corwin);
				max_val_idx(&maxval, &maxidx, PAR);

				if (maxval > DetectionThreshold)
				{
					if (maxval > SV[CurrentSVIndex].DetectionMetric)
					{
						double CodePhase;

						SV[CurrentSVIndex].DetectionMetric = maxval;
						SV[CurrentSVIndex].NormalizedDoppler = NormalizedDoppler[iDoppler];
						SV[CurrentSVIndex].DopplerPhase = 0;
						SV[CurrentSVIndex].f_Detected = true;

						// The FFT returns the code phase corresponding to the beginning of the chip
						// The code phase desired for decoding is the centre of the chip
						// Since the FFT code phase is within half an oversampled period of the ideal code phase, we add an offset of 0.375 of a chip at the oversampled rate
						// This should place the code phase closer to the centre of the chip in most cases while still allowing for the FFT to be out by OSR 						
						CodePhase = (double)maxidx + 0.375 * double(OSR);

						SV[CurrentSVIndex].Tracking.SampleTiming.i_CodePhase = (int)floor(CodePhase);
						SV[CurrentSVIndex].Tracking.SampleTiming.f_CodePhase = CodePhase - floor(CodePhase);
					}
				}
			}
			end_yagpsr_parallel_for_each;
		}
		break;

	default:
		fprintf(stderr, "Invalid SV Correlation Mode specified. Exiting safely.\n");
		fflush(stderr);
		exit(1);
		break;
	}


	// Update the number of samples (equivalently amount of time) spent searching for this SV 
	// For each SV, we proceed as follows:
	// 1. Wake up and search for SV over a maximum of MAX_SV_SEARCH_SAMPLES (see also MAX_SV_SEARCH_TIME_MS)
	// 2. Sleep for SV_SEARCH_COOLDOWN_SAMPLES (SV_SEARCH_COOLDOWN_TIME_MS)

	// Debug messages //
	vint_t SleepList;
	SleepList.resize(0);
	SleepList.reserve(TOTAL_NUMBER_OF_SATELLITES + 1);
	// End Debug messages //

	for (int iSV = 0; iSV < (int)SVIndex.size(); iSV++)
	{
		int	CurrentSVIndex;

		CurrentSVIndex = SVIndex[iSV];

		// Skip if already on cooldown
		if (SV[CurrentSVIndex].SearchCooldown != 0)
		{
			continue;			// Move to the next iteration of the loop without processing the current SV
		}

		// Add the number of samples at the 1.023 MHz code rate
		SV[CurrentSVIndex].IntegrationSamples += (int)x.size() / OSR - 1023 + 1;

		if (SV[CurrentSVIndex].IntegrationSamples > YAGPSRParam.MaxSVAcquisition_Samples)
		{
			// Put the SV to sleep for SV_SEARCH_COOLDOWN_SAMPLES (see also SV_SEARCH_COOLDOWN_MS)			
			SV[CurrentSVIndex].IntegrationSamples = 0;
			SV[CurrentSVIndex].SearchCooldown     = YAGPSRParam.SearchCooldown_Samples;//get_param(SV_SEARCH_COOLDOWN_SAMPLES);
		}
		
		// Disable cooldown if the SV was/is detected, since we will not want a cooldown period should the satellite be lost and/or need to be reaquired
		if (SV[CurrentSVIndex].f_Detected)
		{
			SV[CurrentSVIndex].IntegrationSamples = 0;
			SV[CurrentSVIndex].SearchCooldown     = 0;
		}

		// Debug messages //
		if (SV[CurrentSVIndex].SearchCooldown != 0)
		{
			SleepList.push_back(CurrentSVIndex);
		}
		// End Debug messages //
	}

	// Debug messages //
	//if (SleepList.size() > 0)
	//{
	//	fprintf(stdout, "\nStatus: Sleeping, SV:");
	//}
	//
	//for (int n = 0; n < (int)SleepList.size(); n++)
	//{
	//	fprintf(stdout, " %2d", SleepList[n]);
	//}
	//if (SleepList.size() > 0)
	//{
	//	fprintf(stdout, ".\n\n"); fflush(stdout);
	//}
	// END Debug messages //
}
