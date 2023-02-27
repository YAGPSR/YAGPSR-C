#include "headers.h"


ClSVTrack::ClSVTrack()
{

}


/*
* It may seem there's too much work in this function and that the multiple steps are overly involved, but this structure allows for an easier multi-threaded implementation
*/
void ClSVTrack::track(t_SpaceVehicle *const SVArray, const vcomplex_t &x, const int &OSR, const size_t &TrackingBlockTrimSize)
{
	// Note that this must be the same value for each call to this function since it is used to keep alignment
	size_t TrackingBlockSize = x.size();

	// Process however many samples are passed in, however, the result will not be correct unless
	// the input has a size an integer multiple of the correlation window (i.e. 1023 samples at the code rate. 1023 * OSR at the input rate)
	if ((TrackingBlockSize % (1023 * OSR)) != 0)
	{
		fprintf(stderr, "\nError: SV Tracking - Input must be an exact multiple of 1023 x OSR. Exiting safely.\n"); fflush(stderr);
		exit(1);
	}

	// Create a list of the satellites which may be tracked
	vint_t iDetected;
	iDetected.reserve(TOTAL_NUMBER_OF_SATELLITES);

	// Caution: 1 -> TOTAL_NUMBER_OF_SATELLITES, do not index from zero
	for (int CurrentSVIndex = 1; CurrentSVIndex < TOTAL_NUMBER_OF_SATELLITES + 1; CurrentSVIndex++)
	{
		if (SVArray[CurrentSVIndex].f_Detected == true)
		{
			iDetected.push_back(CurrentSVIndex);
		}
	}

	// Stop immediately if no satellites have been detected
	if (iDetected.size() == 0)
	{
		return;
	}


	////
	// Doppler shift each of the satellites, and store in a processing buffer (ZeroIFBuffer)

	begin_yagpsr_parallel_for_each(iDetected, size_t idx)
	{
		ClFrequencyShifter DopplerShifter;
		vcomplex_t         DopplerShiftedOutput;

		// Shift by the estimate of the Doppler frequency offset (as detected during acquisition); save the final phase for phase continuity on the subsequent calls to this function
		DopplerShifter.set_shift_frequency(SVArray[idx].NormalizedDoppler, SVArray[idx].DopplerPhase);
		DopplerShifter.shift(&DopplerShiftedOutput, x);
		SVArray[idx].DopplerPhase = DopplerShifter.get_phase();

		// Append the new ZIF samples corresponding to this SV to the remaining samples from the previous call to this function
		// or to a block of zeros so that SVs which are detected at different times are still properly aligned relative to each other
		if (SVArray[idx].Tracking.SampleTiming.ZeroIFBuffer.size() == 0)
		{
			SVArray[idx].Tracking.SampleTiming.ZeroIFBuffer.resize(TrackingBlockSize, std::complex<double>(0, 0));
			SVArray[idx].Tracking.SampleTiming.i_CodePhase += (int)TrackingBlockSize;
		}

		append_vector_to_vector(SVArray[idx].Tracking.SampleTiming.ZeroIFBuffer, DopplerShiftedOutput);

	}
	end_yagpsr_parallel_for_each;



	// Tracking each SV
	begin_yagpsr_parallel_for_each(iDetected, size_t idx)
	{
		trackSV(&(SVArray[idx]), OSR);			// Only process SVs that have been correctly detected
	}
	end_yagpsr_parallel_for_each;


	for (int n = 0; n < (int)iDetected.size(); n++)
	{		
		int idx = iDetected[n];

		// Trim the data, note that discontinuties caused by different numbers of bits being processed must be taken care of later
		if (SVArray[idx].Tracking.SampleTiming.ZeroIFBuffer.size() >= 2 * TrackingBlockTrimSize)
		{
			erase_from_beginning(SVArray[idx].Tracking.SampleTiming.ZeroIFBuffer, TrackingBlockTrimSize);
			SVArray[idx].Tracking.SampleTiming.i_CodePhase -= (int)TrackingBlockTrimSize;
		}
	}
}


/*
 *	Track a single SV
*/
void ClSVTrack::trackSV(t_SpaceVehicle *const SV, const int &OSR)
{
	ClFarrowFilter FarrowFilter;
	ClFilter       CostasLoopHalfBandFilter(get_half_band_coeffs());

	constexpr int SamplesPerBit  = 20;		// The number of codes which correspond to a single bit, NOT the number of samples of the input signal
	unsigned  int SamplesPerChip = OSR;		// The number of samples for each single chip, each code consists of 1023 chips and each bit of 20 codes

	double     K_sp, K_si, K_p, K_i, K_t, KFe, alpha_Fe;
	double     dphi, idphi, pdphi, tdphi;
	double     PED, prevPED, Fe, ZFe, ZZFe, Fe_lpf, prevFe_lpf, PED_lpf, prevPED_lpf;			// State variables

	int        sign_count;

	vcomplex_t bpsk_iq_timing_buffer, i_buffer;
	vdouble_t  bit_code_phase_buffer, p_buffer;

	// Work on a local copy to make the code more readable and the indexing easier to follow
	int    i_CodePhase;
	double f_CodePhase;

	i_CodePhase = SV->Tracking.SampleTiming.i_CodePhase;
	f_CodePhase = SV->Tracking.SampleTiming.f_CodePhase;

	// Detect insufficient previous data (i.e. first "early" sample in sample timing is at a negative index) 
	// and move forwards by a single code (at the OSR rate) where necessary
	// This should never happen, since there should always be sufficient data in the buffer
	if ((i_CodePhase + f_CodePhase) < (OSR / 2 + floor(FarrowFilter.get_K() / 2) + 1))
	{
		f_CodePhase += SV->Tracking.SampleTiming.ClockPeriod;
		fprintf(stderr, "Insufficient previous data for tracking. This is an error. Exiting safely.\n"); fflush(stderr);
		exit(1);
	}

	// Sample timing / code phase parameters
	K_sp     = SV->Tracking.SampleTiming.K_sp * OSR;
	K_si     = SV->Tracking.SampleTiming.K_si * OSR;
	
	// Frequency / Phase tracking parameters
	K_p      = SV->Tracking.PhaseFrequencyRecovery.K_p;				// Phase tracking - initially zero, see definition
	K_i      = SV->Tracking.PhaseFrequencyRecovery.K_i;				// Frequency tracking - initially zero, see definition
	K_t      = SV->Tracking.PhaseFrequencyRecovery.K_t;				// Unused
	KFe      = SV->Tracking.PhaseFrequencyRecovery.KFe;				// Initial frequency acquisition
	alpha_Fe = SV->Tracking.PhaseFrequencyRecovery.alpha_Fe;
	
	dphi     = SV->Tracking.PhaseFrequencyRecovery.dphi;
	idphi    = SV->Tracking.PhaseFrequencyRecovery.idphi;
	
	// Restore state variables from previous calls to this function
	PED      = SV->Tracking.PhaseFrequencyRecovery.PED;
	Fe       = SV->Tracking.PhaseFrequencyRecovery.Fe;
	ZFe      = SV->Tracking.PhaseFrequencyRecovery.ZFe;
	Fe_lpf   = SV->Tracking.PhaseFrequencyRecovery.Fe_lpf;
	PED_lpf  = SV->Tracking.PhaseFrequencyRecovery.PED_lpf;
	
	sign_count            = SV->Tracking.BPSKTiming.sign_count;
	bpsk_iq_timing_buffer = SV->Tracking.BPSKTiming.bpsk_iq_timing_buffer;
	bit_code_phase_buffer = SV->Tracking.BPSKTiming.bit_code_phase_buffer;
	
	// Memory reserved for the incoming bit stream
	SV->Tracking.Data.BinarySoft.      reserve(SV->Tracking.Data.BinarySoft.size()       + SV->Tracking.SampleTiming.ZeroIFBuffer.size() / (1023 * OSR * SamplesPerBit) + 1);
	SV->Tracking.Data.BinaryHard.      reserve(SV->Tracking.Data.BinaryHard.size()       + SV->Tracking.SampleTiming.ZeroIFBuffer.size() / (1023 * OSR * SamplesPerBit) + 1);
	SV->Tracking.Data.BinaryQuadrature.reserve(SV->Tracking.Data.BinaryQuadrature.size() + SV->Tracking.SampleTiming.ZeroIFBuffer.size() / (1023 * OSR * SamplesPerBit) + 1);
	SV->Tracking.Data.BinaryCodePhase. reserve(SV->Tracking.Data.BinaryHard.size()       + SV->Tracking.SampleTiming.ZeroIFBuffer.size() / (1023 * OSR * SamplesPerBit) + 1);
	SV->Tracking.Data.BinaryNew.       reserve(SV->Tracking.Data.BinaryNew.size()        + SV->Tracking.SampleTiming.ZeroIFBuffer.size() / (1023 * OSR * SamplesPerBit) + 1);


	////////////////////////////////////////////////////////////////////
	// Break the loop once it is detected that insufficient data is remaining to
	// continue processing; this function is called again once more data has been gathered
	//
	while (1)
	{
		std::complex<double> early_rx, target_rx, late_rx;
		vdouble_t            early_late_phase;
		vcomplex_t           early_late_code_align;
		vcomplex_t           early_late_code_align_nos;
		vcomplex_t           corrsamp;

		double               sample_time_metric;
		double               Tosr, Tcode;												// Sample period at the oversampled rate and the code rate
		double               iq_code_phase;												// Used to store the code phase of a single sample (the most recent), stored where the sample is found to form part of a valid bit
		size_t               EarlyLateOffset = SamplesPerChip / 4; 						// The number of samples (at the oversampled rate) before and after to use for the early-late metric generation

		Tosr  = (SV->Tracking.SampleTiming.ClockPeriod) / (double)(1023.0 * OSR);		// Should be approximately 1.0, adjusted to match the satellite based on the current tracking
		Tcode = (SV->Tracking.SampleTiming.ClockPeriod) / (double)(1023.0);				// Should be approximately the oversampling rate (equivalently the number of SamplesPerChip), adjusted to match the satellite based on the current tracking

		// Detect the underrun error condition (corresponding to insufficient data), this is an error as when operating correctly the code always maintains a sufficient number of samples from the previous call to this function
		if (floor((i_CodePhase + f_CodePhase) - EarlyLateOffset * Tosr) - (1 << (FarrowFilter.get_K())) - 1 < 1)
		{
			fprintf(stderr, "Error: Data underrun for interpolation. Exiting safely."); fflush(stderr);
			exit(1);
		}

		////////////////////////////////////////////////////////////////////
		// Sample Timing - Early-late sampling used to track sampling phase
		// Include the option to update at a lower rate in order to make 
		// it potentially feasible in real-time
		//
		early_rx  = std::complex<double>(0, 0);
		target_rx = std::complex<double>(0, 0);
		late_rx   = std::complex<double>(0, 0);
	
		corrsamp.resize(SamplesPerChip + 2 * EarlyLateOffset + 1);

		early_late_phase.resize(0);
		early_late_phase.reserve((1023 + SamplesPerChip + 2 * EarlyLateOffset) * OSR);

		early_late_code_align_nos.resize(1023, 0);

		for (size_t n = 0; n < 1023 + SamplesPerChip + 2 * EarlyLateOffset; n++)
		{
			for (size_t m = 0; m < (size_t)OSR; m++)
			{
				early_late_phase.push_back((i_CodePhase + f_CodePhase) - EarlyLateOffset * Tosr + n * Tcode + m * Tosr);
			}
		}

		// Detect out-of-bounds (corresponding to insufficient data) and use this to break the while loop
		if (early_late_phase.back() + ((1 << (FarrowFilter.get_K())) + 1) > SV->Tracking.SampleTiming.ZeroIFBuffer.size())
		{
			break;
		}

		// Perform the interpolation and write the result into early_late_code_align
		if (1)
		{
			FarrowFilter.interpolate(&early_late_code_align, SV->Tracking.SampleTiming.ZeroIFBuffer, early_late_phase);
		}
		else
		{
			nearest_neighbour_interpolate(&early_late_code_align, SV->Tracking.SampleTiming.ZeroIFBuffer, early_late_phase);
		}

		// Despread each phase
		for (size_t offset = 0; offset < (SamplesPerChip + 2 * EarlyLateOffset + 1); offset++)
		{
			for (size_t n = 0; n < 1023; n++)
			{
				early_late_code_align_nos[n] = early_late_code_align[n * OSR + offset];
			}

			corrsamp[offset] = correlate_SV(early_late_code_align_nos.data(), 0, SV->ID);
		}

		for (size_t offset = 0; offset < (SamplesPerChip + 2 * EarlyLateOffset + 1); offset++)
		{
			if (offset < SamplesPerChip) 
			{
				early_rx += corrsamp[offset];
			}
			if (offset >= EarlyLateOffset && offset < EarlyLateOffset + SamplesPerChip)
			{
				target_rx += corrsamp[offset];
			}
			if (offset > EarlyLateOffset + EarlyLateOffset)
			{
				late_rx += corrsamp[offset];
			}
		}
		

		// Store the code phase used to generate this sample (i.e. before the code phase is updated by the loop)
		iq_code_phase = (i_CodePhase + f_CodePhase);

		////////////////////////////////////////////////////////////////////
		// Sampling clock loop / tracking (sampling phase and frequency)
		//

		// Generate a metric for the sampling time offset (early-late)
		// For perfect alignment, the early and late values will be the same
		// For sampling TOO EARLY, this metric will be NEGATIVE
		// For sampling TOO LATE,  this metric will be POSITIVE		
		
		// Track real and imaginary until the system has locked, thereafter track only the (phase-adjusted) imaginary component
		if (SV->Tracking.BPSKTiming.f_bpsk_sampling_tracking == false)
		{
			sample_time_metric = (std::abs(early_rx) * std::abs(early_rx) - std::abs(late_rx) * std::abs(late_rx)) / (max(std::abs(early_rx) * std::abs(early_rx), std::abs(late_rx) * std::abs(late_rx)));
		}
		else
		{
			double sampleTimeMetricNormaliztionFactor;
			early_rx *= std::complex<double>(cos(dphi), -1 * sin(dphi));
			late_rx  *= std::complex<double>(cos(dphi), -1 * sin(dphi));

			sampleTimeMetricNormaliztionFactor = (max(std::imag(early_rx) * std::imag(early_rx), std::imag(late_rx) * std::imag(late_rx)));
			if (sampleTimeMetricNormaliztionFactor != 0)
			{
				sample_time_metric = (std::imag(early_rx) * std::imag(early_rx) - std::imag(late_rx) * std::imag(late_rx)) / sampleTimeMetricNormaliztionFactor;
			}
			else
			{
				fprintf(stderr, "Error: Sample Time Metric Divide-By-Zero. Exiting safely.\n");
				fflush(stderr);
				exit(1);
			}
		}

		// Phase update (proportional)
		f_CodePhase += -1 * K_sp * sample_time_metric;

		// Frequency update (integral) components
		constexpr double alpha_stm = 0.9;
		SV->Tracking.SampleTiming.RunningSampleTimeMetric = alpha_stm * SV->Tracking.SampleTiming.RunningSampleTimeMetric + (1.0 - alpha_stm) * sample_time_metric;
		SV->Tracking.SampleTiming.ClockPeriod             = SV->Tracking.SampleTiming.ClockPeriod - 0.4 * K_si * SV->Tracking.SampleTiming.RunningSampleTimeMetric;
		

		f_CodePhase += SV->Tracking.SampleTiming.ClockPeriod;

		// Adjust the i_CodePhase with respect to the f_CodePhase
		i_CodePhase += (int)floor(f_CodePhase);
		f_CodePhase -=      floor(f_CodePhase);

		SV->Tracking.SampleTiming.i_CodePhase = i_CodePhase;
		SV->Tracking.SampleTiming.f_CodePhase = f_CodePhase;

		// Debug
		SV->Debug.push_back((SV->Tracking.SampleTiming.ClockPeriod));
		SV->Debug.push_back(sample_time_metric);
		SV->Debug.push_back(idphi);
		//SV->Debug.push_back((i_CodePhase + f_CodePhase));
		// End Debug

		////////////////////////////////////////////////////////////////////
		// Frequency lock / tracking (Doppler tracking)
		//

		if (SV->Tracking.SampleTiming.CodeCount > SamplesPerBit)
		{
			std::complex<double> vco_shift, iq;
			double LoopPED;

			// VCO - Complex domain phase / frequency shift
			// (Residual Doppler, Doppler tracking, and Carrier phase)
			vco_shift = std::complex<double>(cos(dphi), -1 * sin(dphi));								// exp(-1j * dphi);

			// In-phase and quadrature components form the Phase Error Detection (PED) metric
			iq = target_rx * vco_shift;

			// PLL Implementation
			prevPED = PED;

			if (real(iq) == 0)
			{
				PED = sign(imag(iq)) * (PI / 2.0);
			}
			else
			{
				PED = atan2(real(iq), imag(iq));
			}

			ZZFe = ZFe;
			ZFe = Fe;

			// Frequency Metric
			Fe = PED - prevPED;
			while (Fe > PI)
			{
				Fe = Fe - 2 * PI;
			}
			while (Fe < -PI)
			{
				Fe = Fe + 2 * PI;
			}

			// Remove large spikes that last exactly one sample (due to phase crossing)
			if ((fabs(Fe - ZFe) / PI) > 0.5 && (fabs(ZFe - ZZFe) / PI) > 0.5 && (fabs(Fe - ZZFe) / PI) < 0.5)
			{
				ZFe = (Fe + ZZFe) / 2.0;
			}

			// Phase error metric to be used update the current phase with phase jumps (due to +/-PI) removed
			LoopPED = PED;
			while (LoopPED > PI / 2.0)
			{
				LoopPED = LoopPED - PI;
			}
			while (LoopPED < -PI / 2.0)
			{
				LoopPED = LoopPED + PI;
			}

			// Allow a certain length of time for frequency acquisition, then switch off the frequency acquisition, and enable phase tracking
			if (SV->Tracking.SampleTiming.CodeCount == 500)
			{
				alpha_Fe = alpha_Fe / 8.0;
				KFe = 0;
				K_p = 0.25;				// 2^-2
				//K_i = 0.015625;		// 2^-6;
				K_i = 0.00390625;		// 2^-8;
			}

			if (SV->Tracking.SampleTiming.CodeCount == 1500)
			{
				//SV->Tracking.SampleTiming.K_sp = SV->Tracking.SampleTiming.K_sp / 4.0;
				//SV->Tracking.SampleTiming.K_si = SV->Tracking.SampleTiming.K_si / 4.0;
			}

			prevFe_lpf = Fe_lpf;

			// Low-pass filter the frequency error, note a delay of one sample due to the single sample spike detection and removal
			Fe_lpf = (1 - alpha_Fe) * prevFe_lpf + alpha_Fe * ZFe;
			idphi  = idphi + KFe * ZFe;

			////////////////////////////////////////////////////////////////////
			// Fine phase / frequency tracking
			//

			prevPED_lpf = PED_lpf;

			// alpha = 1.0;
			// PED_lpf = (1 - alpha) * prevPED_lpf + alpha * LoopPED;
			PED_lpf = LoopPED;

			// Proportional-Integral Tracking (Carrier Phase / Frequency)
			tdphi = 0;											// Tracking of Doppler (not used)
			idphi = idphi + K_i * prevPED_lpf;                  // Integrator for frequency tracking
			pdphi = K_p * prevPED_lpf;                          // Proportional for static phase tracking
			dphi = dphi - pdphi - idphi - tdphi;


			////////////////////////////////////////////////////////////////////
			// BPSK sampling
			// We do not need a tracking loop for BPSK timing, since each BPSK symbol is known to be exactly 20 samples since the sampling frequency
			// is tracked as part of the code-sample timing loop
			//

			// Insert at the beginning of the vector (consider changing this later if there is a meaningful performance hit)
			i_buffer.resize(0); i_buffer.reserve(SamplesPerBit + 2);
			i_buffer.push_back(iq);

			p_buffer.resize(0); p_buffer.reserve(SamplesPerBit + 2);
			p_buffer.push_back(iq_code_phase);

			if (bpsk_iq_timing_buffer.size() != (SamplesPerBit + 2) || bit_code_phase_buffer.size() != (SamplesPerBit + 2))
			{
				fprintf(stderr, "Error: BPSK data resolution buffer integrity failure. Exiting safely."); fflush(stderr);
				exit(1);
			}

			i_buffer.insert(i_buffer.end(), bpsk_iq_timing_buffer.begin(), bpsk_iq_timing_buffer.end() - 1);
			bpsk_iq_timing_buffer = i_buffer;

			p_buffer.insert(p_buffer.end(), bit_code_phase_buffer.begin(), bit_code_phase_buffer.end() - 1);
			bit_code_phase_buffer = p_buffer;

			// Do not start BPSK tracking until it is determined that we have sufficiently converged
			if (SV->Tracking.BPSKTiming.f_bpsk_sampling_tracking)
			{
				// Use the sample period counter to align at the beginning of each bit (20 code phases per bit)
				if (SV->Tracking.BPSKTiming.SamplePeriodCounter == 0)
				{
					double acc;
					acc = 0;
					for (int ismpl = 0; ismpl < SamplesPerBit; ismpl++)
					{
						acc += imag(bpsk_iq_timing_buffer[2 + ismpl]);
					}
					SV->Tracking.Data.BinarySoft.     push_back(acc);
					SV->Tracking.Data.BinaryHard.     push_back(((acc > 0) ? +1 : 0));
					SV->Tracking.Data.BinaryCodePhase.push_back(bit_code_phase_buffer[2 + SamplesPerBit - 1]);

					// Debug
					acc = 0;
					for (int ismpl = 0; ismpl < SamplesPerBit; ismpl++)
					{
						acc += real(bpsk_iq_timing_buffer[2 + ismpl]);
					}
					SV->Tracking.Data.BinaryQuadrature.push_back(acc);
					// End Debug //

					SV->Tracking.BPSKTiming.SamplePeriodCounter += 20;
				}

				SV->Tracking.BPSKTiming.SamplePeriodCounter--;
			}
			else
			{
				// Initialize on a zero-crossing, after 20 samples of identical sign have been detected
				if (imag(bpsk_iq_timing_buffer[1]) * imag(bpsk_iq_timing_buffer[2]) > 0)
				{
					sign_count++;
				}
				else 
				{
					sign_count = 0;
				}

				// 20 samples of identical sign after at least 500 ms of acquisition (excluding initial correlation) are required to begin BPSK sampling data detection / decoding
				if (SV->Tracking.SampleTiming.CodeCount > 500 && sign_count >= 20)
				{
					if (imag(bpsk_iq_timing_buffer[0]) * imag(bpsk_iq_timing_buffer[1]) < 0)
					{
						SV->Tracking.BPSKTiming.f_bpsk_sampling_tracking = true;
						SV->Tracking.BPSKTiming.SamplePeriodCounter = 0;
					}
				}

				// If, after one second of tracking, no bit has been detected, then declare the SV undetected
				if (SV->Tracking.SampleTiming.CodeCount > 1000)
				{
					fprintf(stderr, "Status: SV %2d - Unable to lock on. Resetting and recommencing search.\n", SV->ID); fflush(stderr);
					SV->reset_tracking();
					return;
				}

			}
		}

		SV->Tracking.SampleTiming.CodeCount++;
	}

	////////////////////////////////////////////////////////////////////
	// Save state variable related to this SV
	//

	// Do not update K_sp or K_si here, since they are scaled values
	SV->Tracking.PhaseFrequencyRecovery.K_p         = K_p;
	SV->Tracking.PhaseFrequencyRecovery.K_i         = K_i;
	SV->Tracking.PhaseFrequencyRecovery.K_t         = K_t;
	SV->Tracking.PhaseFrequencyRecovery.KFe         = KFe;
	SV->Tracking.PhaseFrequencyRecovery.alpha_Fe    = alpha_Fe;

	SV->Tracking.PhaseFrequencyRecovery.dphi        = dphi;
	SV->Tracking.PhaseFrequencyRecovery.idphi       = idphi;

	SV->Tracking.PhaseFrequencyRecovery.PED         = PED;
	SV->Tracking.PhaseFrequencyRecovery.Fe          = Fe;
	SV->Tracking.PhaseFrequencyRecovery.ZFe         = ZFe;
	SV->Tracking.PhaseFrequencyRecovery.Fe_lpf      = Fe_lpf;
	SV->Tracking.PhaseFrequencyRecovery.PED_lpf     = PED_lpf;

	SV->Tracking.BPSKTiming.sign_count              = sign_count;
	SV->Tracking.BPSKTiming.bpsk_iq_timing_buffer   = bpsk_iq_timing_buffer;
	SV->Tracking.BPSKTiming.bit_code_phase_buffer   = bit_code_phase_buffer;
}
