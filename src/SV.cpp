// Initialize the SV data types and precalculate GPS spreading code
//

#include "headers.h"

/*
 * Initialize the SV
*/

t_SpaceVehicle::t_SpaceVehicle()
{
	f_Detected = false;
	f_Enabled  = true;

	ID         = -1;

	initialize(0, ID);
}

t_SpaceVehicle::t_SpaceVehicle(const int &valOSR, const int &valID)
{
	f_Detected          = false;
	f_Enabled           = true;

	initialize(valOSR, valID);
}

void t_SpaceVehicle::initialize(const int &valOSR, const int &valID)
{
	reset_tracking();

	OSR                = valOSR;
	ID                 = valID;

	initialize_ClockPeriod(valOSR);
}

void t_SpaceVehicle::initialize_ClockPeriod(const int &OSR)
{
	Tracking.SampleTiming.ClockPeriod = 1023 * OSR;
}

void t_SpaceVehicle::set_ID(const int &k)
{
	ID = k;
}

// Reset all internal variables used for acquistion and tracking of satellites
void t_SpaceVehicle::reset_tracking()
{
	f_Detected                                    = false;
	f_Enabled                                     = true;

	DetectionMetric                               = 0;
	NormalizedDoppler                             = 0;
	DopplerPhase                                  = 0;
					                              
	IntegrationSamples                            = 0;
	SearchCooldown                                = 0;

	//Tracking.SampleTiming.CodePhase               = 0;
	Tracking.SampleTiming.CodeCount               = 1;
	Tracking.SampleTiming.i_CodePhase             = 0;
	Tracking.SampleTiming.f_CodePhase             = 0;
	Tracking.SampleTiming.LoopCount               = 0;
	Tracking.SampleTiming.RunningSampleTimeMetric = 0;

	Tracking.SampleTiming.ZeroIFBuffer.resize(0);

	Tracking.SampleTiming.K_sp               = 0.015625;				// Scaled PI sample time tracking parameter (proportional)
	Tracking.SampleTiming.K_si               = 0.0000152587890625;		// Scaled PI sample time tracking parameter (integral)

	Tracking.PhaseFrequencyRecovery.K_p      = 0;				// Phase tracking - initially set to zero (disabled) until frequency tracking is converged
	Tracking.PhaseFrequencyRecovery.K_i      = 0;				// Frequency tracking - initially disabled until frequency tracking is converged
	Tracking.PhaseFrequencyRecovery.K_t      = 0;				// Unused, but kept in case it is needed / desired later
	Tracking.PhaseFrequencyRecovery.KFe      = 0.015625;		// Initial frequency acquisition
	Tracking.PhaseFrequencyRecovery.alpha_Fe = 0.015625;

	Tracking.PhaseFrequencyRecovery.dphi     = 0;
	Tracking.PhaseFrequencyRecovery.idphi    = 0;

	//Tracking.PhaseFrequencyRecovery.prev_stm = 0;

	Tracking.PhaseFrequencyRecovery.PED     = 0;
	Tracking.PhaseFrequencyRecovery.Fe      = 0;
	Tracking.PhaseFrequencyRecovery.ZFe     = 0;
	Tracking.PhaseFrequencyRecovery.Fe_lpf  = 0;
	Tracking.PhaseFrequencyRecovery.PED_lpf = 0;

	Tracking.BPSKTiming.sign_count = 0;
	Tracking.BPSKTiming.f_bpsk_sampling_tracking = false;
	Tracking.BPSKTiming.bit_code_phase_buffer.resize(0);
	Tracking.BPSKTiming.bit_code_phase_buffer.resize(22, 0.0);

	Tracking.BPSKTiming.bpsk_iq_timing_buffer.resize(0);
	Tracking.BPSKTiming.bpsk_iq_timing_buffer.resize(22, 0.0);
	Tracking.BPSKTiming.SamplePeriodCounter = 0;

	Tracking.Data.BinarySoft.resize(0);
	Tracking.Data.BinaryHard.resize(0);
	Tracking.Data.BinaryQuadrature.resize(0);
	Tracking.Data.BinaryCodePhase.resize(0);
	Tracking.Data.BinaryNew.resize(0);

	Tracking.Data.f_GPSBinaryInversion       = false;
	Tracking.Data.f_SubFrameAlignment        = false;
	Tracking.Data.f_isValid                  = false;
	Tracking.Data.f_satellitePositionCurrent = false;

	Tracking.Data.TLMDetectIdx = 0;
	Tracking.Data.TLMCodePhase = 0;
	Tracking.Data.TLMSubframe  = 0;
	Tracking.Data.Subframe_idx = 0;

	Tracking.Decoded.TOW       = 0;

	Tracking.Decoded.GPS_Time     = 0;
	Tracking.Decoded.GPS_Time_TLM = 0;
	
	DetectionMetric               = 0;
	NormalizedDoppler             = 0;
	DopplerPhase                  = 0;

	initialize_ClockPeriod(OSR);

	Debug.resize(0);
}
