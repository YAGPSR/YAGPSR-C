/*
* System parameters and functions to access them
*/

#include "headers.h"

/*
* The default parameters of the input data stream
*/

ClGPSParameters::ClGPSParameters()
{
	// Simple parameters
	GpsDataInputFilename      = GPS_DEFAULT_DATA_INPUT_FILENAME;
	GpsParameterFilename      = GPS_DEFAULT_PARAMETER_INPUT_FILENAME;
						      
	FsData                    = GPS_DEFAULT_FSDATA;						// The sampling rate of the input data, in Hz
	Fc                        = GPS_DEFAULT_FC;							// The centre frequency of the input data, in Hz 
	OSR                       = GPS_PARAM_INVALID_OSR;					// The oversampling rate at which to perform acquisition
	DataType                  = DT_8_BIT_REAL;
						      
	DopplerLowHz              = GPS_DEFAULT_DOPPLERLOWHZ;
	DopplerHighHz             = GPS_DEFAULT_DOPPLERHIGHHZ;
	DopplerStepHz             = GPS_DEFAULT_DOPPLERSTEPHZ;
	DetectionThreshold        = GPS_DEFAULT_DETECTIONTHRESHOLD;
	NumberOfCorrelationPhases = GPS_DEFAULT_NCORRELATION_PHASES;

	// Derived parameters
	CorrelationWindow_ms      = DEFAULT_CORRELATION_WINDOW_MS;
	InputDataBlockSize_ms     = DEFAULT_DATA_BLOCK_SIZE_MS;

	CorrelationWindowSamplesCodeRate = ((unsigned)(1023 * (CorrelationWindow_ms)));
	InputDataBlockSize               = ((unsigned)((double)FsData * (InputDataBlockSize_ms / 1000.0)));

	SearchCooldown_ms                = SV_SEARCH_COOLDOWN_TIME_MS;
	SearchCooldown_Samples           = 1023 * SearchCooldown_ms;
	MaxSVAcquisition_Samples         = MAX_SV_SEARCH_SAMPLES;

	SVDetectionCorrelationMode       = SV_MODE_MULTI_PHASE_CORRELATION;
}

ClGPSParameters::~ClGPSParameters()
{
}