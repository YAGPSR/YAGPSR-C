#pragma once

// Parameter class, used as essentially a structure with an initializer to default values
class ClGPSParameters {
public:
	ClGPSParameters();
	~ClGPSParameters();

	std::string         GpsDataInputFilename;
	std::string         GpsParameterFilename;
	double              FsData;
	double              Fc;
	int                 OSR;
	tdStreamDataFormat  DataType;
	int                 DopplerLowHz;
	int                 DopplerHighHz;
	int                 DopplerStepHz;
	int                 DetectionThreshold;
	unsigned            CorrelationWindow_ms;
	unsigned            CorrelationWindowSamplesCodeRate;
	unsigned            InputDataBlockSize_ms;
	unsigned            InputDataBlockSize;
	unsigned            NumberOfCorrelationPhases;
	int                 SearchCooldown_ms;
	int                 SearchCooldown_Samples;
	int                 MaxSVAcquisition_Samples;

	tdSVCorrelationMode SVDetectionCorrelationMode;
private:
protected:
};
