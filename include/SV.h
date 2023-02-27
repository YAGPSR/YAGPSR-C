#ifndef SVH
#define SVH

typedef struct {
	double    ClockPeriod;		// The clock period at the oversampled rate
	//double    CodePhase;		// The sample index relative to the clock period
	int       i_CodePhase;      // The integer component of the code phase, since the code phase values can get quite large but it's the small differences that are important to timing
	double    f_CodePhase;	    // The fractional part of the code phase, update only f_CodePhase during each tick of the tracking loop, and adjust both i_CodePhase and f_CodePhase appropriately
	int       CodeCount;		// A counter which allows us to enable/disable tracking behaviours after a set period of time
	int       LoopCount;        // A counter used to count samples such that adjustments to the CodePhase are made on the same sample for all SVs

	double    K_sp;				// Scaled PI Sample time tracking parameter (proportional) - muliplied by the OSR to get actual control loop parameter
	double    K_si;				// Scaled PI Sample time tracking parameter (integral) - muliplied by the OSR to get actual control loop parameter

	double    RunningSampleTimeMetric;		// The sample time metric is low-pass filtered, and stored here so that it is always up-to-date 

	vcomplex_t ZeroIFBuffer;			// The GPS samples after being moved to Zero-IF based on the initialization-estimated Doppler frequency corresponding to this satellite

} t_SampleTiming;


typedef struct {
	double K_p;
	double K_i;
	double K_t;
	double KFe;
	double alpha_Fe;

	double dphi;
	double idphi;

	//prev_stm = 0;

	double PED;
	double Fe;
	double ZFe;
	double Fe_lpf;
	double PED_lpf;

} t_PhaseFrequencyRecovery;


typedef struct {
	int        sign_count;
	bool       f_bpsk_sampling_tracking;
	vdouble_t  bit_code_phase_buffer;
	vcomplex_t bpsk_iq_timing_buffer;
	int        SamplePeriodCounter;

} t_BPSKTiming;


typedef struct {
	vdouble_t BinarySoft;
	vint_t    BinaryHard;
	vdouble_t BinaryQuadrature;
	vdouble_t BinaryCodePhase;

	vdouble_t BinaryNew;

	bool      f_GPSBinaryInversion;
	bool      f_SubFrameAlignment;
	bool      f_isValid;
	bool      f_satellitePositionCurrent;

	int       TLMDetectIdx;
	double    TLMCodePhase;
	int       TLMSubframe;
	double    clockCorrect;
	int       Subframe_idx;
	

} t_SVData;


typedef struct {
	uint32_t TOW;

	uint32_t WeekNumber;
	uint32_t Accuracy;
	uint32_t Health;
	uint32_t IODC;
	uint32_t IODE_SF1;
	uint32_t tOC;
   
	double TGD;
	double af2;
	double af1;
	double af0;

	uint32_t IODE_SF2;

	double Crs;
	double Deltan;
	double M0;
	double Cuc;
	double e;
	double Cus;
	double sqrtA;
	double toe;

	uint32_t IODE_SF3;

	double Cic;
	double Omega0;
	double Cis;
	double I0;
	double Crc;
	double omega;
	double OmegaDot;
	double IDOT;

	int64_t GPS_Time;
	int64_t GPS_Time_TLM;

} t_DecodedData;


typedef struct {
	double x;
	double y;
	double z;
} t_XYZCoordinates;


typedef struct {
	t_SampleTiming           SampleTiming;
	t_PhaseFrequencyRecovery PhaseFrequencyRecovery;
	t_BPSKTiming             BPSKTiming;
	t_SVData                 Data;
	t_DecodedData            Decoded;
	t_XYZCoordinates         Coord;

} t_SVParam;

class t_SpaceVehicle {
public:
	t_SpaceVehicle();
	t_SpaceVehicle(const int &, const int &);

	void initialize(const int &, const int &);
	void reset_tracking();
	

	void set_ID(const int &);

	int       ID;

	double    DetectionMetric;
	double    NormalizedDoppler;
	double    DopplerPhase;
	bool      f_Detected;
	bool      f_Enabled;
	int       IntegrationSamples;
	int       SearchCooldown;

	t_SVParam Tracking;

	vcomplex_t Debug;				// Used for the gathering of debug information in a persistent manner

	int        OSR;					// The oversampling ratio at which the data is being processed/tracked

private:
	void initialize_ClockPeriod(const int &);

protected:
};

#endif