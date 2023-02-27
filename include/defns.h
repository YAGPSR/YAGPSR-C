#ifndef DEFNSH
#define DEFNSH

// Define D_YAGPSR_SAT_FAST_SCAN to enable fast scan functionality
// Set D_YAGPSR_SAT_FAST_SCAN_MODE to choose the fast scan mode: 0 to use fast correlation mode using integers, rather than doubles (pretty much the same acquisition performance, but much faster)
// 0 to use sign only (same processing requirements on x86, since bits are stored as integers), included for personal interest and to allow testing of one-bit acquisition
//#define D_YAGPSR_SAT_FAST_SCAN
#define D_YAGPSR_SAT_FAST_SCAN_MODE 0

// Mathematical constants
constexpr auto PI = 3.1415926535897932384626433832795028841972;

// Speed of light in m / s
constexpr double c_in_ms = 2.99792458e8;                                       

// Default parameter values
constexpr auto GPS_DEFAULT_DATA_INPUT_FILENAME             = "data.bin";
constexpr auto GPS_DEFAULT_PARAMETER_INPUT_FILENAME        = "data.dat";
														   
constexpr auto GPS_DEFAULT_FSDATA                          = 12000000;
constexpr auto GPS_DEFAULT_FC                              = 3563000;

constexpr auto GPS_DEFAULT_DOPPLERLOWHZ                    = -5000;
constexpr auto GPS_DEFAULT_DOPPLERHIGHHZ                   = 5000;
constexpr auto GPS_DEFAULT_DOPPLERSTEPHZ                   = 200;
constexpr auto GPS_DEFAULT_DETECTIONTHRESHOLD              = 200;
										      
constexpr auto GPS_PARAM_INVALID_OSR                       = -1;
										      
constexpr auto GPS_DEFAULT_NCORRELATION_PHASES             = 4;

//
constexpr auto TOTAL_NUMBER_OF_SATELLITES                  = 32;
constexpr auto MAX_SIMULTANEOUS_SV_ACQUIRE                 = 32;

constexpr auto DEFAULT_DATA_BLOCK_SIZE_MS                  = 500;
constexpr auto DEFAULT_CORRELATION_WINDOW_MS               = 5;
constexpr auto DEFAULT_CORRELATION_WINDOW_SAMPLES_CODERATE = ((unsigned)(1023 * (DEFAULT_CORRELATION_WINDOW_MS)));

constexpr auto MAX_SV_SEARCH_TIME_MS                       = 30;
constexpr auto MAX_SV_SEARCH_SAMPLES                       = ((unsigned)(1023 * (MAX_SV_SEARCH_TIME_MS)));

constexpr auto SV_SEARCH_COOLDOWN_TIME_MS                  = 1000;
constexpr auto SV_SEARCH_COOLDOWN_SAMPLES                  = ((unsigned)(1023 * (SV_SEARCH_COOLDOWN_TIME_MS)));

// Status flags
#define GPS_READ_STATUS_OK   0
#define GPS_READ_STATUS_EOF -1

#define SELF_TEST_PASS       0
#define SELF_TEST_FAIL       1

#endif
