#include "headers.h"

#include <limits>

void print_usage(char *progname)
{
	fprintf(stderr, "\nUsage: %s -f<infilename> -DopplerSwing <value> -DopplerStep <value> -DetectionThreshold <value> -CorrelationWindow <value> -OSR <value>\n\n", progname);

	fprintf(stderr, "-f<infilename>\t\t\tThe filename of the input data stream. Note that there is NO SPACE between -f and the filename.\n\t\t\t\tSpecifying this as stdin takes input from the console.\n\n");
	fprintf(stderr, "-DopplerLow <value>\t\tThe beginning of the range of Doppler offset frequencies used when searching for each SV.\n\t\t\t\tOverwrites the DopplerLow value where placed later on the command line than DopplerSwing.\n\t\t\t\tInteger only, specified in Hz.\n\n");
	fprintf(stderr, "-DopplerHigh <value>\t\tThe end of the range of Doppler offset frequencies used when searching for each SV.\n\t\t\t\tInteger only, specified in Hz.\n\n");
	fprintf(stderr, "-DopplerSwing <value>\t\tThe the range of Doppler offset frequencies about the centre frequency (+/-DopplerSwing) used when searching for each SV.\n\t\t\t\tOverrides all earlier DopplerLow/DopplerHigh settings on the command line.\n\t\t\t\tPositive integer only, specified in Hz.\n\n");
	fprintf(stderr, "-DopplerStep <value>\t\tThe frequency spacing between consecutive Doppler offset frequencies used when searching for each SV.\n\t\t\t\tIf not an integer divisor of (2 X DopplerSwing) then the maximum Doppler offset is reduced accordingly.\n\t\t\t\tInteger only, specified in Hz.\n\n");
	fprintf(stderr, "-DetectionThreshold <value>\tThe threshold value to use in detection of SVs.\n\t\t\t\tPositive integer only.\n\n");
	fprintf(stderr, "-CorrelationWindow <value>\tThe length of time over which to correlate in SV detection in ms.\n\t\t\t\tPositive integer only.\n\n");
	fprintf(stderr, "-CorrelationMode <value>\tThe mode in which to operate the correlation used in the initial acquisition satellites.\n\t\t\t\tValid values: MultiPhase, FFT, Full.\n\n");
	fprintf(stderr, "-OSR <value>\t\t\tThe oversampling ratio at which to process data (during the tracking phase, but also used to simplify data selection during acquisition).\n\t\t\t\tPositive integer multiple of 4 only.\n\n");
	fprintf(stderr, "-SleepTime <value>\t\t\tThe length of time to wait after failing to acquire a satellite before trying again.\n\t\t\t\tPositive integer only, specified in ms.\n\n");
	//fprintf(stderr, "-d<debugmode>\t\tSet to one to enable debug mode.\n\t\t\tSince this slows operation and prints extraneous information to the console, this is not recommended unless debugging.\n\n");
}

int parse_cmdline(int argc, char **argv, ClGPSParameters *const param)
{
	std::string finname = GPS_DEFAULT_DATA_INPUT_FILENAME;
	char *cmdlinestr;

	if (argc == 1) {
		print_usage(argv[0]);
	}

//	*DEBUG = 0;

	for (int n = 1; n < argc; n++) {
		if (argv[n][0] == '-') {
			switch (argv[n][1]) {
			case 'f':
				finname = &argv[n][2];
				break;

			case 'D':
				cmdlinestr = &argv[n][1];
				if (argc >= n + 2)				// Ensure that the parameter exists before attempting to access it
				{
					if (!strcmp(cmdlinestr, "DopplerSwing"))
					{
						param->DopplerHighHz = (int)atof(&argv[n + 1][0]);
						param->DopplerLowHz = -1 * param->DopplerHighHz;

						if (param->DopplerHighHz < 0)
						{
							fprintf(stderr, "\nError: Command-line argument DopplerSwing must be positive. Exiting safely.\n"); fflush(stderr);
							exit(1);
						}
					}
					else if (!strcmp(cmdlinestr, "DopplerStep"))
					{
						param->DopplerStepHz = (int)atof(&argv[n + 1][0]);

						if (param->DopplerStepHz < 0)
						{
							fprintf(stderr, "\nError: Command-line argument DopplerStep must be positive. Exiting safely.\n"); fflush(stderr);
							exit(1);
						}
					}
					else if (!strcmp(cmdlinestr, "DopplerLow"))
					{
						param->DopplerLowHz = (int)atof(&argv[n + 1][0]);
					}
					else if (!strcmp(cmdlinestr, "DopplerHigh"))
					{
						param->DopplerHighHz = (int)atof(&argv[n + 1][0]);
					}
					else if (!strcmp(cmdlinestr, "DetectionThreshold"))
					{
						param->DetectionThreshold = (int)atof(&argv[n + 1][0]);
						if (param->DetectionThreshold < 0)
						{
							fprintf(stderr, "\nError: Command-line argument DetectionThreshold must be a positive integer. Exiting safely.\n"); fflush(stderr);
							exit(1);
						}
					}
				}
				break;

			case 'C':
				cmdlinestr = &argv[n][1];
				if (argc >= n + 2)				// Ensure that the parameter exists before attempting to access it
				{
					if (!strcmp(cmdlinestr, "CorrelationWindow"))
					{
						int tmp;
						tmp = (int)atof(&argv[n + 1][0]);
						if (tmp < 0)
						{
							fprintf(stderr, "\nError: Command-line argument CorrelationWindow_ms must be a positive value. Exiting safely.\n"); fflush(stderr);
							exit(1);
						}
						param->CorrelationWindow_ms = tmp;
					}
					else if (!strcmp(cmdlinestr, "CorrelationMode"))
					{
						char *modestr = &argv[n + 1][0];
						if (!strcmp(modestr, "MultiPhase"))
						{
							param->SVDetectionCorrelationMode = SV_MODE_MULTI_PHASE_CORRELATION;
						}
						else if (!strcmp(modestr, "FFT"))
						{
							param->SVDetectionCorrelationMode = SV_MODE_FFT_BASED_CORRELATION;
						}
						else if (!strcmp(modestr, "Full"))
						{
							param->SVDetectionCorrelationMode = SV_MODE_FULL_CORRELATION;
							fprintf(stderr, "Warning: Correlation mode ""Full"" not supported. Using default value.\n");
						}
						else {
							fprintf(stderr, "Invalid correlation mode specified at the command line. Defaulting to MultiPhase mode.\n");
							fflush(stderr);
							param->SVDetectionCorrelationMode = SV_MODE_MULTI_PHASE_CORRELATION;
						}
			
					}

				}
				break;

			case 'O':
				cmdlinestr = &argv[n][1];
				if (argc >= n + 2)				// Ensure that the parameter exists before attempting to access it
				{
					if (!strcmp(cmdlinestr, "OSR"))
					{
						param->OSR = (int)atof(&argv[n + 1][0]);
						if (((param->OSR) % 4) != 0)
						{
							fprintf(stderr, "\nError: Command-line argument OSR must be a positive integer multiple of 4. Exiting safely.\n"); fflush(stderr);
							exit(1);
						}
					}
				}
				break;

			case 'S':
				cmdlinestr = &argv[n][1];
				if (argc >= n + 2)				// Ensure that the parameter exists before attempting to access it
				{
					if (!strcmp(cmdlinestr, "SleepTime"))
					{
						int tmp;
						tmp = (int)atof(&argv[n + 1][0]);
						if (tmp < 0)
						{
							fprintf(stderr, "\nError: Command-line argument SleepTime must be a positive value. Exiting safely.\n"); fflush(stderr);
							exit(1);
						}
						param->SearchCooldown_ms = tmp;
					}
				}
				break;

			case '?':
				print_usage(argv[0]);
				exit(0);
				break;

//			case 'd':
//				*DEBUG = (int)atof(&argv[n][2]);
//				if (*DEBUG != 0 && *DEBUG != 1) {
//					fprintf(stderr, "Invalid Flag for Debug Specified at the Command Line. Using Default.\n");
//					*DEBUG = 0;
//				}
//				break;

			default:
				fprintf(stderr, "Unrecognised command line parameter: %s\n", argv[n]);
				break;
			}
		}
	}

	// Correlation window size depends on the OSR
	param->CorrelationWindowSamplesCodeRate = ((unsigned)(1023 * (param->CorrelationWindow_ms)));

	// Convert from ms to samples
	param->SearchCooldown_Samples = 1023 * param->SearchCooldown_ms;

	if (argc == 1) {
		fprintf(stderr, "\n\n");
	}

	param->GpsDataInputFilename = finname;

	size_t dotidx = param->GpsDataInputFilename.find_last_of(".");

	if (dotidx == std::string::npos)
	{
		param->GpsDataInputFilename += ".bin";
	}

	return(0);
}


void parse_parameter_file(ClGPSParameters *const param)
{
	std::string DataTypeString;
	std::string ParameterString;

	int ParseFileOSR = GPS_PARAM_INVALID_OSR;

	size_t dotidx = param->GpsDataInputFilename.find_last_of(".");

	if (dotidx == std::string::npos)
	{
		fprintf(stderr, "Unable to determine parameter filename. Using default values.\n"); fflush(stderr);
		return;
	}

	std::string basename = param->GpsDataInputFilename.substr(0, dotidx);

	param->GpsParameterFilename = basename + ".dat";

	fprintf(stderr, "Parameter file: %s\n", param->GpsParameterFilename.c_str()); fflush(stderr);

	std::ifstream paramfile(param->GpsParameterFilename.c_str(), std::ios::in);

	if (paramfile >> ParameterString)
	{
		param->FsData = atof(ParameterString.c_str());
		paramfile.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
	}
	else 
	{
		fprintf(stderr, "Unable to read FsData from parameter filename. Exiting safely.\n"); fflush(stderr);
		exit(1);
	}

	if (paramfile >> ParameterString)
	{
		param->Fc = atof(ParameterString.c_str());
		paramfile.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
	}
	else 
	{
		fprintf(stderr, "Unable to read Fc from parameter filename. Exiting safely.\n"); fflush(stderr);
		exit(1);
	}

	if (paramfile >> ParameterString)
	{
		ParseFileOSR = (int)atof(ParameterString.c_str());
		paramfile.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
	}
	else
	{
		fprintf(stderr, "Unable to read OSR from parameter filename. Exiting safely.\n"); fflush(stderr);
		exit(1);
	}

	// Only use the OSR from the file if a valid OSR has not previously been specified
	// since the parameter file value is overrideable
	if (param->OSR == GPS_PARAM_INVALID_OSR)
	{
		param->OSR = ParseFileOSR;
	}

	if (paramfile >> ParameterString)
	{
		DataTypeString = ParameterString;
		paramfile.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
	}
	else
	{
		fprintf(stderr, "Unable to read DATA_TYPE from parameter filename. Exiting safely.\n"); fflush(stderr);
		exit(1);
	}
	
	if (!strcmp(DataTypeString.c_str(), "DT_8_BIT_REAL"))
	{
		param->DataType = DT_8_BIT_REAL;
	}
	else if (!strcmp(DataTypeString.c_str(), "DT_8_BIT_COMPLEX"))
	{
		param->DataType = DT_8_BIT_COMPLEX;
	}
	else if (!strcmp(DataTypeString.c_str(), "DT_16_BIT_REAL"))
	{
		param->DataType = DT_16_BIT_REAL;
	}
	else if (!strcmp(DataTypeString.c_str(), "DT_16_BIT_COMPLEX"))
	{
		param->DataType = DT_16_BIT_COMPLEX;
	}

}


void echo_current_parameters(const ClGPSParameters &param)
{
	fprintf(stderr, "\n\nCurrent parameters:\n---------------");
	fprintf(stderr, "\nInput                           : %s\n",        param.GpsDataInputFilename.c_str());
	switch (param.DataType)
	{
	case DT_8_BIT_REAL:
		fprintf(stderr, "Data Type                       : Real Integer (8-bit)\n");
		break;
	case DT_8_BIT_COMPLEX:
		fprintf(stderr, "Data Type                       : Complex Integer (8-bit)\n");
		break;
	case DT_16_BIT_REAL:
		fprintf(stderr, "Data Type                       : Real Integer (16-bit)\n");
		break;
	case DT_16_BIT_COMPLEX:
		fprintf(stderr, "Data Type                       : Complex Integer (16-bit)\n");
		break;
	default:
		fprintf(stderr, "Data Type                       : Unknown/Unsupported\n");
	}
	fprintf(stderr, "Oversampling rate (OSR)         : %9d\n",         param.OSR);
	fprintf(stderr, "\n");
	fprintf(stderr, "Integration Time                : %9d ms\n",      param.CorrelationWindow_ms);
	fprintf(stderr, "Integration Samples (Code Rate) : %9d samples\n", param.CorrelationWindowSamplesCodeRate);
	switch (param.SVDetectionCorrelationMode)
	{
	case SV_MODE_MULTI_PHASE_CORRELATION:
		fprintf(stderr, "Correlation Mode                : Multi-phase direct correlation at the code rate\n");
		break;
	case SV_MODE_FFT_BASED_CORRELATION:
		fprintf(stderr, "Correlation Mode                : FFT-based (at the oversampled rate)\n");
		break;
	case SV_MODE_FULL_CORRELATION:
		fprintf(stderr, "Correlation Mode                : Full direct correlation at the oversampled rate\n");
		break;
	default:
		fprintf(stderr, "Correlation Mode                : INVALID\n");
		fflush(stderr);
		exit(1);
		break;
	}
	fprintf(stderr, "\n");
	fprintf(stderr, "Doppler Low                     : %9d Hz\n",      param.DopplerLowHz);
	fprintf(stderr, "Doppler High                    : %9d Hz\n",      param.DopplerHighHz);
	fprintf(stderr, "Doppler Step                    : %9d Hz\n",      param.DopplerStepHz);
	fprintf(stderr, "\n");											   
	fprintf(stderr, "Detection Threshold             : %9d\n",         param.DetectionThreshold);
	fprintf(stderr, "Input Sampling rate             : %12.2lf Hz\n",  param.FsData);
	fprintf(stderr, "GPS Centre Frequency            : %12.2lf Hz\n",  param.Fc);
	fprintf(stderr, "SV Acquisition Sleep Interval   : %12.2lf s\n",   param.SearchCooldown_ms / 1000.0);
	fprintf(stderr, "\n");

	fflush(stderr);
}
