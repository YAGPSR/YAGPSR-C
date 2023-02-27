//
//

#include "headers.h"

static t_SpaceVehicle SV[TOTAL_NUMBER_OF_SATELLITES+1];		// For simplicity, the SVs will be numbered as per GPS with SV[1] corresponding to Satellite Vehicle 1
															// SV[0], while defined, will remain unused

void write_binary_stream_to_file()
{
	FILE *fpdebug;

	for (int iSV = 1; iSV < TOTAL_NUMBER_OF_SATELLITES + 1; iSV++)
	{
		if (SV[iSV].f_Detected)
		{
			std::string ifname, qfname, dbgname;

			ifname = std::string("binary_stream_soft_") + std::to_string(iSV) + std::string(".txt");
			fpdebug = fopen(ifname.c_str(), "w");
			if (fpdebug == NULL)
			{
				fprintf(stderr, "Error: Failed to open debug file %s\n", ifname.c_str()); fflush(stderr);
				exit(1);
			}
			for (size_t n = 0; n < SV[iSV].Tracking.Data.BinarySoft.size(); n++)
			{
				fprintf(fpdebug, "%lf\n", SV[iSV].Tracking.Data.BinarySoft[n]);
			}
			fclose(fpdebug);

			qfname = std::string("binary_stream_") + std::to_string(iSV) + std::string("_q.txt");
			fpdebug = fopen(qfname.c_str(), "w");
			if (fpdebug == NULL)
			{
				fprintf(stderr, "Error: Failed to open debug file %s\n", qfname.c_str()); fflush(stderr);
				exit(1);
			}
			for (size_t n = 0; n < SV[iSV].Tracking.Data.BinaryQuadrature.size(); n++)
			{
				fprintf(fpdebug, "%lf\n", SV[iSV].Tracking.Data.BinaryQuadrature[n]);
			}
			fclose(fpdebug);

			if (SV[iSV].Debug.size() > 0)
			{
				dbgname = std::string("debug_") + std::to_string(iSV) + std::string(".txt");

				std::string out_string;
				out_string.resize(0);
				out_string.reserve(SV[iSV].Debug.size() * 20);

				for (size_t n = 0; n < SV[iSV].Debug.size(); n++)
				{
					out_string.append(std::to_string(real(SV[iSV].Debug[n])));
					out_string.append("\t");
					out_string.append(std::to_string(imag(SV[iSV].Debug[n])));
					out_string.append("\n");
				}
				std::ofstream dbgstream(dbgname, std::ios::out | std::ios::binary);
				dbgstream.write(&(out_string[0]), out_string.size() * sizeof(char));
				dbgstream.close();
			}
		}
	}
}

int read_gps_raw_data_samples(vcomplex_t *const x, const ClGPSParameters &YAGPSRParam, const size_t &N)
{
	// The file is held open between calls (closed only upon EOF)
	static std::fstream        istream;
	vcomplex_t                 cmplx_buffer;
	static unsigned long long  totalDataRead = 0;

	int Nr;

	// This buffer is void since the data type read may be in multiple different formats
	void *buffer;

	// Open the file only if not already open
	if (!istream.is_open())
	{
		istream.open(YAGPSRParam.GpsDataInputFilename.c_str(), std::ios::in | std::ios::binary);
		if (!istream.is_open())
		{
			fprintf(stderr, "Failed to open %s.\n", YAGPSRParam.GpsDataInputFilename.c_str()); fflush(stderr);
			exit(1);
		}
	}

	// Allocate memory for the worst case, i.e. 2 complex double
	buffer = malloc(N * 2 * sizeof(double));

	// This is a bit messy, but it works
	switch (YAGPSRParam.DataType)
	{
	case DT_8_BIT_REAL:
		istream.read((char *)buffer, 1 * N);
		Nr = (int)istream.gcount();
		totalDataRead += Nr;
		real_array_to_complex(&cmplx_buffer, (int8_t *)buffer, Nr);						// Real
		break;

	case DT_8_BIT_COMPLEX:
		istream.read((char *)buffer, 1 * N);
		Nr = (int)istream.gcount();
		totalDataRead += Nr;
		interleaved_array_to_complex(&cmplx_buffer, (int8_t *)buffer, Nr);				// Complex
		break;

	case DT_16_BIT_REAL:
		istream.read((char *)buffer, 2 * N);
		Nr = (int)istream.gcount();
		totalDataRead += Nr;
		Nr = Nr / 2;
		real_array_to_complex(&cmplx_buffer, (int16_t *)buffer, Nr);					// Real
		break;

	case DT_16_BIT_COMPLEX:
		istream.read((char *)buffer, 2 * N);
		Nr = (int)istream.gcount();
		totalDataRead += Nr;
		Nr = Nr / 2;
		interleaved_array_to_complex(&cmplx_buffer, (int16_t *)buffer, Nr);				// Complex
		break;

	default:
		fprintf(stderr, "Error: Unrecognized or unsupported input data format. Exiting safely.\n"); fflush(stderr);
		exit(1);
	}

	free(buffer);


	if (istream.eof())
	{
		istream.close();
		fprintf(stderr, "Total number of data bytes read before EOF: %llu\n", totalDataRead); fflush(stderr);
		return(GPS_READ_STATUS_EOF);
	}

	// Prepare storage, convert to complex and append additional data
	x->insert(x->end(), cmplx_buffer.begin(), cmplx_buffer.end());

	return(GPS_READ_STATUS_OK);
}


/*
* Select the SVs to search for on the current loop
* Chosen based on the following criteria:
*   1/ Choose a total number of SVs based on the ability to process in real-time (or near real-time/low latency, based on sleeping SVs which are not acquired for a certain length of time)
*   2/ Favour SVs which are most likely (based on Almanac, Ephemeris data, or other) 
*/

int updateSVSearchList(vint_t *const SVIdx, const vcomplex_t &SVDetectBlock, const unsigned &OSR)
{
	SVIdx->resize(0);
	SVIdx->reserve(TOTAL_NUMBER_OF_SATELLITES + 1);

	// Create a list of satellites to search for 
	for (int n = 1; n < TOTAL_NUMBER_OF_SATELLITES + 1; n++)
	{
		if (SV[n].f_Enabled == false)
		{
			continue;	// Never add disabled SVs to the list, used mostly for debugging
		}

		// Only add satellites which are out of cooldown (due to previous non-detection of the SV over a number of samples (period of time))
		if (SV[n].SearchCooldown != 0)
		{
			// Count down samples at the 1.023 MHz code rate
			SV[n].SearchCooldown -= (int)SVDetectBlock.size() / OSR;

			if (SV[n].SearchCooldown <= 0)
			{
				//fprintf(stdout, "Status: SV %2d RESUME search after %8.2lf ms of sleep.\n", n, SV_SEARCH_COOLDOWN_TIME_MS - SV[n].SearchCooldown / 1023.0); fflush(stdout);
				SV[n].SearchCooldown = 0;
			}
		}
		else
		{
			if (SV[n].f_Detected == false)
			{
				SVIdx->push_back(n);
			}

			// Specificy a maximum number of satellites to search for simultaneously
			if (SVIdx->size() >= MAX_SIMULTANEOUS_SV_ACQUIRE)
			{
				break;
			}
		}
	}

	return(0);
}

/*
* GPS Receiver - Main Processing Loop
*/

int gps_receive(const ClGPSParameters &Param)
{
	vcomplex_t x;
	double     FsTargetHz;
	unsigned   CorrelationWindowSamplesOSR;
	size_t     TrackingBlockSize, TrackingBlockTrimSize;

	vcomplex_t ResamplerOutput;
	vcomplex_t FrequencyShiftOutput;
	vcomplex_t SVDetectBlock;
	vcomplex_t processInputBuffer;
	vint_t     SVIdx;

	ClResampler			FrontEndResampler;
	ClFrequencyShifter	ShiftToNearZIF;
	ClFilter			FrontEndHalfBandFilter;
	ClSVScan            SVScanner;
	ClSVTrack           SVTracker;
	ClSVDataDecode      SVDataDecoder;

	////////////////////////////////
	// Main processing loop
	//
	double DeltaT       = 0;
	double TotalTime    = 0;
	double EchoInterval = 0;

	// Calculate derived parameters which depend on the OSR
	FsTargetHz                  = 1023000 * Param.OSR;										// 1023000 cps is the chip rate of the C / A on L1
	CorrelationWindowSamplesOSR = (unsigned)(1023 * Param.OSR * Param.CorrelationWindow_ms);
	
	TrackingBlockSize           = CorrelationWindowSamplesOSR;		// The number of data samples to use for each call to the tracking routine, this is important due to the regular trimming of the data
	TrackingBlockTrimSize       = 5 * TrackingBlockSize;			// The number of data samples to trim while tracking to keep the total memory usage low 

	// Configure the processing blocks
	FrontEndResampler.     initialize(Param.FsData, FsTargetHz);
	ShiftToNearZIF.        initialize(Param.Fc / (double)FsTargetHz);
	FrontEndHalfBandFilter.initialize(get_half_band_coeffs());

	SVScanner.             reset();
	SVScanner.             setDopplerRange(Param.DopplerLowHz, Param.DopplerHighHz, Param.DopplerStepHz, FsTargetHz);
	SVScanner.             setParameters  (Param.NumberOfCorrelationPhases, Param.OSR, Param.DetectionThreshold);

	processInputBuffer.resize(0);

	////
	// Read raw GPS data samples in non-overlapping blocks to allow streaming input data as well as to keep memory consumption down
	while (GPS_READ_STATUS_OK == read_gps_raw_data_samples(&x, Param, Param.InputDataBlockSize))
	{
		// Debug //
		// note that this number can be wrong for the final read, but that's okay
		DeltaT = (double)Param.InputDataBlockSize / (double)Param.FsData;
		if (Param.DataType == DT_8_BIT_COMPLEX || Param.DataType == DT_16_BIT_COMPLEX) {
			DeltaT = DeltaT / 2;
		}
		TotalTime    += DeltaT;
		EchoInterval += DeltaT;
		// End Debug //

		// Resample the data (to an integer multiple of the chip rate in this case)
		FrontEndResampler.resample(&ResamplerOutput, &x);
		x.resize(0);

		// Frequency shift to near-ZIF (offset by unknown-at-this-point Doppler) based on the expected centre frequency of the input
		ShiftToNearZIF.shift(&FrequencyShiftOutput, ResamplerOutput);
		ResamplerOutput.resize(0);

		// Filter some out-of-band noise
		FrontEndHalfBandFilter.filter(&SVDetectBlock, FrequencyShiftOutput);
		FrequencyShiftOutput.resize(0);

		// Buffer the data at the input to the main processing loop to account for any mismatch between input data read rate and processing rate
		// i.e. the buffer will fill during the slow SV search phase, then empty while the SV search has no active search (detected SVs, sleep SVs, etc)
		processInputBuffer.insert(processInputBuffer.end(), SVDetectBlock.begin(), SVDetectBlock.end());

		// Keep processing until the input buffer no longer contains enough samples to process a single correlation window
		while (processInputBuffer.size() >= CorrelationWindowSamplesOSR)
		{
			vcomplex_t y;

			y.resize(0);

			y.insert(y.begin(),      processInputBuffer.begin(), processInputBuffer.begin() + CorrelationWindowSamplesOSR);
			processInputBuffer.erase(processInputBuffer.begin(), processInputBuffer.begin() + CorrelationWindowSamplesOSR);

			// Intelligently decide which SVs to include in the next search
			updateSVSearchList(&SVIdx, y, Param.OSR);

			// Perform detection of SVs by scanning Doppler offset and time phase of the corresponding code correlation
			SVScanner.scanDoppler(SV, SVIdx, y, Param);

			// Debug message //
			
			for (unsigned n = 0; n < SVIdx.size(); n++)
			{
				unsigned CurrentSV;
				CurrentSV = SVIdx[n];

				if (SV[CurrentSV].f_Detected == false)
				{
					// Do nothing
				}
				else
				{
					fprintf(stderr, "Status: SV %2d DETECTED.\tDoppler offset: %+5.0lf Hz. Code phase: %8.3lf (%8.3lf). Detection metric: %10.6lf.\n", CurrentSV, SV[CurrentSV].NormalizedDoppler * FsTargetHz, (SV[CurrentSV].Tracking.SampleTiming.i_CodePhase + SV[CurrentSV].Tracking.SampleTiming.f_CodePhase), (double)(SV[CurrentSV].Tracking.SampleTiming.i_CodePhase + SV[CurrentSV].Tracking.SampleTiming.f_CodePhase) / (double)Param.OSR, SV[CurrentSV].DetectionMetric); fflush(stderr);
				}
			}
			
			// End Debug message //

			// Acquire and track the data stream from any SVs marked as detected
			// Note that we can reuse the samples used for the correlation since they are still in memory
			if (y.size() != TrackingBlockSize)
			{
				fprintf(stderr, "Error: The input data size to the tracking block for each SV must always be the same and equal to TrackingBlockSize\n");
				fflush(stderr);
				exit(1);
			}

			SVTracker.track(SV, y, Param.OSR, TrackingBlockTrimSize);
			
			// Decode GPS subframe data packets to determine the current ephemeris parameters (maximum of one subframe per call)
			SVDataDecoder.decode(SV);

			// Solve the GPS equations based on the ephemeris and time information of each SV
			solveForGPSPosition(SV, TrackingBlockTrimSize);
		}

		if (EchoInterval > 5.00)
		{
			EchoInterval -= 5.00;
			fprintf(stderr, "\nTime elapsed: %5.2lf s.\n\n", TotalTime); fflush(stderr);
		}
		// Break after a set maximum input data has been read
		// Debug: Uncomment to break the loop early without exhausting the data
		//if (TotalTime >= 40)
		//{
			// Do nothing
			// or not, comment/decomment as necessary
			//fprintf(stderr, "** WARNING: ** Breaking early.\n"); fflush(stderr);
			//break;
		//}
	}

	fprintf(stderr, "Successful exit.\n"); fflush(stderr);

	return(0);
}

void on_exit()
{
	// Debug //
	if (0)
	{
		write_binary_stream_to_file();
	}
	// End Debug //

	FILE *ft;
	ft = fopen("LatLonGPS_YAGPSR.kml", "a");
	if (ft != NULL)
	{
		fprintf(ft, "</Document>\n</kml>\n");
		fclose(ft);
	}
}

#ifdef _MSC_VER
BOOL WINAPI CtrlCHandler(DWORD fdwCtrlType)
{
	switch (fdwCtrlType)
	{
		// Handle the CTRL-C signal. 
	case CTRL_C_EVENT:
		on_exit();
		return TRUE;

	default:
		return FALSE;
	}

	return FALSE;
}
#else
void CtrlCHandler(int)
{
	on_exit();
	exit(2);
}
#endif

/*
* Code entry point
*/
int main(int argc, char **argv)
{
	ClGPSParameters YAGPSRParam;

	// Set up a handler for the Ctrl-C signal to allow the code to output meaningful results and debug data even when interrupted
#ifdef _MSC_VER
	if (SetConsoleCtrlHandler(CtrlCHandler, TRUE) == FALSE)
	{
		fprintf(stderr, "Error setting up signal handler. Exiting safely.\n"); fflush(stderr);
		exit(1);
	}
#else
	//struct sigaction act;
	//act.sa_handler = CtrlCHandler;
	//sigaction(SIGINT, &act, NULL);
#endif

	// Parse the command line for input parameters and update YAGPSRParam
	parse_cmdline(argc, argv, &YAGPSRParam);

	// Parse the parameter file associated with the input data file, if it exists
	parse_parameter_file(&YAGPSRParam);

	// Print the current parameter set to the console
	echo_current_parameters(YAGPSRParam);

	// 
	if ((YAGPSRParam.OSR % 4) != 0)
	{
		fprintf(stderr, "\nError: Only oversampling ratios which are an integer multiple of 4 are current supported. Exiting safely.\n"); fflush(stderr);
		exit(1);
	}

	// Set up the satellites
	for (int n = 1; n < TOTAL_NUMBER_OF_SATELLITES + 1; n++)
	{
		// Set the OSR and the SV ID
		SV[n].initialize(YAGPSRParam.OSR, n);
	}

	// Detection, acquisition, tracking, localization
	gps_receive(YAGPSRParam);

	// Clean up on exit (write out debug info and and the appropriate closing lines to the kml)
	on_exit();

	return(0);
}
