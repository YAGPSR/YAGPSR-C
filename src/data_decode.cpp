//

#include "headers.h"

constexpr int NGPSPrembleBits = 8;

// Debug
void DebugEphemerisToFile(t_SpaceVehicle *const SV)
{
	// Create a new variable for simplicity
	t_DecodedData y = SV->Tracking.Decoded;

	// Echo the current ephemeris parameters to file
	FILE *fp;

	static bool f_tv = false;

	// Open for writing on first open (which removes any previous data in the file), and append thereafter
	if (f_tv == false)
	{
		fp = fopen("Ephemeris_SF5.txt", "w");
		f_tv = true;
	}
	else
	{
		fp = fopen("Ephemeris_SF5.txt", "a");
	}
	fprintf(fp, "SV ID: \t\t\t%u\n", SV->ID);
	fprintf(fp, "TOW: \t\t\t%u\n", y.TOW);
	fprintf(fp, "WeekNumber: \t%u\n", y.WeekNumber);
	fprintf(fp, "Accuracy: \t\t%u\n", y.Accuracy);
	fprintf(fp, "Health: \t\t%u\n", y.Health);
	fprintf(fp, "IODC: \t\t\t%u\n", y.IODC);
	fprintf(fp, "IODE_SF1: \t\t%u\n", y.IODE_SF1);
	fprintf(fp, "tOC: \t\t\t%u\n", y.tOC);

	fprintf(fp, "TGD: \t%20.10f\n", y.TGD);
	fprintf(fp, "af2: \t%20.10f\n", y.af2);
	fprintf(fp, "af1: \t%20.10f\n", y.af1);
	fprintf(fp, "af0: \t%20.10f\n", y.af0);

	fprintf(fp, "IODE_SF2: \t\t%u\n", y.IODE_SF2);

	fprintf(fp, "Crs: \t%20.10f\n", y.Crs);
	fprintf(fp, "Deltan:\t%20.10f\n", y.Deltan);
	fprintf(fp, "M0: \t%20.10f\n", y.M0);
	fprintf(fp, "Cuc: \t%20.10f\n", y.Cuc);
	fprintf(fp, "e: \t\t%20.10f\n", y.e);
	fprintf(fp, "Cus: \t%20.10f\n", y.Cus);
	fprintf(fp, "sqrtA: \t%20.10f\n", y.sqrtA);
	fprintf(fp, "toe: \t%20.10f\n", y.toe);

	fprintf(fp, "IODE_SF3: \t\t%u\n", y.IODE_SF3);

	fprintf(fp, "Cic: \t%20.10f\n", y.Cic);
	fprintf(fp, "Omega0:\t%20.10f\n", y.Omega0);
	fprintf(fp, "Cis: \t%20.10f\n", y.Cis);
	fprintf(fp, "I0: \t%20.10f\n", y.I0);
	fprintf(fp, "Crc: \t%20.10f\n", y.Crc);
	fprintf(fp, "omega: \t%20.10f\n", y.omega);
	fprintf(fp, "OmegaDot:\t%20.10f\n", y.OmegaDot);
	fprintf(fp, "IDOT: \t%20.10f\n", y.IDOT);
	fprintf(fp, "\n");
	fprintf(fp, "GPS_Time_TLM: \t%d\n\n", (int32_t)SV->Tracking.Decoded.GPS_Time_TLM);
	fprintf(fp, "x: \t\t\t\t%20.10f\n", SV->Tracking.Coord.x);
	fprintf(fp, "y: \t\t\t\t%20.10f\n", SV->Tracking.Coord.y);
	fprintf(fp, "z: \t\t\t\t%20.10f\n", SV->Tracking.Coord.z);
	fprintf(fp, "clockCorrect: \t%20.10f\n", SV->Tracking.Data.clockCorrect);
	fprintf(fp, "TLMCodePhase: \t%20.10f\n", SV->Tracking.Data.TLMCodePhase);
	fprintf(fp, "\n\n");

	fclose(fp);
}
// End Debug



// Nothing to initialize
ClSVDataDecode::ClSVDataDecode()
{

}

//
// Decode binary GPS data
//
// Search data for the TLM and HOW 30-bit sequences, verify veracity and use
// these to align and decode the data bits
//
// This function receives a maximum of one subframe per call
//

void ClSVDataDecode::decode(t_SpaceVehicle *const SVArray)
{
	for (int CurrentSVIndex = 1; CurrentSVIndex < TOTAL_NUMBER_OF_SATELLITES + 1; CurrentSVIndex++)
	{
		if (SVArray[CurrentSVIndex].f_Detected == true)
		{
			decodeSV(&(SVArray[CurrentSVIndex]));
		}

	}
}

// Return a list of all indices which match the preamble values, +ve for normal alignment and -ve for inverted alignment
void ClSVDataDecode::find_gps_preamble_align(vint_t *align_idx, const vint_t &data)
{
	align_idx->resize(0);
	align_idx->reserve(data.size() - 8);

	for (int n = 0; n < (int)data.size() - 8; n++)
	{
		int metric;

		metric = -1 * data[n] + 1 * data[n + 1] + 1 * data[n + 2] + 1 * data[n + 3] - 1 * data[n + 4] + 1 * data[n + 5] - 1 * data[n + 6] - 1 * data[n + 7];

		// Yes, these are the correct way around, negative metric is a positive alignment, and positive metric is an inverted alignment
		if (metric == -4)
		{
			align_idx->push_back(n);
		}
		if (metric == 4)
		{
			align_idx->push_back(-1 * n);
		}
	}
}


bool ClSVDataDecode::check_parity(vint_t *const data_bits, const vint_t &GPS_BinaryHard, const int &idx)
{
	bool   f_parity_pass;
	vint_t D;
	int    DS29, DS30;

	f_parity_pass = false;
	D.resize(30);

	DS29  = GPS_BinaryHard[idx - 2];
	DS30  = GPS_BinaryHard[idx - 1];

	D[24] = DS29 + GPS_BinaryHard[idx+0] + GPS_BinaryHard[idx+1] + GPS_BinaryHard[idx+2] + GPS_BinaryHard[idx+4] + GPS_BinaryHard[idx+5] + GPS_BinaryHard[idx+9] + GPS_BinaryHard[idx+10] + GPS_BinaryHard[idx+11] + GPS_BinaryHard[idx+12] + GPS_BinaryHard[idx+13] + GPS_BinaryHard[idx+16] + GPS_BinaryHard[idx+17] + GPS_BinaryHard[idx+19] + GPS_BinaryHard[idx+22];
	D[25] = DS30 + GPS_BinaryHard[idx+1] + GPS_BinaryHard[idx+2] + GPS_BinaryHard[idx+3] + GPS_BinaryHard[idx+5] + GPS_BinaryHard[idx+6] + GPS_BinaryHard[idx+10] + GPS_BinaryHard[idx+11] + GPS_BinaryHard[idx+12] + GPS_BinaryHard[idx+13] + GPS_BinaryHard[idx+14] + GPS_BinaryHard[idx+17] + GPS_BinaryHard[idx+18] + GPS_BinaryHard[idx+20] + GPS_BinaryHard[idx+23];
	D[26] = DS29 + GPS_BinaryHard[idx+0] + GPS_BinaryHard[idx+2] + GPS_BinaryHard[idx+3] + GPS_BinaryHard[idx+4] + GPS_BinaryHard[idx+6] + GPS_BinaryHard[idx+7] + GPS_BinaryHard[idx+11] + GPS_BinaryHard[idx+12] + GPS_BinaryHard[idx+13] + GPS_BinaryHard[idx+14] + GPS_BinaryHard[idx+15] + GPS_BinaryHard[idx+18] + GPS_BinaryHard[idx+19] + GPS_BinaryHard[idx+21];
	D[27] = DS30 + GPS_BinaryHard[idx+1] + GPS_BinaryHard[idx+3] + GPS_BinaryHard[idx+4] + GPS_BinaryHard[idx+5] + GPS_BinaryHard[idx+7] + GPS_BinaryHard[idx+8] + GPS_BinaryHard[idx+12] + GPS_BinaryHard[idx+13] + GPS_BinaryHard[idx+14] + GPS_BinaryHard[idx+15] + GPS_BinaryHard[idx+16] + GPS_BinaryHard[idx+19] + GPS_BinaryHard[idx+20] + GPS_BinaryHard[idx+22];
	D[28] = DS30 + DS30 + GPS_BinaryHard[idx+0] + GPS_BinaryHard[idx+2] + GPS_BinaryHard[idx+4] + GPS_BinaryHard[idx+5] + GPS_BinaryHard[idx+6] + GPS_BinaryHard[idx+8] + GPS_BinaryHard[idx+9] + GPS_BinaryHard[idx+13] + GPS_BinaryHard[idx+14] + GPS_BinaryHard[idx+15] + GPS_BinaryHard[idx+16] + GPS_BinaryHard[idx+17] + GPS_BinaryHard[idx+20] + GPS_BinaryHard[idx+21] + GPS_BinaryHard[idx+23];
	D[29] = DS30 + DS29 + GPS_BinaryHard[idx+2] + GPS_BinaryHard[idx+4] + GPS_BinaryHard[idx+5] + GPS_BinaryHard[idx+7] + GPS_BinaryHard[idx+8] + GPS_BinaryHard[idx+9] + GPS_BinaryHard[idx+10] + GPS_BinaryHard[idx+12] + GPS_BinaryHard[idx+14] + GPS_BinaryHard[idx+18] + GPS_BinaryHard[idx+21] + GPS_BinaryHard[idx+22] + GPS_BinaryHard[idx+23];
	
	// Check parity
	f_parity_pass = true;
	for (int n = 24; n < (int)D.size(); n++)
	{
		if ((D[n] % 2) != GPS_BinaryHard[idx + n])
		{
			f_parity_pass = false;
			break;
		}
	}

	data_bits->resize(0);
	if (f_parity_pass == true)
	{
		data_bits->reserve(24);

		for (int n = 0; n < 24; n++)
		{
			data_bits->push_back(binary_xor(GPS_BinaryHard[idx + n], DS30));
		}
	}

	return(f_parity_pass);
}


// Check for TLM and HOW alignment by verifying parity and the trailing zeros of the HOW
bool ClSVDataDecode::is_gps_frame_aligned(const vint_t &GPS_BinaryHard, const int &idx)
{
	bool   f_TLM_Parity_Check, f_HOW_Parity_Check, f_HOW_Zeros_Check, f_FrameAligned;
	vint_t discard;

	// TLM Parity
	f_TLM_Parity_Check = check_parity(&discard, GPS_BinaryHard, idx);

	// HOW Parity
	f_HOW_Parity_Check = check_parity(&discard, GPS_BinaryHard, idx + 30);

	// Check for HOW trailing zeros
	if (f_HOW_Parity_Check == true)
	{
		if (GPS_BinaryHard[idx + 30 + 28] | GPS_BinaryHard[idx + 30 + 29])
		{
			f_HOW_Zeros_Check = false;
		}
		else
		{
			f_HOW_Zeros_Check = true;
		}
	}
	else
	{
		f_HOW_Zeros_Check = false;
	}

	// Frame is detected only if all the above checks on the bit stream are true
	if (f_TLM_Parity_Check == true && f_HOW_Parity_Check == true && f_HOW_Zeros_Check == true)
	{
		f_FrameAligned = true;
	}
	else
	{
		f_FrameAligned = false;
	}

	return(f_FrameAligned);

}


void ClSVDataDecode::decodeSubframe(t_DecodedData *const Decoded, const vint_t *const data_word, const int &Subframe)
{

	constexpr double GPS_PI = 3.1415926535898;               // Use this value of pi for GPS calculations

	t_DecodedData *y;

	y = Decoded;

	int64_t TGDi, tOCi, af2i, af1i, af0i, Crsi, Deltani, M0i, Cuci, ei, Cusi, sqrtAi, Cici, Omega0i, Cisi, I0i, Crci, omegai, OmegaDoti, IDOTi;
	//int64_t SVPage;

	switch (Subframe)
	{
	case 1:
			//fprintf(stderr, "Subframe 1\n");

			y->WeekNumber = convert_bits_to_int(data_word[2].data(),     10, true);

			//fprintf(stderr, "Week number: %d\n", y->WeekNumber); fflush(stderr);

			y->Accuracy   = convert_bits_to_int(data_word[2].data() + 12, 4, true);
			y->Health     = convert_bits_to_int(data_word[2].data() + 16, 6, true);
			y->IODC       = convert_bits_to_int(data_word[2].data() + 22, 2, true) * (1<<8) + convert_bits_to_int(data_word[7].data(), 8, true);
			y->IODE_SF1   = y->IODC & 255;	// bitand(y->IODC, 255)
			TGDi          = convert_bits_to_int(data_word[6].data() + 16, 8, true);
			tOCi          = convert_bits_to_int(data_word[7].data() + 8, 16, true);
			af2i          = convert_bits_to_int(data_word[8].data(),      8, true);
			af1i          = convert_bits_to_int(data_word[8].data() + 8, 16, true);
			af0i          = convert_bits_to_int(data_word[9].data(),     22, true);

			// 2s complement and scaling
			y->tOC = (uint32_t)(tOCi * (1<<4));

			TGDi   = convert_2s_to_signed(TGDi, 8);
			y->TGD = (double)(TGDi) / (double)((int64_t)1 << 31);

			af2i   = convert_2s_to_signed(af2i, 8);
			y->af2 = af2i / (double)((int64_t)1 << 55);

			af1i  = convert_2s_to_signed(af1i, 16);
			y->af1 = af1i / (double)((int64_t)1 << 43);

			af0i   = convert_2s_to_signed(af0i, 22);
			y->af0 = af0i / (double)((int64_t)1 << 31);

			break;

	case 2:
			//fprintf(stderr, "Subframe 2\n");

			// Ephemeris parameters
			y->IODE_SF2 = convert_bits_to_int(data_word[2].data(),       8, true);
			Crsi        = convert_bits_to_int(data_word[2].data() +  8, 16, true);
			Deltani     = convert_bits_to_int(data_word[3].data(),      16, true);
			M0i         = convert_bits_to_int(data_word[3].data() + 16,  8, true) * ((int64_t)1 << 24) + convert_bits_to_int(data_word[4].data(), 24, true);;
			Cuci        = convert_bits_to_int(data_word[5].data(),      16, true);
			ei          = convert_bits_to_int(data_word[5].data() + 16,  8, true) * ((int64_t)1 << 24) + convert_bits_to_int(data_word[6].data(), 24, true);
			Cusi        = convert_bits_to_int(data_word[7].data(),      16, true);
			sqrtAi      = convert_bits_to_int(data_word[7].data() + 16,  8, true) * ((int64_t)1 << 24) + convert_bits_to_int(data_word[8].data(), 24, true);
			y->toe      = convert_bits_to_int(data_word[9].data(),      16, true);

			// 2s complement and scaling
			Crsi      = convert_2s_to_signed(Crsi, 16);
			y->Crs    = Crsi / (double)((int64_t)1 << 5);

			Deltani   = convert_2s_to_signed(Deltani, 16);
			y->Deltan = Deltani / (double)((int64_t)1 << 43);
			y->Deltan = y->Deltan * GPS_PI;                   // Transmitted as semi - circles / sec

			M0i       = convert_2s_to_signed(M0i, 32);
			y->M0     = M0i / (double)((int64_t)1 << 31);
			y->M0     = y->M0 * GPS_PI;                           // Transmitted as semi - circles

			Cuci      = convert_2s_to_signed(Cuci, 16);
			y->Cuc    = Cuci / (double)((int64_t)1 << 29);

			// Not 2s complement
			y->e      = ei / (double)((int64_t)1 << 33);

			Cusi      = convert_2s_to_signed(Cusi, 16);
			y->Cus    = Cusi / (double)((int64_t)1 << 29);

			// Not 2s complement
			y->sqrtA  = sqrtAi / (double)((int64_t)1 << 19);

			// Not 2s complement
			y->toe    = y->toe * ((int64_t)1 << 4);                            // Not a mistake, 2^4 multiplier

			break;

	case 3:
		//fprintf(stderr, "Subframe 3\n");

		// Ephemeris parameters
		Cici        = convert_bits_to_int(data_word[2].data(),      16, true);
		Omega0i     = convert_bits_to_int(data_word[2].data() + 16,  8, true) * ((int64_t)1 << 24) + convert_bits_to_int(data_word[3].data(), 24, true);
		Cisi        = convert_bits_to_int(data_word[4].data(),      16, true);
		I0i         = convert_bits_to_int(data_word[4].data() + 16,  8, true) * ((int64_t)1 << 24) + convert_bits_to_int(data_word[5].data(), 24, true);
		Crci        = convert_bits_to_int(data_word[6].data(),      16, true);
		omegai      = convert_bits_to_int(data_word[6].data() + 16,  8, true) * ((int64_t)1 << 24) + convert_bits_to_int(data_word[7].data(), 24, true);
		OmegaDoti   = convert_bits_to_int(data_word[8].data(),      24, true);
		y->IODE_SF3 = convert_bits_to_int(data_word[9].data(),       8, true);
		IDOTi       = convert_bits_to_int(data_word[9].data() + 8,  14, true);

		// 2s complement and scaling
		Cici        = convert_2s_to_signed(Cici, 16);
		y->Cic      = Cici / (double)((int64_t)1 << 29);

		Omega0i     = convert_2s_to_signed(Omega0i, 32);
		y->Omega0   = Omega0i / (double)((int64_t)1 << 31);
		y->Omega0   = y->Omega0 * GPS_PI;								// Transmitted as semi-circles
				    
		Cisi        = convert_2s_to_signed(Cisi, 16);
		y->Cis      = Cisi / (double)((int64_t)1 << 29);
				    
		I0i         = convert_2s_to_signed(I0i, 32);
		y->I0       = I0i / (double)((int64_t)1 << 31);
		y->I0       = y->I0 * GPS_PI;									// Transmitted as semi-circles
				    
		Crci        = convert_2s_to_signed(Crci, 16);
		y->Crc      = Crci / (double)((int64_t)1 << 5);
				    
		omegai      = convert_2s_to_signed(omegai, 32);
		y->omega    = omegai / (double)((int64_t)1 << 31);
		y->omega    = y->omega * GPS_PI;								// Transmitted as semi-circles

		OmegaDoti   = convert_2s_to_signed(OmegaDoti, 24);
		y->OmegaDot = OmegaDoti / (double)((int64_t)1 << 43);
		y->OmegaDot = y->OmegaDot * GPS_PI;								// Transmitted as semi-circles / sec

		IDOTi       = convert_2s_to_signed(IDOTi, 14);
		y->IDOT     = IDOTi / (double)((int64_t)1 << 43);
		y->IDOT     = y->IDOT * GPS_PI;									// Transmitted as semi-circles / sec
		break;

	case 4:
		//SVPage = convert_bits_to_int(data_word[2].data() + 2, 6, true);
		//fprintf(stderr, "Subframe 4, Satellite ID: %d\n", (int)SVPage);
		break;

	case 5:
		//fprintf(stderr, "Subframe 5, not used\n");
		break;
					

	default:
		fprintf(stderr, "Invalid or unsupported subframe type\n");
		y->WeekNumber = 0;
		y->Accuracy   = 0;
		y->Health     = 0;
		y->IODC       = 0;
		y->TGD        = 0;
		y->tOC        = 0;
		y->af2        = 0;
		y->af1        = 0;
		y->af0        = 0;
		break;
	}

}

void ClSVDataDecode::decodeSV(t_SpaceVehicle *const SV)
{
	// 62 is the minimum length of data that may contain two complete 30-bit words plus two leading bits (for parity check)
	constexpr int MinimumNumberOfBitsForTLMHOWDecode = 62;

	vint_t    BinaryHard;
	//vdouble_t BitCodePhase;

	// Make local copies of the data for ease of use and readability (very low overhead)
	if (SV->Tracking.Data.f_GPSBinaryInversion == true)
	{
		binary_invert(&BinaryHard, SV->Tracking.Data.BinaryHard);
	}
	else
	{
		BinaryHard = SV->Tracking.Data.BinaryHard;
	}

	//BitCodePhase = SV->Tracking.Data.BinaryCodePhase;

	// Where a valid subframe has not yet been detected, search for alignment
	if (SV->Tracking.Data.f_SubFrameAlignment == false)
	{
		vint_t InvertedBinary;
		vint_t align_candidate;
		int    align_idx;

		align_idx = 0;

		// Ensure there is enough data to contain two complete 30-bit words plus two leading bits (for parity check) at least once
		if (BinaryHard.size() < MinimumNumberOfBitsForTLMHOWDecode)
		{
			// Wait for more data before proceeding
			return;
		}

		binary_invert(&InvertedBinary, BinaryHard);

		// Preamble binary sequence alignment, if any
		find_gps_preamble_align(&align_candidate, BinaryHard);

		if (align_candidate.size() == 0)
		{
			// No tentative bit stream alignments detected
			// Trim the binary sequence since we know that there is no alignment detected
			erase_from_beginning(SV->Tracking.Data.BinaryHard,      SV->Tracking.Data.BinaryHard.size()      - (NGPSPrembleBits - 1));
			erase_from_beginning(SV->Tracking.Data.BinaryCodePhase, SV->Tracking.Data.BinaryCodePhase.size() - (NGPSPrembleBits - 1));
			return;
		}

		////
		// Use both TLM and HOW word parity to align bit stream based on the preamble candidate alignment value
		for (int n = 0; n < (int)align_candidate.size(); n++)
		{
			if ((int)BinaryHard.size() < std::abs(align_candidate[n]) + MinimumNumberOfBitsForTLMHOWDecode - 2)
			{
				// Insufficient data to contain a complete TLM and HOW for alignment, return to wait for more data
				// Trim the binary sequence since we know that there is no alignment detected at previous align_candidate values
				// but this - and subsequent - alignment candidate(s) have not yet been tested so leave all data associated with the current alignment candidate
				// Note that align_candidate[n] must be greater than 1 to detect a subframe, and thus we erase such that align_candidate[n] is the third element
				erase_from_beginning(SV->Tracking.Data.BinaryHard,      std::abs(align_candidate[n]) - 2);
				erase_from_beginning(SV->Tracking.Data.BinaryCodePhase, std::abs(align_candidate[n]) - 2);
				return;
				//break;
			}

			// Ensure two bits previous to the current bit are available for parity checking
			if (align_candidate[n] > 1)
			{
				SV->Tracking.Data.f_SubFrameAlignment = is_gps_frame_aligned(BinaryHard, align_candidate[n]);

				if (SV->Tracking.Data.f_SubFrameAlignment == true)
				{
					align_idx = std::abs(align_candidate[n]);

					// Note that alignment has been detected on a non-inverted binary stream and break to stop looking for alignment
					SV->Tracking.Data.f_GPSBinaryInversion = false;
					break;
				}
			}
			else if (align_candidate[n] < -1)
			{

				SV->Tracking.Data.f_SubFrameAlignment = is_gps_frame_aligned(InvertedBinary, std::abs(align_candidate[n]));

				if (SV->Tracking.Data.f_SubFrameAlignment == true)
				{
					align_idx = std::abs(align_candidate[n]);

					// Note that alignment has been detected on an inverted binary stream and break to stop looking for alignment
					SV->Tracking.Data.f_GPSBinaryInversion = true;
					break;
				}
			}
		}

		// Debug //
		if (SV->Tracking.Data.f_SubFrameAlignment == true)
		{
			fprintf(stderr, "Status: SV %2d Subframe Alignment FOUND at bit alignment index %d.\n", SV->ID, align_idx); fflush(stderr);
		}
		// End Debug //

		// Remove all (but two) bits previous to the detected TLM frame, since these
		// cannot be used (the two bits previous to the TLM are used as part of parity check and decoding)
		if (SV->Tracking.Data.f_SubFrameAlignment == true)
		{
			SV->Tracking.Data.f_SubFrameAlignment = true;
			SV->Tracking.Data.BinaryHard.erase     (SV->Tracking.Data.BinaryHard.begin(),      SV->Tracking.Data.BinaryHard.begin()      + (align_idx - 2));
			SV->Tracking.Data.BinaryCodePhase.erase(SV->Tracking.Data.BinaryCodePhase.begin(), SV->Tracking.Data.BinaryCodePhase.begin() + (align_idx - 2));

			SV->Tracking.Data.TLMDetectIdx = 2;// align_idx;
			SV->Tracking.Data.TLMCodePhase = SV->Tracking.Data.BinaryCodePhase[2]; //CodePhase[align_idx];		// ??

			SV->Tracking.Data.Subframe_idx = 2;
		}
		else
		{
			// Trim the binary sequence since we know that there is no alignment detected at any align_candidate values
			// At this point in the code, all alignments within BinaryHard have been fully tested (including TLM/HOW) so remove all but the final bits
			erase_from_beginning(SV->Tracking.Data.BinaryHard,      SV->Tracking.Data.BinaryHard.size()      - (NGPSPrembleBits - 1));
			erase_from_beginning(SV->Tracking.Data.BinaryCodePhase, SV->Tracking.Data.BinaryCodePhase.size() - (NGPSPrembleBits - 1));
			return;
		}

	}
	else
	{
		bool   f_TLM_Parity_Check, f_HOW_Parity_Check;
		vint_t data_word_TLM, data_word_HOW;
		vint_t data_word[10];								// Each subframe consists of 10 data words
		int    Subframe;									// The subframe number of the subframe being decoded

		// Wait for a complete subframe, plus the first two 30-bit words of the
		// next frame to be available before decoding
		// The data has already been aligned such that the TLM begins on the
		// third bit (bits one and two also being required for decoding)
		if (BinaryHard.size() < 363)
		{
			return;
		}

		// Verify that the data is still being tracked and received correctly by verifying TLM and HOW parity check bits
		// Drop the SV immediately where a parity error is found (this is not strictly necessary, but since this information
		// is used to calculate the ephemeris and there is no current check, it is safer to drop the SV for now. A better
		// solution to be implemented once the entire codebase is working acceptably)
		f_TLM_Parity_Check = check_parity(&data_word_TLM, BinaryHard, SV->Tracking.Data.Subframe_idx);

		if (f_TLM_Parity_Check == false)
		{
			fprintf(stderr, "Status: SV %2d PARITY ERROR. (TLM).\n", SV->ID); fflush(stderr);

			SV->reset_tracking();

			//erase_from_beginning(SV->Tracking.Data.BinaryHard,      300);
			//erase_from_beginning(SV->Tracking.Data.BinaryCodePhase, 300);
			//SV->Tracking.Data.Subframe_idx = 2;
			return;
		}

		if ((int)BinaryHard.size() < SV->Tracking.Data.Subframe_idx + 30 + 29)
		{
			return;
		}

		f_HOW_Parity_Check = check_parity(&data_word_HOW, BinaryHard, SV->Tracking.Data.Subframe_idx + 30);

		if (f_HOW_Parity_Check == false)
		{
			fprintf(stderr, "Status: SV %2d PARITY ERROR. (HOW).\n", SV->ID); fflush(stderr);

			SV->reset_tracking();

			//erase_from_beginning(SV->Tracking.Data.BinaryHard,      300);
			//erase_from_beginning(SV->Tracking.Data.BinaryCodePhase, 300);
			//SV->Tracking.Data.Subframe_idx = 2;
			return;
		}

		Subframe = 4 * data_word_HOW[19] + 2 * data_word_HOW[20] + data_word_HOW[21];

		// Read each of the data words of this subframe
		for (int n = 2; n < 10; n++)
		{
			bool f_Parity_Check;

			if ((int)BinaryHard.size() < SV->Tracking.Data.Subframe_idx + 30 * n + 29)
			{
				return;
			}

			f_Parity_Check = check_parity(&data_word[n], BinaryHard, SV->Tracking.Data.Subframe_idx + 30 * n);
			if (f_Parity_Check == false)
			{
				fprintf(stderr, "Status: SV %2d PARITY ERROR. Data word %d, subframe %d.\n", SV->ID, n + 1, Subframe); fflush(stderr);

				SV->reset_tracking();

				//erase_from_beginning(SV->Tracking.Data.BinaryHard,      300);
				//erase_from_beginning(SV->Tracking.Data.BinaryCodePhase, 300);
				//SV->Tracking.Data.Subframe_idx = 2;
				return;
			}
		}

		//
		// Ensure that the subsequent frame is also correctly aligned
		// and declare a loss of satellite tracking where it is not
		//
		if ((int)BinaryHard.size() < SV->Tracking.Data.Subframe_idx + 300 + 59)
		{
			return;
		}

		bool f_aligned = is_gps_frame_aligned(BinaryHard, SV->Tracking.Data.Subframe_idx + 300);
		if (f_aligned == false)
		{
			SV->reset_tracking();
			fprintf(stderr, "Status: SV %2d TLM ALIGNMENT LOST.\n", SV->ID); fflush(stderr);
			return;
		}

		// Store the code phase corresponding to the edge of the TLM of the
		// current subframe, later used for positioning
		// Note that this corresponds to the code phase at the beginning of the
		// subsequent subframe because of the call to is_gps_frame_aligned
		// which aligns on the next subframe boundary
		//SV->Tracking.Data.TLMCodePhase = CodePhase[SV->Tracking.Data.Subframe_idx - 1];
		SV->Tracking.Data.TLMCodePhase = SV->Tracking.Data.BinaryCodePhase[SV->Tracking.Data.Subframe_idx + 300];

		// Discard decoded data, and keep code phase values aligned
		erase_from_beginning(SV->Tracking.Data.BinaryHard,      300);
		erase_from_beginning(SV->Tracking.Data.BinaryCodePhase, 300);
		SV->Tracking.Data.Subframe_idx = 2;

		// Interpret the data bits
		int TOW_HOW = 0;
		for (int n = 0; n < 17; n++)
		{
			TOW_HOW += data_word_HOW[n] * (1 << (16 - n));
		}
		SV->Tracking.Decoded.TOW = 4 * TOW_HOW;

		decodeSubframe(&(SV->Tracking.Decoded), data_word, Subframe);

		// f_isValid indicates that Ephemeris data is available for this SV and that the SV is "healthy"
		//if (Subframe == 2 || SV->Tracking.Data.f_isValid == true)
		//{
			fprintf(stderr, "INFORMATION: SV %2d Subframe %d. IODE Values: %d, %d, %d.\n", SV->ID, Subframe, SV->Tracking.Decoded.IODE_SF1, SV->Tracking.Decoded.IODE_SF2, SV->Tracking.Decoded.IODE_SF3); fflush(stderr);

			// Test frame validity by comparing IODE values from each of the subframes 1, 2, 3
			if (SV->Tracking.Decoded.IODE_SF1 == SV->Tracking.Decoded.IODE_SF2 && SV->Tracking.Decoded.IODE_SF2 == SV->Tracking.Decoded.IODE_SF3 && SV->Tracking.Decoded.IODE_SF2 != 0)
			{
				if (SV->Tracking.Decoded.Health == 0)
				{
					SV->Tracking.Data.f_isValid = true;
				}
				else
				{
					// Exclude satellites marked as unhealthy from being considered for positioning
					SV->Tracking.Data.f_isValid = false;
				}
			}
			else
			{
				// Data set cutover, new data must be collected
				SV->Tracking.Data.f_isValid = false;
			}

			if (SV->Tracking.Data.f_isValid)
			{
				// If valid, determine the current Ephemeris for the satellite
				SV->Tracking.Decoded.GPS_Time     = (int64_t)(1024 + SV->Tracking.Decoded.WeekNumber) * 604800 + (int64_t)SV->Tracking.Decoded.TOW + (int64_t)SV->Tracking.Decoded.TOW / 2;		// Note integer arithmetic, but TOW is guaranteed a multiple of 6
				SV->Tracking.Decoded.GPS_Time_TLM = SV->Tracking.Decoded.GPS_Time;       // GPS_Time is the time at the beginning of the subsequent subframe, note that the code phase measurement is already aligned to this boundary (see where TLMCodePhase is set for full details); corresponds to the TLM at the time at which the satellite position will be calculated

				calculateEphemeris(SV, SV->Tracking.Decoded.GPS_Time_TLM);

				//SV->Tracking.Data.TLMCodePhase = BitCodePhase[SV->Tracking.Data.Subframe_idx];
				SV->Tracking.Data.TLMSubframe  = Subframe;

				SV->Tracking.Data.f_satellitePositionCurrent = true;

				// Debug 
				DebugEphemerisToFile(SV);
				// End Debug
			}

		//}
	}

}


/*
* Calculate the ephemeris
*/
void ClSVDataDecode::calculateEphemeris(t_SpaceVehicle *const SV, const int64_t &RefTime)
{
	// Constants specified for use with GPS(official WGS - 84 specified values)
	constexpr double c          = 2.99792458e8;							// Speed of light in m / s
	constexpr double mu         = 3.986005e14;							// Earth's universal gravitational parameter in m^3/s^2
	constexpr double OmegaDot_e = 7.2921151467e-5;						// Earth's rotation rate in rad/s
	constexpr double GPS_PI     = 3.1415926535898;						// Use this value of pi for GPS calculations

	double Delta_t_SV_L1, ik, Omegak, xdash, ydash;

	// Create a new variable for simplicity
	t_DecodedData Data = SV->Tracking.Decoded;

	// Time Correction
	int64_t ttoc64;
	ttoc64 = RefTime - Data.tOC;
	while (ttoc64 > 302400)
	{
		ttoc64 = ttoc64 - 604800;
	}

	while (ttoc64 < -302400)
	{
		ttoc64 = ttoc64 + 604800;
	}

	int32_t ttoc = (int32_t)ttoc64;

	// Satellite code phase offset, p40, SPS Signal Specification 1995
	// Adjusting the time for satellite clock biases, relativistic effects, etc
	double Delta_tr = 0;

	// Run the loop thrice since the relativistic offset is calculated based on
	// parameters calculated based on it = D
	for (int rcdx = 0; rcdx < 3; rcdx++)
	{
		Delta_t_SV_L1 = Data.af0 + Data.af1 * ttoc + Data.af2 * ttoc * ttoc + Delta_tr - Data.TGD;

		// The phase delay of the antenna means that actual transmission of the
		// GPS signal occurs slightly later than the GPS time, as given in the
		// subframe header, and thus the time index at which we calculate
		// satellite position must be correspondingly adjusted to be later than
		// the time transmitted by the subframe
		double GPS_Time_Corrected = RefTime + Delta_t_SV_L1;

		// Calculation of ephemeris based on Table 2 - 15, SPS Signal Specification 1995

		// Semi - major axis
		double A = (Data.sqrtA) * (Data.sqrtA);

		// Computed mean motion - rad / sec
		double n0 = sqrt(mu / (A * A * A));

		// Time from ephemeris reference epoch
		double tk = GPS_Time_Corrected - Data.toe;

		while (tk > 302400)
		{
			tk = tk - 604800;
		}
		while (tk < -302400)
		{
			tk = tk + 604800;
		}

		// Corrected mean motion - rad / sec
		double n = n0 + Data.Deltan;

		// Mean anomaly
		double Mk = Data.M0 + n * tk;

		// Eccentric anomaly
		// Solve for Ek, iteratively
		int nbEk = 32;
		double Ek = 0;
		double EkC[3];
		for (int ibk = 0; ibk < nbEk; ibk++)
		{
			double Met1_0, Met1_1, Met1_2;

			EkC[0] = Ek + 1.0 / (double)((int64_t)1 << ibk); //2 ^ -ibk;
			EkC[1] = Ek;
			EkC[2] = Ek - 1.0 / (double)((int64_t)1 << ibk); // 2 ^ -ibk;

			Met1_0 = Mk - (GPS_PI * EkC[0] - Data.e * sin(GPS_PI * EkC[0]));
			Met1_1 = Mk - (GPS_PI * EkC[1] - Data.e * sin(GPS_PI * EkC[1]));
			Met1_2 = Mk - (GPS_PI * EkC[2] - Data.e * sin(GPS_PI * EkC[2]));

			if ((fabs(Met1_0) < fabs(Met1_1)) && (fabs(Met1_0) < fabs(Met1_2)))
			{
				Ek = EkC[0];
			}
			else if ((fabs(Met1_1) < fabs(Met1_0)) && (fabs(Met1_1) < fabs(Met1_2)))
			{
				Ek = EkC[1];
			}
			else
			{
				Ek = EkC[2];
			}

		}

		Ek = Ek * GPS_PI;

		// True anolamy
		double nuk_Num = (sqrt(1 - Data.e * Data.e) * sin(Ek));
		double nuk_Den = (cos(Ek) - Data.e);
		double nuk = atan2(nuk_Num, nuk_Den);

		//
		double Phi_k = nuk + Data.omega;

		// Corrected argument of latitude
		double uk = Phi_k + Data.Cuc * cos(2 * Phi_k) + Data.Cus * sin(2 * Phi_k);

		// Radial distance
		double rk = A * (1 - Data.e * cos(Ek)) + Data.Crc * cos(2 * Phi_k) + Data.Crs * sin(2 * Phi_k);

		// Inclination
		ik = Data.I0 + Data.IDOT * tk + Data.Cic * cos(2 * Phi_k) + Data.Cis * sin(2 * Phi_k);

		//
		Omegak = Data.Omega0 + (Data.OmegaDot - OmegaDot_e) * tk - OmegaDot_e * Data.toe;

		// Co-ordinates
		xdash = rk * cos(uk);
		ydash = rk * sin(uk);

		// Relativistic correction term
		double F = -2 * sqrt(mu) / (c * c);
		Delta_tr = F * Data.e * Data.sqrtA * sin(Ek);

	}

	SV->Tracking.Coord.x = xdash * cos(Omegak) - ydash * cos(ik) * sin(Omegak);
	SV->Tracking.Coord.y = xdash * sin(Omegak) + ydash * cos(ik) * cos(Omegak);
	SV->Tracking.Coord.z = ydash * sin(ik);

	SV->Tracking.Data.clockCorrect = Delta_t_SV_L1;
}

