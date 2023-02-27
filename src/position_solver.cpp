#include "headers.h"

void   gn_solve_gps_equations(double *const, double *const, double *const, double *const, const vdouble_t &, const vdouble_t &, const vdouble_t &, const vdouble_t &);
void   gps_residual(t_RealMatrix *const, const double &, const double &, const double &, const double &, const vdouble_t &, const vdouble_t &, const vdouble_t &, const vdouble_t &);
double fjacob(const int &, const int &, const double &, const double &, const double &, const double &, const vdouble_t &, const vdouble_t &, const vdouble_t &, const vdouble_t &);
void   ecef2lla_wgs84(double *const, double *const, double *const, const vdouble_t &);


bool is_healthy(const t_SpaceVehicle &SV)
{
	// Bit AND the MSB of the six-bit field Health
	if ((SV.Tracking.Decoded.Health & (1 << 5)) == 0)
		return(true);
	else
		return(false);
}

//
//
//
void solveForGPSPosition(t_SpaceVehicle *const SVArray, const size_t &TrackingBlockTrimSize)
{
	vdouble_t x, y, z;
	vdouble_t codePhase, clockCorrect, clockPeriod, deltaCodePhase;
	vdouble_t pos_dt;
	double    minCodePhase;

	vint_t tv_GPSTime, tv_Subframe;

	minCodePhase = std::numeric_limits<double>::max();

	// Create an ordered list of all valid satellite co-ordinates and their respective timing data
	for (int n = 1; n < TOTAL_NUMBER_OF_SATELLITES + 1; n++)
	{
		if (SVArray[n].Tracking.Data.f_isValid && SVArray[n].Tracking.Data.f_satellitePositionCurrent && is_healthy(SVArray[n]))
		{
			x.           push_back(SVArray[n].Tracking.Coord.x);
			y.           push_back(SVArray[n].Tracking.Coord.y);
			z.           push_back(SVArray[n].Tracking.Coord.z);
			codePhase.   push_back(SVArray[n].Tracking.Data.TLMCodePhase);
			clockCorrect.push_back(SVArray[n].Tracking.Data.clockCorrect);
			clockPeriod. push_back(SVArray[n].Tracking.SampleTiming.ClockPeriod);
			
			if (SVArray[n].Tracking.Data.TLMCodePhase < minCodePhase || codePhase.size() == 1)
			{
				minCodePhase = SVArray[n].Tracking.Data.TLMCodePhase;
			}

			tv_GPSTime.push_back((int32_t)SVArray[n].Tracking.Decoded.GPS_Time_TLM);
			tv_Subframe.push_back(SVArray[n].Tracking.Data.TLMSubframe);
		}
	}

	// Do not attempt to find a GPS position with fewer than 4 satellites
	if ((int)x.size() < 4)
	{
		return;
	}

	// Correct for code phase discontinuities introduced when adjusted the code phase value to account for trimming the input data vector during SV tracking
	deltaCodePhase.reserve(x.size());
	for (size_t m = 0; m < x.size(); m++)
	{
		double MAX_PHASE_DISCONTINUITY = (double)TrackingBlockTrimSize;

		double dCP = codePhase[m] - minCodePhase;
		while (dCP > MAX_PHASE_DISCONTINUITY / 2.0)
		{
			dCP = dCP - MAX_PHASE_DISCONTINUITY;
		}
		while (dCP < -1 * MAX_PHASE_DISCONTINUITY / 2.0)
		{
			dCP = dCP + MAX_PHASE_DISCONTINUITY;
		}
	
		deltaCodePhase.push_back(dCP);
	}

	for (size_t m = 0; m < x.size(); m++)
	{
		// Since we do not have a local reference time, adjust the times to be relative to each other, with a 70ms offset
		// This is a rough starting point for iteration. Note: these times are NOT CORRECT
		//pos_dt.push_back(70e-3 + (codePhase[m] - minCodePhase) / (clockPeriod[m] * 1e3) + clockCorrect[m]);
		pos_dt.push_back(70e-3 + deltaCodePhase[m] / (clockPeriod[m] * 1e3) + clockCorrect[m]);
	}
	
	// Debug //
	// Write out the (x, y, z, dt) quadruples to a file to test in MATLAB
	FILE *fp;

	// Open for writing on first open, and append thereafter
	static int32_t prevTime = 0;
	static bool f_tv = false;

	if (x.size() > 0)
	{
		if (prevTime != (int32_t)tv_GPSTime[0])
		{
			FILE *ft;

			if (f_tv == false)
			{
				fp   = fopen("xyzdt.txt", "w");
				f_tv = true;

				ft = fopen("LatLonGPS_YAGPSR.kml", "w");
				fprintf(ft, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<kml xmlns=\"http://earth.google.com/kml/2.2\">\n<Document>\n  <name>GPS Data Points</name>\n  <Style><PolyStyle><fill>0</fill><outline>1</outline></PolyStyle></Style>\n<Style id=\"gpspin\">  <IconStyle>  <Icon>  <href>http://maps.google.com/mapfiles/kml/pushpin/pink-pushpin.png</href>  </Icon>  </IconStyle>  </Style>\n");
				fclose(ft);
			}
			else
			{
				fp = fopen("xyzdt.txt", "a");
			}

			for (size_t m = 0; m < x.size(); m++)
			{
				fprintf(fp, "%d\t%d\t%20.10f\t%20.10f\t%20.10f\t%20.10f\t%20.10f\n", (int)(m + 1), tv_GPSTime[m], x[m], y[m], z[m], pos_dt[m] - 70e-3, codePhase[m]);
			}
			fclose(fp);

			// End Debug //

			// Solve the GPS equations using an iterative Gauss-Newton approach
			double local_x, local_y, local_z;
			double time_error = 0;
			for (int itn = 0; itn < 10; itn++)
			{
				vdouble_t ex, ey;
				ex.resize(0); ex.resize(x.size());
				ey.resize(0); ey.resize(y.size());

				for (size_t m = 0; m < x.size(); m++)
				{
					constexpr double OmegaDot_e = 7.2921151467e-5;               // Earth's rotation rate in rad/s

					pos_dt[m] = pos_dt[m] - time_error;

					// Add a correction for earth rotation to the position
					// Note that since we don't know the actual propagation time, this is initially an estimate, but should still provide good results
					// Iterating once we have a better estimate for t (by solving the GPS equations) will improve the estimate further
					ex[m] = x[m] + OmegaDot_e * y[m] * pos_dt[m];
					ey[m] = y[m] - OmegaDot_e * x[m] * pos_dt[m];
				}

				gn_solve_gps_equations(&local_x, &local_y, &local_z, &time_error, ex, ey, z, pos_dt);
			}

			vdouble_t ecef;
			ecef.reserve(3);
			ecef.push_back(local_x);
			ecef.push_back(local_y);
			ecef.push_back(local_z);

			double Latitude, Longitude, Altitlude;

			// Perform a simple validity test on the co-ordinates (i.e. they are closer to the centre of the earth than the satellites)
			double deviceDistanceFromCentre, minSatelliteDistanceFromCentre;
			deviceDistanceFromCentre       = sqrt(local_x * local_x + local_y * local_y + local_z * local_z);
			minSatelliteDistanceFromCentre = std::numeric_limits<double>::max();
			for (size_t n = 0; n < x.size(); n++)
			{
				double satelliteDistanceFromCentre;
				satelliteDistanceFromCentre = sqrt(x[n] * x[n] + y[n] * y[n] + z[n] * z[n]);
				if (satelliteDistanceFromCentre < minSatelliteDistanceFromCentre)
				{
					minSatelliteDistanceFromCentre = satelliteDistanceFromCentre;
				}
			}

			if (deviceDistanceFromCentre > minSatelliteDistanceFromCentre)
			{
				fprintf(stderr, "Warning: Solution to GPS equations yields invalid position (outside satellite orbit). This could be simply convergence to the wrong solution. Not writing this position to file.\n");
				fflush(stderr);
			}
			else
			{
				// Convert from XYZ to longitude and latitude using the WGS84 model
				ecef2lla_wgs84(&Latitude, &Longitude, &Altitlude, ecef);

				ft = fopen("LatLonGPS_YAGPSR.kml", "a");
				fprintf(ft, "<Placemark><styleUrl>#gpspin</styleUrl><Point><coordinates>%12.9f,%12.9f</coordinates></Point></Placemark>\n", Longitude, Latitude);
				fclose(ft);
			}
		}
		prevTime = (int32_t)tv_GPSTime[0];
	}
}

//
void gn_solve_gps_equations(double *const x, double *const y, double *const z, double *const t, const vdouble_t &A, const vdouble_t &B, const vdouble_t &C, const vdouble_t &D2)
{
	constexpr int32_t MAXITERATIONS = 1000;

	int32_t iteration_count = 0;

	t_RealMatrix f(A.size(), 1);
	t_RealMatrix J(A.size(), 4);

	(*x) = 0.0;
	(*y) = 0.0;
	(*z) = 0.0;
	(*t) = 0.0;

	vdouble_t D;
	for (size_t n = 0; n < D2.size(); n++)
		D.push_back(D2[n] - 70e-3);

	while (1)
	{
		t_RealMatrix JT, JTJ, JTJI, JTfM;
		vdouble_t    R, JTf;

		gps_residual(&f, *x, *y, *z, *t, A, B, C, D);

		for (int k = 0; k < (int)A.size(); k++)
		{
			for (int32_t ell = 0; ell < 4; ell++)
			{
				double jcb;

				jcb = fjacob(k, ell, (*x), (*y), (*z), (*t), A, B, C, D);
				J.set(k, ell, jcb);
			}
		}

		JT   = J.Transpose();
		JTJ  = JT * J;
		JTfM = JT * f;
		JTf  = JTfM.data;

		R    = JTJ.SolvePositiveDefinite(JTf);

		if (R.size() == 0)
		{
			// If the matrix can't be inverted then we cannot solve the GPS equations so return 
			// Consider using a different version of lambda in the Levenberg–Marquardt algorithm if this happens regularly
			fprintf(stderr, "Warning: Unable to solve the GPS equations. Consider increasing the value of lambda in the Levenberg Marquardt algorithm if this happens frequently\n");
			fflush(stderr);
			return;
		}

		(*x) += -0.5 * R[0];
		(*y) += -0.5 * R[1];
		(*z) += -0.5 * R[2];
		(*t) += -0.5 * R[3];

		iteration_count++;

		if (iteration_count > MAXITERATIONS)
		{
			break;
		}
	}
}


// x, y, z and t are the variables to be optimized, A, B, C and D are the
// coefficients which define the equations
void gps_residual(t_RealMatrix *const r, const double &x, const double &y, const double &z, const double &t, const vdouble_t &A, const vdouble_t &B, const vdouble_t &C, const vdouble_t &D)
{
	constexpr double c = c_in_ms;

	if ((*r).data.size() < A.size())
	{
		fprintf(stderr, "Error: Mismatched input/output variable size in function gps_residual. This should never happen. Exiting safely."); 
		fflush(stderr);
		exit(1);
	}

	for (size_t n = 0; n < A.size(); n++)
	{
		(*r).data[n] = (x - A[n]) * (x - A[n]) + (y - B[n]) * (y - B[n]) + (z - C[n]) * (z - C[n]) - (c * (t - D[n])) * (c * (t - D[n]));
	}
}


// The Jacobian for the Gauss-Newton iterations which solve the GPS equations
double fjacob(const int &k, const int &ell, const double &x, const double &y, const double &z, const double &t, const vdouble_t &A, const vdouble_t &B, const vdouble_t &C, const vdouble_t &D)
{
	constexpr double c = c_in_ms;

	double jcb;

	switch (ell)
	{
	case 0:
		jcb = 2 * (x - A[k]);
		break;
	case 1:
		jcb = 2 * (y - B[k]);
		break;
	case 2:
		jcb = 2 * (z - C[k]);
		break;
	case 3:
		jcb = -2 * c * c * (t - D[k]);
		break;
	default:
		fprintf(stderr, "Invalid parameter to Jacobian element calculation. Exiting safely.\n"); fflush(stderr);
		exit(1);
		break;
	}

	return(jcb);
}


//
// Convert from ECEF to LLA
// "Understanding GPS Principles and Applications"
// Section 2.2.3 - World Geodetic System
//

void ecef2lla_wgs84(double *const Latitude, double *const Longitude, double *const Altitlude, const vdouble_t &ecef)
{
	// Constants for WGS-84
	constexpr double a   = 6378137;							// Semi - major axis, in m(mean radius of the earth)
	constexpr double b   = 6356752.3142;					// Semi - minor axis, in m
	constexpr double e2  = 0.00669437999014;				// e ^ 2, WGS - 84 value, not equal to the GRS - 80 value
	constexpr double ed2 = 0.00673949674228;				// e'^2, WGS-8 value

	double x, y, z, lambda, p, tu;
	double c2u, s2u, tp, phi, s2p, c2p;
	double N, h;

	x = ecef[0];
	y = ecef[1];
	z = ecef[2];

	// Geodetic longitude
	if (x > 0)
	{
		lambda = atan2(y, x);
	}
	else if (y >= 0)
	{
		lambda = atan2(y, x) + PI;
	}
	else
	{
		lambda = atan2(y, x) - PI;
	}

	p = sqrt(x*x + y*y);

	tu = (z/p) * (a/b);

	// Iterate until convergence of tu
	for (int32_t n = 0; n < 1000; n++)
	{
		c2u = 1.0 / (1 + tu*tu);
		s2u = 1.0 - c2u;
		tp = (z + ed2*b*s2u*sqrt(s2u)) / (p - e2*a*c2u*sqrt(c2u));
		tu = (b/a) * tp;
	}

	phi = atan2((z + ed2*b*s2u*sqrt(s2u)), (p - e2*a*c2u*sqrt(c2u)));
	s2p = sin(phi);
	c2p = cos(phi);

	N = a / sqrt(1 - e2*s2p);
	h = (p / c2p) - N;

	// Latitude(N / S), Longitude(E / W), Altitude
	(*Latitude)  = phi * (180.0 / PI);
	(*Longitude) = lambda * (180.0 / PI);
	(*Altitlude) = h;
}