#include "headers.h"

ClFrequencyShifter::ClFrequencyShifter()
{
	set_shift_frequency(0.0);
	reset_phase();
}

ClFrequencyShifter::ClFrequencyShifter(const double &Fshift)
{
	set_shift_frequency(Fshift);
	reset_phase();
}

ClFrequencyShifter::ClFrequencyShifter(const double &Fshift, const double &phase)
{
	set_shift_frequency(Fshift);
	set_phase(phase);
}

void ClFrequencyShifter::initialize(const double &Fshift)
{
	set_shift_frequency(Fshift);
	reset_phase();
}

void ClFrequencyShifter::set_shift_frequency(const double &Fshift)
{
	Fn = Fshift;
	reset_phase();
}

void ClFrequencyShifter::set_shift_frequency(const double &Fshift, const double &phase)
{
	Fn = Fshift;
	set_phase(phase);
}

void ClFrequencyShifter::reset_phase()
{
	phi = 0.0;
}

void ClFrequencyShifter::set_phase(const double &phase)
{
	phi = phase;
}

double ClFrequencyShifter::get_phase()
{
	return(phi);
}

template void ClFrequencyShifter::shift(vcomplex_t *const, const vdouble_t  &);
template void ClFrequencyShifter::shift(vcomplex_t *const, const vcomplex_t &);
template<typename T>
void ClFrequencyShifter::shift(vcomplex_t *const y, const T &x)
{
	y->resize(0);
	y->reserve(x.size());

	for (unsigned n = 0; n < x.size(); n++)
	{
		// Left-shift
		y->push_back(x[n] * std::complex<double>(cos(2 * PI * phi), -1 * sin(2 * PI * phi)));
		phi += Fn;

		// Restrict the angle to lie between -2*PI and +2*PI (i.e. between -1 and +1)
		if (phi > 1)
		{
			phi--;
		}
		else if (phi < -1)
		{
			phi++;
		}
	}

}
