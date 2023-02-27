#pragma once

class ClFrequencyShifter {
public:
	ClFrequencyShifter();
	ClFrequencyShifter(const double &);
	ClFrequencyShifter(const double &, const double &);

	void initialize(const double &);
	void set_shift_frequency(const double &);
	void set_shift_frequency(const double &, const double &);
	
	void set_phase(const double &);
	void reset_phase();

	double get_phase();

	//void shift(vcomplex_t *const, const vdouble_t &x);
	template<typename T>
	void shift(vcomplex_t *const, const T &x);

private:
	// Shift is by normalized frequency, Fn
	double Fn;
	double phi;

protected:
};