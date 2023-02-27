#pragma once

class ClResampler {
public:
	ClResampler();
	ClResampler(const double &, const double &);

	void initialize(const double &, const double &);
	void reset();

	//template<typename T>
	//int resample(T *const, T *const);
	//int resample(vdouble_t *const, vdouble_t *const);
	int resample(vcomplex_t *const, vcomplex_t *const);

	//int self_test();

private:
	int        K;
	double     FsTarget;
	double     FsData;
	double     Delta_mu;
	double     current_rsidx;
	//vdouble_t data;
	vcomplex_t data;

	ClFarrowFilter FarrowFilter;

protected:
};