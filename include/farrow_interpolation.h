#pragma once

class ClFarrowFilter {
public:
	ClFarrowFilter();
	ClFarrowFilter(const int &);
	
	void initialize(const int &);

	//void interpolate(vdouble_t  *const, vdouble_t  const &, vdouble_t const &);
	//void interpolate(vcomplex_t *const, vcomplex_t const &, vdouble_t const &);
	template<typename T>
	void interpolate(std::vector<T> *const, std::vector<T> const &, vdouble_t const &);

	unsigned get_K();

private:
	void set_coefficients();

	//double custom_pow(const double &x, const int &n);			// This does not support general usage, and is strictly restricted to this class

	int       K;
	vdouble_t c;

protected:

};