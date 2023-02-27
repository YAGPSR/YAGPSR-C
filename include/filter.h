#pragma once

class ClFilter {
public:
	ClFilter();
	ClFilter(const vdouble_t &);

	void initialize(const vdouble_t &);
	void reset_state();

	void filter(vdouble_t *const,  const vdouble_t &);
	void filter(vcomplex_t *const, const vcomplex_t &);

	int self_test();

private:
	vdouble_t  c;			// Real coefficients only, complex coefficients not supported
	vdouble_t  state;
	vdouble_t  state_j;		// Imaginary component of the state, for use with complex input filters
	vcomplex_t cmplx_state;

	void set_coefficients_and_reset(const vdouble_t &);

protected:
};