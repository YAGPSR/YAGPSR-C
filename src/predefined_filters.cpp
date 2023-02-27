#include "headers.h"

vdouble_t get_half_band_coeffs()
{
	vdouble_t c;

	// 10-bit halfband filter
	c.push_back(1 / 511.0);
	c.push_back(0 / 511.0);
	c.push_back(-2 / 511.0);
	c.push_back(0 / 511.0);
	c.push_back(3 / 511.0);
	c.push_back(0 / 511.0);
	c.push_back(-6 / 511.0);
	c.push_back(0 / 511.0);
	c.push_back(10 / 511.0);
	c.push_back(0 / 511.0);
	c.push_back(16 / 511.0);
	c.push_back(0 / 511.0);
	c.push_back(27 / 511.0);
	c.push_back(0 / 511.0);
	c.push_back(51 / 511.0);
	c.push_back(0 / 511.0);
	c.push_back(62 / 511.0);
	c.push_back(56 / 511.0);
	c.push_back(62 / 511.0);
	c.push_back(0 / 511.0);
	c.push_back(51 / 511.0);
	c.push_back(0 / 511.0);
	c.push_back(27 / 511.0);
	c.push_back(0 / 511.0);
	c.push_back(16 / 511.0);
	c.push_back(0 / 511.0);
	c.push_back(10 / 511.0);
	c.push_back(0 / 511.0);
	c.push_back(-6 / 511.0);
	c.push_back(0 / 511.0);
	c.push_back(3 / 511.0);
	c.push_back(0 / 511.0);
	c.push_back(-2 / 511.0);
	c.push_back(0 / 511.0);
	c.push_back(1 / 511.0);

	return(c);
}