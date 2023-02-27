#ifndef MISCH
#define MISCH

extern int convert_to_byte_stream(unsigned char *const, const unsigned char *const, const unsigned int);
extern int convert_to_bit_stream(unsigned char *const, const unsigned char *const, const unsigned int);

extern int convert_bits_to_int(const int *, const int &, bool);
extern int convert_2s_to_signed(const int &, const int &);
extern int64_t convert_2s_to_signed(const int64_t &, const int &);

extern int binary_xor(const int &, const int &);

extern void nearest_neighbour_interpolate(std::vector<std::complex<double>> *const, std::vector<std::complex<double>> const &, vdouble_t const &);

/*
	Erase N elements from the beginning of std::vector type x
	This function exists because I've had to look it up too many times
*/
template <typename T>
void erase_from_beginning(std::vector<T> &x, const int &N)
{
	if ((N < 0) || (N > (int)x.size()))
	{
		fprintf(stderr, "Error: Invalid erasure request to erase_from_beginning. Exiting safely.\n"); fflush(stderr);
		exit(1);
	}
	if (N == 0)
		return;

	x.erase(x.begin(), x.begin() + N);
}


template <typename T>
void append_vector_to_vector(std::vector<T> &x, const std::vector<T> &y)
{
	x.insert(x.end(), y.begin(), y.end());
}


template <typename T>
void interleaved_array_to_complex(vcomplex_t *const y, const T *x, const int &N)
{
	if (N % 2 != 0)
	{
		fprintf(stderr, "Warning: conversion of interleaved to complex ignorning final element\n"); fflush(stderr);
	}

	y->resize(0);
	y->reserve(N / 2);
	for (int n = 0; n < N / 2; n++)
	{
		y->push_back(std::complex<double>((double)x[2 * n], (double)x[2 * n + 1]));
	}
}


template <typename T>
void real_array_to_complex(vcomplex_t *const y, const T *x, const int &N)
{
	y->resize(0);
	y->reserve(N);
	for (int n = 0; n < N; n++)
	{
		y->push_back(std::complex<double>((double)x[n], 0.0));
	}
}

#endif
