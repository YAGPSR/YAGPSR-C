#include "headers.h"

#include <stdio.h>
#include <string.h>

/*
   IMPORTANT: len is the total number of bits
*/

int convert_to_byte_stream(unsigned char *const byte_stream, const unsigned char *const bit_stream, const unsigned int len)
{
   unsigned int n;

   // This routine will only be successful if len is an exact multiple of 8
   if ((len % 8) || (len == 0)) {
      fprintf(stderr, "Error: convert_to_byte_stream requires an exact multiple of 8 as the length of bit stream\n");
      return(0);
   }

   // Initialize the output byte stream to zero
   memset((void *)byte_stream, 0, len / 8);

   // Note: MSB first
   for (n = 0; n < (len / 8); n++) {
      byte_stream[n] = (bit_stream[n*8 + 7]) + 
                       (bit_stream[n*8 + 6] << 1) +
                       (bit_stream[n*8 + 5] << 2) + 
                       (bit_stream[n*8 + 4] << 3) +
                       (bit_stream[n*8 + 3] << 4) +
                       (bit_stream[n*8 + 2] << 5) +
                       (bit_stream[n*8 + 1] << 6) +
                       (bit_stream[n*8] << 7);
   }

   return(1);
}


/*
   IMPORTANT: len is the length of the byte stream
*/

int convert_to_bit_stream(unsigned char *const bit_stream, const unsigned char *const byte_stream, const unsigned int len)
{
   unsigned int n;

   // Note: MSB first
   for (n = 0; n < len; n++) {
      bit_stream[n*8]     = (byte_stream[n] >> 7) & 1;
      bit_stream[n*8 + 1] = (byte_stream[n] >> 6) & 1;
      bit_stream[n*8 + 2] = (byte_stream[n] >> 5) & 1;
      bit_stream[n*8 + 3] = (byte_stream[n] >> 4) & 1;
      bit_stream[n*8 + 4] = (byte_stream[n] >> 3) & 1;
      bit_stream[n*8 + 5] = (byte_stream[n] >> 2) & 1;
      bit_stream[n*8 + 6] = (byte_stream[n] >> 1) & 1;
      bit_stream[n*8 + 7] = (byte_stream[n]) & 1;
   }

   return(1);
}

/*
* Convert an set of N bits to an integer, either MSB or LSB first
*/
int convert_bits_to_int(const int *bit_stream, const int &N, bool MSBFirst)
{
	int acc;

	acc = 0;

	if (MSBFirst == true)
	{
		for (int n = 0; n < N; n++)
		{
			acc += bit_stream[n] * (1 << (N - n - 1));
		}
	}
	else
	{
		for (int n = 0; n < N; n++)
		{
			acc += bit_stream[n] * (1 << n);
		}
	}

	return(acc);
}


int convert_2s_to_signed(const int &x, const int &NBits)
{
	if (x & (1 << (NBits - 1)))
	{
		return(x - (1 << NBits));
	}
	else
	{
		return(x);
	}
}

int64_t convert_2s_to_signed(const int64_t &x, const int &NBits)
{
	if (x & ((int64_t)1 << (NBits - 1)))
	{
		return(x - ((int64_t)1 << NBits));
	}
	else
	{
		return(x);
	}
}

// Note, the integers are assumed to take on values of 0, 1 ONLY
int binary_xor(const int &b1, const int &b2)
{
	if (b1 == b2)
	{
		return(0);
	}
	else
	{
		return(1);
	}
}


void nearest_neighbour_interpolate(std::vector<std::complex<double>> *const y, std::vector<std::complex<double>> const &x, vdouble_t const &p)
{
	y->resize(p.size());

	for (size_t n = 0; n < p.size(); n++)
	{
		int idx;

		idx = (int)round(p[n]);
		
		(*y)[n] = x[idx];
	}
}