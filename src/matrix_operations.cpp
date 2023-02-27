//
// Some common matrix operations, as required for this code
// These are implemented rather than using a library since there are very few
// and this allows all licensing issues to be avoided, plus I wanted to learn about functors
//

#include "headers.h"

t_RealMatrix::t_RealMatrix()
{
	NRows = 0;
	NCols = 0;

	data.resize(0);
}

t_RealMatrix::t_RealMatrix(const size_t &nr, const size_t &nc)
{
	NRows = nr;
	NCols = nc;

	data.resize(nr * nc, 0.0);
}

t_RealMatrix::~t_RealMatrix()
{
}

// Ordering is (row, col)
// Return the index corresponing to the row/col position
size_t t_RealMatrix::index(const size_t &r, const size_t &c) const
{
	return(c * NRows + r);
}


// Return the data at the specified index
double t_RealMatrix::element(const size_t &r, const size_t &c) const
{
	return(data[c * NRows + r]);
}


// Set a specific index to a value
void t_RealMatrix::set(const size_t &r, const size_t &c, const double &val)
{
	this->data[this->index(r, c)] = val;
}


void t_RealMatrix::initialize(const size_t &nr, const size_t &nc)
{
	NRows = nr;
	NCols = nc;

	data.resize(0);
	data.resize(nr * nc, 0.0);
}

//
// Transpose a matrix (in-place)
//
void t_RealMatrix::InplaceTranspose()
{
	t_RealMatrix LT;

	// Number of rows and number of columns of the transposed matrix (reversed wrt to the original)
	LT.initialize(NCols, NRows);

	for (size_t nr = 0; nr < NRows; nr++)
	{
		for (size_t nc = 0; nc < NCols; nc++)
		{
			LT.set(nc, nr, this->element(nr, nc));
		}
	}

	this->NCols = LT.NCols;
	this->NRows = LT.NRows;
	this->data  = LT.data;
}


//
// Transpose a matrix (in-place)
//
t_RealMatrix t_RealMatrix::Transpose()
{
	t_RealMatrix LT;

	// Number of rows and number of columns of the transposed matrix (reversed wrt to the original)
	LT.initialize(NCols, NRows);

	for (size_t nr = 0; nr < NRows; nr++)
	{
		for (size_t nc = 0; nc < NCols; nc++)
		{
			LT.set(nc, nr, this->element(nr, nc));
		}
	}

	return(LT);
}

//
// The Cholesky Decomposition using the Cholesky-Banachiewicz Algorithm
// This is the algorithm implemented for the real decomposition only
// Reference: https://en.wikipedia.org/wiki/Cholesky_decomposition
//

t_RealMatrix t_RealMatrix::CholeskyDecompositionReal()
{
	t_RealMatrix L(this->NRows, this->NCols);
	double acc;

	if (this->NRows != this->NCols)
	{
		fprintf(stderr, "Warning: Cholesky decomposition requires a square, positive-definite matrix. This matrix is not square. Returning input matrix.\n"); 
		fflush(stderr);
		L.initialize(0, 0);
		return(L);
	}

	for (size_t nr = 0; nr < this->NRows; nr++)
	{
		for (size_t nc = 0; nr < this->NCols; nc++)
		{
			if (nr < nc)
			{
				break;
			}

			acc = 0;
			for (size_t k = 0; k < nc; k++)
			{
				//acc = acc + L(nr, k) * conj(L(nc, k));
				acc = acc + L(nr, k) * L(nc, k);
			}

			if (nr == nc)
			{
				double t;

				t = this->element(nr, nc) - acc;

				if (t <= 0)
				{
					fprintf(stderr, "Warning: Numerical precision or non-positive definite input matrix generated the square root of a negative number (or an exact zero which would cause a divide-by-zero error). Invalid result.\n");
					fflush(stderr);
					L.initialize(0, 0);
					return(L);
				}

				L.set(nr, nc, sqrt(t));
			}
			else
			{
				if (L(nc, nc) == 0)
				{
					fprintf(stderr, "Warning: Numerical precision or non-positive definite input matrix generated a divide-by-zero error. Invalid result.\n");
					fflush(stderr);
					L.initialize(0, 0);
					return(L);
				}

				L.set(nr, nc, (this->element(nr, nc) - acc) / L(nc, nc));
			}
		}
	}

	return(L);
}


//
// Solve the system of equations Lx = b by back substitution, where L is a square, lower-triangular matrix
// with no zero elements on the diagonal (L is not verified as being lower triangular)
//
vdouble_t t_RealMatrix::SolveBackSubstitutionLower(vdouble_t const &b) const
{
	vdouble_t v, x;

	v.resize(this->NRows);

	for (size_t n = 0; n < this->NRows; n++)
	{
		if(this->element(n,n) == 0)
		{
			fprintf(stderr, "Error: Divide-by-zero error in back substitution (lower). Invalid result.\n");
			fflush(stderr);
			x.resize(0);
			return(x);
		}
	}

	for (size_t n = 0; n < this->NRows; n++)
	{
		double tmp;

		tmp = b[n];
		for (size_t m = 0; m < n; m++)
		{
			tmp -= v[m] * this->element(n, m);
		}

		v[n] = tmp / this->element(n, n);
	}

	x = v;

	return(x);
}


//
// Solve the system of equations Lx = b by back substitution, where L is a square, upper-triangular matrix
// with no zero elements on the diagonal (L is not verified as being upper triangular)
//
vdouble_t t_RealMatrix::SolveBackSubstitutionUpper(vdouble_t const &b) const
{
	vdouble_t v, x;

	v.resize(this->NCols);

	for (size_t n = 0; n < this->NRows; n++)
	{
		if (this->element(n, n) == 0)
		{
			fprintf(stderr, "Error: Divide-by-zero error in back substitution (upper). Invalid result.\n");
			fflush(stderr);
			x.resize(0);
			return(x);
		}
	}

	for (size_t n = 0; n < this->NRows; n++)
	{
		double tmp;

		tmp = b[this->NRows - n - 1];
		for (size_t m = 0; m < n; m++)
		{
			tmp -= v[this->NRows - m - 1] * this->element(this->NRows - n - 1, this->NRows - m - 1);
		}

		v[this->NRows - n - 1] = tmp / this->element(this->NRows - n - 1, this->NRows - n - 1);
	}

	x = v;

	return(x);
}

//
// Solve for x, in the equation Ax = b
// A (input) is a real, square, positive-definite matrix
// b (output) is a real vector
// x (output) is a real vector
//
vdouble_t t_RealMatrix::SolvePositiveDefinite(vdouble_t const &b)
{
	t_RealMatrix L;
	t_RealMatrix LT;
	vdouble_t    v, w, x;

	// Step 1: Cholesky decomposition
	L = this->CholeskyDecompositionReal();

	if (L.NRows == 0)
	{
		x.resize(0);
		return(x);
	}

	// The equation is now L L^T x = b
	// Step 2  : Solve using back-substitution
	// Step 2-a: Solve for v, in the equation: L v = b,
	v = L.SolveBackSubstitutionLower(b);

	if (v.size() == 0)
	{
		x.resize(0);
		return(x);
	}

	L.InplaceTranspose();

	// Step 2-b: Solve for x in the equation: L^T x = v
	w = L.SolveBackSubstitutionUpper(v);

	if (w.size() == 0)
	{
		x.resize(0);
		return(x);
	}

	x = w;

	return(x);
}


// Test routine
void TestCholeskyBasedInversion()
{
	t_RealMatrix A, X, Y;
	vdouble_t    b, x;

	A.initialize(6, 6);

	A.data[0]  = 11.3100;
	A.data[1]  = -9.6990;
	A.data[2]  = -3.5040;
	A.data[3]  = -1.1050;
	A.data[4]  = 1.3800;
	A.data[5]  = -2.7290;
	A.data[6]  = -9.6990;
	A.data[7]  = 31.8130;
	A.data[8]  = 6.3430;
	A.data[9]  = 2.9130;
	A.data[10] = -0.8830;
	A.data[11] = 1.4950;
	A.data[12] = -3.5040;
	A.data[13] = 6.3430;
	A.data[14] = 3.3180;
	A.data[15] = 4.0000;
	A.data[16] = -0.3380;
	A.data[17] = 2.6270;
	A.data[18] = -1.1050;
	A.data[19] = 2.9130;
	A.data[20] = 4.0000;
	A.data[21] = 9.0750;
	A.data[22] = 1.9370;
	A.data[23] = 0.1190;
	A.data[24] = 1.3800;
	A.data[25] = -0.8830;
	A.data[26] = -0.3380;
	A.data[27] = 1.9370;
	A.data[28] = 2.6360;
	A.data[29] = -3.2810;
	A.data[30] = -2.7290;
	A.data[31] = 1.4950;
	A.data[32] = 2.6270;
	A.data[33] = 0.1190;
	A.data[34] = -3.2810;
	A.data[35] = 14.6410;

	Y = A.CholeskyDecompositionReal();

	for (size_t n = 0; n < 36; n++)
		fprintf(stderr, "Idx %2d: %20.10f\n", (int)n, Y.data[n]);

	fprintf(stderr, "\n");
	fflush(stderr);

	b.resize(6);
	b[0] =  0.10;
	b[1] =  0.75;
	b[2] = -2.60;
	b[3] = -0.99;
	b[4] =  1.00;
	b[5] = -0.34;

	x = A.SolvePositiveDefinite(b);

	for (size_t n = 0; n < 6; n++)
		fprintf(stderr, "Idx %2d: %20.10f\n", (int)n, x[n]);

	fflush(stderr);

	// Test matrix multiplication

	X.initialize(4, 4);
	Y.initialize(4, 4);

	X.data[0]  = 0.5377;
	X.data[1]  = 1.8339;
	X.data[2]  = -2.2588;
	X.data[3]  = 0.8622;
	X.data[4]  = 0.3188;
	X.data[5]  = -1.3077;
	X.data[6]  = -0.4336;
	X.data[7]  = 0.3426;
	X.data[8]  = 3.5784;
	X.data[9]  = 2.7694;
	X.data[10] = -1.3499;
	X.data[11] = 3.0349;
	X.data[12] = 0.7254;
	X.data[13] = -0.0631;
	X.data[14] = 0.7147;
	X.data[15] = -0.2050;

	Y.data[0]  = -0.1241;
	Y.data[1]  = 1.4897;
	Y.data[2]  = 1.4090;
	Y.data[3]  = 1.4172;
	Y.data[4]  = 0.6715;
	Y.data[5]  = -1.2075;
	Y.data[6]  = 0.7172;
	Y.data[7]  = 1.6302;
	Y.data[8]  = 0.4889;
	Y.data[9]  = 1.0347;
	Y.data[10] = 0.7269;
	Y.data[11] = -0.3034;
	Y.data[12] = 0.2939;
	Y.data[13] = -0.7873;
	Y.data[14] = 0.8884;
	Y.data[15] = -1.1471;
			
	A = X * Y;

	fprintf(stderr, "\n\nInversion:\n\n");

	for (size_t n = 0; n < 16; n++)
		fprintf(stderr, "Idx %2d: %20.10f\n", (int)n, A.data[n]);

	fflush(stderr);


}
