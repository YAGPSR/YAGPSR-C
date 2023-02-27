#pragma once

extern void TestCholeskyBasedInversion();


#define MATRIX_OPERATION_FAIL          -1
#define MATRIX_OPERATION_SUCCESS        0


class t_RealMatrix
{
public:
	size_t    NRows;
	size_t    NCols;
	vdouble_t data;

	t_RealMatrix();
	t_RealMatrix(const size_t &, const size_t &);
	~t_RealMatrix();
	

	double element(const size_t &, const size_t &) const;
	size_t index(const size_t &, const size_t &) const;
	void   set(const size_t &, const size_t &, const double &);
	void   initialize(const size_t &, const size_t &);

	void         InplaceTranspose();
	t_RealMatrix Transpose();
	t_RealMatrix CholeskyDecompositionReal();
	vdouble_t    SolvePositiveDefinite(vdouble_t const &);

	// Functor for data access into this matrix library
	double operator()(size_t r, size_t c) const { return data[c * NRows + r]; }

	// Matrix multiplication
	t_RealMatrix operator*(const t_RealMatrix& X) {

		t_RealMatrix Y(this->NRows, X.NCols);
		int          c;

		c = 0;

		for (size_t nc = 0; nc < X.NCols; nc++)
		{
			for (size_t cr = 0; cr < this->NRows; cr++)
			{
				for (size_t ic = 0; ic < this->NCols; ic++)
				{
					Y.data[c] += this->element(cr, ic) * X.element(ic, nc);
				}

				c++;
			}
		}

		return(Y);
	}

protected:
private:

	vdouble_t SolveBackSubstitutionLower(vdouble_t const &) const;
	vdouble_t SolveBackSubstitutionUpper(vdouble_t const &) const;
};

