#ifndef _MATRIX_H
#define _MATRIX_H

#include <stdlib.h>
#include <math.h>

// use this for tol variable in invertMatrix()
#define SVD_TOL 0.0001

class matrix
{
 public:
  matrix(int r, int c);
  ~matrix();

  void setValue(double d, int r, int c) {
    m[r][c] = d;
  }
  double getValue(int r, int c) {
    return m[r][c];
  }
  int getnRows() {
    return nRows;
  }
  int getnCols() {
    return nCols;
  }

  // for debugging
  void printMatrix();

  void computeTranspose(matrix *trans);
  // multiply the matrix in this class by matrix mul, and put result in matrix result
  void computeMatrixMul(matrix *mul, matrix *result);
  // invert matrix, and put result in CInv
  // tol is SVD_TOL
  // uses SVD method - seems to work like pinv() function in MATLAB
  // returns rank
  int invertMatrix(matrix *CInv, double tol);

 private:
  double **m; // the matrix elements
  int nRows;
  int nCols;
};

#endif
