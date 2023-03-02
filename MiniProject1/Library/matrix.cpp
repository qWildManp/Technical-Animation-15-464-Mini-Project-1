#include <iostream>
#include "matrix.hpp"
#include "svdcmp.hpp"

matrix::matrix(int r, int c)
{
  //m = new (double *)[r];
  m = new double *[r];
  for (int i=0; i<r; i++)
    m[i] = new double[c];

  for (int i=0; i<r; i++)
    for (int j=0; j<c; j++)
      m[i][j] = 0.0;

  nRows = r;
  nCols = c;
}

matrix::~matrix()
{
  for (int i=0; i<nRows; i++) {
    delete [] m[i];
    m[i] = NULL;
  }
  delete [] m;
  m = NULL;
}

void matrix::printMatrix()
{
  for (int i=0; i<nRows; i++) {
    for (int j=0; j<nCols; j++)
      std::cout << m[i][j] << " ";
    std::cout << std::endl;
  }
}

void matrix::computeTranspose(matrix *trans)
{
  if (nRows != trans->getnCols()) {
    std::cout << "in matrix.cxx, transpose matrix size problem" << std::endl;
    exit(1);
  }
  if (nCols != trans->getnRows()) {
    std::cout << "in matrix.cxx, transpose matrix size problem" << std::endl;
    exit(1);
  }

  for (int i=0; i<nRows; i++)
    for (int j=0; j<nCols; j++)
      trans->setValue(m[i][j], j, i);
}

void matrix::computeMatrixMul(matrix *mul, matrix *result)
{
  if (nCols != mul->getnRows()) {
    std::cout << "in matrix.cxx, multiply matrix size problem" << std::endl;
    exit(1);
  }
  if (nRows != result->getnRows()) {
    std::cout << "in matrix.cxx, multiply matrix size problem" << std::endl;
    exit(1);
  }
  if (mul->getnCols() != result->getnCols()) {
    std::cout << "in matrix.cxx, multiply matrix size problem" << std::endl;
    exit(1);
  }

  double total;
  for (int i=0; i<nRows; i++) {
    for (int j=0; j<mul->getnCols(); j++) {
      total = 0;
      for (int k=0; k<nCols; k++)
	total += m[i][k] * mul->getValue(k, j);
      result->setValue(total, i, j);
    }
  }
}

/* ****************************************************************
 * invertMatrix -- using SVD 
 *
 *  returns rank
 * ****************************************************************/
//int invertMatrix(double **C, double **CInv, int rows, int cols, double tol)
int matrix::invertMatrix(matrix *CInv, double tol)
{
  if (nRows != CInv->getnCols()) {
    std::cout << "in matrix.cxx, invert matrix size problem" << std::endl;
    exit(1);
  }
  if (nCols != CInv->getnRows()) {
    std::cout << "in matrix.cxx, invert matrix size problem" << std::endl;
    exit(1);
  }

  // just make variables correspond in old code
  int rows = nRows;
  int cols = nCols;

  int i, j, k, rank;
  double *diag, **V, **T, diagMax, diagMin;
  double **CP;

  /* copy C over to CP and allocate space CPInv */
  CP = (double**)calloc(rows+1, sizeof(double*));
  for (i = 1; i <= rows; i++) {
    CP[i] = (double*)calloc(cols+1, sizeof(double));
    for (j = 1; j <= cols; j++) {
      //      CP[i][j] = C[i-1][j-1];
      CP[i][j] = m[i-1][j-1];
    }
  }

  /* do the SVD decomposition */
  diag = (double*)calloc(cols+1, sizeof(double));
  V = (double**)calloc(cols+1, sizeof(double*));
  T = (double**)calloc(cols+1, sizeof(double*));
  for (i = 1; i <= cols; i++) {
    V[i] = (double*)calloc(cols+1, sizeof(double));    
    T[i] = (double*)calloc(rows+1, sizeof(double));
  }
  svdcmp(CP, rows, cols, diag, V);

  /* zero small diagonal elements and invert all others */
  diagMax = 0.0;
  rank = 0;
  for (i = 1; i <= cols; i++) {
    if (diag[i] > diagMax) diagMax = diag[i];
  }
  diagMin = tol;
  for (i = 1; i <= cols; i++) {
    if (diag[i] < diagMin) {
      diag[i] = 0.0;
    } else {
      diag[i] = 1.0/diag[i];
      rank++;
    }
  }

  /* compute inverse, which is V W^-1 U^T
   *   .. U is now stored in C              */
  
  /* W^-1 U^T */
  for (i = 1; i <= cols; i++) {
    for (j = 1; j <= rows; j++) {
      T[i][j] = diag[i] * CP[j][i];
    }
  }

  /* V W^-1 U^T */
  for (i = 1; i <= cols; i++) {
    for (j = 1; j <= rows; j++) {
      //      CInv[i-1][j-1] = 0.0;
      CInv->setValue(0.0, i-1, j-1);
      for (k = 1; k <= cols; k++) {
	//	CInv[i-1][j-1] += V[i][k] * T[k][j];
	CInv->setValue(CInv->getValue(i-1,j-1) + V[i][k] * T[k][j], i-1, j-1);
      }
    }
  }

  /* cleanup diag, V, T, CP */
  for (i = 1; i <= cols; i++) {
    free(V[i]);
    free(T[i]);
  }
  free(V);
  free(T);

  for (i = 1; i <= rows; i++) {
    free(CP[i]);
  }
  free(CP);

  free(diag);

  return (rank);
}
