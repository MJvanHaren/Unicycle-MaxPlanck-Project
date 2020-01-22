#include "matrix_calculations.h"
/**************************************************************************
 *
 *  Functions
 *
 **************************************************************************/
//! \brief calcultes matrix vector product vec C = Matrix A * vec B
//! \param[in] K, M sizes of matrices
//! \param[in]  Mat_A, matrix (on the right side of equation)
//! \param[in] Mat_B, vector which should be multiplied with matrix
//! \param[in] Mat_C, vector to store result in.
void multiply_matrix_vector(int K, int M, const float Mat_A[][M], float Mat_B[M], float Mat_C[K]) {
  int i, j;
  for (i = 0; i < K; i++) { // looping through row i of Matrix A
    Mat_C[i] = 0;
    for (j = 0; j < M; j++) { // looping through column j of Matrix A
      Mat_C[i] += Mat_A[i][j] * Mat_B[j];
    }
  }
} //! \end of mutliply_matrix_vector function