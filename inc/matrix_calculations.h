#ifndef matrix_calculations__
#define matrix_calculations__
/**************************************************************************
 *
 *  Header Files to include
 *
 **************************************************************************/
/* subystem header files */

/* global program files */
#include "m_general.h"
#include "config.h"
#include "m_usb.h"

/* External header files */

/**************************************************************************
 *
 *  Function declarations
 *
 **************************************************************************/
void multiply_matrix_vector(int K, int M, const float Mat_A[][M], float Mat_B[M], float Mat_C[K]);

#endif
