/*
	Copyright 2017-2020 Dino Spiller (dinospiller@gmail.com)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
 * mat_math.c
 *
 *  Created on: 07/nov/2014
 *      Author: mac_daino
 */

#include "stdint.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include "mat_math.h"


/*
   Recursive definition of determinate using expansion by minors.

double Determinant(double **a,int n)
{
   int i,j,j1,j2;
   double det = 0;
   double **m = NULL;

   if (n < 1) {

   } else if (n == 1) {
      det = a[0][0];
   } else if (n == 2) {
      det = a[0][0] * a[1][1] - a[1][0] * a[0][1];
   } else {
      det = 0;
      for (j1=0;j1<n;j1++) {
         m = malloc((n-1)*sizeof(double *));
         for (i=0;i<n-1;i++)
            m[i] = malloc((n-1)*sizeof(double));
         for (i=1;i<n;i++) {
            j2 = 0;
            for (j=0;j<n;j++) {
               if (j == j1)
                  continue;
               m[i-1][j2] = a[i][j];
               j2++;
            }
         }
         det += pow(-1.0,j1+2.0) * a[0][j1] * Determinant(m,n-1);
         for (i=0;i<n-1;i++)
            free(m[i]);
         free(m);
      }
   }
   return(det);
}*/

/*
   Find the cofactor matrix of a square matrix

void CoFactor(double **a,int n,double **b)
{
   int i,j,ii,jj,i1,j1;
   double det;
   double **c;

   c = malloc((n-1)*sizeof(double *));
   for (i=0;i<n-1;i++)
     c[i] = malloc((n-1)*sizeof(double));

   for (j=0;j<n;j++) {
      for (i=0;i<n;i++) {

         //Form the adjoint a_ij
         i1 = 0;
         for (ii=0;ii<n;ii++) {
            if (ii == i)
               continue;
            j1 = 0;
            for (jj=0;jj<n;jj++) {
               if (jj == j)
                  continue;
               c[i1][j1] = a[ii][jj];
               j1++;
            }
            i1++;
         }

         // Calculate the determinate
         det = Determinant(c,n-1);

         // Fill in the elements of the cofactor
         b[i][j] = pow(-1.0,i+j+2.0) * det;
      }
   }
   for (i=0;i<n-1;i++)
      free(c[i]);
   free(c);
}
*/

//------------------------------- GLOBALS ----------------------------------
matrix_t* matrix_clone(matrix_t* input){
    matrix_t* clone = NULL;
    if(input!=NULL){
        uint8_t i;
        clone = matrix_create(input->n_rows,input->n_cols);
        for(i=0;i<input->n_rows;i++)
            memcpy(clone->matr[i],input->matr[i],(input->n_cols*sizeof(MATRIX_DATATYPE)));
    }
	return clone;
}

void matrix_transpose(matrix_t* input,matrix_t* output){
    if(input==NULL) return;

	matrix_t* result=matrix_create(input->n_cols, input->n_rows);
	uint8_t i,j;

	for (i=0;i<input->n_rows;i++) {
	  for (j=0;j<input->n_cols;j++) {
		  result->matr[j][i]=input->matr[i][j];
	  }
	}

    matrix_destroy(output);
    output=matrix_clone(result);
    matrix_destroy(result);
}

matrix_return_t matrix_cofactors(matrix_t* input, matrix_t* output){
	if(input->n_rows!=input->n_cols) return matrix_return_incoherent_matrix_dimensions;
	uint8_t i,j,ii,jj,i1,j1;
	MATRIX_DATATYPE det;
	matrix_t* c = matrix_create(input->n_rows-1,input->n_cols-1);
    /* Fill in the elements of the cofactor */
    if(output==NULL) return matrix_return_NULL_matrices_args;
    if(output->n_rows!=output->n_cols){
        matrix_destroy(output);
        output = matrix_create(input->n_rows,input->n_cols);
    }

	for (j=0;j<input->n_rows;j++) {
	  for (i=0;i<input->n_rows;i++) {

		 /* Form the adjoint input_ij */
		 i1 = 0;
		 for (ii=0;ii<input->n_rows;ii++) {
			if (ii == i)
			   continue;
			j1 = 0;
			for (jj=0;jj<input->n_rows;jj++) {
			   if (jj == j)
				  continue;
			   c->matr[i1][j1] = input->matr[ii][jj];
			   j1++;
			}
			i1++;
		 }

		 /* Calculate the determinate */
		 det = matrix_determinant(c);
		 output->matr[i][j] = pow(-1.0,i+j+2.0) * det;
	  }
	}
	matrix_destroy(c);
	return matrix_return_OK;
}

MATRIX_DATATYPE matrix_determinant(matrix_t* input){
	if(input->n_rows!=input->n_cols) return (MATRIX_DATATYPE) 0;

	uint8_t i,j,j1,j2;
	MATRIX_DATATYPE det = 0;
	matrix_t *m;

	if (input->n_rows == 1) { /* Shouldn't get used */
	  det = input->matr[0][0];
	} else if (input->n_rows == 2) {
	  det = input->matr[0][0] * input->matr[1][1] - input->matr[1][0] * input->matr[0][1];
	} else {
	  det = 0;
	  for (j1=0;j1<input->n_rows;j1++) {
		 m = matrix_create(input->n_rows-1,input->n_cols-1);
		 for (i=1;i<m->n_rows;i++) {
			j2 = 0;
			for (j=0;j<m->n_rows;j++) {
			   if (j == j1)
				  continue;
			   m->matr[i-1][j2] = input->matr[i][j];
			   j2++;
			}
		 }
		 det += pow(-1.0,j1+2.0) * input->matr[0][j1] * matrix_determinant(m);
		 matrix_destroy(m);
	  }
	}
	return(det);
}

matrix_return_t matrix_invert(matrix_t* input, matrix_t* output){
	if(input->n_rows!=input->n_cols) return matrix_return_incoherent_matrix_dimensions;
    if(matrix_determinant(input)==0) return matrix_return_singular_matrix;
    if(output==NULL) return matrix_return_NULL_matrices_args;
    if(output->n_rows!=output->n_cols){
        matrix_destroy(output);
        output = matrix_create(input->n_rows,input->n_cols);
    }
    matrix_cofactors(input, output);
    matrix_transpose(output, output);
    return matrix_return_OK;
}

matrix_return_t matrix_sum(matrix_t* a, matrix_t* b, matrix_t* output){
	matrix_return_t ret = matrix_return_OK;
	if((a->n_rows!=b->n_rows)||(a->n_cols!=b->n_cols)){
		ret= matrix_return_incoherent_matrix_dimensions;
	}else{
		uint8_t i,j;

        if(output==NULL) return matrix_return_NULL_matrices_args;

        if((output->n_rows!=a->n_rows)||(output->n_cols!=a->n_cols)){
            matrix_destroy(output);
            output = matrix_create(a->n_rows,a->n_cols);
        }

		for (i=0;i<a->n_rows;i++) {
			  for (j=0;j<a->n_cols;j++) {
					output->matr[i][j] = a->matr[i][j] + b->matr[i][j];
			  }
		}
	}
	return ret;
}
matrix_return_t matrix_subtract(matrix_t* a, matrix_t* b, matrix_t* output){
	matrix_return_t ret = matrix_return_OK;
	if((a->n_rows!=b->n_rows)||(a->n_cols!=b->n_cols)){
		ret= matrix_return_incoherent_matrix_dimensions;
	}else{
		uint8_t i,j;

		if(output==NULL) return matrix_return_NULL_matrices_args;

        if((output->n_rows!=a->n_rows)||(output->n_cols!=a->n_cols)){
            matrix_destroy(output);
            output = matrix_create(a->n_rows,a->n_cols);
        }

		for (i=0;i<a->n_rows;i++) {
			  for (j=0;j<a->n_cols;j++) {
					output->matr[i][j] = a->matr[i][j] - b->matr[i][j];
			  }
		}
	}
	return ret;
}

matrix_return_t matrix_multiply(matrix_t* a, matrix_t* b, matrix_t* output){
    matrix_return_t result = matrix_return_OK;
    // check coherence of inputs dimensions
    if((a==NULL)||(b==NULL)||(output==NULL))
        result=matrix_return_NULL_matrices_args;
    else if(a->n_cols!=b->n_rows)
        result=matrix_return_incoherent_matrix_dimensions;
    else{
        if((output->n_rows!=a->n_rows)||(output->n_cols!=b->n_cols)){
            matrix_destroy(output);
            output = matrix_create(a->n_rows,b->n_cols);
        }

        uint8_t i,j,k;
        for(i=0;i<output->n_rows;i++){
            memset(output->matr[i], 0, (output->n_cols*sizeof(MATRIX_DATATYPE)));
        }

        for(i=0;i<output->n_rows;i++){
            for(j=0;j<output->n_cols;j++){
                for(k=0;k<a->n_cols;k++){
                    output->matr[i][j]+=a->matr[i][k]*b->matr[k][j];
                }
            }
        }

    }

    return result;
}

/**
 * @brief: 	creates a zero-filled matrix of specified dimensions
 * @params:	rows=num of wanted rows
 * @params:	cols=num of wanted columns
 * @return: a pointer to the matrix: it uses dynamic allocation (heap)
 */
matrix_t* matrix_create(uint8_t rows, uint8_t cols){
	matrix_t* matrix = malloc(sizeof(matrix_t));
    uint8_t n_rows;
	matrix->n_rows = rows;
	matrix->n_cols = cols;
	matrix->matr = malloc(rows*sizeof(MATRIX_DATATYPE *));
    for(n_rows=0;n_rows<rows;n_rows++)
        matrix->matr[n_rows] = malloc(cols*sizeof(MATRIX_DATATYPE));
	return matrix;
}

/**
 * @brief: frees the (dynamically allocated) memory associated to a matrix
 */
void matrix_destroy(matrix_t* matrix){
	if(matrix!=NULL){
        uint8_t n_rows;
        for(n_rows=0;n_rows<matrix->n_rows;n_rows++)
            free(matrix->matr[n_rows]);
        free(matrix->matr);
		free(matrix);
	}
}

