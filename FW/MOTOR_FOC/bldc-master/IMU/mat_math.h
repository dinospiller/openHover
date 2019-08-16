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
 * mat_math.h
 *
 *  Created on: 07/nov/2014
 *      Author: mac_daino
 */

#ifndef MAT_MATH_H_
#define MAT_MATH_H_

#define MATRIX_DATATYPE	float
/* matrices are in the format:
 * float Q[2][2]={	{0.1,	0},
 * 					{0,		0.1}}
 * float Q[1][2]={	{0},
 * 					{0}}
 * Pay attention: the 1st index is the row, the second is the column
 */

typedef enum{
	matrix_return_OK = 0,
	matrix_return_incoherent_matrix_dimensions =-1,
	matrix_return_singular_matrix =-2,
	matrix_return_NULL_matrices_args =-3,
}matrix_return_t;

typedef struct{
	uint8_t n_rows;
	uint8_t n_cols;
	MATRIX_DATATYPE** matr;
}matrix_t;

matrix_t* matrix_create(uint8_t rows, uint8_t cols);
void matrix_destroy(matrix_t* matrix);

matrix_t* matrix_clone(matrix_t* input);
void matrix_transpose(matrix_t* input,matrix_t* output);
matrix_return_t matrix_cofactors(matrix_t* input, matrix_t* output);
MATRIX_DATATYPE matrix_determinant(matrix_t* input);
matrix_return_t matrix_invert(matrix_t* input, matrix_t* output);
matrix_return_t matrix_sum(matrix_t* a, matrix_t* b, matrix_t* output);
matrix_return_t matrix_subtract(matrix_t* a, matrix_t* b, matrix_t* output);
matrix_return_t matrix_multiply(matrix_t* a, matrix_t* b, matrix_t* output);

#endif /* MAT_MATH_H_ */
