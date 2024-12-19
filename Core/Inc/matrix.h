#ifndef __MATRIX_H__
#define __MATRIX_H__

#include "main.h"
#include "math.h"
#ifdef __cplusplus
extern "C" {
#endif

#define MATRIX_SIZE 6 // ��������� 6x6 �Ĺ̶���С

void matrix_inverse(float src[MATRIX_SIZE][MATRIX_SIZE], float res[MATRIX_SIZE][MATRIX_SIZE], int size);
void matrix__mul(float matrix[MATRIX_SIZE][MATRIX_SIZE], float vector[MATRIX_SIZE], float result[MATRIX_SIZE]);
#endif
