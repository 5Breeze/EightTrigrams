#include "matrix.h"

// 矩阵乘法
void matrix_mul(float A[MATRIX_SIZE][MATRIX_SIZE], float B[MATRIX_SIZE][MATRIX_SIZE], float C[MATRIX_SIZE][MATRIX_SIZE], int size) {
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            C[i][j] = 0;
            for (int k = 0; k < size; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

// 矩阵求逆
void matrix_inverse(float src[MATRIX_SIZE][MATRIX_SIZE], float res[MATRIX_SIZE][MATRIX_SIZE], int size) {
    int i, j, k, principal;
    float L[MATRIX_SIZE][MATRIX_SIZE] = {0};
    float U[MATRIX_SIZE][MATRIX_SIZE] = {0};
    float tmp[MATRIX_SIZE][MATRIX_SIZE];
    float Lsum, Usum, p, Max;

    // 复制输入矩阵到临时矩阵
    memcpy(tmp, src, sizeof(float) * size * size);

    // 初始化 L 为单位矩阵
    for (i = 0; i < size; i++) {
        L[i][i] = 1.0f;
    }

    // 选主元并进行行交换
    for (j = 0; j < size; j++) {
        principal = j;
        Max = fabs(tmp[principal][j]);
        for (i = j + 1; i < size; i++) {
            if (fabs(tmp[i][j]) > Max) {
                principal = i;
                Max = fabs(tmp[i][j]);
            }
        }
        if (j != principal) {
            for (k = 0; k < size; k++) {
                p = tmp[principal][k];
                tmp[principal][k] = tmp[j][k];
                tmp[j][k] = p;
            }
        }
    }

    // 计算 L 和 U
    for (i = 0; i < size; i++) {
        for (j = 0; j < size; j++) {
            if (i <= j) {
                Usum = 0;
                for (k = 0; k < i; k++) {
                    Usum += L[i][k] * U[k][j];
                }
                U[i][j] = tmp[i][j] - Usum;
            } else {
                Lsum = 0;
                for (k = 0; k < j; k++) {
                    Lsum += L[i][k] * U[k][j];
                }
                L[i][j] = (tmp[i][j] - Lsum) / U[j][j];
            }
        }
    }

    // L 的逆
    for (i = 0; i < size; i++) {
        for (j = 0; j <= i; j++) {
            if (i == j) {
                L[i][j] = 1.0f / L[i][j];
            } else {
                Lsum = 0;
                for (k = j; k < i; k++) {
                    Lsum += L[i][k] * L[k][j];
                }
                L[i][j] = -Lsum / L[i][i];
            }
        }
    }

    // U 的逆
    for (j = size - 1; j >= 0; j--) {
        for (i = j; i >= 0; i--) {
            if (i == j) {
                U[i][j] = 1.0f / U[i][j];
            } else {
                Usum = 0;
                for (k = j; k >= i + 1; k--) {
                    Usum += U[i][k] * U[k][j];
                }
                U[i][j] = -Usum / U[i][i];
            }
        }
    }

    // 计算逆矩阵 res = U^-1 * L^-1
    matrix_mul(U, L, res, size);
}

// 矩阵与向量相乘
void matrix__mul(float matrix[MATRIX_SIZE][MATRIX_SIZE], float vector[MATRIX_SIZE], float result[MATRIX_SIZE]) {
    for (int i = 0; i < MATRIX_SIZE; i++) {
        result[i] = 0; // 初始化结果向量的第 i 个元素
        for (int j = 0; j < MATRIX_SIZE; j++) {
            result[i] += matrix[i][j] * vector[j]; // 计算矩阵第 i 行和向量的点积
        }
    }
}