#ifndef KalmanMath_h
#define KalmanMath_h

#include <stdlib.h>
#include <math.h>

//Gaussian noise
double randn(double mean, double stddev) {
    double u1, u2, rand_stdnormal;
    u1 = rand() / (RAND_MAX + 1.0);
    u2 = rand() / (RAND_MAX + 1.0);
    // Box-Muller transform
    rand_stdnormal = sqrt(-2.0 * log(u1)) * cos(2.0 * 3.14 * u2);
    return mean + stddev * rand_stdnormal;
}

void addMatrices(double *mat1, double *mat2, double *result, int rows, int cols) {
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            result[i * cols + j] = mat1[i * cols + j] + mat2[i * cols + j];
        }
    }
}

void subtractMatrices(double *mat1, double *mat2, double *result, int rows, int cols) {
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            result[i * cols + j] = mat1[i * cols + j] - mat2[i * cols + j];
        }
    }
}

void multiplyMatrices(double *mat1, double *mat2, double *result, int rows1, int cols1, int rows2, int cols2) {
    for (int i = 0; i < rows1; ++i) {
        for (int j = 0; j < cols2; ++j) {
            *(result + i * cols2 + j) = 0; // Initialize result element to zero
            for (int k = 0; k < cols1; ++k) {
                *(result + i * cols2 + j) += *(mat1 + i * cols1 + k) * *(mat2 + k * cols2 + j);
            }
        }
    }
}

void transposeMatrix(double *inputMatrix, double *outputMatrix, int rows, int cols) {
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            // Use pointers to access matrix elements
            double *inputElement = inputMatrix + i * cols + j;
            double *outputElement = outputMatrix + j * rows + i;

            // Copy elements to the transposed matrix
            *outputElement = *inputElement;
        }
    }
}

// Function to perform Gauss-Jordan elimination to find the inverse of a matrix
void gaussJordan(double **matrix, int n) {
    // Create an identity matrix as the augmented matrix
    double **augmented = (double **)malloc(n * sizeof(double *));
    for (int i = 0; i < n; i++) {
        augmented[i] = (double *)malloc(2 * n * sizeof(double));
        for (int j = 0; j < n; j++) {
            augmented[i][j] = matrix[i][j];
            augmented[i][j + n] = (i == j) ? 1.0 : 0.0;
        }
    }

    // Apply Gauss-Jordan elimination
    for (int i = 0; i < n; i++) {
        // Make the diagonal element 1
        double pivot = augmented[i][i];
        for (int j = 0; j < 2 * n; j++) {
            augmented[i][j] /= pivot;
        }

        // Make the other elements in the column 0
        for (int k = 0; k < n; k++) {
            if (k != i) {
                double factor = augmented[k][i];
                for (int j = 0; j < 2 * n; j++) {
                    augmented[k][j] -= factor * augmented[i][j];
                }
            }
        }
    }

    // Extract the inverse matrix
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            matrix[i][j] = augmented[i][j + n];
        }
    }

    // Free the memory allocated for the augmented matrix
    for (int i = 0; i < n; i++) {
        free(augmented[i]);
    }
    free(augmented);
}

#endif