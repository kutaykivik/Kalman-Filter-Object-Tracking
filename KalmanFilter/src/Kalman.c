#include "Kalman.h"

void kalman_init(KalmanFilter* kf) 
{
    int i = 0;

    for (i = 0; i < 4; i++) 
    {
        kf->est[i] = 0;
        kf->RData[i] = 0;
    }

    for (i = 0; i < 16; i++) 
    {
        kf->AData[i] = 0;
        kf->QData[i] = 0;
        kf->PData[i] = 0;
        kf->IData[i] = 0;
    }
    
    for (i = 0; i < 8; i++) 
    {
        kf->HData[i] = 0;
        kf->KGData[i] = 0;
    }

    // State transition matrix
    kf->AData[0] = 1;
    kf->AData[1] = dt;
    kf->AData[5] = 1;
    kf->AData[10] = 1;
    kf->AData[11] = dt;
    kf->AData[15] = 1;

    // Measurement matrix
    kf->HData[0] = 1;
    kf->HData[6] = 1;
    
    // Identity matrix
    kf->IData[0] = 1;
    kf->IData[5] = 1;
    kf->IData[10] = 1;
    kf->IData[15] = 1;

    // Process noise matrix
    kf->QData[0] = 5;
    kf->QData[5] = 5;
    kf->QData[10] = 5;
    kf->QData[15] = 5;

    // Measurement covariance matrix
    kf->RData[0] = 5;
    kf->RData[3] = 5;
    
    // Covariance matrix of estimation
    kf->PData[0] = 500;
    kf->PData[5] = 500;
    kf->PData[10] = 500;
    kf->PData[15] = 500;
}

void kalmanFilter(KalmanFilter* kf, double *measured) 
{
    int i = 0;
    int j = 0;
    int c = 0;

    // Predict
    double result[16];
    double transformed_matrix[2][2];
    
    double temp1[16];
    double temp2[16];
    double temp3[16];
    double temp4[16];
    double temp5[4];
    
    multiplyMatrices(&kf->AData[0], &kf->est[0], result, 4, 4, 4, 1);
    kf->est[0] = result[0];
    kf->est[1] = result[1];
    kf->est[2] = result[2];
    kf->est[3] = result[3];
    
    // A * P
    multiplyMatrices(kf->AData, kf->PData, temp1, 4, 4, 4, 4);
    // (A * P) * A^T
    transposeMatrix(kf->AData, temp2, 4, 4);
    multiplyMatrices(temp1, temp2, temp3, 4, 4, 4, 4);

    // (A * P * A^T) + Q
    addMatrices(temp3, kf->QData, result, 4, 4);

    for (i = 0; i < 16; ++i) 
    {
        kf->PData[i] = result[i];
    }
    
    // Update
    
    // H^T
    transposeMatrix(kf->HData, temp1, 2, 4);
    // (H * P * H^T) + R
    multiplyMatrices(kf->HData, kf->PData, temp2, 2, 4, 4, 4);
    multiplyMatrices(temp2, temp1, temp3, 2, 4, 4, 2);
    addMatrices(temp3, kf->RData, result, 2, 2);
    
    for (i = 0; i < 2; i++) 
    {
        for (j = 0; j < 2; j++) 
        {
            transformed_matrix[i][j] = result[c];
            c++;
        }
    }
    c = 0;
    
    // Allocate memory for the matrix
    double **matrix = (double **)malloc(2 * sizeof(double *));
    for (int i = 0; i < 2; i++) 
    {
        matrix[i] = (double *)malloc(2 * sizeof(double));
        for (int j = 0; j < 2; j++) 
        {
            matrix[i][j] = transformed_matrix[i][j];
        }
    }
    
    // Perform Gauss-Jordan elimination
    // ((H * P * H^T) + R)^-1
    gaussJordan(matrix, 2);
    for (i = 0; i < 2; i++) 
    {
        for (j = 0; j < 2; j++) 
        {
            result[c] = matrix[i][j];
            c++;
        }
    }
    // Free allocated memory
    for (int i = 0; i < 2; i++) 
    {
        free(matrix[i]);
    }
    free(matrix);

    // K = P * H(transpose) * ((H * P * H^T) + R)^-1
    multiplyMatrices(kf->PData, temp1, temp4, 4, 4, 4, 2);
    multiplyMatrices(temp4, result, kf->KGData, 4, 2, 2, 2);
    
    // Estimate current state
    multiplyMatrices(kf->HData, kf->est, temp4, 2, 4, 4, 1);
    subtractMatrices(&measured[0], &temp4[0], result, 2, 1);
    
    multiplyMatrices(kf->KGData, result, temp4, 4, 2, 2, 1);
    addMatrices(kf->est, temp4, result, 4, 1);
    
    kf->est[0] = result[0];
    kf->est[1] = result[1];
    kf->est[2] = result[2];
    kf->est[3] = result[3];
    
    // Update the current estimate uncertainty
    multiplyMatrices(kf->KGData, kf->HData, temp1, 4, 2, 2, 4);
    subtractMatrices(kf->IData, temp1, temp2, 4, 4); // temp2 = (I - K * H)
    multiplyMatrices(temp2, kf->PData, temp3, 4, 4, 4, 4); // temp3 = (I - K * H) * P
    transposeMatrix(temp2, temp4, 4, 4); // temp4 = (I - K * H) (transpose)
    multiplyMatrices(temp3, temp4, temp1, 4, 4, 4, 4); // temp1 = (I - K * H) * P * (I - K * H) (transpose)
    
    multiplyMatrices(kf->KGData, kf->RData, temp2, 4, 2, 2, 2); // temp2 = K * R
    transposeMatrix(kf->KGData, temp3, 4, 2); // temp3 = K(transpose)
    multiplyMatrices(temp2, temp3, temp4, 4, 2, 2, 4); // temp4 = K * R * K(transpose)
    
    addMatrices(temp1, temp4, result, 4, 4);
    
    for (i = 0; i < 15; i++) 
    {
        kf->PData[i] = result[i];
    }
}
