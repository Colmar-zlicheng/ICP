#include <math.h>
#include <iostream>
#include <map>
using namespace std;

bool Jacobi(double *matrix, int dim, double *eigenvectors, double *eigenvalues, double precision, int max)
{

    for (int i = 0; i < dim; i++)
    {
        eigenvectors[i * dim + i] = 1.0f;
        for (int j = 0; j < dim; j++)
        {
            if (i != j)
                eigenvectors[i * dim + j] = 0.0f;
        }
    }

    int nCount = 0; // current iteration
    while (1)
    {
        // find the largest element on the off-diagonal line of the matrix
        double dbMax = matrix[1];
        int nRow = 0;
        int nCol = 1;
        for (int i = 0; i < dim; i++)
        { // row
            for (int j = 0; j < dim; j++)
            { // column
                double d = fabs(matrix[i * dim + j]);
                if ((i != j) && (d > dbMax))
                {
                    dbMax = d;
                    nRow = i;
                    nCol = j;
                }
            }
        }

        if (dbMax < precision) // precision check
            break;

        if (nCount > max) // iterations check
            break;

        nCount++;

        double dbApp = matrix[nRow * dim + nRow];
        double dbApq = matrix[nRow * dim + nCol];
        double dbAqq = matrix[nCol * dim + nCol];

        // compute rotate angle
        double dbAngle = 0.5 * atan2(-2 * dbApq, dbAqq - dbApp);
        double dbSinTheta = sin(dbAngle);
        double dbCosTheta = cos(dbAngle);
        double dbSin2Theta = sin(2 * dbAngle);
        double dbCos2Theta = cos(2 * dbAngle);

        matrix[nRow * dim + nRow] = dbApp * dbCosTheta * dbCosTheta +
                                    dbAqq * dbSinTheta * dbSinTheta + 2 * dbApq * dbCosTheta * dbSinTheta;

        matrix[nCol * dim + nCol] = dbApp * dbSinTheta * dbSinTheta +
                                    dbAqq * dbCosTheta * dbCosTheta - 2 * dbApq * dbCosTheta * dbSinTheta;
        matrix[nRow * dim + nCol] = 0.5 * (dbAqq - dbApp) * dbSin2Theta + dbApq * dbCos2Theta;
        matrix[nCol * dim + nRow] = matrix[nRow * dim + nCol];

        for (int i = 0; i < dim; i++)
        {
            if ((i != nCol) && (i != nRow))
            {
                int u = i * dim + nRow; // p
                int w = i * dim + nCol; // q

                dbMax = matrix[u];
                matrix[u] = matrix[w] * dbSinTheta + dbMax * dbCosTheta;
                matrix[w] = matrix[w] * dbCosTheta - dbMax * dbSinTheta;
            }
        }

        for (int j = 0; j < dim; j++)
        {
            if ((j != nCol) && (j != nRow))
            {
                int u = nRow * dim + j; // p
                int w = nCol * dim + j; // q

                dbMax = matrix[u];
                matrix[u] = matrix[w] * dbSinTheta + dbMax * dbCosTheta;
                matrix[w] = matrix[w] * dbCosTheta - dbMax * dbSinTheta;
            }
        }

        // compute eigenvector
        for (int i = 0; i < dim; i++)
        {

            int u = i * dim + nRow; // p
            int w = i * dim + nCol; // q

            dbMax = eigenvectors[u];
            eigenvectors[u] = eigenvectors[w] * dbSinTheta + dbMax * dbCosTheta;
            eigenvectors[w] = eigenvectors[w] * dbCosTheta - dbMax * dbSinTheta;
        }
    }

    // sort eigenvalues

    std::map<double, int> mapEigen;
    for (int i = 0; i < dim; i++)
    {
        eigenvalues[i] = matrix[i * dim + i];
        mapEigen.insert(make_pair(eigenvalues[i], i));
    }

    double *pdbTmpVec = new double[dim * dim];
    std::map<double, int>::reverse_iterator iter = mapEigen.rbegin();
    for (int j = 0; iter != mapEigen.rend(), j < dim; ++iter, ++j)
    {
        for (int i = 0; i < dim; i++)
        {
            pdbTmpVec[i * dim + j] = eigenvectors[i * dim + iter->second];
        }
        eigenvalues[j] = iter->first;
    }

    for (int i = 0; i < dim; i++)
    {
        double dSumVec = 0;
        for (int j = 0; j < dim; j++)
            dSumVec += pdbTmpVec[j * dim + i];

        if (dSumVec < 0)
        {
            for (int j = 0; j < dim; j++)
                pdbTmpVec[j * dim + i] *= -1;
        }
    }

    memcpy(eigenvectors, pdbTmpVec, sizeof(double) * dim * dim);

    delete[] pdbTmpVec;

    return true;
}
