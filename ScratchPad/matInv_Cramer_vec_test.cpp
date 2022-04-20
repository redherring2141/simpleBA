#include <iostream>
#include <vector>
#include <bits/stdc++.h>

using namespace std;
typedef vector<vector<double>> Mat2D_t;
typedef vector<vector<vector<double>>> Mat3D_t;

Mat2D_t Create2DMat(unsigned nRows, unsigned nCols)
{
   return Mat2D_t(nRows, vector<double>(nCols, 0));
};

Mat2D_t GetCofactorMat(Mat2D_t matA, unsigned row_p, unsigned col_q)
{// Function to get cofactor of matA[p][q] in matCof. n is current dimension of matA.
    int nRows_matA = matA.size();
    int nCols_matA = matA[0].size();
    int n = 0;
    int i = 0, j = 0;

    if(nRows_matA == nCols_matA)
    {
        n = nRows_matA;
    }
    else
    {
        cout << "Only available for a square matrix. Function: GetCofactorMat" << endl;
        abort();
    }

    Mat2D_t matCof = Create2DMat(n, n);

    // Looping for each element of the matrix
    for(int row=0; row<n; row++)
    {
        for(int col=0; col<n; col++)
        {
            // Copying into temporary matrix only those element
            // which are not in given row and column
            if(row!=row_p && col!=col_q)
            {
                cout << "<<<<<debugging>>>>> j = " << j << endl;
                matCof[i][j] = matA[row][col];
                j++;

                // Row is filled, so increase row index and reset col index
                if(j==n-1)
                {
                    j=0;
                    i++;
                }
            }
        }
    }
    return matCof;
};

double GetDeterminant(Mat2D_t matA)
{// Recursive function for finding determinant of a matrix. n is current dimension of matA.
    double det = 0;
    int nRows_matA = matA.size();
    int nCols_matA = matA[0].size();
    int n = 0;

    if(nRows_matA == nCols_matA)
    {
        n = nRows_matA;
    }
    else
    {
        cout << "Only available for a square matrix. Function: GetDeterminant" << endl;
        abort();
    }

    if(n==1) // Base case: if matrix contains single element
    {
        det = matA[0][0];
    }
    else
    {
        //cout << "<<<<<debugging>>>>> n = " << n << endl;
        Mat2D_t matCof = Create2DMat(n, n); // To store cofactors
        int sign = 1; // To store sign multiplier
        for(int col=0; col<n; col++)
        {// Iterate for each element of first row
            matCof = GetCofactorMat(matA, 0, col);
            det += sign * matA[0][col] * GetDeterminant(matCof);
            // Terms are to be added with alternate sign
            sign = -sign;
        }
    }

    return det;
};

Mat2D_t GetAdjointMat (Mat2D_t matA)
{// Function to get adjoint of matA in matAdj
    int nRows_matA = matA.size();
    int nCols_matA = matA[0].size();
    int n = 0;
    int i = 0, j = 0;

    if(nRows_matA == nCols_matA)
    {
        n = nRows_matA;
    }
    else
    {
        cout << "Only available for a square matrix. Function: GetAdjointMat" << endl;
        abort();
    }

    Mat2D_t matAdj = Create2DMat(n, n);
    int sign = 1;
    Mat2D_t matCof = Create2DMat(n, n); // matCof is used to store cofactors of matA.

    if(n==1)
    {
        matAdj[0][0] = 1;
        return matAdj;
    }
    else
    {
        for(int row=0; row<nRows_matA; row++)
        {
            for(int col=0; col<nCols_matA; col++)
            {
                //cout << "<<<<<debugging>>>>>" << endl;
                // Get cofactor of matA
                matCof = GetCofactorMat(matA, i, j);

                // Sign of matAdj[j][i] is positive if sum of row and column indexes is even.
                sign = ((i+j)%2==0)? 1: -1;

                // Interchanging rows and columns to get the transpose of the cofactor matrix
                matAdj[j][i] = (sign)*GetDeterminant(matCof);
            }
        }
        return matAdj;
    }
};

Mat2D_t GetInverseMat(Mat2D_t matA)
{// Function to calculate and store inverse, returns false if matrix is singular
    int nRows_matA = matA.size();
    int nCols_matA = matA[0].size();
    int n = 0;

    if(nRows_matA == nCols_matA)
    {
        n = nRows_matA;
    }
    else
    {
        cout << "Only available for a square matrix. Function: GetInverseMat" << endl;
        abort();
    }

    double det = GetDeterminant(matA);
    
    if(det==0) // Find determinant of matA
    {
        cout << "Singular matrix (det=0), can't get the inverse matrix." << endl;
        abort();
    }
    else
    {
        Mat2D_t matAdj = Create2DMat(n, n);
        Mat2D_t matInv = Create2DMat(n, n);
        matAdj = GetAdjointMat(matA); // Find adjoint
        for(int row=0; row<n; row++)
        {
            for(int col=0; col<n; col++)
            {
                //cout << "<<<<<debugging>>>>>" << endl;
                matInv[row][col] = matAdj[row][col]/det;
            }
        }
        return matInv;
    }
};

void Show2DMat(Mat2D_t mat)
{
    int nRows = mat.size();
    int nCols = mat[0].size();

    cout << "2D matrix display: Rows = " << nRows << ", Cols = " << nCols << endl;

    for(int row=0;row<nRows;row++)
    {
        for(int col=0;col<nCols;col++)
        {
            cout << mat[row][col] << "\t";
            //cout << ((abs(mat[row][col])<EPSILON) ? 0 : mat[row][col]) << "\t";
        }
        cout << endl;
    }
    cout << endl;    
};


int main (int argc, char** argv)
{
    Mat2D_t matA = Create2DMat(4, 4);
    Mat2D_t invA = Create2DMat(4, 4);
    matA = { {5, -2, 2, 7}, {1, 0, 0, 3}, {-3, 1, 5, 0}, {3, -1, -9, 4}};
    cout << "The input matrix A:" << endl;
    Show2DMat(matA);
    invA = GetInverseMat(matA);
    cout << "The inverse matrix of A:" << endl;
    Show2DMat(invA);

    return 0;
};