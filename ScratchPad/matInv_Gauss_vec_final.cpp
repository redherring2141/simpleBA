#include <iostream>
#include <cmath>
#include <vector>

using namespace std;
typedef vector<vector<double>> Mat2D_t;
typedef vector<vector<vector<double>>> Mat3D_t;

Mat2D_t Create2DMat(unsigned nRows, unsigned nCols)
{
   return Mat2D_t(nRows, vector<double>(nCols, 0));
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

Mat2D_t GetAugmentedMat(Mat2D_t matA)
{
    int nRows_matA = matA.size();
    int nCols_matA = matA[0].size();
    int n = 0;

    if(nRows_matA == nCols_matA)
    {
        n = nRows_matA;
    }
    else
    {
        cout << "Only available for a square matrix. Function: GetAugmentedMat" << endl;
        abort();
    }

    Mat2D_t matAug = Create2DMat(n, 2*n);
    for(int row=0; row<n; row++)
    {
        for(int col=0; col<n; col++)
        {
            matAug[row][col] = matA[row][col];
        }
        matAug[row][row+n] = 1;
    }
    return matAug;
};

Mat2D_t GetInverseMat(Mat2D_t matA)
{
    int nRows_matA = matA.size();
    int nCols_matA = matA[0].size();
    int n = 0;

    if((nRows_matA) == nCols_matA)
    {
        n = nRows_matA;
    }
    else
    {
        cout << "Only available for a square matrix. Function: GetInverseMat" << endl;
        abort();
    }

    Mat2D_t matAug = Create2DMat(n, 2*n);
    matAug = GetAugmentedMat(matA);


    for(int i=0; i<n; i++)
    {// Search for the maximum in this column
        double maxEl = abs(matAug[i][i]);
        int maxRow = i;
        for(int k=i+1; k<n; k++)
        {
            if(abs(matAug[k][i]) > maxEl)
            {
                maxEl = matAug[k][i];
                maxRow = k;
            }
        }

        for(int k=i; k<2*n; k++)
        {// Swap the maximum row with current row (column by column)
            double tmp = matAug[maxRow][k];
            matAug[maxRow][k] = matAug[i][k];
            matAug[i][k] = tmp;
        }

        for(int k=i+1; k<n; k++)
        {// Make all rows below this one 0 in current column
            double c = -matAug[k][i]/matAug[i][i];
            for(int j=i; j<2*n; j++)
            {
                if(i==j)
                {
                    matAug[k][j] = 0;
                }
                else
                {
                    matAug[k][j] += c * matAug[i][j];
                }
            }
        }
    }

    for(int i=n-1; i>=0; i--)
    {// Solve equation Ax=b for an upper triangular matrix A
        for(int k=n; k<2*n; k++)
        {
            matAug[i][k] /= matAug[i][i];
        }
        // This is not necessary, but the output looks nicer:
        matAug[i][i] = 1;

        for(int rowModify=i-1; rowModify>=0; rowModify--)
        {
            for(int colModify=n; colModify<2*n; colModify++)
            {
                matAug[rowModify][colModify] -= matAug[i][colModify] * matAug[rowModify][i];
            }
            // This is not necessary, but the output looks nicer:
            matAug[rowModify][i] = 0;
        }
    }

    Mat2D_t matInv = Create2DMat(n, n);
    for(int row=0; row<n; row++)
    {
        for(int col=0; col<n; col++)
        {
            matInv[row][col] = matAug[row][col+n];
        }
    }
    return matInv;
};



int main(int argc, char **argv)
{
    int n = 4;

    Mat2D_t matA = Create2DMat(n, n);
    matA = { {5, -2, 2, 7}, {1, 0, 0, 3}, {-3, 1, 5, 0}, {3, -1, -9, 4} };
    Mat2D_t matAug = Create2DMat(n, 2*n);
    matAug = GetAugmentedMat(matA);
    Mat2D_t matInv = Create2DMat(n, n);
    matInv = GetInverseMat(matA);

    cout << "<<< Input matrix >>>" << endl;
    Show2DMat(matA);

    cout << "<<< Augmented matrix >>>" << endl;
    Show2DMat(matAug);

    cout << "<<< Inverse matrix >>>" << endl;
    Show2DMat(matInv);
};