#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>
#include <random>

#include <cstdio>
#include <cmath>
#include <cassert>
#include <cstdlib>
#include <ctime>
#include <cstring>


#define NPOSES 4
#define NPTS 50
#define NUM_ITERATIONS  10
#define START_POSE  2 // (Originally 3 in MATLAB's indexing system)
#define NPOSES_OPT  (NPOSES - START_POSE + 1)

#define RANDOM_FIXED 1 // 1 for load random data generated from MATLAB, 0 for random generation here inside

#define EPSILON 1e-15
#define ROTATION_NOISE_STD (0.5/180.0*M_PI)
#define POSITION_NOISE_STD 0.7

// STD deviation on image noise
#define FOCAL_LENGTH    500
#define IMAGE_NOISE_STD (0.3/FOCAL_LENGTH)
#define OUTLIER_PROB    0.1 // Probability of a bad outlier
#define OUTLIER_IMAGE_NOISE_STD (30.0/FOCAL_LENGTH)

using namespace std;

typedef vector<vector<double>> Mat2D_t;
typedef vector<vector<vector<double>>> Mat3D_t;


Mat2D_t Create2DMat(unsigned nRows, unsigned nCols)
{
   return Mat2D_t(nRows, vector<double>(nCols, 0));
};

Mat3D_t Create3DMat(unsigned nDepth, unsigned nRows, unsigned nCols)
{
   return Mat3D_t(nDepth, Mat2D_t(nRows, vector<double>(nCols,0)));
};

vector<double> Create1DVec(unsigned length)
{
    return vector<double>(length,0);
};

double CalcDeterminant(Mat2D_t Matrix)
{
    //this function is written in c++ to calculate the determinant of matrix
    // it's a recursive function that can handle matrix of any dimension
    double det = 0; // the determinant value will be stored here
    if (Matrix.size() == 1)
    {
        return Matrix[0][0]; // no calculation needed
    }
    else if (Matrix.size() == 2)
    {
        //in this case we calculate the determinant of a 2-dimensional matrix in a 
        //default procedure
        det = (Matrix[0][0] * Matrix[1][1] - Matrix[0][1] * Matrix[1][0]);
        return det;
    }
    else
    {
        //in this case we calculate the determinant of a squared matrix that have 
        // for example 3x3 order greater than 2
        for (int p = 0; p < Matrix[0].size(); p++)
        {
            //this loop iterate on each elements of the first row in the matrix.
            //at each element we cancel the row and column it exist in
            //and form a matrix from the rest of the elements in the matrix
            Mat2D_t TempMatrix; // to hold the shaped matrix;
            for (int i = 1; i < Matrix.size(); i++)
            {
                // iteration will start from row one cancelling the first row values
                vector<double> TempRow;
                for (int j = 0; j < Matrix[i].size(); j++)
                {
                    // iteration will pass all cells of the i row excluding the j 
                    //value that match p column
                    if (j != p)
                    {
                        TempRow.push_back(Matrix[i][j]);//add current cell to TempRow 
                    }
                }
                if (TempRow.size() > 0)
                    TempMatrix.push_back(TempRow);
                //after adding each row of the new matrix to the vector tempx
                //we add it to the vector temp which is the vector where the new 
                //matrix will be formed
            }
            det = det + Matrix[0][p] * pow(-1, p) * CalcDeterminant(TempMatrix);
            //then we calculate the value of determinant by using a recursive way
            //where we re-call the function by passing to it the new formed matrix
            //we keep doing this until we get our determinant
            cout << "p, det = " << p << ", " << det << endl;
        }
        return det;
    }
};

Mat2D_t randn(unsigned nRows, unsigned nCols)
{
    Mat2D_t mat = Create2DMat(nRows, nCols);
    for(int row=0; row<nRows; row++)
    {
        for(int col=0; col<nCols; col++)
        {
            mat[row][col] = (double)(rand())/(double)(RAND_MAX);
        }
    }
    return mat;
};

int main(int argc, char** argv)
{
    //Mat2D_t mat_test = Create2DMat(4, 4);
    //mat_test = { {1, 0, 4, -6}, {2, 5, 0, 3}, {-1, 2, 3, 5}, {2, 1, -2, 3} };
    Mat2D_t mat_test = randn(10, 10);
    double det = 0;
    det = CalcDeterminant(mat_test);

    cout << "Determinant = " << det << endl;

    return 0;
}