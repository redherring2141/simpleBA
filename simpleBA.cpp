#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>

#include <cstdio>
#include <cmath>
#include <cassert>
#include <cstdlib>
#include <ctime>
#include <cstring>

#include <random>
//#include <Eigen/Dense>


#define NPOSES 4
#define NPTS 50
#define EPSILON 1e-15
//#define _USE_MATH_DEFINES
//#define PI 3.141592
#define ROTATION_NOISE_STD (0.5/180.0*M_PI)
#define POSITION_NOISE_STD 0.7

// STD deviation on image noise
#define FOCAL_LENGTH    500
#define IMAGE_NOISE_STD (0.3/FOCAL_LENGTH)
#define OUTLIER_PROB    0.1 // Probability of a bad outlier
#define OUTLIER_IMAGE_NOISE_STD (30.0/FOCAL_LENGTH)

#define NUM_ITERATIONS  10
#define START_POSE  3 // (Originally 3 in MATLAB's indexing system)
#define NPOSES_OPT  (NPOSES - START_POSE + 1)

#define RANDOM_FIXED 1 // 1 for load random data generated from MATLAB, 0 for random generation here inside


using namespace std;
//using namespace Eigen;
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
}

Mat2D_t Add2DMat(Mat2D_t matA, Mat2D_t matB)
{
    int nRows_matA = matA.size();
    int nCols_matA = matA[0].size();
    int nRows_matB = matB.size();
    int nCols_matB = matB[0].size();

    if( (nRows_matA==nRows_matB) && (nCols_matA==nCols_matB) )
    {
        assert((nRows_matA==nRows_matB) && (nCols_matA==nCols_matB));
        Mat2D_t matC = Create2DMat(nRows_matA, nCols_matA);
        for(int row=0; row<nRows_matA; row++)
        {
            for(int col=0; col<nCols_matB; col++)
            {
                matC[row][col] = matA[row][col] + matB[row][col];
            }
        }
        return matC;       
    }
    else
    {
        cout << "Matrix dimensions do not match." << endl;
        abort();
    }
};

vector<double> Add1DVec(vector<double> vecA, vector<double> vecB)
{
    int len_vecA = vecA.size();
    int len_vecB = vecB.size();
    if( len_vecA==len_vecB )
    {
        assert(len_vecA==len_vecB);
        vector<double> vecC = Create1DVec(len_vecA);
        for(int idx=0; idx<len_vecA; idx++)
        vecC[idx] = vecA[idx] + vecB[idx];
    }
    else
    {
        cout << "Matrix dimensions do not match" << endl;
        abort();
    }
};

Mat2D_t AddScala2DMat(Mat2D_t mat, double scala)
{
    for(int row=0; row<mat.size(); row++)
    {
        for(int col=0; col<mat[0].size(); col++)
        {
            mat[row][col] += scala;
        }
    }
    return mat;
};

Mat2D_t Mul2DMat(Mat2D_t matA, Mat2D_t matB)
{
    int nRows_matA = matA.size();
    int nCols_matA = matA[0].size();
    int nRows_matB = matB.size();
    int nCols_matB = matB[0].size();
    if(nCols_matA == nRows_matB)
    {
        assert(nCols_matA == nRows_matB);
        Mat2D_t matC = Create2DMat(nRows_matA, nCols_matB);
        for(int row=0; row<nRows_matA; row++)
        {
            for(int col=0; col<nCols_matB; col++)
            {
                for(int idx=0; idx<nCols_matA; idx++)
                {
                    matC[row][col] += matA[row][idx]*matB[idx][col];
                }
            }
        }
        return matC;
    }
    else
    {
        cout << "Input dimensions do not match." << endl;
        cout << "Function: Mul2DMat" << endl;
        cout << "Input matA size = (" << nRows_matA << "," << nCols_matA << ")" << endl;
        cout << "Input matB size = (" << nRows_matB << "," << nCols_matB << ")" << endl;
        abort();
    }
};

Mat2D_t MulScala2DMat(Mat2D_t matX, double scala)
{
    int nRows_matX = matX.size();
    int nCols_matX = matX[0].size();    
    Mat2D_t matY = Create2DMat(nRows_matX, nCols_matX);
    for(int row=0; row<nRows_matX; row++)
    {
        for(int col=0; col<nCols_matX; col++)
        {
            matY[row][col] = matX[row][col] * scala;
        }
    }
    return matY;
};

Mat2D_t Trans2DMat(Mat2D_t mat)
{
    int nRows = mat.size();
    int nCols = mat[0].size();
    Mat2D_t mat_trans = Create2DMat(nCols, nRows);
    for(int row=0; row<nRows; row++)
    {
        for(int col=0; col<nCols; col++)
        {
            mat_trans[col][row] = mat[row][col];
        }
    }
    return mat_trans;
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

void Show1DVec(vector<double> vec)
{
    int len = vec.size();
    cout << "[1D vector display: Length = " << len << "]" << endl;
    for(int idx=0; idx<len; idx++)
    {
        cout << ((abs(vec[idx])<EPSILON) ? 0 : vec[idx]) << "\t";
        //cout << vec[len] << "\t";
    }
    cout << endl << endl;
}

void Show2DMat(Mat2D_t mat)
{
    int nRows = mat.size();
    int nCols = mat[0].size();
    cout << "[2D matrix display: Rows = " << nRows << ", Cols = " << nCols << "]" << endl;

    for(int row=0;row<nRows;row++)
    {
        for(int col=0;col<nCols;col++)
        {
            cout << mat[row][col] << " ";
            //cout << ((abs(mat[row][col])<EPSILON) ? 0 : mat[row][col]) << "\t";
        }
        cout << endl;
    }
    cout << endl;    
};

void Show3DMat(Mat3D_t mat)
{
    int nDepth = mat.size();
    int nRows = mat[0].size();
    int nCols = mat[0][0].size();

    cout << "[3D matrix display: Depths = " << nDepth << ", Rows = " << nRows << ", Cols = " << nCols << "]" << endl;

    for(int depth=0;depth<nDepth;depth++)
    {
        for(int row=0;row<nRows;row++)
        {
            for(int col=0;col<nCols;col++)
            {
                cout << ((abs(mat[depth][row][col])<EPSILON) ? 0 : mat[depth][row][col]) << "\t";
            }
            cout << endl;
        }
        cout << endl;
    }
    cout << endl;    
};

Mat2D_t rot_x(double theta)
{
    Mat2D_t R = Create2DMat(3,3);
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            R[i][j]=0;
        }
    }

    R[0][0] = 1;
    R[0][1] = 0;
    R[0][2] = 0;

    R[1][0] = 0;
    R[1][1] = cos(theta);
    R[1][2] = -sin(theta);

    R[2][0] = 0;
    R[2][1] = sin(theta);
    R[2][2] = cos(theta);

    return R;
};

Mat2D_t rot_y(double theta)
{
    Mat2D_t R = Create2DMat(3,3);
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            R[i][j]=0;
        }
    }

    R[0][0] = cos(theta);
    R[0][1] = 0;
    R[0][2] = sin(theta);

    R[1][0] = 0;
    R[1][1] = 1;
    R[1][2] = 0;

    R[2][0] = -sin(theta);
    R[2][1] = 0;
    R[2][2] = cos(theta);

    return R;
};

Mat2D_t rot_z(double theta)
{
    Mat2D_t R = Create2DMat(3,3);
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            R[i][j]=0;
        }
    }

    R[0][0] = cos(theta);
    R[0][1] = -sin(theta);
    R[0][2] = 0;

    R[1][0] = sin(theta);
    R[1][1] = cos(theta);
    R[1][2] = 0;

    R[2][0] = 0;
    R[2][1] = 0;
    R[2][2] = 1;

    return R;
};

Mat2D_t randn(unsigned nRows, unsigned nCols)
{
    Mat2D_t mat = Create2DMat(nRows, nCols);
    //srand(time(NULL));
    //srand(time(0));
    for(int row=0; row<nRows; row++)
    {
        for(int col=0; col<nCols; col++)
        {
            mat[row][col] = (double)(rand())/(double)(RAND_MAX);
        }
    }
    return mat;
};

Mat2D_t bsxfun_minus(Mat2D_t matA, vector<double> vecB)
{
    int nRows_matA = matA.size();
    int nCols_matA = matA[0].size();
    int len_vecB = vecB.size();
    Mat2D_t ans = Create2DMat(nRows_matA, nCols_matA);

    if(nCols_matA == len_vecB)
    {
        assert(nCols_matA == len_vecB);
        for(int row_A=0; row_A<nRows_matA; row_A++)
        {
            for(int col_A=0; col_A<nCols_matA; col_A++)
            {
                ans[row_A][col_A] = matA[row_A][col_A] - vecB[col_A];
            }
        }
        return ans;
    }
    else
    {
        cout << "Input dimensions do not match." << endl;
        cout << "Function: bsxfun_minus" << endl;
        cout << "Input matA size = (" << nRows_matA << "," << nCols_matA << ")" << endl;
        cout << "Input vecB size = " << len_vecB << endl;
        abort();
    }
};

Mat2D_t bsxfun_rdivide(Mat2D_t matA, vector<double> vecB)
{
    int nRows_matA = matA.size();
    int nCols_matA = matA[0].size();
    int len_vecB = vecB.size();
    Mat2D_t ans = Create2DMat(nRows_matA, nCols_matA);

    if(nCols_matA == len_vecB)
    {
        assert(nCols_matA == len_vecB);
        for(int row_A=0; row_A<nRows_matA; row_A++)
        {
            for(int col_A=0; col_A<nCols_matA; col_A++)
            {
                ans[row_A][col_A] = matA[row_A][col_A] / vecB[col_A];
            }
        }
        return ans;
    }
    else
    {
        cout << "Input dimensions do not match." << endl;
        cout << "Function: bsxfun_minus" << endl;
        cout << "Input matA size = (" << nRows_matA << "," << nCols_matA << ")" << endl;
        cout << "Input vecB size = " << len_vecB << endl;    
        abort();
    }
};

vector<double> binomial_rndgen(unsigned nTrials, double prob)
{
    random_device rnd_seed{};       // use to seed the rng
    mt19937 rnd_engine{rnd_seed()}; // rng

    double p = 0.1; // probability
    bernoulli_distribution dist(p);

    //int nTrials = 50;
    unsigned int nnz = 0;

    vector<double> result = vector<double>(nTrials+1,0);

    // generate 5 runs
    for (size_t i = 0; i < nTrials; ++i)
    {
        result[i] = (double)dist(rnd_engine);
        //cout << result[i] << " ";
        (result[i] == 0.0) ? nnz : nnz++;
    }
    //cout << endl << "Number of nonzeros = " << nnz << endl;

    result[nTrials] = (double) nnz;

    return result;
};

unsigned int nnz(vector<double> outliers)
{
    unsigned int len = outliers.size();
    unsigned ans = 0;
    for(int idx=0; idx<len; idx++)
    {
        if(outliers[idx]>0)
        {
            ans++;
        }
    }
    return ans;
    //return (unsigned int) outliers[outliers.size()-1];
};

unsigned int idx_min(vector<double> vec)
{
    double val_min = vec[0];
    unsigned int idx_ret = 0;
    unsigned len = vec.size();
    for(int idx=0; idx<len; idx++)
    {
        if(vec[idx]<val_min)
        {
            val_min = vec[idx];
        }
    }

    for(int idx=0; idx<len; idx++)
    {
        if(vec[idx]==val_min)
        {
            idx_ret = idx;
            break;
        }
    }
    return idx_ret;
};

void WriteToPLYFile(const string &filename, Mat3D_t cams, Mat2D_t pts)
{
    unsigned num_cameras = cams.size();
    unsigned num_points = pts.size();
    ofstream of(filename.c_str());

    of << "ply"
       << '\n' << "format ascii 1.0"
       << '\n' << "element vertex " << num_cameras + num_points
       << '\n' << "property float x"
       << '\n' << "property float y"
       << '\n' << "property float z"
       << '\n' << "property uchar red"
       << '\n' << "property uchar green"
       << '\n' << "property uchar blue"
       << '\n' << "end_header" << endl;

    // Export extrinsic data (i.e. camera centers) as green points.
    for(int idx_cam = 0; idx_cam < num_cameras; idx_cam++)
    {
        of << cams[idx_cam][0][0] << ' ' << cams[idx_cam][1][0] << ' ' << cams[idx_cam][2][0] << " 0 255 0" << '\n';
    }

    // Export the structure (i.e. 3D Points) as white points.
    for (int idx_pts = 0; idx_pts < num_points; idx_pts++)
    {
        of << pts[idx_pts][0] << ' ' << pts[idx_pts][1] << ' ' << pts[idx_pts][2] << " 255 255 255" << '\n';
    }

    of.close();
};

Mat2D_t skew3(Mat2D_t vecA)
{
    Mat2D_t vecRet = Create2DMat(3, 3);
    vecRet = {  {0, (-1)*vecA[2][0], vecA[1][0]},
                {vecA[2][0], 0, (-1)*vecA[0][0]},
                {(-1)*vecA[1][0], vecA[0][0], 0}  };

    return vecRet;
};

double norm_Mat(Mat2D_t mat)
{
    unsigned len=mat.size();
    double partial_sum = 0;
    double L2norm = 0;
    for(int idx=0; idx<len; idx++)
    {
        partial_sum += pow(mat[idx][0], 2);
    }
    L2norm = sqrt(partial_sum);
    return L2norm;
};

Mat2D_t rodrigues(Mat2D_t omega)
{// Rodrigues formula for 3x1 vector only
    int nRows = omega.size();
    int nCols = omega[0].size();
    if(nRows == 3 && nCols == 1)
    {
        assert(nRows == 3 && nCols == 1);
    }
    else
    {
        cout << "Wrong vector size. Aborting..." << endl;
        abort();
    }

    double theta = norm_Mat(omega);
    double theta2 = 0;
    double sinthetatheta = 0;
    double onecosthetatheta2 = 0;
    if(theta < (sqrt(EPSILON)*100))
    {
        theta2 = Mul2DMat(Trans2DMat(omega), omega)[0][0];
        sinthetatheta = 1 - theta2/6;
        onecosthetatheta2 = 1 - theta2/24;
    }
    else
    {
        theta2 = theta*theta;
        onecosthetatheta2 = (1-cos(theta))/theta2;
        sinthetatheta = sin(theta)/theta;
    }
    Mat2D_t omegav = Create2DMat(3, 3);
    omegav = skew3(omega);
    Mat2D_t R = Create2DMat(3, 3);
    R = Add2DMat(Add2DMat({ {1, 0, 0}, {0, 1, 0}, {0, 0, 1} }, MulScala2DMat(omegav, sinthetatheta)), MulScala2DMat(Mul2DMat(omegav, omegav), onecosthetatheta2));

    return R;
};

vector<double> LoadRawData(const string &filename)
{
   vector<double> data_raw;
   ifstream in(filename);
   for(string line; getline(in, line);)
   {
      stringstream data_str(line);
      //double data;
      for(double data; data_str >> data;)
      {
         data_raw.push_back(data);
      }
   }
   return data_raw;
};

Mat2D_t Reshape1Dto2D(vector<double> rawdata, unsigned nRows, unsigned nCols)
{
   Mat2D_t mat2D = Create2DMat(nRows, nCols);
   for(int idx_row=0; idx_row<nRows; idx_row++)
   {
      for(int idx_col=0; idx_col<nCols; idx_col++)
      {
         mat2D[idx_row][idx_col] = rawdata[idx_row*nCols + idx_col];
      }
   }
   return mat2D;
};

Mat3D_t Reshape1Dto3D(vector<double> rawdata, unsigned nDepths, unsigned nRows, unsigned nCols)
{
   Mat3D_t mat3D = Create3DMat(nDepths, nRows, nCols);
   for(int idx_depth=0; idx_depth<nDepths; idx_depth++)
   {
      for(int idx_row=0; idx_row<nRows; idx_row++)
      {
         for(int idx_col=0; idx_col<nCols; idx_col++)
         {
            mat3D[idx_depth][idx_row][idx_col] = rawdata[idx_depth*nRows*nCols + idx_row*nCols + idx_col];
         }
      }
   }
   return mat3D;
};

int main(int argc, char** argv)
{
    cout << setprecision(4) << fixed;

    // Load random variable data
    vector<double> randn_angs_NPOSES_raw = LoadRawData("./randn_angs_NPOSES.txt");
    Mat3D_t randn_angs_NPOSES_3D = Reshape1Dto3D(randn_angs_NPOSES_raw, NPOSES, 3, 1);
    //Show3DMat(randn_angs_NPOSES_3D);

    vector<double> randn_pos_NPOSES_raw = LoadRawData("./randn_pos_NPOSES.txt");
    Mat3D_t randn_pos_NPOSES_3D = Reshape1Dto3D(randn_pos_NPOSES_raw, NPOSES, 3, 1);
    //Show3DMat(randn_pos_NPOSES_3D);

    vector<double> randn_pts_world_NPTS_raw = LoadRawData("./randn_pts_world_NPTS.txt");
    Mat3D_t randn_pts_world_NPTS_3D = Reshape1Dto3D(randn_pts_world_NPTS_raw, NPTS, 3, 1);
    //Show3DMat(randn_pts_world_NPTS_3D);

    vector<double> randn_pts_img_noisy_NPTS_NPOSES_raw = LoadRawData("./randn_pts_img_noisy_NPTS_NPOSES.txt");
    Mat3D_t randn_pts_img_noisy_NPTS_NPOSES_3D = Reshape1Dto3D(randn_pts_img_noisy_NPTS_NPOSES_raw, NPOSES, NPTS, 2);
    //Show3DMat(randn_pts_img_noisy_NPTS_NPOSES_3D);

    vector<double> outlier_idx_NPOSES_NPTS_raw = LoadRawData("./outlier_idx_NPOSES_NPTS.txt");
    Mat2D_t outlier_idx_NPOSES_NPTS_2D = Reshape1Dto2D(outlier_idx_NPOSES_NPTS_raw, NPOSES, NPTS);
    //Show2DMat(outlier_idx_NPOSES_NPTS_2D);

    vector<double> randn_nnz_outliers_NPOSES_raw = LoadRawData("./randn_nnz_outliers_NPOSES.txt");
    //Show1DVec(randn_nnz_outliers_NPOSES_raw);
    


    // Input 4 initial poses (add more here and increment NPOSES appropriately)
    Mat3D_t wRb_cams = Create3DMat(NPOSES, 3, 3);
    Mat3D_t p_cams = Create3DMat(NPOSES, 3, 1);
    
    wRb_cams[0] = rot_x(-M_PI/2.0);
    p_cams[0] = {{0}, {0}, {0}};

    wRb_cams[1] = Mul2DMat(rot_z(0.4), wRb_cams[0]);
    p_cams[1] = {{1.0}, {0}, {0}};

    wRb_cams[2] = Mul2DMat(rot_z(0.1), wRb_cams[1]);
    p_cams[2] = {{1.3}, {0}, {0}};

    wRb_cams[3] = Mul2DMat(rot_z(-0.5), wRb_cams[0]);
    p_cams[3] = {{-1.3}, {0}, {0}};

    //Show3DMat(wRb_cams);
    //Show3DMat(p_cams);

    // Generate noisy initial guess poses
    Mat3D_t wRb_cams_noisy = Create3DMat(NPOSES, 3, 3);
    Mat3D_t p_cams_noisy = Create3DMat(NPOSES, 3, 1);

    srand(time(NULL));
    for(int idx_cam=0; idx_cam<NPOSES; idx_cam++)
    {
        double noise_scale = (double)max((idx_cam-1),0) / (double)(NPOSES-2);
        //cout << "idx = " << idx << "\tnoise_scale = " << noise_scale << endl;
        Mat2D_t randn_angs = Create2DMat(3, 1);
        Mat2D_t randn_pos = Create2DMat(3, 1);
        if(RANDOM_FIXED)
        {
            randn_angs = randn_angs_NPOSES_3D[idx_cam];
            randn_pos = randn_pos_NPOSES_3D[idx_cam];
        }
        else
        {
            randn_angs = randn(3, 1);
            randn_pos = randn(3, 1);
        }
        Mat2D_t angs = MulScala2DMat(randn_angs, noise_scale*ROTATION_NOISE_STD);        
        Mat2D_t noise_rot = Mul2DMat(Mul2DMat(rot_x(angs[0][0]), rot_y(angs[1][0])), rot_z(angs[2][0]));
        Mat2D_t noise_pos = MulScala2DMat(randn_pos, noise_scale*POSITION_NOISE_STD);
        wRb_cams_noisy[idx_cam] = Mul2DMat(noise_rot, wRb_cams[idx_cam]);
        p_cams_noisy[idx_cam] = Add2DMat(noise_pos, p_cams[idx_cam]);
        //Show2DMat(angs);
        //cout<< "noise_scale*ROTATION_NOISE_STD = " << noise_scale*ROTATION_NOISE_STD << endl;
    }
    //Show3DMat(wRb_cams_noisy);
    //Show3DMat(p_cams_noisy);

    
    // Generate point cloud
    Mat2D_t point_center = Create2DMat(3, 1);
    point_center = {{0}, {4.0}, {0}};
    double point_rad = 1;
    Mat2D_t point_std = Create2DMat(3, 1);
    point_std = {{0.01}, {0.01}, {0.01}};
    Mat2D_t points_world = Create2DMat(NPTS, 3);
    //Mat3D_t points_world = Create3DMat(NPTS, 3, 1);

    srand(time(NULL));
    for(int idx_pts=0; idx_pts<NPTS; idx_pts++)
    {
        Mat2D_t noise_pt = Create2DMat(3, 1);
        Mat2D_t randn_pts_world = Create2DMat(3, 1);
        if(RANDOM_FIXED)
        {
            randn_pts_world = randn_pts_world_NPTS_3D[idx_pts];
        }
        else
        {
            randn_pts_world = randn(3, 1);
        }
        noise_pt = { {point_std[0][0]*randn_pts_world[0][0]}, {point_std[1][0]*randn_pts_world[1][0]}, {point_std[2][0]*randn_pts_world[2][0]} };
        //Show2DMat(noise_pt);

        Mat2D_t R = Mul2DMat(rot_y((double)(idx_pts+1)/(double)NPTS*2*M_PI), rot_z((double)(idx_pts+1)/(double)NPTS*M_PI/3)); // MATLAB idx starts from 1, while C++ starts from 0
        double rad = 0.5 + point_rad * ((double)(idx_pts+1)/(double)NPTS);  // MATLAB idx starts from 1, while C++ starts from 0
        Mat2D_t rad_tmp = Create2DMat(3, 1);
        rad_tmp = {{rad}, {0}, {0}};
        Mat2D_t point = Mul2DMat(R, rad_tmp);
        //vector<double> rad_tmp = {rad, 0, 0};        

        Mat2D_t points_world_tmp = Add2DMat(Add2DMat(point_center, point), noise_pt);
        
        points_world[idx_pts] = {points_world_tmp[0][0], points_world_tmp[1][0], points_world_tmp[2][0]};
        //points_world[idx] = Add1DVec(Add1DVec(point_center, point), noise_pt);
        //Show2DMat(points_world_tmp);  
    }
    //Show2DMat(points_world);

    
    // Project points into images
    Mat3D_t points_image = Create3DMat(NPOSES, 3, NPTS);
    Mat3D_t points_image_noisy = Create3DMat(NPOSES, 3, NPTS);
    for(int idx_cam=0; idx_cam<NPOSES; idx_cam++)
    {
        for(int idx_pts=0; idx_pts<NPTS; idx_pts++)
        {
            points_image_noisy[idx_cam][2][idx_pts] = 1.0;
        }
    }

    //Show3DMat(points_image_noisy);

    
    unsigned total_outliers = 0;
    //Mat2D_t bsxfun_minus_tmp = Create2DMat(NPTS, 3);
    for(int idx_cam=0; idx_cam<NPOSES; idx_cam++)
    {
        Mat2D_t wRb = wRb_cams[idx_cam];
        //Mat2D_t p = Create2DMat(3,1);
        //vector<double> p = Create1DVec(3);
        //Show2DMat(p_cams[idx_cam]);
        vector<double> p = {p_cams[idx_cam][0][0], p_cams[idx_cam][1][0], p_cams[idx_cam][2][0]};
        //p = {p_cams[idx_cam][0][0], p_cams[idx_cam][1][0], p_cams[idx_cam][2][0]};

        //Show2DMat(p);
        //cout << p_cams[idx_cam][0][0] << endl;
        //Show1DVec(p_cams[idx_cam][1]);
        //Show1DVec(p);
        //Show1DVec((vector<double>){p_cams[idx_cam][0][0], p_cams[idx_cam][1][0], p_cams[idx_cam][2][0]});

        //bsxfun_minus_tmp = bsxfun_minus(points_world, p);
        //points_image[idx_cam] = Mul2DMat(Trans2DMat(wRb), bsxfun_minus(points_world, p));
        points_image[idx_cam] = Mul2DMat(Trans2DMat(wRb), Trans2DMat(bsxfun_minus(points_world, p)));
        //points_image[idx_cam] = Mul2DMat(wRb, Trans2DMat(bsxfun_minus(points_world, p)));
        //Show2DMat(points_image[idx_cam]);

        // Divide by camera z coordinate
        vector<double> z_tmp = Create1DVec(NPTS);
        for(int idx_pts=0; idx_pts<NPTS; idx_pts++)
        {
            z_tmp[idx_pts] = points_image[idx_cam][2][idx_pts];
        }
        //Show1DVec(z_tmp);
        
        //points_image[idx_cam] = bsxfun_rdivide(points_image[idx_cam], points_image[idx_cam][3]);
        //Show2DMat(bsxfun_rdivide(points_image[idx_cam], points_image[idx_cam][3]));
        points_image[idx_cam] = bsxfun_rdivide(points_image[idx_cam], z_tmp);
        //Show2DMat(points_image[idx_cam]);
        //cout << "<<<<<<<<<<<<<<<<<<<<<<<<DEBUG MARKER1>>>>>>>>>>>>>>>>>>>>>>" << endl << endl;
        // Add synthetic noise on all features
        //Mat2D_t noise_points_image = Create2DMat(NPTS, 2);
        Mat2D_t noise_points_image = Create2DMat(NPTS, 2);
        Mat2D_t randn_pts_img_noisy = Create2DMat(NPTS, 2);
        if(RANDOM_FIXED)
        {
            randn_pts_img_noisy = randn_pts_img_noisy_NPTS_NPOSES_3D[idx_cam];
        }
        else
        {
            randn_pts_img_noisy = randn(NPTS, 2);
        }
        noise_points_image = MulScala2DMat(randn_pts_img_noisy, IMAGE_NOISE_STD);
        //noise_points_image = MulScala2DMat(randn(NPTS, 2), IMAGE_NOISE_STD);
        //cout << "<<<<<<<<<<<<<<<<<<<<<<<<DEBUG MARKER2>>>>>>>>>>>>>>>>>>>>>>" << endl << endl;
        //Show2DMat(randn_pts_img_noisy);
        //Show2DMat(noise_points_image);
        
        for(int idx_pts=0; idx_pts<NPTS; idx_pts++)
        {
            for(int idx_dim=0; idx_dim<2; idx_dim++)
            {
                //cout << "[idx_cam idx_pts idx] = [" << idx_cam << " " << idx_pts << " " << idx << "]" << endl;
                //cout << "points_image[idx_cam][idx_pts][idx] = " << points_image[idx_cam][idx][idx_pts] << endl;
                //cout << "noise_points_image[idx_pts][idx] = " << noise_points_image[idx_pts][idx] << endl << endl;                
                //cout << "<<<<<<<<<<<<<<<<<<<<<<<<DEBUG MARKER3>>>>>>>>>>>>>>>>>>>>>>" << endl << endl;
                points_image_noisy[idx_cam][idx_dim][idx_pts] = points_image[idx_cam][idx_dim][idx_pts] + noise_points_image[idx_pts][idx_dim];
            }
        }
        //Show2DMat(points_image_noisy[idx_cam]);

        
        // Generate indices of outliers
        vector<double> outlier_idx = Create1DVec(NPTS);
        if(RANDOM_FIXED)
        {
            outlier_idx = outlier_idx_NPOSES_NPTS_2D[idx_cam];
        }
        else
        {
            outlier_idx = binomial_rndgen(NPTS, OUTLIER_PROB);
        }
        //Show1DVec(outlier_idx);
        
        unsigned int num_outliers = nnz(outlier_idx);
        Mat2D_t noise_outliers_image = Create2DMat(num_outliers, 2);
        Mat2D_t randn_nnz_outliers = Create2DMat(num_outliers, 2);
        //cout << "debugging 0" << endl;
        //Show1DVec(randn_nnz_outliers_NPOSES_raw);
        if(RANDOM_FIXED)
        {
            //cout << "num_outliers = " << num_outliers << endl;
            for(int idx=0; idx<num_outliers; idx++)
            {
                //cout << "debugging 1" << endl;
                randn_nnz_outliers[idx][0] = randn_nnz_outliers_NPOSES_raw[2*(idx+total_outliers)];
                randn_nnz_outliers[idx][1] = randn_nnz_outliers_NPOSES_raw[2*(idx+total_outliers)+1];
                //cout << "debugging 2" << endl;
            }
        }
        else
        {
            //cout << "debugging 3" << endl;
            randn_nnz_outliers = randn(num_outliers, 2);
        }
        //Show2DMat(randn_nnz_outliers);
        noise_outliers_image = MulScala2DMat(randn_nnz_outliers, OUTLIER_IMAGE_NOISE_STD);
        //noise_outliers_image = MulScala2DMat(randn(num_outliers, 2), OUTLIER_IMAGE_NOISE_STD);
        //Show2DMat(randn(num_outliers, 2));
        //Show2DMat(MulScala2DMat(randn(num_outliers, 2), OUTLIER_IMAGE_NOISE_STD));
        //Show1DVec(outlier_idx);
        //Show2DMat(noise_outliers_image);

        /*
        cout << "\n<<<<<points_image_noisy>>>>>" << endl;
        Show3DMat(points_image_noisy);
        cout << "\n<<<<<points_image>>>>>" << endl;
        Show3DMat(points_image);
        cout << "\n<<<<<noise_outliers_image>>>>>" << endl;
        Show2DMat(noise_outliers_image);
        cout << "\n<<<<<noise_points_image>>>>>" << endl;
        Show2DMat(noise_points_image);
        */

        total_outliers += num_outliers;
        //cout << "num_outliers = " << num_outliers << endl;
        unsigned idx_outlier = 0;
        //for(int idx_pts=0; idx_pts<num_outliers; idx_pts++)
        for(int idx_pts=0; idx_pts<NPTS; idx_pts++)
        {
            if(outlier_idx[idx_pts]>0)
            {            
                for(int idx_dim=0; idx_dim<2; idx_dim++)
                {

                    points_image_noisy[idx_cam][idx_dim][idx_pts] = points_image[idx_cam][idx_dim][idx_pts] + noise_outliers_image[idx_outlier][idx_dim];
                    /*
                    if(idx_cam==1 && idx_pts==2)
                    {
                        cout << "idx_cam = " << idx_cam << endl;
                        cout << "idx_pts = " << idx_pts << endl;
                        cout << "outlier_idx[idx_pts] = " << outlier_idx[idx_pts] << endl;
                        cout << "points_image_noisy[idx_cam][idx_dim][idx_pts] = " << points_image_noisy[idx_cam][idx_dim][idx_pts] << endl;
                        cout << "points_image[idx_cam][idx_dim][idx_pts] = " << points_image[idx_cam][idx_dim][idx_pts] << endl;
                        cout << "noise_outliers_image[idx_pts][idx_dim] = " << noise_outliers_image[idx_pts][idx_dim] << endl;
                        cout << "randn_nnz_outliers[idx_pts][idx_dim] = " << randn_nnz_outliers[idx_pts][idx_dim] << endl;
                    } 
                    */                   
                    
                }
                idx_outlier++;

                        
                //else
                //{
                //    points_image_noisy[idx_cam][idx_dim][idx_pts] = points_image[idx_cam][idx_dim][idx_pts] + noise_points_image[idx_pts][idx_dim];
                //}
                //

            }
        }
        //Show2DMat(noise_outliers_image);
        //Show2DMat(points_image[idx_cam]);
        //Show2DMat(points_image_noisy[idx_cam]);
        //cout << "Total number of outliers: " << total_outliers << endl;
    }
    
    //Show3DMat(points_image_noisy);
    cout << "Total number of outliers: " << total_outliers << endl;


    
    // Estimated poses
    Mat3D_t wRb_cams_estimate = wRb_cams_noisy;
    Mat3D_t p_cams_estimate = p_cams_noisy;


    //Show3DMat(points_image_noisy);
    //Show3DMat(p_cams_estimate);
    // Triangulate initial guesses on all feature with least squares
    Mat3D_t points_world_estimate = Create3DMat(NPTS, 3, 1);
    
    for(int idx_pts=0; idx_pts<NPTS; idx_pts++)
    {
        Mat2D_t A = Create2DMat(3, 3);
        Mat2D_t b = Create2DMat(3, 1);
        Mat2D_t u = Create2DMat(3, NPOSES);
        Mat2D_t u_new = Create2DMat(3, NPOSES);
        vector<double> u_sqrt = Create1DVec(NPOSES);
        Mat2D_t u_pose = Create2DMat(3, 1);
        Mat2D_t v = Create2DMat(3, 1);
        Mat2D_t B = Create2DMat(3, 3);
        Mat2D_t eye = Create2DMat(3, 3);
        eye = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

        // All observations of this feature, normalized
        //MATLAB_ref: u = squeeze( points_image_noisy(:,i,:)); 
        for(int idx_cam=0; idx_cam<NPOSES; idx_cam++)
        {
            for(int idx_dim=0; idx_dim<3; idx_dim++)
            {
                u[idx_dim][idx_cam] = points_image_noisy[idx_cam][idx_dim][idx_pts];
            }
        }

        //MATLAB_ref: u = bsxfun(@rdivide, u, sqrt(sum(u.^2,1))); 
        for(int idx_cam=0; idx_cam<NPOSES; idx_cam++)
        {
            double sum_col = 0;
            for(int idx_dim=0; idx_dim<3; idx_dim++)
            {
                sum_col += pow(u[idx_dim][idx_cam],2);
            }
            u_sqrt[idx_cam] = sqrt(sum_col);
        }
        u_new = bsxfun_rdivide(u, u_sqrt);


        for(int idx_cam=0; idx_cam<NPOSES; idx_cam++)
        {
            for(int idx_dim=0; idx_dim<3; idx_dim++)
            {
                u_pose[idx_dim][0] = u_new[idx_dim][idx_cam];
            }
            v = Mul2DMat(wRb_cams_estimate[idx_cam], u_pose);
            B = Add2DMat(eye, MulScala2DMat(Mul2DMat(v, Trans2DMat(v)), -1.0));
            A = Add2DMat(A, B);
            //cout <<"Show2DMat(B);" << endl;
            //cout << "[i, j] = [" << idx_pts << ", " << idx_cam << "]" << endl;
            //Show2DMat(u_pose);
            //cout <<"Show2DMat(p_cams_estimate[idx_cam]);" << endl;
            //Show2DMat(p_cams_estimate[idx_cam]);
            //cout << "Show2DMat(Mul2DMat(B, p_cams_estimate[idx_cam]));" << endl;
            //Show2DMat(Mul2DMat(B, p_cams_estimate[idx_cam]));
            b = Add2DMat(b, Mul2DMat(B, p_cams_estimate[idx_cam]));
        }

        // Solve
        points_world_estimate[idx_pts] = Mul2DMat(GetInverseMat(A), b);

        //Show2DMat(b);
    }
    //Show3DMat(points_world_estimate);


    // Find best point
    Mat2D_t point_deltas = Create2DMat(NPTS, 3);
    Mat2D_t points_world_estimate_tmp = Create2DMat(NPTS, 3);
    for(int idx_pts=0; idx_pts<NPTS; idx_pts++)
    {
        for(int idx_dim=0; idx_dim<3; idx_dim++)
        {
            points_world_estimate_tmp[idx_pts][idx_dim] = points_world_estimate[idx_pts][idx_dim][0];
        }
    }
    point_deltas = Add2DMat(points_world, MulScala2DMat(points_world_estimate_tmp, -1));

    vector<double> point_deltas_sqrt = Create1DVec(NPTS);
    for(int idx_pts=0; idx_pts<NPTS; idx_pts++)
    {
        double squared_col_sum = 0;
        for(int idx_dim=0; idx_dim<3; idx_dim++)
        {
            squared_col_sum += pow(point_deltas[idx_pts][idx_dim], 2);
        }
        //cout << "squared_col_sum = " << squared_col_sum << endl;
        point_deltas_sqrt[idx_pts] = sqrt(squared_col_sum);
    }
    unsigned best_point_idx = idx_min(point_deltas_sqrt);
    //Show1DVec(point_deltas_sqrt);
    //cout << "best_point_idx = " << best_point_idx << endl;

    // Convert poses to SE3
    Mat3D_t cam_pose_estimates = Create3DMat(NPOSES, 4, 4);
    for(int idx_cam=0; idx_cam<NPOSES; idx_cam++)
    {
        Mat2D_t wRb = wRb_cams[idx_cam];
        Mat2D_t p = {{p_cams[idx_cam][0][0]}, {p_cams[idx_cam][1][0]}, {p_cams[idx_cam][2][0]}};

        for(int row=0; row<3; row++)
        {
            for(int col=0; col<3; col++)
            {
                cam_pose_estimates[idx_cam][row][col] = Trans2DMat(wRb)[row][col];
            }
            cam_pose_estimates[idx_cam][row][3] = Mul2DMat(MulScala2DMat(Trans2DMat(wRb), -1), p)[row][0];
        }
        cam_pose_estimates[idx_cam][3] = {0, 0, 0, 1};
    }
    Show3DMat(cam_pose_estimates);

    // Run bundle adjustment
    // We will optimize only the poses from START_POSE to NPOSES (inclusive)
    Mat2D_t H = Create2DMat(NPTS*3+NPOSES_OPT*6, NPTS*3+NPOSES_OPT*6);
    for(int iter=0; iter<NUM_ITERATIONS; iter++)
    {
        // Formulate Jacobian and residual
        Mat2D_t J = Create2DMat(NPTS*NPOSES*2, NPTS*3+NPOSES_OPT*6);
        Mat2D_t r = Create2DMat(NPTS*NPOSES*2, 1);

        // Structure of Jacobian
        // [point_0 ... point_i ... point_N | pose_0 ... pose_i ... pose_N]
        for(int idx_pts=0; idx_pts<NPTS; idx_pts++)
        {
            Mat2D_t p_world = points_world_estimate[idx_pts];
            for(int idx_cam=0; idx_cam<NPOSES; idx_cam++)
            {
                // Camera pose
                Mat2D_t H_cam = cam_pose_estimates[idx_cam];
                Mat2D_t p_cam = Mul2DMat(H_cam, (Mat2D_t){{p_world[0][0]}, {p_world[1][0]}, {p_world[2][0]}, {1}});
                Mat2D_t p_cam_truncated = {{p_cam[0][0]}, {p_cam[1][0]}, {p_cam[2][0]}}; // Truncate to remove 1
                double xc = p_cam_truncated[0][0];
                double yc = p_cam_truncated[1][0];
                double zc = p_cam_truncated[2][0];

                // Projection Jacobian (2x3)
                Mat2D_t Jproj = Create2DMat(2, 3);
                Jproj = {{1/zc, 0, -1*xc/(zc*zc)}, {0, 1/zc, -1*yc/(zc*zc)}};

                // Project to image coordinates and calculate residual
                Mat2D_t h_est = Create2DMat(3, 1);
                h_est = MulScala2DMat(p_cam_truncated, 1/p_cam_truncated[2][0]);
                unsigned row = idx_cam*NPTS*2 + idx_pts*2;
                //cout << "row = " << row << endl;
                for(int idx_row=row; idx_row<(row+2); idx_row++)
                {
                    //cout << "<<<<< debugging 8>>>>>" << endl;         
                    r[idx_row][0] = points_image_noisy[idx_cam][idx_row-row][idx_pts] - h_est[idx_row-row][0];
                    //cout << "<<<<< debugging 9>>>>>" << endl;                         
                }
                
                // Pose Jacobian (3x6)
                Mat2D_t Jpose = Create2DMat(3, 6);
                Mat2D_t p_cam_skew = MulScala2DMat(skew3(p_cam), -1);
                Jpose = { {1, 0, 0, p_cam_skew[0][0], p_cam_skew[0][1], p_cam_skew[0][2]},
                          {0, 1, 0, p_cam_skew[1][0], p_cam_skew[1][1], p_cam_skew[1][2]},
                          {0, 0, 1, p_cam_skew[2][0], p_cam_skew[2][1], p_cam_skew[2][2]} };

                // Transform to camera
                Mat2D_t Jpoint = Create2DMat(3, 3);
                for(int idx_row=0; idx_row<3; idx_row++)
                {
                    for(int idx_col=0; idx_col<3; idx_col++)
                    {
                        Jpoint[idx_row][idx_col] = H_cam[idx_row][idx_col];
                    }
                }

                // Insert Jacobians
                if(idx_cam >= (START_POSE-1))
                {// Optimizing pose also
                    int cols_pose = NPTS*3 + (idx_cam - (START_POSE-1))*6;
                    vector<int> idx_cols_pose = vector<int> (6, 0);
                    idx_cols_pose = {cols_pose, cols_pose+1, cols_pose+2, cols_pose+3, cols_pose+4, cols_pose+5};
                    for(int idx_row=row; idx_row<(row+2); idx_row++)
                    {
                        for(int idx_vec=0; idx_vec<6; idx_vec++)
                        {
                            J[idx_row][idx_cols_pose[idx_vec]] = Mul2DMat(Jproj, Jpose)[idx_row-row][idx_vec];
                        }
                    }
                }
                else
                {
                    // Optimizing only points
                }
                int cols_pts = idx_pts*3;
                vector<int> idx_cols_pts = vector<int> (3, 0);
                idx_cols_pts = {cols_pts, cols_pts+1, cols_pts+2};
                for(int idx_row=row; idx_row<(row+2); idx_row++)
                {
                    for(int idx_vec=0; idx_vec<3; idx_vec++)
                    {
                        J[idx_row][idx_cols_pts[idx_vec]] = Mul2DMat(Jproj, Jpoint)[idx_row-row][idx_vec];
                    }
                }                
            }
            //Show2DMat(r);              
        }
        double norm_r = norm_Mat(r);
    
        cout << "Iter " << iter << ", magnitude " << norm_r << endl;

        // Calculate cauchy weights
        vector<double> r2 = vector<double> (r.size(), 0);
        double partial_sum = 0;
        
        for(int idx=0; idx<r.size(); idx++)
        {
            r2[idx]=r[idx][0] * r[idx][0];//pow(r[idx][0],2);
            partial_sum += r2[idx];
        }
        double sigsqrd = partial_sum / r2.size();
        Mat2D_t W = Create2DMat(NPTS*NPOSES*2, NPTS*NPOSES*2);
        for(int idx=0; idx<NPTS*NPOSES*2; idx++)
        {
            W[idx][idx] = 1 / (1 + r2[idx]/sigsqrd);
        }

        //Show2DMat(W);
        //Show2DMat(J);

        // Calculate update (slow and simple method)
        //Mat2D_t H = Create2DMat(NPTS*3+NPOSES_OPT*6, NPTS*3+NPOSES_OPT*6);
        H = Mul2DMat(Mul2DMat(Trans2DMat(J), W), J);
        //Show2DMat(H);
        //Show2DMat(GetInverseMat(H));
        //cout << "H's determinant = " << 
        Mat2D_t dx = Create2DMat(NPTS*3+NPOSES_OPT*6, 1);
        dx = Mul2DMat(GetInverseMat(H), Mul2DMat(Mul2DMat(Trans2DMat(J), W), r));

        //Show2DMat(dx);

        
        // Update points
        Mat3D_t dx_points = Create3DMat(NPTS, 3, 1);
        for(int idx_pts=0; idx_pts<NPTS; idx_pts++)
        {
            for(int idx_dim=0; idx_dim<3; idx_dim++)
            {
                dx_points[idx_pts][idx_dim][0] = dx[idx_pts*3+idx_dim][0];
            }
        }
        
        for(int idx_pts=0; idx_pts<NPTS; idx_pts++)
        {
            for(int idx_dim=0; idx_dim<3; idx_dim++)
            {
                points_world_estimate[idx_pts][idx_dim][0] += dx_points[idx_pts][idx_dim][0];
            }
        }
        

        // Update poses
        Mat2D_t dx_poses = Create2DMat(6, NPOSES_OPT);
        for(int idx_cam=0; idx_cam<NPOSES_OPT; idx_cam++)
        {
            for(int idx_dim=0; idx_dim<6; idx_dim++)
            {
                dx_poses[idx_dim][idx_cam] = dx[idx_cam*6+idx_dim][0];
            }
        }
        
        Mat2D_t twist = Create2DMat(6, 1);
        for(int idx_cam=0; idx_cam<NPOSES_OPT; idx_cam++)
        {
            for(int idx_dim=0; idx_dim<6; idx_dim++)
            {
                twist[idx_dim][0] = dx_poses[idx_dim][idx_cam];
            }
            // Approximate the exponential map
            Mat2D_t twist_4_6 = Create2DMat(3, 1);
            Mat2D_t twist_1_3 = Create2DMat(3, 1);
            twist_1_3 = { {twist[0][0]}, {twist[1][0]}, {twist[2][0]} };
            twist_4_6 = { {twist[3][0]}, {twist[4][0]}, {twist[5][0]} };
            Mat2D_t S = Create2DMat(3, 3);
            S = skew3(twist_4_6);
            Mat2D_t V = Create2DMat(3, 3);
            V = Add2DMat(Add2DMat({ {1, 0, 0}, {0, 1, 0}, {0, 0, 1} }, MulScala2DMat(S, 0.5)), MulScala2DMat(Mul2DMat(S, S), 1/6));
            Mat2D_t update = Create2DMat(4, 4);
            for(int idx_row=0; idx_row<3; idx_row++)
            {
                for(int idx_col=0; idx_col<3; idx_col++)
                {
                    update[idx_row][idx_col] = rodrigues(twist_4_6)[idx_row][idx_col];
                }
                update[idx_row][3] = Mul2DMat(V, twist_1_3)[idx_row][0];
            }
            update[3] = { 0, 0, 0, 1};
            cam_pose_estimates[idx_cam] = Mul2DMat(update, cam_pose_estimates[idx_cam]);
        }      
    }

    // Convert poses back to R, p form
    for(int idx_cam=0; idx_cam<NPOSES; idx_cam++)
    {
        H = cam_pose_estimates[idx_cam];
        Mat2D_t wRb = Create2DMat(3, 3);
        Mat2D_t p = Create2DMat(3, 1);
        Mat2D_t H_13_4 = Create2DMat(3, 1);
        for(int idx_row=0; idx_row<3; idx_row++)
        {
            for(int idx_col=0; idx_col<3; idx_col++)
            {
                wRb[idx_row][idx_col] = H[idx_row][idx_col];
            }
            p[idx_row][0] = Mul2DMat( MulScala2DMat(wRb, -1), H_13_4)[idx_row][0];
        }
        wRb_cams_estimate[idx_cam] = Trans2DMat(wRb);
        p_cams_estimate[idx_cam] = p;
    }

    

    
    //Show3DMat(cam_pose_estimates);
    //cout << "best_point_idx = " << best_point_idx << endl;
    
    //Show1DVec(point_deltas_sqrt);
    

    //Show3DMat(points_world_estimate);

    //Show2DMat(bsxfun_minus_tmp);
    //Show3DMat(points_image);
    //WriteToPLYFile("input_test.ply", p_cams_noisy, points_world);


    

    WriteToPLYFile("input_test.ply", p_cams, points_world);


    Mat2D_t points_world_estimate_2D = Create2DMat(NPTS, 3);
    for(int idx_pts=0; idx_pts<NPTS; idx_pts++)
    {
        for(int idx_dim=0; idx_dim<3; idx_dim++)
        {
            points_world_estimate_2D[idx_pts][idx_dim] = points_world_estimate[idx_pts][idx_dim][0];
        }
    }    
    WriteToPLYFile("output_test.ply", p_cams_estimate, points_world_estimate_2D);
    

    return 0;
};