#include <iostream>
#include <fstream>
#include <vector>

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
#define ROTATION_NOISE_STD (0.5/180*M_PI)
#define POSITION_NOISE_STD 0.7

// STD deviation on image noise
#define FOCAL_LENGTH    500
#define IMAGE_NOISE_STD (0.3/FOCAL_LENGTH)
#define OUTLIER_PROB    0.1 // Probability of a bad outlier
#define OUTLIER_IMAGE_NOISE_STD (30/FOCAL_LENGTH)


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

Mat2D_t MulScala2DMat(Mat2D_t mat, double scala)
{
    for(int row=0; row<mat.size(); row++)
    {
        for(int col=0; col<mat[0].size(); col++)
        {
            mat[row][col] *= scala;
        }
    }
    return mat;
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

void Show1DVec(vector<double> vec)
{
    int len = vec.size();
    cout << "1D vector display: Length = " << len << endl;
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

    cout << "2D matrix display: Rows = " << nRows << ", Cols = " << nCols << endl;

    for(int row=0;row<nRows;row++)
    {
        for(int col=0;col<nCols;col++)
        {
            cout << ((abs(mat[row][col])<EPSILON) ? 0 : mat[row][col]) << "\t";
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

    cout << "3D matrix display: Depths = " << nDepth << ", Rows = " << nRows << ", Cols = " << nCols << endl;

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

vector<unsigned int> binomial_rndgen(unsigned nTrials, double prob)
{
    random_device rnd_seed{};       // use to seed the rng
    mt19937 rnd_engine{rnd_seed()}; // rng

    double p = 0.1; // probability
    bernoulli_distribution dist(p);

    //int nTrials = 50;
    int nnz = 0;

    vector<unsigned int> result = vector<unsigned int>(nTrials+1,0);

    // generate 5 runs
    for (size_t i = 0; i < nTrials; ++i)
    {
        result[i] = (unsigned int)dist(rnd_engine);
        //cout << result[i] << " ";
        (result[i] == true) ? nnz++ : nnz;
    }
    //cout << endl << "Number of nonzeros = " << nnz << endl;

    result[nTrials] = nnz;

    return result;
};

unsigned int nnz(vector<unsigned int> outliers)
{
    return outliers[outliers.size()];
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
}

int main(int argc, char** argv)
{  
    // Input 4 initial poses (add more here and increment NPOSES appropriately)
    Mat3D_t wRb_cams = Create3DMat(NPOSES, 3, 3);
    Mat3D_t p_cams = Create3DMat(NPOSES, 3, 1);
    
    wRb_cams[0] = rot_x(-M_PI/2);
    p_cams[0] = {{0}, {0}, {0}};

    wRb_cams[1] = Mul2DMat(rot_z(0.4), wRb_cams[0]);
    p_cams[1] = {{1.0}, {0}, {0}};

    wRb_cams[2] = Mul2DMat(rot_z(0.1), wRb_cams[1]);
    p_cams[2] = {{1.3}, {0}, {0}};

    wRb_cams[3] = Mul2DMat(rot_z(-0.5), wRb_cams[0]);
    p_cams[3] = {{-1.3}, {0}, {0}};

    // Generate noisy initial guess poses
    Mat3D_t wRb_cams_noisy = Create3DMat(NPOSES, 3, 3);
    Mat3D_t p_cams_noisy = Create3DMat(NPOSES, 3, 1);

    srand(time(NULL));
    for(int idx=0; idx<NPOSES; idx++)
    {
        double noise_scale = (double)max((idx-1),0) / (double)(NPOSES-2);
        //cout << "idx = " << idx << "\tnoise_scale = " << noise_scale << endl;
        Mat2D_t angs = MulScala2DMat(randn(3,1), noise_scale*ROTATION_NOISE_STD);        
        Mat2D_t noise_rot = Mul2DMat(Mul2DMat(rot_x(angs[0][0]), rot_y(angs[1][0])), rot_z(angs[2][0]));
        Mat2D_t noise_pos = MulScala2DMat(randn(3,1), noise_scale*POSITION_NOISE_STD);
        wRb_cams_noisy[idx] = Mul2DMat(noise_rot, wRb_cams[idx]);
        p_cams_noisy[idx] = Add2DMat(noise_pos, p_cams[idx]);
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
    for(int idx=0; idx<NPTS; idx++)
    {
        Mat2D_t noise_pt = Create2DMat(3, 1);
        Mat2D_t pt_tmp= randn(3, 1);
        noise_pt = { {point_std[0][0]*pt_tmp[0][0]}, {point_std[1][0]*pt_tmp[1][0]}, {point_std[2][0]*pt_tmp[2][0]} };
        //Show2DMat(noise_pt);

        Mat2D_t R = Mul2DMat(rot_y((double)idx/(double)NPTS*2*M_PI), rot_z((double)idx/(double)NPTS*M_PI/3));
        double rad = 0.5 + point_rad * ((double)idx/(double)NPTS);
        Mat2D_t rad_tmp = Create2DMat(3, 1);
        rad_tmp = {{rad}, {0}, {0}};
        Mat2D_t point = Mul2DMat(R, rad_tmp);
        //vector<double> rad_tmp = {rad, 0, 0};        

        Mat2D_t points_world_tmp = Add2DMat(Add2DMat(point_center, point), noise_pt);
        
        points_world[idx] = {points_world_tmp[0][0], points_world_tmp[1][0], points_world_tmp[2][0]};
        //points_world[idx] = Add1DVec(Add1DVec(point_center, point), noise_pt);
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
        for(int idx=0; idx<NPTS; idx++)
        {
            z_tmp[idx] = points_image[idx_cam][2][idx];
        }
        //Show1DVec(z_tmp);
        
        //points_image[idx_cam] = bsxfun_rdivide(points_image[idx_cam], points_image[idx_cam][3]);
        //Show2DMat(bsxfun_rdivide(points_image[idx_cam], points_image[idx_cam][3]));
        points_image[idx_cam] = bsxfun_rdivide(points_image[idx_cam], z_tmp);
        //Show2DMat(points_image[idx_cam]);
        //cout << "<<<<<<<<<<<<<<<<<<<<<<<<DEBUG MARKER1>>>>>>>>>>>>>>>>>>>>>>" << endl << endl;
        // Add synthetic noise on all features
        //Mat2D_t noise_points_image = Create2DMat(NPTS, 2);
        Mat2D_t noise_points_image = MulScala2DMat(randn(NPTS, 2), IMAGE_NOISE_STD);
        //cout << "<<<<<<<<<<<<<<<<<<<<<<<<DEBUG MARKER2>>>>>>>>>>>>>>>>>>>>>>" << endl << endl;

        /*
        for(int idx_pts=0; idx_pts<NPTS; idx_pts++)
        {
            for(int idx=0; idx<2; idx++)
            {
                //cout << "[idx_cam idx_pts idx] = [" << idx_cam << " " << idx_pts << " " << idx << "]" << endl;
                //cout << "points_image[idx_cam][idx_pts][idx] = " << points_image[idx_cam][idx][idx_pts] << endl;
                //cout << "noise_points_image[idx_pts][idx] = " << noise_points_image[idx_pts][idx] << endl << endl;                
                //cout << "<<<<<<<<<<<<<<<<<<<<<<<<DEBUG MARKER3>>>>>>>>>>>>>>>>>>>>>>" << endl << endl;
                points_image_noisy[idx_cam][idx][idx_pts] = points_image[idx_cam][idx][idx_pts] + noise_points_image[idx_pts][idx];
            }
        }
        */
        //Show2DMat(points_image_noisy[idx_cam]);

        // Generate indices of outliers
        vector<unsigned int> outlier_idx = binomial_rndgen(NPTS, OUTLIER_PROB);
        unsigned int num_outliers = nnz(outlier_idx);
        total_outliers += num_outliers;
        Mat2D_t noise_outliers_image = MulScala2DMat(randn(num_outliers, 2), OUTLIER_IMAGE_NOISE_STD);
        for(int idx_pts=0; idx_pts<NPTS; idx_pts++)
        {
            for(int idx=0; idx<2; idx++)
            {
                if(outlier_idx[idx_pts]==1)
                {
                    points_image_noisy[idx_cam][idx][idx_pts] = points_image[idx_cam][idx][idx_pts] + noise_outliers_image[idx_pts][idx];
                }
                else
                {
                    points_image_noisy[idx_cam][idx][idx_pts] = points_image[idx_cam][idx][idx_pts] + noise_points_image[idx_pts][idx];
                }
            }
        }
    }
    //Show2DMat(bsxfun_minus_tmp);
    //Show3DMat(points_image);
    WriteToPLYFile("input_test.ply", p_cams_noisy, points_world);

    return 0;
};