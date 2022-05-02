#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <string>
#include <vector>

#include <cstdio>
#include <cmath>
#include <cassert>
#include <cstdlib>
#include <ctime>
#include <cstring>

#include <random>

#define NPOSES 4
#define NPTS 50
#define EPSILON 1e-15

using namespace std;

using TYPE = double;

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


//------------------------------------------------

template< typename T > vector< vector<T> > loaDTxtStream( const string &filename )
{
   vector< vector<T> > data;
   ifstream in( filename );
   for ( string line; getline( in, line ); )
   {
      stringstream ss( line );
      vector<T> row;
      for ( T d; ss >> d; ) row.push_back( d );
      data.push_back( row );
   }
   return data;
}

//------------------------------------------------

template< typename T > void print( const vector< vector<T> > &data )
{
   for ( auto &row : data )
   {
      for ( auto &item : row ) cout << setw( 1 ) << item << ' ';
      cout << '\n';
   }
}


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

Mat2D_t LoadMat2D( const string &filename)
{
   Mat2D_t mat2D;
   ifstream in(filename);
   for(string line; getline(in, line);)
   {
      stringstream ss(line);
      vector<double> data_row;
      for(double data_col; ss >> data_col;)
      {
         data_row.push_back(data_col);
      }
      mat2D.push_back(data_row);
   }
   return mat2D;
}

void WriteMat2DTxtStream(Mat2D_t mat2D, const string &filename)
{
   ofstream txtFile(filename);
   unsigned int nRows = mat2D.size();
   unsigned int nCols = mat2D[0].size();

   if (txtFile.is_open())
   {
      for(int idx_row=0; idx_row<nRows; idx_row++)
      {
         for(int idx_col=0; idx_col<nCols; idx_col++)
         {
            txtFile << mat2D[idx_row][idx_col] << endl;
         }
      }
   }
   txtFile.close();
};

void WriteMat3DTxtStream(Mat3D_t mat3D, const string &filename)
{
   ofstream txtFile(filename);
   unsigned int nDepths = mat3D.size();
   unsigned int nRows = mat3D[0].size();
   unsigned int nCols = mat3D[0][0].size();

   if (txtFile.is_open())
   {
      for(int idx_depth=0; idx_depth<nDepths; idx_depth++)
      {      
         for(int idx_row=0; idx_row<nRows; idx_row++)
         {
            for(int idx_col=0; idx_col<nCols; idx_col++)
            {
               txtFile << setw(8) << mat3D[idx_depth][idx_row][idx_col] << endl;
            }
         }
      }
   }
   txtFile.close();
};

void WriteMat3DTxtFprintf(Mat3D_t mat3D, const char* filename)
{
   unsigned int nDepths = mat3D.size();
   unsigned int nRows = mat3D[0].size();
   unsigned int nCols = mat3D[0][0].size();

   FILE* fp = fopen(filename, "w");

   for(int idx_depth=0; idx_depth<nDepths; idx_depth++)
   {      
      for(int idx_row=0; idx_row<nRows; idx_row++)
      {
         for(int idx_col=0; idx_col<nCols; idx_col++)
         {
            fprintf(fp, "%4.3f\n", mat3D[idx_depth][idx_row][idx_col]);
         }
      }
   }
   fclose(fp);
};

void WriteMat2DTxtFprintf(Mat2D_t mat2D, char* filename)
{
   unsigned int nRows = mat2D.size();
   unsigned int nCols = mat2D[0].size();

   FILE* fp = fopen(filename, "w");

   for(int idx_row=0; idx_row<nRows; idx_row++)
   {
      for(int idx_col=0; idx_col<nCols; idx_col++)
      {
         fprintf(fp, "%4.3f\n", mat2D[idx_row][idx_col]);
      }
   }
   fclose(fp);
};

//======================================================================

int main()
{

   cout << setprecision(4) << fixed;
   /*
   auto data = loaDTxtStream<TYPE>( "../randn_nnz_outliers_NPOSES.txt" );
   for(int idx_row=0; idx_row<data.size(); idx_row++)
   {
       for(int idx_col=0; idx_col<data[0].size(); idx_col++)
       {
           cout << data[idx_row][idx_col] << " ";
       }
       cout << endl;
   }
   Show2DMat(data);
   */
   
    
    vector<double> randn_angs_NPOSES_raw = LoadRawData("../randn_angs_NPOSES.txt");
    Mat3D_t randn_angs_NPOSES_3D = Reshape1Dto3D(randn_angs_NPOSES_raw, NPOSES, 3, 1);
    Show3DMat(randn_angs_NPOSES_3D);

    vector<double> randn_pos_NPOSES_raw = LoadRawData("../randn_pos_NPOSES.txt");
    Mat3D_t randn_pos_NPOSES_3D = Reshape1Dto3D(randn_pos_NPOSES_raw, NPOSES, 3, 1);
    Show3DMat(randn_pos_NPOSES_3D);

    vector<double> randn_pts_world_NPTS_raw = LoadRawData("../randn_pts_world_NPTS.txt");
    Mat3D_t randn_pts_world_NPTS_3D = Reshape1Dto3D(randn_pts_world_NPTS_raw, NPTS, 3, 1);
    Show3DMat(randn_pts_world_NPTS_3D);

    vector<double> randn_pts_img_noisy_NPTS_NPOSES_raw = LoadRawData("../randn_pts_img_noisy_NPTS_NPOSES.txt");
    Mat3D_t randn_pts_img_noisy_NPTS_NPOSES_3D = Reshape1Dto3D(randn_pts_img_noisy_NPTS_NPOSES_raw, NPOSES, NPTS, 2);
    Show3DMat(randn_pts_img_noisy_NPTS_NPOSES_3D);

    vector<double> outlier_idx_NPOSES_NPTS_raw = LoadRawData("../outlier_idx_NPOSES_NPTS.txt");
    Mat2D_t outlier_idx_NPOSES_NPTS_2D = Reshape1Dto2D(outlier_idx_NPOSES_NPTS_raw, NPOSES, NPTS);
    Show2DMat(outlier_idx_NPOSES_NPTS_2D);

    vector<double> randn_nnz_outliers_NPOSES_raw = LoadRawData("../randn_nnz_outliers_NPOSES.txt");
    Show1DVec(randn_nnz_outliers_NPOSES_raw);

    //WriteMat3DTxtStream(randn_angs_NPOSES_3D, "../randn_angs_NPOSES_fileouttest.txt");
    //WriteMat2DTxtStream(outlier_idx_NPOSES_NPTS_2D, "../outlier_idx_NPOSES_NPTS_fileouttest.txt");

    WriteMat3DTxtFprintf(randn_angs_NPOSES_3D, "../randn_angs_NPOSES_fileouttest.txt");
    WriteMat2DTxtFprintf(outlier_idx_NPOSES_NPTS_2D, "../outlier_idx_NPOSES_NPTS_fileouttest.txt");
    

    



   //print( data );
}