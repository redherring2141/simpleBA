#include "../include/common.hpp"
#include "../include/arrayMat.hpp"

int main(void)
{
    unsigned int dim0 = K;
    unsigned int dim1 = C;
    unsigned int dim2 = R;
    unsigned int dim3 = S;
    int size = dim0*dim1*dim2*dim3;
    string filename = "IA";

    
    unsigned int *mat_IA_dram = new unsigned int [size];
    unsigned int *mat_OA_dram = new unsigned int [size];
    unsigned int mat_OA [dim0][dim1][dim2][dim3];
    for(int i=0; i<size; i++)
    {
        mat_IA_dram[i] = 0;
        mat_OA_dram[i] = 0;
    }

    for(int i=0; i<dim0; i++)
    {
        for(int j=0; j<dim1; j++)
        {
            for(int k=0; k<dim2; k++)
            {
                for(int l=0; l<dim3; l++)
                {
                    mat_OA[i][j][k][l] = 0;
                }
            }
        }
    }    

    
    int num_nonzero = int(size*FW_DENSITY/100); // The number of nonzero elements
    cout << "num_nonzero = " << num_nonzero << " / size = " << size << endl << endl;
    int idx_list[num_nonzero];  // The position list of nonzero elements
    for (int i=0; i<num_nonzero; i++)   // Initialize the list
    {
        idx_list[i]=0;
    }

    // Generate (num_nonzero) number of coprime positions randomly
    // positions = from 1 to H*W
    int idx=1;  // idx = from 1 to (num_nonzero)
    int idx_tmp=0;  // Random number
    bool is_exist=false;   // Flag to check
    
    srand((unsigned int)time(NULL)); 
    idx_tmp = rand()%size; // Random number generated
    idx_list[0] = idx_tmp;

    while(idx<num_nonzero)
    {
        idx_tmp = rand()%size; // Random number generated

        for(int i=0; i<idx; i++)
        {
            if(idx_list[i]==idx_tmp)
            {
                is_exist = true;
                break;
            }
            is_exist = false;
        }

        if(is_exist == false)
        {
            idx_list[idx]=idx_tmp;
            idx++;
        }
    }
    sort(idx_list, idx_list+num_nonzero);

    for(int i=0; i<num_nonzero; i++)
    {
        unsigned int pos0 = (unsigned int) (idx_list[i] / (dim1*dim2*dim3));
        unsigned int pos1 = (unsigned int) ((idx_list[i] - pos0*dim1*dim2*dim3) / (dim2*dim3));
        unsigned int pos2 = (unsigned int) ((idx_list[i] - pos0*dim1*dim2*dim3 - pos1*dim2*dim3) / dim3);
        unsigned int pos3 = (unsigned int) ((idx_list[i] - pos0*dim1*dim2*dim3 - pos1*dim2*dim3 - pos2*dim3) % dim3);
        mat_IA_dram[pos0*dim1*dim2*dim3+pos1*dim2*dim3+pos2*dim3+pos3] = 1;
    }

    for(int i=0; i<dim0; i++)
    {
        for(int j=0; j<dim1; j++)
        {
            cout << "[dim0=" << i << ", dim1=" << j << "]" << endl;
            for(int k=0; k<dim2; k++)
            {
                for(int l=0; l<dim3; l++)
                {
                    cout << mat_IA_dram[i*dim1*dim2*dim3+j*dim2*dim3+k*dim3+l] << " ";
                }
                cout << endl;
            }
            cout << endl;
        }
        cout << endl;
    }
    cout << endl;        

    



    cout << "<<<<<<<<<<Exporting a matrix to a binary file>>>>>>>>>>" << endl;

    string txtFileName = filename + ".txt";
    string binFileName = filename + ".bin";
    string format_txt = filename + "_format.txt";

    FILE* binFile_IA_write = fopen(binFileName.c_str(), "wb");
    fwrite(mat_IA_dram, sizeof(*mat_IA_dram)*size, 1, binFile_IA_write);
    fclose(binFile_IA_write);

    ofstream txtFile(txtFileName);
    ofstream formatFile(format_txt);

    if (txtFile.is_open() && formatFile.is_open())
    {
        for(int i=0; i<dim0; i++)
        {
            for(int j=0; j<dim1; j++)
            {
                txtFile << "[dim0 = " << i << ", dim1 = " << j << "]" << endl;
                for(int k=0; k<dim2; k++)
                {
                    for(int l=0; l<dim3; l++)
                    {
                        txtFile << *(mat_IA_dram+i*dim1*dim2*dim3+j*dim2*dim3+k*dim3+l) << " ";
                        formatFile << *(mat_IA_dram+i*dim1*dim2*dim3+j*dim2*dim3+k*dim3+l) << " ";
                    }
                    txtFile << endl;
                }
                txtFile << endl;
            }
            txtFile << endl;
        }
    }
    else
    {
        cout << "File open error." << endl;
    }

    txtFile.close();
    formatFile.close();





    FILE* binFile_OA_read = fopen(binFileName.c_str(), "rb");
    fread(mat_OA_dram, sizeof(*mat_IA_dram)*size, 1, binFile_OA_read);
    fclose(binFile_OA_read);

    for(int i=0; i<dim0; i++)
    {
        for(int j=0; j<dim1; j++)
        {
            cout << "[dim0=" << i << ", dim1=" << j << "]" << endl;
            for(int k=0; k<dim2; k++)
            {
                for(int l=0; l<dim3; l++)
                {
                    cout << mat_OA_dram[i*dim1*dim2*dim3+j*dim2*dim3+k*dim3+l] << " ";
                }
                cout << endl;
            }
            cout << endl;
        }
        cout << endl;
    }
    cout << endl;
        

    /*
    for(int i=0; i<size; i++)
    {
        unsigned int pos3 = (unsigned int) (i % dim3);
        unsigned int pos2 = (unsigned int) (((i-pos3)/dim3)%dim2);
        unsigned int pos1 = (unsigned int) (((i-pos3-pos2*dim3)/dim2)%dim1);
        unsigned int pos0 = (unsigned int) (((i-pos3-pos2*dim3-pos1*dim2*dim3)/dim1)%dim0);
        mat_OA[pos0][pos1][pos2][pos3] = mat_OA_dram[i];        
    }
    */

    for(int i=0; i<dim0; i++)
    {
        for(int j=0; j<dim1; j++)
        {
            for(int k=0; k<dim2; k++)
            {
                for(int l=0; l<dim3; l++)
                {
                    mat_OA[i][j][k][l] = mat_OA_dram[i*dim1*dim2*dim3+j*dim2*dim3+k*dim3+l];
                }
            }
        }
    }   

    for(int i=0; i<dim0; i++)
    {
        for(int j=0; j<dim1; j++)
        {
            cout << "[dim0=" << i << ", dim1=" << j << "]" << endl;
            for(int k=0; k<dim2; k++)
            {
                for(int l=0; l<dim3; l++)
                {
                    cout << mat_OA[i][j][k][l] << " ";
                }
                cout << endl;
            }
            cout << endl;
        }
        cout << endl;
    }
    cout << endl;    


    delete [] mat_IA_dram;
    delete [] mat_OA_dram;


    return 0;
}