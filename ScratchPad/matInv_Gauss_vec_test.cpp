#include <iostream>
#include <cmath>
#include <vector>

using namespace std;

void print(vector< vector<double>> &A) {
    int n = A.size();
    std::cout << "A.size() --> " << n << std::endl;
    for (int i=0; i<n; i++) {
        for (int j=0; j<2*n; j++) {
            cout << A[i][j] << "\t";
            if (j == n-1) {
                cout << "| ";
            }
        }
        cout << "\n";
    }
    cout << endl;
}

void calculateInverse(vector< vector<double> >& A) {
    int n = A.size();

    for (int i=0; i<n; i++) {
        // Search for maximum in this column
        double maxEl = abs(A[i][i]);
        int maxRow = i;
        for (int k=i+1; k<n; k++) {
            if (abs(A[k][i]) > maxEl) {
                maxEl = A[k][i];
                maxRow = k;
            }
        }

        // Swap maximum row with current row (column by column)
        for (int k=i; k<2*n;k++) {
            double tmp = A[maxRow][k];
            A[maxRow][k] = A[i][k];
            A[i][k] = tmp;
        }

        // Make all rows below this one 0 in current column
        for (int k=i+1; k<n; k++) {
            double c = -A[k][i]/A[i][i];
            for (int j=i; j<2*n; j++) {
                if (i==j) {
                    A[k][j] = 0;
                } else {
                    A[k][j] += c * A[i][j];
                }
            }
        }
    }

    // Solve equation Ax=b for an upper triangular matrix A
    for (int i=n-1; i>=0; i--) {
        for (int k=n; k<2*n;k++) {
            A[i][k] /= A[i][i];
        }
        // this is not necessary, but the output looks nicer:
        A[i][i] = 1;

        for (int rowModify=i-1;rowModify>=0; rowModify--) {
            for (int columModify=n;columModify<2*n;columModify++) {
                A[rowModify][columModify] -= A[i][columModify]
                                             * A[rowModify][i];
            }
            // this is not necessary, but the output looks nicer:
            A[rowModify][i] = 0;
        }
    }
}

int main() {
    int n;
    /*
    cout << "Size: ";
    cin >> n;
    */
    n=4;

    vector<double> line(2*n,0.0);
    vector< vector<double> > A(n,line);


    // Read input data
    /*
    for (int i=0; i<n; i++) {
        for (int j=0; j<n; j++) {
            cout << "A[" << i << "][" << j << "]: ";
            cin >> A[i][j];
        }
    }
    */
    //A = { {5, -2, 2, 7, 0, 0, 0, 0}, {1, 0, 0, 3, 0, 0, 0, 0}, {-3, 1, 5, 0, 0, 0, 0, 0}, {3, -1, -9, 4, 0, 0, 0, 0} };
    A = { {5, -2, 2, 7}, {1, 0, 0, 3}, {-3, 1, 5, 0}, {3, -1, -9, 4} };
    // A = { {5.0, -2.0, 2.0, 7.0}, {1.0, 0.0, 0.0, 3.0}, {-3.0, 1.0, 5.0, 0.0}, {3.0, -1.0, -9.0, 4.0} };

    // double input[n][n] = { {5, -2, 2, 7}, {1, 0, 0, 3}, {-3, 1, 5, 0}, {3, -1, -9, 4} };
    // for (int i=0; i<n; i++) {
        // for (int j=0; j<n; j++) {
            // A[i][j] = input[i][j];
        // }
    // }



    for (int i=0; i<n; i++) {
        A[i][n+i] = 1.0;
    }

    // Tae
    // for (int i = 0; i < 4; i++){
        // for(int j = 0; j < 8; j++){
            // std::cout << i << " " << j << std::endl;
            // std::cout << A[i][j] << std::endl;
        // }
    // }

    // Print input
    print(A);

    // Calculate solution
    calculateInverse(A);

    // Print result
    cout << "Result:" << endl;
    print(A);
}