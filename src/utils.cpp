#include "utils.h"

namespace simutils {
    void reSizeMat(vector<int> *A, int prevDim, int newDim) {
    vector<int> B(newDim*newDim);
    fill(B.begin(),B.begin() + newDim*newDim, 0);
    for(int i=0;i<newDim*newDim;i++) {
        int r = i/newDim;
        int c = i%newDim;
        if(r < prevDim && c < prevDim) {
            B[i] = A->at(r*prevDim + c);
        }
    }
    *A = B;
}

void printmat(vector<int> *H) {
    int n = sqrt(H->size());
    for(int i=0;i<H->size();i++) {
        int c = i%n;
        int r = i/n;
        cout<<H->at(i)<<" ";
        if(c==n-1) {
            cout<<endl;
        }
    }
    cout<<endl;
}

void blockDiag(vector<int> *H, real_t *Hn, int HnRows) {
    int HnLen = HnRows*HnRows;
    int currDim = sqrt(H->size());
    int nblocks = currDim/HnRows;
    int newDim = HnRows*(nblocks+1);
    reSizeMat(H, currDim, newDim);

    for(int i=0;i<(newDim*newDim);i++) {
        int c = i%newDim;
        int r = i/newDim;
        if(r > currDim -1 || c > currDim -1) {
            if(r<currDim && c > currDim-1) {
                H->at(i) = 0;
            }
            else if(r>currDim-1 && c < currDim) {
                H->at(i) = 0;
            }
            else {
                int r_n = r%HnRows;
                int c_n = c%HnRows;
                H->at(i) = (Hn[r_n*HnRows + c_n]);
            }
        }
    }
}
}