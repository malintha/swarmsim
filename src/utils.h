#include <qpOASES.hpp>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include <algorithm> 

using namespace std;
USING_NAMESPACE_QPOASES

namespace simutils {
    void blockDiag(vector<int> *H, real_t *Hn, int HnRows);
    void printmat(vector<int> *H);
    void reSizeMat(vector<int> *A, int prevDim, int newDim);
}