#include "solver.h"
#include "ros/console.h"

Solver::Solver(int nDrones, double maxVel, double maxAcc, int nChecks, double frequency)
    : K(nDrones), maxVel(maxVel), maxAcc(maxAcc), nChecks(nChecks) {
        dt = 1/frequency;
    }

MatrixXf Solver::getHblock(double t0, double t1) {
    int n = 7;
    MatrixXf hblock(n,n);
    hblock.fill(0);
    hblock(0,0) = 2*72*360*(pow(t1,5)-pow(t0,5));
    hblock(1,1) = 2*40*120*(pow(t1,3)-pow(t0,3));
    hblock(2,2) = 2*576*(t1-t0);
    hblock(1,0) = 30*720*(pow(t1,4)-pow(t0,4));
    hblock(0,1) = 30*720*(pow(t1,4)-pow(t0,4));
    hblock(2,1) = 24*120*(pow(t1,2)-pow(t0,2));
    hblock(1,2) = 24*120*(pow(t1,2)-pow(t0,2));
    hblock(0,2) = 16*360*(pow(t1,3)-pow(t0,3));
    hblock(2,0) = 16*360*(pow(t1,3)-pow(t0,3));
    return hblock;
}

vector<Trajectory> Solver::solve(vector<Trajectory> droneWpts, vector<double> tList) {
    nwpts = droneWpts[0].pos.size();
    int nc = getnConstraints();
    int nx = getnVariables();
    MatrixXf A(nc, nx);
    vector<double> lba;
    vector<double> uba;
    ROS_DEBUG_STREAM("nx: "<<nx<<" nc: "<<nc);

    // construct Hessian
    MatrixXf H(K*M*D*n, K*M*D*n);
    for(int k=0;k<K;k++) {
        MatrixXf mstacked(M*D*n, M*D*n);
        for(int m=0;m<M;m++) {
            double t0 = tList[0];
            double t1 = tList[tList.size()-1];
            MatrixXf dstacked(D*n,D*n);
            dstacked.fill(0);
            for(int d=0;d<D;d++) {
                MatrixXf hblock = getHblock(t0, t1);
                dstacked.block(d*7,d*7,n,n) = hblock;
            }
            mstacked.block(7*D*m,7*D*m,7*D,7*D) = dstacked;
        }
        H.block(n*D*M*k,n*D*M*k,M*D*7,M*D*7) = mstacked;
    }

    //construct A and constraint matrices
    for(int k=0;k<K;k++) {
        vector<Vector3d> posList = droneWpts[k].pos;
        MatrixXf dstacked(nc/(K), n*D);
        dstacked.fill(0);
        for(int d=0;d<D;d++) {
            int dConstraints = nc/(K*D);
            MatrixXf mstacked(dConstraints, n);
            mstacked.fill(0);
            vector<double> lba_d;
            vector<double> uba_d;
            for(int m=0;m<nwpts;m++) {
                vector<double> lba_m;
                vector<double> uba_m;
                int ccount = 0;
                double t = tList[m];
                int mdConstraints;
                m==0 || m== nwpts-1 ? mdConstraints = 3: mdConstraints = 1;
                if(nwpts > 0) {
                    m < nwpts - 1 ? mdConstraints += 2*nChecks : mdConstraints = mdConstraints;
                }
                MatrixXf mdblock(mdConstraints, n);
                mdblock.fill(0);

                //position equality
                MatrixXf tPos = getPosTimeVec(t);
                mdblock.block(ccount++,0,1,n) = tPos;

                lba.push_back(posList[m][d]);
                uba.push_back(posList[m][d]);
                if(m==0 || m==posList.size()-1) {
                    //add velocity equality
                    MatrixXf tVel = getVelTimeVec(t);
                    mdblock.block(ccount++,0,1,n) = tVel;
                    lba.push_back(0);
                    uba.push_back(0);
                    //acceleration equality
                    MatrixXf tAcc = getAccTimeVec(t);
                    mdblock.block(ccount++,0,1,n) = tAcc;
                    lba.push_back(0);
                    uba.push_back(0);
                }
                //add interim velocity and acceleration limits
                if(m < nwpts-1) {
                    double t1 = tList[m+1];
                    for(int i=1;i <= nChecks;i++) {
                        double tCheck = t + ((double)i/(nChecks+1))*(t1 - t);
                        MatrixXf tVel = getVelTimeVec(tCheck);
                        mdblock.block(ccount++, 0,1,n) = tVel;                        
                        lba.push_back(-maxVel);
                        uba.push_back(maxVel);

                        MatrixXf tAcc = getAccTimeVec(tCheck);
                        mdblock.block(ccount++,0,1,n) = tAcc;

                        lba.push_back(-maxAcc);
                        uba.push_back(maxAcc);
                    }
                }          

                int rowIdx;
                int mcrows = mdblock.rows();
                m == 0? rowIdx = 0 : rowIdx = 2+m*(2*nChecks + 1);
                mstacked.block(rowIdx, 0, mcrows, n) = mdblock;
            }
            int dColIdx = n*d;
            int dRowIdx = dConstraints*d;
            int dcrows = mstacked.rows();
            int dccols = mstacked.cols();
            dstacked.block(dcrows*d, dccols*d, dcrows, dccols) = mstacked;
        }

        int kRowIdx = dstacked.rows()*k;
        int kColIdx = dstacked.cols()*k;
        A.block(kRowIdx, kColIdx,nc/K,n*D) = dstacked;
    }

    real_t lb_r[lba.size()];
    real_t ub_r[uba.size()];
    copy(lba.begin(), lba.begin()+nc,lb_r);
    copy(uba.begin(), uba.begin()+nc,ub_r);
    real_t* A_r = matrix2realt(A);
    real_t* H_r = matrix2realt(H);

    ROS_DEBUG_STREAM("lba: "<<lba.size()<<endl<<"uba: "<<uba.size());
    real_t g[nx];
    fill(g, g+nx, 0);
    QProblem qp(nx, nc);
    Options options;
    options.setToMPC();
    options.printLevel = PrintLevel::PL_NONE;
	qp.setOptions( options );
	qp.init(H_r,g,A_r,nullptr,nullptr,lb_r,ub_r, nWSR);
	real_t* xOpt = new real_t[nx];
	qp.getPrimalSolution(xOpt);
}

real_t* Solver::matrix2realt(MatrixXf mat) {
    real_t* mat_r = new real_t[mat.size()];
    MatrixXf mat_t = mat.transpose();
    float* ap = mat_t.data();
    for(int i=0;i<mat.size();i++) {
        mat_r[i] = *ap++;
    } 
    return mat_r;
}

Trajectory Solver::calculateTrajectory(vector<double> coefficients, double t0, double t1) {
    Trajectory traj;
        vector<double> xCoeffs[n];
        vector<double> yCoeffs[n];
        vector<double> zCoeffs[n];

        copy(coefficients.begin(), coefficients.begin()+(n-1), xCoeffs);
        copy(coefficients.begin()+n, coefficients.begin()+2*n-1, yCoeffs);
        copy(coefficients.begin()+2*n, coefficients.begin()+3*n-1, zCoeffs);

        for(double t=t0; t<=t1; t+=dt) {
            Vector3d pos;
            // pos[0] = 
    }

}

int Solver::getnConstraints() {
    return (nwpts + 4 + 2*nChecks*(nwpts-1))*D*K;
}

int Solver::getnVariables() {
    return n*K*D;
}