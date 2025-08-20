#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv) 
{
    double ar = 1.0, br = 2.0, cr = 1.0 ; // real sensor parameters
    double ae = 2.0, be = -1.0, ce = 5.0; // estimated sensor parameters
    
    int N = 100; // number of data points
    double w_sigma = 0.1; // noise standard deviation
    double inv_sigma = 1.0 / w_sigma;
    cv::RNG rng; // random number generator

    vector<double> x_data, y_data;
    for (int i = 0; i < N; i++)
    {
        double x = i / 100.0; // generate x data
        x_data.push_back(x);
        y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma)); // generate y data with noise
    }

    // Gauss-Newton optimization
    int step = 100;
    double cost = 0.0 ,lastCost = 0.0;

    chrono::steady_clock::time_point start = chrono::steady_clock::now();
    for(int iter = 0; iter < step; iter++)
    {
        Matrix3d H = Matrix3d::Zero(); // Hessian matrix
        Vector3d b = Vector3d::Zero(); // bias
        cost = 0.0; // reset cost

        for(int j = 0; j < N; j++)
        {
            double xj = x_data[j], yj = y_data[j];
            double loss = yj - exp(ae * xj * xj + be * xj + ce); // loss function
            Vector3d J; // Jacobian vector
            J[0] = -xj * xj * exp(ae * xj * xj + be * xj + ce); // derivative w.r.t. ae
            J[1] = -xj * exp(ae * xj * xj + be * xj + ce); // derivative w.r.t. be
            J[2] = -exp(ae * xj * xj + be * xj + ce); // derivative w.r.t. ce

            H +=  inv_sigma * inv_sigma * J * J.transpose(); // accumulate Hessian
            b +=  -inv_sigma * inv_sigma * loss * J; // accumulate bias

            cost += loss * loss; // accumulate cost
        }

        // solve for parameter update
        Vector3d dx = H.ldlt().solve(b); // solve linear system
        if (isnan(dx[0])) 
        {
            cout << "result in NaN" << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost)
        {
            cout << "cost: " << cost << ", lastCost: " << lastCost << endl;
            break;
        }

        ae += dx[0]; // update ae
        be += dx[1]; // update be
        ce += dx[2]; // update ce
        lastCost = cost; // update last cost

        cout << "Iteration " << iter << ": cost = " << cost << ", ae = " << ae << ", be = " << be << ", ce = " << ce  << ",\n update" << dx.transpose() << endl;
    }

    chrono::steady_clock::time_point end = chrono::steady_clock::now();
    auto duration = chrono::duration_cast<chrono::milliseconds>(end - start);
    cout << "Optimization finished in " << duration.count() << " ms" << endl;
    cout << "Final estimated abc: ae = " << ae << ", be = " << be << ", ce = " << ce << endl;
    return 0;
}