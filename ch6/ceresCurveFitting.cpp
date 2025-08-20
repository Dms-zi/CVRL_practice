#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>

using namespace std;

struct CURVE_FITTING_COST
{
    CURVE_FITTING_COST(double x, double y) : x_(x), y_(y) {}

    template <typename T>
    bool operator()(const T* const abc, T* residual) const 
    {
        residual[0] = T(y_) - ceres::exp(abc[0] * x_ * x_ + abc[1] * x_ + abc[2]);
        return true;
    }

    double x_;
    double y_;
};

int main(int argc, char **argv) 
{
    double ar = 1.0, br = 2.0, cr = 1.0; // real sensor parameters
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
    double abc[3] = {ae, be, ce}; // initial guess for parameters

    // Ceres optimization setup
    ceres::Problem problem;
    for (int i = 0; i < N; i++)
    {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
                new CURVE_FITTING_COST(x_data[i], y_data[i])
            ),
            nullptr,
            abc
        );
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;
    
    ceres::Solver::Summary summary;
    chrono::steady_clock::time_point start = chrono::steady_clock::now();
    
    ceres::Solve(options, &problem, &summary);
    
    chrono::steady_clock::time_point end = chrono::steady_clock::now();
    auto duration = chrono::duration_cast<chrono::milliseconds>(end - start);
    
    cout << "Optimization finished in " << duration.count() << " ms" << endl;

    cout << summary.BriefReport() << endl;
    cout << "estimated parameters: "
         << "a = " << abc[0] << ", b = " << abc[1] << ", c = " << abc[2] << endl;


    return 0;

};