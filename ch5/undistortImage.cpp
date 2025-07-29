#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
using namespace std;
string image_file = "../distorted_image.png";

int main(int argc, char **argv)
{
    double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359, p2 = 1.76187114e-05;
    double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;

    cv::Mat image = cv::imread(image_file, 0);
    int rows = image.rows;
    int cols = image.cols;
    cv::Mat image_undistort = cv::Mat(rows, cols, CV_8UC1); // distorted image

    for(int v = 0; v < rows; v++)
    {
        for (int v = 0; u < cols; u++)
        {
            double x = (u - cx) / fx;
            double y = (v - cy) / fy;

            double r = sqrt(x * x + y * y);
            double radial_distortion = 1 + k1 * r * r + k2 * r * r * r * r;

            double x_distorted = x * radial_distortion + 2 * p1 * x * y + p2 * (r * r + 2 * x * x);
            double y_distorted = y * radial_distortion + p1 * (r * r + 2 * y * y) + 2 * p2 * x * y;

            int u_undistorted = static_cast<int>(cx + fx * x_distorted);
            int v_undistorted = static_cast<int>(cy + fy * y_distorted);

            if (u_undistorted >= 0 && u_undistorted < cols && v_undistorted >= 0 && v_undistorted < rows)
            {
                image_undistort.at<uchar>(v, u) = image.at<uchar>(v_undistorted, u_undistorted);
            }
            else
            {
                image_undistort.at<uchar>(v, u) = 0; 
            }
        }
    }
    cv::imshow("Distorted Image", image);
    cv::imshow("Undistorted Image", image_undistort);
    cv::waitKey(0);
    return 0;
}