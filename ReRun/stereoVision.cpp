#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <unistd.h>
#include <rerun.hpp>
#include "include/CollectionAdapter.hpp"


using namespace std;
using namespace Eigen;
using namespace rerun;

string left_file = "../left.png";
string right_file = "../right.png";


int main(int argc, char **argv) 
{
    float fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
    float b = 0.573;

    cv::Mat left = cv::imread(left_file, 0);
    cv::Mat right = cv::imread(right_file, 0);
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32);    
    cv::Mat disparity_sgbm, disparity;
    sgbm->compute(left, right, disparity_sgbm);
    disparity_sgbm.convertTo(disparity, CV_32F, 1.0 / 16.0f);

    vector<Eigen::Vector3f> pointcloud;
    vector<rerun::Color> colors;

    for (int v = 0; v < left.rows; v++)
    {
        for (int u = 0; u < left.cols; u++) 
        {
            if (disparity.at<float>(v, u) <= 0.0 || disparity.at<float>(v, u) >= 96.0) continue; // invalid disparity

            Vector3f point(0, 0, 0);

            double x = (u - cx) / fx;
            double y = (v - cy) / fy;
            double depth = fx * b / (disparity.at<float>(v, u));
            point[0] = x * depth;
            point[1] = y * depth; 
            point[2] = depth;

            float intensity = left.at<uchar>(v, u);
            colors.push_back(rerun::Color(intensity, intensity, intensity));

            pointcloud.push_back(point);

        }
    }
    
    // cv::Mat to rerun::Collection
    rerun::Collection<uint8_t> left_collection = rerun::CollectionAdapter<uint8_t, cv::Mat>()(left);
    rerun::Collection<uint8_t> right_collection = rerun::CollectionAdapter<uint8_t, cv::Mat>()(right);
    // Vector to rerun::Collection
    rerun::Collection<rerun::Position3D> pointcloud_collection = rerun::CollectionAdapter<rerun::Position3D, std::vector<Eigen::Vector3f>>()(pointcloud);

    const auto rec = rerun::RecordingStream("stereoVision example with OpenCV");
    // rec.spawn().exit_on_failure();

    // Draw stereo left image
    rec.log("left_image",
            rerun::archetypes::Image::from_grayscale8(left_collection,{static_cast<uint32_t>(left.cols), static_cast<uint32_t>(left.rows)}));
    rec.log("right_image",
            rerun::archetypes::Image::from_grayscale8(right_collection,{static_cast<uint32_t>(right.cols), static_cast<uint32_t>(right.rows)}));
    // pinhole camera model
    rec.log("stereo_camera",
            rerun::Pinhole(rerun::Pinhole::from_focal_length_and_resolution({fx, fy}, {static_cast<float>(left.cols), static_cast<float>(left.rows)})));
    // point cloud
    rec.log("world/landmarks", rerun::Points3D(pointcloud_collection).with_colors(colors));

    // save the recording
    rec.save("stereoVision_example.rrd").exit_on_failure();
    return 0;
}
