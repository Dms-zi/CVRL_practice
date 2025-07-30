#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <boost/format.hpp>  // for formating strings
#include <pangolin/pangolin.h>
#include <sophus/se3.hpp>
#include <rerun.hpp>
#include "include/CollectionAdapter.hpp"

using namespace std;

// typedef vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> TrajectoryType;
// typedef Eigen::Matrix<double, 6, 1> Vector6d;

vector<rerun::components::RotationQuat> q_rotation;
vector<rerun::components::Translation3D> translations;
vector<rerun::archetypes::Transform3D> poses;
vector<rerun::components::Color> colors;
rerun::components::LineStrip3D trajectory;
// void showPointCloud(
//     const vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud);

int main(int argc, char **argv) {
    vector<cv::Mat> colorImgs, depthImgs;    

    ifstream fin("../pose.txt");
    if (!fin) {
        cerr << "not pose.txt in this directory" << endl;
        return 1;
    }
    rerun::archetypes::Transform3D pose;
    for (int i = 0; i < 5; i++) {
        boost::format fmt("./%s/%d.%s"); 
        colorImgs.push_back(cv::imread((fmt % "color" % (i + 1) % "png").str()));
        depthImgs.push_back(cv::imread((fmt % "depth" % (i + 1) % "pgm").str(), -1)); 

        float data[7] = {0};
        for (auto &d:data)
            fin >> d;
        q_rotation.push_back(rerun::datatypes::Quaternion::from_xyzw(data[6], data[3], data[4], data[5]));
        translations.push_back(rerun::datatypes::Vec3D(data[0], data[1], data[2]));

        pose.with_quaternion(q_rotation[i]);
        pose.with_translation(translations[i]);

        poses.push_back(pose);
    }
    cout << "Loaded " << q_rotation.size() << " poses" << endl;
    cout << "Loaded " << translations.size() << " translations" << endl;


    float cx = 325.5;
    float cy = 253.5;
    float fx = 518.0;
    float fy = 519.0;
    float depthScale = 1000.0;

    vector<Vector3f> pointcloud;
    pointcloud.reserve(1000000);

    for (int i = 0; i < 5; i++) {
        cout << "converting image ...: " << i + 1 << endl;
        cv::Mat color = colorImgs[i];
        cv::Mat depth = depthImgs[i];
        for (int v = 0; v < color.rows; v++)
            for (int u = 0; u < color.cols; u++) {
                unsigned int d = depth.ptr<unsigned short>(v)[u]; 
                if (d == 0) continue; 
                Eigen::Vector3f point;
                point[2] = float(d) / depthScale;
                point[0] = (u - cx) * point[2] / fx;
                point[1] = (v - cy) * point[2] / fy;
                pointcloud = translations[i] * point;

                colors.push_back(color.data[v * color.step + u * color.channels()]);   // blue
                colors.push_back(color.data[v * color.step + u * color.channels() + 1]); // green
                colors.push_back(color.data[v * color.step + u * color.channels() + 2]); // red
            }
    }
    // image to rerun::Collection
    rerun::Collection<uint8_t> color_collection = rerun::CollectionAdapter<uint8_t, cv::Mat>()(colorImgs);
    rerun::Collection<uint8_t> depth_collection = rerun::CollectionAdapter<uint8_t, cv::Mat>()(depthImgs);

    // pointcloud to rerun::Collection
    rerun::Collection<rerun::Position3D> pointcloud_collection = rerun::CollectionAdapter<rerun::Position3D, std::vector<Eigen::Vector3f>>()(pointcloud);

     
    const auto rec = rerun::RecordingStream("RGBD example with OpenCV");
    rec.set_time("time", timestamp=time.timestamp())

    rec.log
    (
        "poses",
        Points3D(t[0]).with_colors(rerun::Color(0, 255, 0)),
        // Boxes3D::from_half_sizes({{.5f, .5f, .5f}}).with_fill_mode(rerun::FillMode::Solid),
        Transform3D().with_axis_length(5.0)
    );

    rec.send_columns
    (
        "poses",
        TimeColumn::from_sequence("timestamp", ts),
        Transform3D()
        .with_many_translation(t)
        .with_many_quaternion(q)
        .columns()
    );

    trajectory = LineStrip3D(translations);
    rec.log
    (
        "trajectory",
        lineStrips3D(trajectory);
        .with_colors({rerun::Color(255, 0, 0)})
    );
      
    rec.log
    (
        "world/camera/",
        rerun::Pinhole(rerun::Pinhole::from_focal_length_and_resolution({fx, fy}, {static_cast<float>(color.cols), static_cast<float>(color.rows)})) 
    );
    // depth image
    rec.log
    (
        "world/camera/depth",
        rerun::archetypes::Image::from_rgb24(depth_collection, {static_cast<uint32_t>(depth.cols), static_cast<uint32_t>(depth.rows)})
    );

    // color image
    rec.log
    (
        "world/camera/color",
        rerun::archetypes::Image::from_rgb24(color_collection, {static_cast<uint32_t>(color.cols), static_cast<uint32_t>(color.rows)})
    );



    

    // cout << "global point cloud size : " << pointcloud.size() << endl;
    // showPointCloud(pointcloud);
    return 0;
}



void showPointCloud(const vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud) {

    if (pointcloud.empty()) {
        cerr << "Point cloud is empty!" << endl;
        return;
    }

    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto &p: pointcloud) {
            glColor3d(p[3] / 255.0, p[4] / 255.0, p[5] / 255.0);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
    return;
}
