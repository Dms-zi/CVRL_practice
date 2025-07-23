#include <iostream>
#include <fstream>

#include <vector>
#include <rerun.hpp>
#include <rerun/demo_utils.hpp>
#include <rerun/datatypes/quaternion.hpp>

using namespace std;
using namespace rerun::demo;
using namespace rerun::components;
using namespace rerun::datatypes;


int main(){
    // // Create a new RecoidngStream which sends data over gRPC to the viewer process
    // const auto rec = rerun::RecordingStream("rerun_example_cpp");

    // // new viewer instance spawn
    // rec.spawn().exit_on_failure();

    // // Create some data using the `grid` utility function.
    // std::vector<rerun::Position3D> points = grid3d<rerun::Position3D, float>(-10.f, 10.f, 10);
    // std::vector<rerun::Color> colors = grid3d<rerun::Color, uint8_t>(0, 255, 10);

    // // Log the "my_points" entity with our data, using the `Points3D` archetype.
    // rec.log("my_points", rerun::Points3D(points).with_colors(colors).with_radii({0.5f}));

    float x,y,z,w;
    x = 0.011409802;
    y = 0.010697415;
    z = 0.002189494;
    w = 0.999875307;

    
    Quaternion q = Quaternion::from_xyzw(x,y,z,w);
    cout << q.w() << endl;
}