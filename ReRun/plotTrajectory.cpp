#include <iostream>
#include <fstream>
#include <unistd.h>
#include <vector>

#include <rerun.hpp>
#include <rerun/demo_utils.hpp>
#include <rerun/datatypes/quaternion.hpp>
#include <rerun/archetypes.hpp>

using namespace std;
using namespace rerun::demo;
using namespace rerun::components;
using namespace rerun::datatypes;
using namespace rerun::archetypes;
using rerun::TimeColumn;

string trajectory_file = "../Trajectory.txt";

/*
Quaternion -> x,y,z,w
RotationQuat -> q for rotation
translation3D -> translation
LineStrips3D -> set of Linestrip3D
tick -> timecolumn, rerun viewer timeline
*/
int main(int argc, char **argv)
{
    vector<RotationQuat> q;
    vector<Translation3D> t;
    vector<int64_t> ts;

    ifstream fin(trajectory_file);

    if(!fin)
    {
        cout << "cannot find trajectory file at" << trajectory_file << endl;
        return 1;
    }

    while (!fin.eof())
    {
        double tx, ty, tz, qx, qy, qz, qw;
        string time;
        fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        
        time.erase(0, 8);
        int64_t timestamp = static_cast<int64_t>(stod(time) * 1e9); // nano seconds

        ts.push_back(timestamp);
        q.push_back(Quaternion::from_xyzw(
            static_cast<float>(qx),
            static_cast<float>(qy),
            static_cast<float>(qz),
            static_cast<float>(qw)));
        t.push_back(Vec3D(
            static_cast<float>(tx),
            static_cast<float>(ty),
            static_cast<float>(tz)));  
    }

    cout << "Loaded " << ts.size() << " points" <<endl;

    // for (size_t i = 0; i < q.size(); ++i) 
    // {
    //      auto quat = static_cast<Quaternion>(q[i]);

    //      cout << "Index " << i << ": "
    //           << "w = " << quat.w()
    //           << ", x = " << quat.x()
    //           << ", y = " << quat.y()
    //           << ", z = " << quat.z()
    //           << endl;
    // }

    // for (size_t i = 0; i < ts.size(); ++i) 
    // {
    // cout << "timestamp[" << i << "] = " << ts[i] << " ns" << endl;
    // }
    const auto rec = rerun::RecordingStream("plotTrajectory with Rerun");
    rec.spawn().exit_on_failure();

    rec.set_time_sequence("timestamp", ts[0]);

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


    LineStrip3D trajectory(t);

    rec.log
    (
        "trajectory",
        LineStrips3D(trajectory)
        .with_colors({rerun::Color(255, 0, 0)})
    );

    return 0;
}