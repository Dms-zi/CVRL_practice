#include <iostream>
#include <fstream>
#include <unistd.h>
#include <vector>

#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <GL/gl.h>
#include <GL/glu.h>

using namespace std;
using namespace Eigen;

string trajectory_file = "/home/cvrl/ubuntu_20.04/vslam_ws/ch3/Trajectory.txt";

void DrawTrajectory(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>>);

int main(int argc, char **argv){
    vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses;
    ifstream fin(trajectory_file);

    if(!fin){
        cout << "cannot find trajectory file at" << trajectory_file << endl;
        return 1;
    }

    while (!fin.eof()){
        double time, tx, ty, tz, qx, qy, qz, qw;
        fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        
        Isometry3d Twr(Quaterniond(qw,qx,qy,qz));
        Twr.pretranslate(Vector3d(tx,ty,tz));
        poses.push_back(Twr);

        //poses.push_back(Isometry3d(Quaterniond(qw, qx, qy, qz)).pretranslate(Vector3d(tx, ty, tz)));

    }
    cout << "Read total" << poses.size() << " pose entries" << endl;

    // draw trajectory in pangolin
    DrawTrajectory(poses);
    return 0;
}

// Visualize pose trajectory represented by Isometry3d on Pangloin 3d Viewer
void DrawTrajectory(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses){
    //create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);

    glEnable(GL_DEPTH_TEST); // Enables dpeth testing, objects that are closer to the camera appear in front of farther objects.
    glEnable(GL_BLEND); // allow blend color => useful for transparency effects
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // control how colors mix
    
    // camera view setting
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024,758,500,500,512,389,0.1,1000), //resolution, forcal length, central point, near, far
        pangolin::ModelViewLookAt(0,-0.1,-1.8,0,0,0,0.0,1.0,0.0) // camera pose (xyz), viewer poses, upvector(0,1,0 ->y)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0,1.0,0.0,1.0,-1024.0f/768.0f) 
    .SetHandler(new pangolin::Handler3D(s_cam)); // interaction with mouse

    while (pangolin::ShouldQuit() == false){
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f); // RGB, transpearency
        glLineWidth(2);

        for(size_t i=0; i < poses.size(); i++){
            // Draw 3 axes of each pose
            Vector3d Ow = poses[i].translation();
            Vector3d Xw = poses[i] * (0.1 * Vector3d(1,0,0));
            Vector3d Yw = poses[i] * (0.1 * Vector3d(0,1,0));
            Vector3d Zw = poses[i] * (0.1 * Vector3d(0,0,1));
            
            glBegin(GL_LINES);
            glColor3f(1.0,0.0,0.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Xw[0], Xw[1], Xw[2]);

            glColor3f(0.0,1.0,0.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Yw[0], Yw[1], Yw[2]);

            glColor3f(0.0,0.0,1.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Zw[0], Zw[1], Zw[2]);
            
            glEnd;
        }

        // draw a connection
        for(size_t i=0; i<poses.size() - 1; i++){
            glColor3f(0.0,0.0,0.0);
            
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i+1];

            glVertex3d(p1.translation()[0],p1.translation()[1],p1.translation()[2]);
            glVertex3d(p2.translation()[0],p2.translation()[1],p2.translation()[2]);
            glEnd();
        }

        pangolin::FinishFrame();
        usleep(5000); // sleep 5ms
    }

}