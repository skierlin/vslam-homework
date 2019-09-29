#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sophus/so3.h>
// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace std;

// path to trajectory file
string trajectory_file = "../data/trajectory.txt";
string trajectory_file1 = "../data/groundtruth.txt";
string trajectory_file2 = "../data/estimated.txt";


// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);

void cal_rmse(){

    ifstream posefile1,posefile2;
    posefile1.open(trajectory_file1, ios::in);
    posefile2.open(trajectory_file2, ios::in);

    double sum = 0;
    for(int i = 0; i < 612; i++){
        double data[8] = {0};
        for(auto& d1 : data){
            posefile1 >> d1;
        }
        Eigen::Quaterniond q1(data[7], data[4], data[5], data[6]);
        Eigen::Vector3d t1(data[1], data[2], data[3]); 
        Sophus::SE3 SE3_Rt1(q1, t1);

        for(auto& d2 : data){
            posefile2 >> d2;
        }
        Eigen::Quaterniond q2(data[7], data[4], data[5], data[6]);
        Eigen::Vector3d t2(data[1], data[2], data[3]);
        Sophus::SE3 SE3_Rt2(q2, t2);

        double e = ((SE3_Rt1.inverse()*SE3_Rt2).log()).squaredNorm();

        sum += e;
    }

    double rmse = sqrt(sum/612);

    cout << "rmse = " << rmse << endl;



}

int main(int argc, char **argv) {

    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses;

    /// implement pose reading code
    // start your code here (5~10 lines)
    ifstream posefile;
    posefile.open(trajectory_file, ios::in);
    ifstream posefile1,posefile2;
    posefile1.open(trajectory_file1, ios::in);
    posefile2.open(trajectory_file2, ios::in);
    if (!posefile) {
        cout << "Open file failure." << endl;
        return 1;
    }

    for (int i = 0; i<620; i++) {
        double data[8] = { 0 };
        for (auto& d : data)
            posefile >> d;
        Eigen::Quaterniond q(data[7], data[4], data[5], data[6]);
        Eigen::Vector3d t(data[1], data[2], data[3]);
        Sophus::SE3 SE3_traj(q, t);
        poses.push_back(SE3_traj);
    } 

    cal_rmse();

    for (int i = 0; i<612; i++) {
        double data[8] = { 0 };
        for (auto& d : data)
            posefile1 >> d;
        Eigen::Quaterniond q(data[7], data[4], data[5], data[6]);
        Eigen::Vector3d t(data[1], data[2], data[3]);
        Sophus::SE3 SE3_traj(q, t);
        poses.push_back(SE3_traj);
    } 

    for (int i = 0; i<612; i++) {
        double data[8] = { 0 };
        for (auto& d : data)
            posefile2 >> d;
        Eigen::Quaterniond q(data[7], data[4], data[5], data[6]);
        Eigen::Vector3d t(data[1], data[2], data[3]);
        Sophus::SE3 SE3_traj(q, t);
        poses.push_back(SE3_traj);
    } 

    // end your code here

    // draw trajectory in pangolin
    DrawTrajectory(poses);
    return 0;
}

/*******************************************************************************************/
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses) {
    if (poses.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
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

        glLineWidth(2);
        for (size_t i = 0; i < poses.size() - 1; i++) {
            glColor3f(1 - (float) i / poses.size(), 0.0f, (float) i / poses.size());
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}