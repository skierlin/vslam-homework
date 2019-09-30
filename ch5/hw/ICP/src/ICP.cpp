#include <sophus/se3.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <unistd.h>

// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

using namespace std;
typedef vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> vecSE3;

// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);
void DrawTrajectory(vector<Eigen::Vector3d> poses1, vector<Eigen::Vector3d> poses2);
void ReadData(const string &filename, vector<Eigen::Vector3d> &input1, vector<Eigen::Vector3d> &input2);
void ICP_SVD();
void ICP_GN(int iter_num );
double compute_ess(Sophus::SE3 in1, Sophus::SE3 in2);

int main(int argc, char **argv) {

    //ICP_SVD();  // SVD分解法
    ICP_GN(300);  // 高斯牛顿非线性优化法

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

            glColor3f(1, 0.0f, 0.0f);
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

void ReadData(const string &filename, vector<Eigen::Vector3d> &input1, vector<Eigen::Vector3d> &input2)
{
    ifstream infile;
    infile.open(filename);

    Eigen::Vector3d t1, t2;
    Eigen::Quaterniond q1, q2;
    double one_time1, one_time2;


    if(!infile)
    {
        cout << "input file err" << endl;
    }
    while(!infile.eof())
    {
        infile >> one_time1 >> t1(0) >> t1(1) >> t1(2) >> q1.x() >> q1.y() >> q1.z() >> q1.w() \
               >> one_time2 >> t2(0) >> t2(1) >> t2(2) >> q2.x() >> q2.y() >> q2.z() >> q2.w();

        input1.push_back(t1);
        input2.push_back(t2);
    }
    infile.close();
    return;
};

double compute_ess(Sophus::SE3 in1, Sophus::SE3 in2)
{
    double ess(0.0);
    Eigen::VectorXd log_ess;
    log_ess = (in1.inverse() * in2).log();
    for (int i = 0; i < log_ess.size(); ++i)
    {
        ess += log_ess(i) * log_ess(i);
    }
    ess = sqrt(ess);

    return ess;
}

void ICP_SVD()
{
    vector<Eigen::Vector3d> P1, P2, P2_;
    ReadData("/compare.txt", P1, P2);

    Eigen::Vector3d p1_mean, p2_mean;
    for (int i = 0; i < P1.size(); ++i) {
        p1_mean += P1[i];
        p2_mean += P2[i];
    }
    p1_mean = p1_mean / P1.size();
    p2_mean = p2_mean / P2.size();
    Eigen::Matrix<double, 3, 3 > W;
    for (int i = 0; i < P1.size(); ++i) {
        Eigen::Vector3d q1, q2;
        q1 = P1[i] - p1_mean;
        q2 = P2[i] - p2_mean;
        W += q1 * q2.transpose();
    }
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullV | Eigen::ComputeFullU);
    Eigen::Vector3d x = svd.singularValues();
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    Eigen::Matrix3d R = U * V.transpose();
    Eigen::Vector3d t = p1_mean - R * p2_mean;

    for (int i = 0; i < P2.size(); ++i) {
        P2_.push_back(R * P2[i] + t);
    }
    // draw trajectory in pangolin
    DrawTrajectory(P1, P2_);

    return;
}

void ICP_GN(int iter_num)
{
    vector<Eigen::Vector3d> P1, P2, P2_;
    ReadData("../compare.txt", P1, P2);
    long double cost = 0, last_cost = 0;
    Sophus::SE3 T(Eigen::Matrix3d::Identity(3, 3), Eigen::Vector3d(0, 0, 0));
    for (int i = 0; i < iter_num; ++i) {
        Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
        Sophus::Vector6d b = Sophus::Vector6d::Zero();
        Eigen::Vector3d e;

        cost = 0;
        for (int j = 0; j < P1.size(); ++j) {
            Eigen::Matrix<double , 4, 1> p22;
            p22 << P2[j][0], P2[j][1], P2[j][2], 1;
            e = P1[j] - (T.matrix() * p22).topRows(3);
            cost += 0.5 * sqrt(e.transpose() * e);

            Eigen::Matrix<double , 3, 6> J;
            Eigen::Vector3d p222 = (T.matrix() * p22).topRows(3);

            J(0, 0) = -1;
            J(0, 1) = 0;
            J(0, 2) = 0;
            J(0, 3) = 0;
            J(0, 4) = -p222(2);
            J(0, 5) = p222(1);
            J(1, 0) = 0;
            J(1, 1) = -1;
            J(1, 2) = 0;
            J(1, 3) = p222(2);
            J(1, 4) = 0;
            J(1, 5) = -p222(0);
            J(2, 0) = 0;
            J(2, 1) = 0;
            J(2, 2) = -1;
            J(2, 3) = -p222(1);
            J(2, 4) = p222(0);
            J(2, 5) = 0;

            H += J.transpose() * J;
            b += -J.transpose() * e;

            //cout << "J: " << endl << J << endl;
            //cout << "H: " << endl << H << endl;
            //cout << "b: " << endl << b << endl;
        }

        Sophus::Vector6d dx;
        dx = H.ldlt().solve(b);
        cout << "dx = " << dx.transpose() << endl;
        if (isnan(dx[0])) {
            cout << "result is nan!" << endl;
            break;
        }

        if (i > 0 && cost >= last_cost) {
            // cost increase, update is not good
            cout << "cost: " << cost << ", last cost: " << last_cost << endl;
            cout << last_cost << endl;
            break;
        }
        //if (sqrt(dx.transpose() * dx) < 0.01) {
        //    // cost increase, update is not good
        //    cout << "cost: " << cost << ", last cost: " << last_cost << endl;
        //    break;
        //}
        T = Sophus::SE3::exp(dx) * T;

        last_cost = cost;

        //cout << "iteration " << i << " cost=" << cout.precision(12) << cost << endl;
        cout << "iteration " << i << " cost=" << cost << " , last_cost = " << last_cost << endl;
    }
    cout << "estimated pose: \n" << T.matrix() << endl;
    for (int i = 0; i < P2.size(); ++i) {
        Eigen::Matrix<double , 4, 1> p22;
        p22 << P2[i][0], P2[i][1], P2[i][2], 1;
        P2_.push_back(( T.matrix() * p22).topRows(3));
    }
    DrawTrajectory(P1, P2_);
    return;
}

void DrawTrajectory(vector<Eigen::Vector3d> poses1, vector<Eigen::Vector3d> poses2) {
    if (poses1.empty() || poses2.empty()) {
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
        for (size_t i = 0; i < poses1.size() - 1; i++) {

            glColor3f(1, 0.0f, 0.0f);
            glBegin(GL_LINES);
            auto p1 = poses1[i], p2 = poses1[i + 1];
            glVertex3d(p1[0], p1[1], p1[2]);
            glVertex3d(p2[0], p2[1], p2[2]);

            glColor3f(0.0f, 1, 0.0f);
            glBegin(GL_LINES);
            auto p3 = poses2[i], p4 = poses2[i + 1];
            glVertex3d(p3[0], p3[1], p3[2]);
            glVertex3d(p4[0], p4[1], p4[2]);

            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}
