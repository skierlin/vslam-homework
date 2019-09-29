#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;

int main(){
	
	Eigen::Quaterniond q1(0.55,0.3,0.2,0.2);
	Eigen::Quaterniond q1_norm = q1.normalized();
	Eigen::Isometry3d T1_cw = Eigen::Isometry3d::Identity();
	T1_cw.rotate(q1_norm);
	T1_cw.pretranslate ( Eigen::Vector3d ( 0.7,1.1,0.2 ) );

	Eigen::Quaterniond q2(-0.1,0.3,-0.7,0.2);
	Eigen::Quaterniond q2_norm = q2.normalized();
	Eigen::Isometry3d T2_cw = Eigen::Isometry3d::Identity();
	T2_cw.rotate(q2_norm);
	T2_cw.pretranslate ( Eigen::Vector3d ( -0.1,0.4,0.8 ) );
	
	Eigen::Vector3d p1(0.5,-0.1,0.2);
	Eigen::Vector3d p_w = T1_cw.inverse()*p1;
	Eigen::Vector3d p2 = T2_cw*p_w;

	cout << "p2 = " << endl << p2 << endl;

}