#include <iostream>
using namespace std;
#include <ctime>
// Eigen 部分
#include <Eigen/Core>
// 稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>

#define MATRIX_SIZE 100

int main( int argc, char** argv )
{

    Eigen::Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_100;
    matrix_100 = Eigen::MatrixXd::Random(100,100);

    Eigen::Matrix< double, MATRIX_SIZE,  1> v_Nd;
    v_Nd = Eigen::MatrixXd::Random( MATRIX_SIZE,1 );
    
    Eigen::Matrix<double,MATRIX_SIZE,1> x = matrix_100.colPivHouseholderQr().solve(v_Nd);
    cout << "the first answer is " << x << endl;

    Eigen::Matrix<double,MATRIX_SIZE,1> y = matrix_100.ldlt().solve(v_Nd);
    cout << "the second answer is " <<  y << endl;


    return 0;
}
