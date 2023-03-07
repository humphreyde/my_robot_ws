#include <stdlib.h>
#include <cstring>
#include <iostream>
#include <string>
#include<Eigen/Dense>
#include<Eigen/Core>
#include<math.h>


using namespace std;
using namespace Eigen;

void main() {

    //fk运算
    float q1,q2,q3,q4,q5,q6,q7;
    ///赋予各关节角度
    q1 = 0;  q2 = -0.37353;   q3 = -0.0728801;  q4 = -1.57675;   q5 = 2.71527;  q6 = -0.415837;   q7 = 0.420669;
    Eigen::Matrix<float, 4,4>T1;
    T1 << cos(q1), 0, - sin(q1), 0, sin(q1), 0, cos(q1), 0, 0, - 1, 0, - 0.1654, 0, 0, 0 ,1;
    Eigen::Matrix<float, 4, 4 > T2;
    T2 << cos(q2), 0, - sin(q2), 0, sin(q2), 0, cos(q2), 0, 0, - 1, 0 ,0, 0, 0, 0 ,1;
    Eigen::Matrix<float, 4, 4>T3;
    T3 << cos(q3), 0, sin(q3), 0, sin(q3), 0, -cos(q3), 0, 0, 1, 0, 0.244, 0, 0, 0, 1;
    Eigen::Matrix<float, 4, 4>T4;
    T4 << cos(q4), 0, -sin(q4), 0, sin(q4), 0, cos(q4), 0, 0, -1, 0, 0, 0, 0, 0, 1;
    Eigen::Matrix<float, 4, 4>T5;
    T5 <<cos(q5), 0, sin(q5), 0, sin(q5), 0, - cos(q5), 0, 0, 1, 0 ,0.211, 0, 0, 0 ,1;
    Eigen::Matrix<float, 4, 4>T6;
    T6 <<cos(q6), 0 ,- sin(q6), 0, sin(q6), 0 ,cos(q6) ,0, 0, - 1, 0 ,0, 0, 0 ,0, 1;
    Eigen::Matrix<float, 4, 4>T7;
    T7 <<cos(q7), - sin(q7), 0, 0, sin(q7), cos(q7), 0, 0, 0 ,0 ,1, 0.1539, 0, 0 ,0 ,1;
    Eigen::Matrix<float, 4, 4>T;
    T = T1 * T2 * T3 * T4 * T5 * T6 * T7;
    cout << T << endl;
}