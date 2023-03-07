#include <stdlib.h>
#include <cstring>
#include <iostream>
#include <string>
#include<Eigen/Dense>
#include<Eigen/Core>
#include<math.h>


using namespace std;
using namespace Eigen;

void main(int argc, char** argv) {

    //ik运算
    float x, y, z, r, p, y1;
    cout << "请输入平移量x y z(需要运算做差)\n";
    cin >> x >> y >> z;
    Matrix4f transl_44;
    transl_44 << 1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1;
    cout << "请输入旋转量r p y1\n";
    cin >> r >> p >> y1;
    Matrix4f rpy2tr_44;
    rpy2tr_44 << cos(p) * cos(y1), cos(y1)* sin(p)* sin(r) - cos(r) * sin(y1), sin(r)* sin(y1) + cos(r) * cos(y1) * sin(p), 0,
        cos(p)* sin(y1), cos(r)* cos(y1) + sin(p) * sin(r) * sin(y1), cos(r)* sin(p)* sin(y1) - cos(y1) * sin(r), 0,
        -sin(p), cos(p)* sin(r), cos(p)* cos(r), 0,
        0, 0, 0, 1;
    /*Matrix4f T = transl_44 * rpy2tr_44;*/
    Matrix4f T;
    T << 0.960531, -0.156023, 0.230298, 0.308958,
        0.194706, 0.968372, -0.156026, -0.0218177,
        -0.198671, 0.194709, 0.96053, -0.0562809,
        0, 0, 0, 1;
    cout << "矩阵是：\n" << T << endl;

    float q1, q2, q3, q4, q5, q6, q7;
    float ws, A2, B2, C2, D2;
    cout << "指定关节一参数\n";
    cin >> q1;
    //下面计算关节参数
    Eigen::Matrix<float, 4, 4>Tn = T.inverse();
    Eigen::Matrix<float, 3, 1> p70;
    p70 << Tn(0, 3), Tn(1, 3), Tn(2, 3);
    Eigen::Matrix<float, 3, 1>p07;
    p07 << T(0, 3), T(1, 3), T(2, 3);
    Eigen::Matrix<float, 3, 3>R07;
    R07 << T(0, 0), T(0, 1), T(0, 2), T(1, 0), T(1, 1), T(1, 2), T(2, 0), T(2, 1), T(2, 2);
    Eigen::Matrix<float, 3, 1>d1;
    d1 << 0, 0, -0.090;
    Eigen::Matrix<float, 3, 1>d7;
    d7 << 0, 0, 0.248;
    Eigen::Matrix<float, 3, 1>w;
    w << p07 - d1 + R07 * d7;
    ws = float(sqrt(w(0, 0) * w(0, 0) + w(1, 0) * w(1, 0) + w(2, 0) * w(2, 0)));
    q4 = -acos((0.247 * 0.247 + 0.211 * 0.211 - ws * ws) / 2 * 0.247 * 0.211);
    Eigen::Matrix<float, 3, 1>Tz = -R07 * (p70 + d7);
    q3 = asin((double(Tz(0, 0) * sin(q1) - Tz(1, 0) * cos(q1))) / -0.211 * sin(q4));
    A2 = 0.247 + 0.211 * cos(q4);
    B2 = 0.211 * sin(q4) * cos(q3);
    C2 = double(Tz(0, 0) * cos(q1) + Tz(1, 0) * sin(q1));
    D2 = Tz(2, 0) + 0.090;
    q2 = atan((A2 * C2 - B2 * D2) / (B2 * C2 + A2 * D2));
    Eigen::Matrix<float, 4, 4>T1;
    T1 << cos(q1), 0, -sin(q1), 0, sin(q1), 0, cos(q1), 0, 0, -1, 0, -0.090, 0, 0, 0, 1;
    Eigen::Matrix<float, 4, 4>T2;
    T2 << cos(q2), 0, -sin(q2), 0, sin(q2), 0, cos(q2), 0, 0, -1, 0, 0, 0, 0, 0, 1;
    Eigen::Matrix<float, 4, 4>T3;
    T3 << cos(q3), 0, sin(q3), 0, sin(q3), 0, -cos(q3), 0, 0, 1, 0, 0.247, 0, 0, 0, 1;
    Eigen::Matrix<float, 4, 4>T4;
    T4 << cos(q4), 0, -sin(q4), 0, sin(q4), 0, cos(q4), 0, 0, -1, 0, 0, 0, 0, 0, 1;
    Eigen::Matrix<float, 4, 4>T04 = T1 * T2 * T3 * T4;
    Eigen::Matrix<float, 4, 4>T47 = T04.inverse() * T;
    q6 = -acos(T47(2, 2));
    q7 = atan2(-T47(2, 1) / sin(q6), T47(2, 0) / sin(q6));
    q5 = atan2(-T47(1, 2) / sin(q6), -T47(0, 2) / sin(q6));
    Eigen::Matrix<float, 1, 7>q;
    q << q1, q2, q3, q4, q5, q6, q7;
    cout << q << endl;

}