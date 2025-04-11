#include "ekf_catkin/kalman.h"
#include <iostream>
int main(){
    Eigen::MatrixXd A(2,2);
    Eigen::MatrixXd B(2,1);
    Eigen::MatrixXd C(2,2);
    Eigen::MatrixXd Q(2,2);
    Eigen::MatrixXd R(2,2);
    Eigen::MatrixXd P(2,2);
    Eigen::VectorXd x(2);
    Eigen::VectorXd u(1);
    Eigen::VectorXd y(2);
    Eigen::VectorXd y_1(1);

    A << 1, 1,
         0, 1;
    B << 0.5,
         1;
    C << 1, 0,
          0,1;
    Q << 0.05,0,
         0, 0.05;
    R << 0.1, 0,
         0, 0.1;
    P << 1, 0,
         0, 1;
    x << 1,
         1;
    u << 0;
    y_1<<1;
    y << 1.2,
          1;

    kalman kf(A,B,C,Q,R,P,x);
    kf.setY(y_1);
    kf.setY(y);
    Eigen::VectorXd v_1= (Eigen::VectorXd(1) << 1.2).finished();
     Eigen::VectorXd v_2= (Eigen::VectorXd(1) << 1).finished();
    kf.setY_partial(0,v_1);
    kf.setY_partial(1,v_2);

    //kf.setC_matrix(C);
    Eigen::MatrixXd first_row(1,2);
    Eigen::MatrixXd second_row(1,2);
     first_row << 1, 0;
     second_row << 0, 1;
    kf.setC_matrix_partial(0,first_row);
    kf.setC_matrix_partial(1,second_row);

//    kf.estimate(u);
     kf.predict(u);
     kf.update();
    std::cout << "C " << kf.getX() << std::endl;
    std::cout << "y " << kf.getP() << std::endl;
    return 0;
}