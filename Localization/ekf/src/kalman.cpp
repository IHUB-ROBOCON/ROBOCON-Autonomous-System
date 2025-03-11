#include "kalman.h"
kalman::kalman(){
}
kalman ::kalman(const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& B,
    const Eigen::MatrixXd& C,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& P,
    const Eigen::VectorXd& x
):A(A),B(B),C(C),Q(Q),R(R),P(P),x(x)
{
    I.setIdentity();
}
void kalman::estimate(const Eigen::VectorXd& u){
    Eigen::VectorXd x_old=x;
    Eigen::VectorXd x_bar=A*x+B*u;
    Eigen::MatrixXd P_bar=A*P*A.transpose()+R;
    Eigen::MatrixXd k = P_bar * C.transpose() * (C * P * C.transpose() + Q).inverse();
    x=x_bar+k*(y+C*x_bar);
    P=(I-k*C)*P_bar;
}
void kalman::setA_matrix(const Eigen::MatrixXd& A){
    this->A=A;
}
void kalman::setB_matrix(const Eigen::MatrixXd& B){
    this->B=B;
}
void kalman::setC_matrix(const Eigen::MatrixXd& C){
    this->C=C;
}
void kalman::setY(const Eigen::VectorXd& y){
    this->y=y;
}
void kalman::setY_partial(int startRow, const Eigen::VectorXd& y){
    this->y.segment(startRow,y.size())=y;
}
void kalman::setC_matrix_partial(int startRow, const Eigen::MatrixXd& C){
    this->C.block(startRow,0,C.rows()-startRow,C.cols())=C;
}


