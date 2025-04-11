#include "ekf_catkin/kalman.h"
#include <iostream>
kalman::kalman(){
}
kalman ::kalman(const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& B,
    const Eigen::MatrixXd& P,
    const Eigen::VectorXd& x
):A(A),B(B),P(P),x(x)
{
    I.setIdentity();
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
void kalman::predict(const Eigen::VectorXd& u){
    x=A*x+B*u;
    P=A*P*A.transpose()+R;
}
void kalman::update(){
    Eigen::MatrixXd k = (P * C.transpose()) * (C * P * C.transpose() + Q).inverse();
    x=x+k*(y-C*x);
    Eigen::MatrixXd kC=k*C;
    I=Eigen::MatrixXd::Identity(
        (kC).rows(),      // Rows of original matrix
        (kC).cols()       // Columns of original matrix
    );
    P=(I-kC)*P;
}
void kalman::estimate(const Eigen::VectorXd& u){
    Eigen::VectorXd x_old=x;
    Eigen::VectorXd x_bar=A*x+B*u;
    Eigen::MatrixXd P_bar=A*P*A.transpose()+R;
    Eigen::MatrixXd k = (P_bar * C.transpose()) * (C * P_bar * C.transpose() + Q).inverse();
    x=x_bar+k*(y-C*x_bar);
    Eigen::MatrixXd kC=k*C;
    I=Eigen::MatrixXd::Identity(
        (kC).rows(),      // Rows of original matrix
        (kC).cols()       // Columns of original matrix
    );
    P=(I-kC)*P_bar;
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
void kalman::setQ_matrix(const Eigen::MatrixXd& Q){
    this->Q=Q;
}
void kalman::setR_matrix(const Eigen::MatrixXd& R){
    this->R=R;
}

void kalman::setY(const Eigen::VectorXd& y){
    this->y=y;
}
void kalman::setY_partial(int startRow, const Eigen::VectorXd& y){
    this->y.segment(startRow,y.size())=y;
}
void kalman::setC_matrix_partial(int startRow, const Eigen::MatrixXd& C){
    this->C.block(startRow,0,C.rows(),C.cols())=C;
}
Eigen::VectorXd kalman::getY() const {
    return y;
}
Eigen::MatrixXd kalman::getC() const {
    return C;
}
Eigen::VectorXd kalman::getX() const {
    return x;
}
Eigen::MatrixXd kalman::getP() const {
    return P;
}


