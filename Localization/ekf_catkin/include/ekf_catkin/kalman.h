#include <Eigen/Dense>
#ifndef EKF_KALMAN_H
#define EKF_KALMAN_H
class kalman{
    public:
    kalman();
    kalman(const Eigen::MatrixXd& A,
        const Eigen::MatrixXd& B,
        const Eigen::MatrixXd& C,
        const Eigen::MatrixXd& Q,
        const Eigen::MatrixXd& R,
        const Eigen::MatrixXd& P,
        const Eigen::VectorXd& x);
    kalman(
        const Eigen::MatrixXd& Q,
        const Eigen::MatrixXd& R,
        const Eigen::MatrixXd& P,
        const Eigen::VectorXd& x
    );
    void predict(const Eigen::VectorXd& u);
    void update();
    void estimate(const Eigen::VectorXd& u);
    void setA_matrix(const Eigen::MatrixXd& A);
    void setB_matrix(const Eigen::MatrixXd& B);
    void setC_matrix(const Eigen::MatrixXd& C);
    void setR_matrix(const Eigen::MatrixXd& R);
    void setQ_matrix(const Eigen::MatrixXd& Q);
    void setY(const Eigen::VectorXd& y);
    void setY_partial(int startRow, const Eigen::VectorXd& y);
    void setC_matrix_partial(int startRow, const Eigen::MatrixXd& C);
    Eigen::VectorXd getX() const;
    Eigen::MatrixXd getP() const;
    Eigen::MatrixXd getC() const;
    Eigen::VectorXd getY() const;
    
    private:
    Eigen::MatrixXd A;      //State transition matrix
    Eigen::MatrixXd B;      //Input matrix
    Eigen::MatrixXd C;      //Output matrix
    Eigen::MatrixXd Q;      //Sensor uncertainties (measurement nois)
    Eigen::MatrixXd R;      //model uncertainties (process noise)
    Eigen::MatrixXd P;      //covariance matrix
    Eigen::VectorXd x;      //state vector
    Eigen::MatrixXd I;      //identity matrix
    Eigen::VectorXd y;      //output vector
};
#endif
