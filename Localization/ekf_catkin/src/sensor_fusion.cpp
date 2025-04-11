#include "ros/ros.h"
#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
int main(int argc, char **argv){
    ros::init(argc, argv, "sensor_fusion");
    ros::NodeHandle n;
    SensorFusion sensor_fusion(n);
    while (ros::ok()){
        ros::spin();
    }
}
class SensorFusion{
private:
    Eigen::VectorXd state(5);
    state<<0,0,0,0,0;
    double dt =0.01;
    MatrixXd A(5,5);
    A<<1,0,0,dt,0,
      0,1,0,0,dt,
      0,0,1,0,0,
      0,0,0,1,0,
      0,0,0,0,1;
    MatrixXd B(5,3);
    B<<0,0,0,
      0,0,0,
      0,0,dt,
      dt,0,0,
      0,dt,0;
    MatrixXd P(5,5);
    P<<0.01,0,0,0,0,
      0,0.01,0,0,0,
      0,0,0.01,0,0,
      0,0,0,0.01,0,
      0,0,0,0,0.01;
      kalman kf(A,B,P,state);
public:
    SensorFusion(ros::NodeHandle& nh) {
        // Initialize subscribers and publishers
        imu_sub = nh.subscribe("imu", 1000, &SensorFusion::imuCallback, this);
        odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1000);
    }
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    // Process IMU data
    ROS_INFO("IMU data received: orientation: [%f, %f, %f, %f]", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    // Convert quaternion to Euler angles
    Eigen::quaterniond q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    Eigen::Vector3d RPY_angles= q.toRotationMatrix().eulerAngles(0, 1, 2);
    // Creating the required vectors and matrices
    VectorXd y(1);
    y<<RPY_angles(2);
    Vector3d u(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->angular_velocity.z);
    MatrixXd C(1,5);
    C<<0,0,1,0,0;
    matrixXd Q(1,1);
    Q<<msg->angular_velocity_covariance[8];
    MatrixXd R(5,5);
    R=B*Q*B.transpose();
    // apply the kalman filter
    kf.setY(y);
    kf.setC_matrix(C);
    kf.setR_matrix(R);
    kf.setQ_matrix(Q);
    kf.estimate(u);
    
    // Create an Odometry message
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    
    // Fill in the odometry message with orientation and position
    geometry_msgs::Quaternion q_msg;
    yaw= kf.getX()(2);
    Eigen::Quaternionf q = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
    q_msg.w = q.w();
    q_msg.x = q.x();
    q_msg.y = q.y();
    q_msg.z = q.z();
o   odom.pose.pose.orientation = q_msg;
    odom.pose.pose.position.x = kf.getX()(0);
    odom.pose.pose.position.y = kf.getX()(1);
    odom.twist.twist.linear.x = kf.getX()(3);
    odom.twist.twist.linear.y = kf.getX()(4);
    odom.pose.pose.position.z = 0.0; 
    //adjusting the covariances for testing
    odom.pose.covariance[0] = kf.getP()(0,0);
    odom.pose.covariance[6] = kf.getP()(1,1);
    odom.pose.covariance[18] = kf.getP()(2,2);
    odom.twist.covariance[0] = kf.getP()(3,3);
    odom.twist.covariance[6] = kf.getP()(4,4);
    // Publish the odometry message
    odom_pub.publish(odom);
}
};