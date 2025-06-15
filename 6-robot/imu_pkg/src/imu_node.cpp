#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>

ros::Publisher vel_pub;

void IMUCal(sensor_msgs::Imu msg)
{
    // 检测消息包中四元数数据是否存在
    if (msg.orientation_covariance[0] < 0)
        return;
    // 四元数转成欧拉角
    tf::Quaternion quaternion(
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    // 弧度换算成角度
    roll = roll * 180 / M_PI;   // roll: 滚转
    pitch = pitch * 180 / M_PI; // pitch: 俯仰
    yaw = yaw * 180 / M_PI;     // yaw: 朝向
    ROS_INFO("roll: %.0f, pitch: %.0f, yaw: %.0f", roll, pitch, yaw);

    // 速度消息包
    geometry_msgs::Twist vel_cmd;
    // 目标朝向角
    double target_yaw = 90;
    // 计算速度
    double diff_angle = target_yaw - yaw;
    vel_cmd.angular.z = diff_angle * 0.01;
    // vel_cmd.linear.x = 0.1; // 设置前进速度
    vel_pub.publish(vel_cmd);
}
int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "imu_node");
    ros::NodeHandle nh;
    // 订阅 IMU 的数据话题
    ros::Subscriber imu_sub = nh.subscribe("/imu/data", 10, IMUCal);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::spin();
    return 0;
}