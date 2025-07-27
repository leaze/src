#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

class PosePublisher
{
public:
    PosePublisher()
    {
        // 初始化ROS节点
        ros::NodeHandle nh;

        // 创建发布者，分别发布g1_12dof_gazebo和movable_sphere的位姿信息
        position_pub_g1_ = nh.advertise<geometry_msgs::Point>("/g1_12dof_gazebo/position", 10);
        orientation_pub_g1_ = nh.advertise<geometry_msgs::Quaternion>("/g1_12dof_gazebo/orientation", 10);
        linear_velocity_pub_g1_ = nh.advertise<geometry_msgs::Vector3>("/g1_12dof_gazebo/linear_velocity", 10);
        angular_velocity_pub_g1_ = nh.advertise<geometry_msgs::Vector3>("/g1_12dof_gazebo/angular_velocity", 10);

        position_pub_sphere_ = nh.advertise<geometry_msgs::Point>("/movable_sphere/position", 10);
        orientation_pub_sphere_ = nh.advertise<geometry_msgs::Quaternion>("/movable_sphere/orientation", 10);
        linear_velocity_pub_sphere_ = nh.advertise<geometry_msgs::Vector3>("/movable_sphere/linear_velocity", 10);
        angular_velocity_pub_sphere_ = nh.advertise<geometry_msgs::Vector3>("/movable_sphere/angular_velocity", 10);

        // 创建服务客户端，用于调用Gazebo的模型状态服务
        client_ = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

        // 设置发布频率
        rate_ = new ros::Rate(10); // 10 Hz
    }

    void run()
    {
        while (ros::ok())
        {
            // 获取g1_12dof_gazebo的模型状态
            gazebo_msgs::GetModelState srv_g1;
            srv_g1.request.model_name = "g1_12dof_gazebo";
            srv_g1.request.relative_entity_name = "";

            // 获取movable_sphere的模型状态
            gazebo_msgs::GetModelState srv_sphere;
            srv_sphere.request.model_name = "movable_sphere";
            srv_sphere.request.relative_entity_name = "";

            if (client_.call(srv_g1) && client_.call(srv_sphere))
            {
                // 发布g1_12dof_gazebo的位姿信息
                position_pub_g1_.publish(srv_g1.response.pose.position);
                orientation_pub_g1_.publish(srv_g1.response.pose.orientation);
                linear_velocity_pub_g1_.publish(srv_g1.response.twist.linear);
                angular_velocity_pub_g1_.publish(srv_g1.response.twist.angular);

                // 发布movable_sphere的位姿信息
                position_pub_sphere_.publish(srv_sphere.response.pose.position);
                orientation_pub_sphere_.publish(srv_sphere.response.pose.orientation);
                linear_velocity_pub_sphere_.publish(srv_sphere.response.twist.linear);
                angular_velocity_pub_sphere_.publish(srv_sphere.response.twist.angular);
            }
            else
            {
                ROS_ERROR("Failed to call service /gazebo/get_model_state");
            }

            rate_->sleep();
        }
    }

private:
    ros::Publisher position_pub_g1_;
    ros::Publisher orientation_pub_g1_;
    ros::Publisher linear_velocity_pub_g1_;
    ros::Publisher angular_velocity_pub_g1_;

    ros::Publisher position_pub_sphere_;
    ros::Publisher orientation_pub_sphere_;
    ros::Publisher linear_velocity_pub_sphere_;
    ros::Publisher angular_velocity_pub_sphere_;

    ros::ServiceClient client_;
    ros::Rate* rate_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_publisher");
    PosePublisher publisher;
    publisher.run();
    return 0;
}