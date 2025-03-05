#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <xv_sdk/PoseStampedConfidence.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/mutex.hpp>
#include <Eigen/Geometry>
#include <iomanip>

struct SharedData {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr path;
    Eigen::Affine3f pose;
    bool pose_updated;
    boost::mutex mutex;

    int cloud_width = 0;
    int cloud_height = 0;
    double cloud_fps = 0.0;
    double pose_fps = 0.0;
    ros::Time last_cloud_time;
    ros::Time last_pose_time;

    SharedData() : cloud(new pcl::PointCloud<pcl::PointXYZ>),
                   path(new pcl::PointCloud<pcl::PointXYZ>),
                   pose(Eigen::Affine3f::Identity()),
                   pose_updated(false) {}
};

struct Config {
    struct {
        float y_threshold = 1.0;
        bool enable = true;
    } filter;

    struct {
        float coordinate_size = 0.3;
        float line_width = 2.0;
        float triangle_size = 0.5;  // 三角形尺寸参数
        bool swap_xz = true;        // 坐标系交换参数
        bool invert_y = true;       // Y轴反转参数
    } visualization;
};

SharedData g_data;
Config g_config;

void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *temp);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto &p : *temp) {
        // 坐标系转换
        pcl::PointXYZ transformed;
        if (g_config.visualization.swap_xz) {
            transformed.x = p.z;
            transformed.z = p.x;
        } else {
            transformed.x = p.x;
            transformed.z = p.z;
        }
        transformed.y = g_config.visualization.invert_y ? -p.y : p.y;

        if (g_config.filter.enable && std::abs(transformed.y) < g_config.filter.y_threshold) {
            filtered->push_back(transformed);
        }
    }

    boost::mutex::scoped_lock lock(g_data.mutex);
    g_data.cloud->swap(*filtered);
    g_data.cloud_width = msg->width;
    g_data.cloud_height = msg->height;

    // 计算帧率
    ros::Time now = ros::Time::now();
    if (g_data.last_cloud_time.isValid()) {
        double delta = (now - g_data.last_cloud_time).toSec();
        if (delta > 0) g_data.cloud_fps = 1.0 / delta;
    }
    g_data.last_cloud_time = now;
}

void poseCallback(const xv_sdk::PoseStampedConfidence::ConstPtr &msg) {
    Eigen::Vector3f position(
        msg->poseMsg.pose.position.x,
        msg->poseMsg.pose.position.y,
        msg->poseMsg.pose.position.z);

    Eigen::Quaternionf q_orig(
        msg->poseMsg.pose.orientation.w,
        msg->poseMsg.pose.orientation.x,
        msg->poseMsg.pose.orientation.y,
        msg->poseMsg.pose.orientation.z);

    // 坐标系修正
    static const Eigen::Quaternionf q_correct(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()));
    Eigen::Affine3f pose = Eigen::Affine3f::Identity();
    pose.translate(position);
    pose.rotate(q_orig * q_correct);

    // 应用可视化参数转换
    if (g_config.visualization.swap_xz) {
        Eigen::Vector3f t = pose.translation();
        pose.translation() << t.z(), t.y(), t.x();
    }
    if (g_config.visualization.invert_y) {
        pose.translation().y() *= -1;
    }

    boost::mutex::scoped_lock lock(g_data.mutex);
    g_data.path->push_back(pcl::PointXYZ(pose.translation().x(), 
                                       pose.translation().y(), 
                                       pose.translation().z()));
    g_data.pose = pose;
    g_data.pose_updated = true;

    // 计算帧率
    ros::Time now = ros::Time::now();
    if (g_data.last_pose_time.isValid()) {
        double delta = (now - g_data.last_pose_time).toSec();
        if (delta > 0) g_data.pose_fps = 1.0 / delta;
    }
    g_data.last_pose_time = now;
}

void drawCameraIndicator(pcl::visualization::PCLVisualizer& viewer, 
                        const Eigen::Affine3f& pose) {
    const float size = g_config.visualization.triangle_size;
    
    // 定义局部坐标系三角形顶点（相机坐标系）
    Eigen::Vector3f local_points[3] = {
        Eigen::Vector3f(0, 0, 0),            // 原点
        Eigen::Vector3f(-size, -size, size), // 左下
        Eigen::Vector3f(size, -size, size)   // 右下
    };

    // 转换到世界坐标系
    pcl::PointXYZ world_points[3];
    for(int i=0; i<3; ++i){
        Eigen::Vector3f p = pose * local_points[i];
        world_points[i] = pcl::PointXYZ(p.x(), p.y(), p.z());
    }

    // 绘制三角形边
    viewer.removeShape("cam_line1");
    viewer.removeShape("cam_line2");
    viewer.removeShape("cam_line3");
    viewer.addLine(world_points[0], world_points[1], 0.0, 1.0, 0.0, "cam_line1");
    viewer.addLine(world_points[1], world_points[2], 0.0, 1.0, 0.0, "cam_line2");
    viewer.addLine(world_points[2], world_points[0], 0.0, 1.0, 0.0, "cam_line3");

    // 添加方向箭头
    Eigen::Vector3f tip = pose * Eigen::Vector3f(0, 0, size*1.5);
    viewer.removeShape("cam_arrow");
    viewer.addArrow(pcl::PointXYZ(tip.x(), tip.y(), tip.z()),
                   world_points[0],
                   0.0, 1.0, 0.0, false, "cam_arrow");
}

void updateVisualization(pcl::visualization::PCLVisualizer &viewer) {
    static size_t last_path_size = 0;

    // 更新点云
    if (!g_data.cloud->empty()) {
        viewer.removePointCloud("cloud");
        viewer.addPointCloud<pcl::PointXYZ>(g_data.cloud, "cloud");
        viewer.setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_COLOR,
            0.7, 0.7, 0.7, "cloud");
    }

    // 更新路径
    if (g_data.path->size() >= 2) {
        for (size_t i=0; i<last_path_size; ++i)
            viewer.removeShape("line_"+std::to_string(i));

        for (size_t i=0; i<g_data.path->size()-1; ++i) {
            const auto& p1 = (*g_data.path)[i];
            const auto& p2 = (*g_data.path)[i+1];
            viewer.addLine<pcl::PointXYZ>(p1, p2, 1.0, 0.0, 0.0, "line_"+std::to_string(i));
            viewer.setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
                g_config.visualization.line_width, "line_"+std::to_string(i));
        }
        last_path_size = g_data.path->size()-1;
    }

    // 更新相机指示器
    if (g_data.pose_updated) {
        drawCameraIndicator(viewer, g_data.pose);
        viewer.removeCoordinateSystem("cam_coord");
        viewer.addCoordinateSystem(g_config.visualization.coordinate_size, 
                                 g_data.pose, "cam_coord");
        g_data.pose_updated = false;
    }

    // 更新文本信息
    Eigen::Vector3f pos = g_data.pose.translation();
    Eigen::Vector3f euler = g_data.pose.rotation().eulerAngles(2,1,0);

    std::ostringstream oss;
    oss << "Camera Pose:\n"
        << std::fixed << std::setprecision(2)
        << "Position(m)\nX:" << pos.x() << " Y:" << pos.y() << " Z:" << pos.z()
        << "\nRotation(deg)\nYaw:" << euler[0]*180/M_PI 
        << " Pitch:" << euler[1]*180/M_PI 
        << " Roll:" << euler[2]*180/M_PI
        << "\n\nFrame Rates\nCloud:" << std::setprecision(1) << g_data.cloud_fps
        << "Hz\nPose:" << g_data.pose_fps << "Hz"
        << "\n\nResolution: " << g_data.cloud_width << "x" << g_data.cloud_height;

    viewer.removeShape("info_text");
    viewer.addText(oss.str(), 10, 30, 12, 1,1,1, "info_text");
}

void visualizationThread() {
    pcl::visualization::PCLVisualizer viewer("SLAM Visualizer");
    viewer.setBackgroundColor(0.1, 0.1, 0.1);
    viewer.initCameraParameters();
    viewer.setCameraPosition(0, -5, 3, 0, 0, 0);

    while (!viewer.wasStopped() && ros::ok()) {
        {
            boost::mutex::scoped_lock lock(g_data.mutex);
            updateVisualization(viewer);
        }
        viewer.spinOnce(50);
        ros::spinOnce();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "slam_visualizer");
    ros::NodeHandle nh, pnh("~");

    pnh.param("filter/enable", g_config.filter.enable, true);
    pnh.param("filter/y_threshold", g_config.filter.y_threshold, 1.0f);
    pnh.param("visualization/coordinate_size", g_config.visualization.coordinate_size, 0.5f);
    pnh.param("visualization/line_width", g_config.visualization.line_width, 2.0f);
    pnh.param("visualization/triangle_size", g_config.visualization.triangle_size, 0.5f);
    pnh.param("visualization/swap_xz", g_config.visualization.swap_xz, true);
    pnh.param("visualization/invert_y", g_config.visualization.invert_y, true);

    ros::Subscriber sub_cloud = nh.subscribe<sensor_msgs::PointCloud2>(
        "/xv_sdk/xv_dev/tof_camera/point_cloud", 1, pointcloudCallback);
    ros::Subscriber sub_pose = nh.subscribe<xv_sdk::PoseStampedConfidence>(
        "/xv_sdk/xv_dev/slam/pose", 1, poseCallback);

    visualizationThread();
    return 0;
}