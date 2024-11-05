//
// Created by benny on 24-10-29.
//
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>
#include <filesystem>
#include <pcl/common/transforms.h>

// 假设你有一个全局的 ROS 节点句柄和发布者

ros::Publisher point_cloud_pub;

// 函数原型声明
void publishPointCloudFromFile(const std::string& file_path, const std::string& topic_name);

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_publisher");
    ros::NodeHandle nh;
    point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/lidar", 1);

    // 调用函数发布点云,读取路径为 '/data/bag/extra/gml_2024-10-15-10-45-17/_livox_lidar'下的所有PCD文件，进行循环处理
    // 指定要读取的 PCD 文件目录
    std::string pcd_directory = "/data/bag/extra/gml_2024-10-15-10-45-17/_livox_lidar/";
    ros::Rate loop_rate(10);
    // 使用文件系统库遍历目录
    // for (const auto& entry : std::filesystem::directory_iterator(pcd_directory))
    // {
    //     if (entry.path().extension() == ".pcd")
    //     {
    //         publishPointCloudFromFile(entry.path(), "/lidar");
    //     }
    //     loop_rate.sleep();
    // }
    std::string pcd_path;
    nh.getParam("/pcd_file", pcd_path);
    for (int i = 0; i < 10; ++i)
    {
        publishPointCloudFromFile(pcd_path, "/lidar");
        loop_rate.sleep();
    }
    // ros::spin();

    return 0;
}

std::string vectorToString(const std::vector<double>& vec, const std::string& delimiter = ", ") {
    std::ostringstream oss;
    for (size_t i = 0; i < vec.size(); ++i) {
        if (i != 0) {
            oss << delimiter;
        }
        // 可以选择不同的精度
        oss << std::fixed << std::setprecision(6) << vec[i];
    }
    return oss.str();
}

void publishPointCloudFromFile(const std::string& file_path, const std::string& topic_name) {
    // 加载 PCD 文件
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *cloud) == -1) {
        ROS_ERROR("Couldn't read file %s \n", file_path.c_str());
        return;
    }

    // 定义位姿（位移和旋转四元数）
    // -2.62103 -2.3705 0.0553439 0.00853753 -0.026726 -0.713 -0.700603
    std::string depth_pose_str;
    if (!ros::param::get("lidar_pose", depth_pose_str)) {
        ROS_ERROR("Failed to get param 'depth_pose'");
        return;
    }
    std::vector<double> pose_array;
    std::stringstream ss(depth_pose_str);
    std::string item;
    while (std::getline(ss, item, ' ')) {
        pose_array.push_back(std::stod(item)); // 将字符串转换为double并添加到数组中
    }
    if (pose_array.size() != 7) {
        ROS_ERROR("Invalid depth pose");
    }
    ROS_INFO("lidar pose: %s", vectorToString(pose_array).c_str());
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() << pose_array[0], pose_array[1], pose_array[2];
    // 注意四元数的顺序：w, x, y, z
    Eigen::Quaterniond rotation(pose_array[6], pose_array[3], pose_array[4], pose_array[5]);
    pose.rotate(rotation);

    // 应用位姿变换到点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *transformed_cloud, pose.matrix());
    *cloud = *transformed_cloud;
    // 将 PCL 点云转换为 ROS 点云消息
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*cloud, ros_cloud);

    // 设置消息头信息（例如，时间戳和帧ID）
    ros_cloud.header.stamp = ros::Time::now(); // 或者使用文件中的时间戳，如果可用的话
    ros_cloud.header.frame_id = "map"; // 或者其他适当的帧ID

    // 发布点云消息
    point_cloud_pub.publish(ros_cloud);
    ROS_INFO("Published point cloud from file: %s to topic: %s", file_path.c_str(), topic_name.c_str());
}