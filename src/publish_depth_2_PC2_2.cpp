// 根据这个位姿和相机内参，计算出每个像素点的世界坐标并且保存为点云(PCL格式)
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <sstream>
#include <string>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;

// 定义相机内参
struct CameraIntrinsics
{
    float fx, fy, cx, cy;
};

ros::Publisher point_cloud_pub;

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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_to_pointcloud_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/depth/cloud", 1);
    std::string depth_image_path;
    nh.getParam("/depth_png_file", depth_image_path);
    std::string frame_id = "map";
    ROS_INFO("image path: %s", depth_image_path.c_str());
    cv::Mat cv_image = cv::imread(depth_image_path, cv::IMREAD_UNCHANGED);
    if (cv_image.empty())
    {
        ROS_ERROR("Failed to load image.");
        return -1;
    }
    // cv::imshow("Depth Image", cv_image);
    // 转换图像到sensor_msgs::Image消息
    std::string encoding = sensor_msgs::image_encodings::TYPE_16UC1; // 根据实际情况设置编码
    if (cv_image.type() != CV_16UC1)
    {
        ROS_WARN("Converting image to %s.", encoding.c_str());
        // 如果图像类型不匹配，可能需要进行转换
        cv_image.convertTo(cv_image, CV_16UC1);
    }

    // 相机内参
    CameraIntrinsics intrinsics{};
    intrinsics.fx = 431.574523925781; // 焦距 x
    intrinsics.fy = 431.574523925781; // 焦距 y
    intrinsics.cx = 429.674438476562; // 光心 x
    intrinsics.cy = 237.917541503906; // 光心 y
    // -2.518551 -2.610104 0.097342 0.7122784687391646 0.006027449494040417 0.0010280845644780004 -0.7018703554541365
    // 定义位姿（位移和旋转四元数）
    std::string depth_pose_str;
    if (!ros::param::get("depth_pose", depth_pose_str)) {
        ROS_ERROR("Failed to get param 'depth_pose'");
        return -1;
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
    ROS_INFO("depth pose: %s", vectorToString(pose_array).c_str());
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() << pose_array[0], pose_array[1], pose_array[2];
    // 注意四元数的顺序：w, x, y, z
    Eigen::Quaterniond rotation(pose_array[6], pose_array[3], pose_array[4], pose_array[5]);
    pose.rotate(rotation);

    // 创建点云对象
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = cv_image.cols;
    cloud.height = cv_image.rows;
    cloud.points.resize(cloud.width * cloud.height);
    cloud.is_dense = true;

    float minDepth = 0.01;
    float maxDepth = 3.0;
    // 遍历深度图像并计算世界坐标
    for (int v = 0; v < cv_image.rows; ++v)
    {
        for (int u = 0; u < cv_image.cols; ++u)
        {
            // 获取深度值（注意类型转换）
            uint16_t depth = cv_image.at<uint16_t>(v, u);
            if (depth == 0) continue; // 跳过无效深度值

            double depthInMeters = depth / 1000.0;
            if (depthInMeters < minDepth || depthInMeters > maxDepth)
                continue;

            // 计算像素点在相机坐标系下的坐标
            double z = depthInMeters; // 假设深度单位为毫米，转换为米
            double x = (u - intrinsics.cx) * z / intrinsics.fx;
            double y = (v - intrinsics.cy) * z / intrinsics.fy;
            Eigen::Vector3d point_camera(x, y, z);

            // 转换到世界坐标系下
            Eigen::Vector3d point_world = pose * point_camera;

            // 添加到点云中
            pcl::PointXYZ pcl_point;
            pcl_point.x = point_world.x();
            pcl_point.y = point_world.y();
            pcl_point.z = point_world.z();
            cloud.points[v * cloud.width + u] = pcl_point;
        }
    }

    // 转换点云为ROS消息并发布
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.frame_id = frame_id;
    // 加载深度图像并发布点云
    for (int i = 0; i < 10; ++i)
    {
        point_cloud_pub.publish(cloud_msg);
        loop_rate.sleep();
    }
    // 保持节点运行直到关闭
    // ros::spin();

    return 0;
}
