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

void rotatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const AngleAxisd &rotation_x, const AngleAxisd &rotation_y)
{
    // 创建绕x轴旋转的变换矩阵
    Matrix3d rotation_x_matrix = AngleAxisd(rotation_x).matrix();
    // 创建绕y轴旋转的变换矩阵
    Matrix3d rotation_y_matrix = AngleAxisd(rotation_y).matrix();

    // 组合旋转矩阵
    Matrix3d combined_rotation = rotation_x_matrix * rotation_y_matrix;

    // 应用变换到点云
    Affine3d rotation_transform(Affine3d(combined_rotation.cast<double>()));
    pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *rotated_cloud, rotation_transform);

    // 替换原始点云
    *cloud = *rotated_cloud;
}

void applyTransformation(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const Matrix4d &transformation_matrix)
{
    Matrix4d custom_transformation_matrix = Matrix4d::Zero();
    // 提取旋转矩阵部分（3x3）
    Matrix3d rotation_matrix = transformation_matrix.block<3, 3>(0, 0);
    // 计算旋转矩阵的逆
    Matrix3d rotation_matrix_inv = rotation_matrix.inverse();
    custom_transformation_matrix.block<3, 3>(0, 0) = rotation_matrix_inv;
    for (int i = 0; i < 3; ++i)
    {
        custom_transformation_matrix(i, 3) = -transformation_matrix(i, 3);
    }
    // 应用变换到点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *transformed_cloud, custom_transformation_matrix.cast<float>());
    // 替换原始点云
    *cloud = *transformed_cloud;
}

void convertAndPublishDepthToPointCloud(cv::Mat depthImage, CameraIntrinsics intrinsics, std::string frame_id, const Matrix4d &transformation_matrix)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // 创建点云对象

    // 遍历深度图像，将符合条件的点添加到点云中
    float minDepth = 0.01; // 最小深度阈值（单位：米）
    float maxDepth = 4.0; // 最大深度阈值（单位：米）
    for (int y = 0; y < depthImage.rows; ++y)
    {
        for (int x = 0; x < depthImage.cols; ++x)
        {
            if (x % 5 != 0 && y % 5 != 0)
                continue;
            // 获取深度值
            uint16_t depth = depthImage.at<uint16_t>(y, x);
            // 忽略深度值为0的点d
            if (depth == 0)
                continue;
            double depthInMeters = depth / 1000.0;

            // ROS_INFO("%d %d depth=%d", x, y, depthInMeters);
            // 如果深度值不在指定的范围内，则忽略该点
            if (depthInMeters < minDepth || depthInMeters > maxDepth)
                continue;

            // 计算三维坐标
            double X = (x - intrinsics.cx) * depthInMeters / intrinsics.fx;
            double Y = (y - intrinsics.cy) * depthInMeters / intrinsics.fy;
            double Z = depthInMeters;

            // 创建点云点并设置坐标
            pcl::PointXYZ point;
            point.x = X;
            point.y = Y;
            point.z = Z;
            // ROS_INFO("convertAndPublishDepthToPointCloud: depth=%d, x=%f, y=%f, z=%f", depth, X, Y, Z);
            // 将点添加到点云中
            cloud->push_back(point);
        }
    }
    // 将点云转换为sensor_msgs::PointCloud2消息
    // 更新点云的大小信息
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true; // 若所有点均有效，则设置为true



    // 旋转点云,z轴朝前
    AngleAxisd rotation_x(-M_PI_2, Eigen::Vector3d::UnitX()); // 绕x轴旋转-90度（M_PI_2是90度的弧度值）
    AngleAxisd rotation_y(M_PI_2, Eigen::Vector3d::UnitY()); // 绕y轴旋转yaw_angle度
    rotatePointCloud(cloud, rotation_x, rotation_y);

    // 应用变换矩阵 carmer to imu
    applyTransformation(cloud, transformation_matrix);

    //imu to lidar
    Matrix4d custom_transformation_matrix = Matrix4d::Identity();
    custom_transformation_matrix(0, 3) = -0.011;
    custom_transformation_matrix(1, 3) = -0.02329;
    custom_transformation_matrix(2, 3) = 0.04412;
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *transformed_cloud, custom_transformation_matrix.cast<float>());
    *cloud = *transformed_cloud;

    // 将PCL点云转换为ROS消息格式
    sensor_msgs::PointCloud2 rosCloud;
    pcl::toROSMsg(*cloud, rosCloud);
    rosCloud.header.frame_id = frame_id;
    rosCloud.header.stamp = ros::Time::now();

    // 发布点云消息
    point_cloud_pub.publish(rosCloud);
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


    // lidar到摄像头的变换矩阵
    Matrix4d transformation_matrix;
    transformation_matrix << -0.0347702, -0.999329, -0.0115237, 0.113479,
                              0.0359214, 0.0102735, -0.999302, -0.216314,
                              0.99875, -0.0351599, 0.0355401, -0.00212184,
                              0, 0, 0, 1;


    // 加载深度图像并发布点云
    for (int i = 0; i < 10; ++i)
    {
        convertAndPublishDepthToPointCloud(cv_image, intrinsics, frame_id, transformation_matrix);
        loop_rate.sleep();
    }
    // 保持节点运行直到关闭
    // ros::spin();

    return 0;
}
