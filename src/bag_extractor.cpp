#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/filesystem.hpp>
#include <string>
#include <algorithm>

void saveImage(const sensor_msgs::Image::ConstPtr& img_msg, const std::string& dir_name, const std::string& encoding) {
    // 将ROS的Image消息转换为OpenCV格式
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(img_msg, encoding);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // 获取时间戳
    ros::Time timestamp = img_msg->header.stamp;
    std::stringstream ss;
    std::stringstream nss;
    nss << std::setw(9) << std::setfill('0')  << timestamp.nsec;
    ss << dir_name << "/" << timestamp.sec << "_" << nss.str() << ".png";

    // 保存图片
    cv::imwrite(ss.str(), cv_ptr->image);
    ROS_INFO("Saved image: %s, Image format: %s", ss.str().c_str(), encoding.c_str());
}

void savePointCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, const std::string& dir_name) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);

    // 获取时间戳
    ros::Time timestamp = cloud_msg->header.stamp;
    std::stringstream ss;
    std::stringstream nss;
    nss << std::setw(9) << std::setfill('0')  << timestamp.nsec;
    ss << dir_name << "/" << timestamp.sec << "_" << nss.str() << ".pcd";

    // 保存点云
    pcl::io::savePCDFileBinary(ss.str(), cloud);
    ROS_INFO("Saved point cloud: %s, Point cloud format: PCD (Point Cloud Data)", ss.str().c_str());
}

int main(int argc, char** argv) {
    if (argc < 2) {
        ROS_ERROR("Usage: rosrun your_package extract_images_and_pointclouds_from_bag <bag_file>");
        return 1;
    }

    std::string bag_file = argv[1];

    // 提取bag文件的目录路径
    boost::filesystem::path bag_path(bag_file);
    std::string parent_path = bag_path.parent_path().string();
    std::string bag_name = bag_path.stem().string();  // 获取不带扩展名的文件名

    // 设置保存路径为父目录下，与bag文件同名的子目录
    boost::filesystem::path save_dir = parent_path + "/" + bag_name;
    // 检查提取的路径是否存在
    if (!boost::filesystem::exists(save_dir)) {
        ROS_ERROR("Extracted directory path does not exist: %s", save_dir.c_str());
        boost::filesystem::create_directories(save_dir);
        ROS_WARN("Directory created successfully !!!.");
    }

    rosbag::Bag bag;
    bag.open(bag_file, rosbag::bagmode::Read);

    // 创建一个视图，遍历bag中的所有sensor_msgs/Image和sensor_msgs/PointCloud2类型的消息
    rosbag::View view(bag);  // 不指定话题，遍历所有消息

    // 处理每一条消息
    for (rosbag::MessageInstance const m : view) {
        // 获取原始话题名
        std::string topic_name = m.getTopic();

        // 获取话题名并将斜杠替换为下划线
        std::string modified_topic_name = topic_name;
        std::replace(modified_topic_name.begin(), modified_topic_name.end(), '/', '_');

        // 创建对应的文件夹
        boost::filesystem::path dir(save_dir / modified_topic_name);
        if (!boost::filesystem::exists(dir)) {
            boost::filesystem::create_directories(dir);
        }

        // 提取图片
        if (m.getDataType() == "sensor_msgs/Image") {
            sensor_msgs::Image::ConstPtr img_msg = m.instantiate<sensor_msgs::Image>();
            if (img_msg != nullptr) {
                std::string encoding;
                if (topic_name == "/camera/color/image_rect_raw") {
                    encoding = sensor_msgs::image_encodings::RGB8;
                } 
                else if (topic_name == "/camera/depth/image_rect_raw") {
                    encoding = sensor_msgs::image_encodings::TYPE_16UC1;
                } 
                else if (topic_name == "/camera/infra1/image_rect_raw" || topic_name == "/camera/infra2/image_rect_raw") {
                    encoding = sensor_msgs::image_encodings::MONO8;
                } 
                else 
                {
                    encoding = sensor_msgs::image_encodings::BGR8; // 默认编码
                }
                saveImage(img_msg, dir.string(), encoding);
            }
        }

        // 提取点云
        if (m.getDataType() == "sensor_msgs/PointCloud2") {
            sensor_msgs::PointCloud2::ConstPtr cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
            if (cloud_msg != nullptr) {
                savePointCloud(cloud_msg, dir.string());
            }
        }
    }

    bag.close();
    return 0;
}
