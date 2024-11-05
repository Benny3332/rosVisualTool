//
// Created by jin on 24-3-24.
//

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <fstream>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <filesystem>

#include <filesystem>

enum converter_flag
{
    TUM=1 , 
    CLOUD = 2 ,
    TAC = 3
};

class PathConverter {
public:
    PathConverter(ros::NodeHandle nh, std::string topic, int flag, std::string topic_cfg, std::string path_dir, std::string name)
    :   path_topic(topic), convt_flag(flag) ,path_topic_cfg(topic_cfg), save_path(path_dir), file_name(name)
    {
        // 订阅 nav_msgs/Path 消息
        path_sub = nh.subscribe(path_topic, 1000, &PathConverter::pathCallback, this);
//        path_sub1 = nh.subscribe("/path_global", 10, &PathConverter::pathCallback, this);
    }

    void pathCallback(const nav_msgs::Path::ConstPtr& path_msg) 
    {
        switch (convt_flag)
        {
        case (TUM):
            pathToTUM(path_msg);
            break;
        case (CLOUD):
            pathToCloud(path_msg);
            break;
        case (TAC):
            pathToTUM(path_msg);
            pathToCloud(path_msg);
            break;
        default:
           printf("Error Convert Type");
            break;
        } 

    }

    void pathToTUM(const nav_msgs::Path::ConstPtr& path_msg) 
    {
        // Check if the directory exists, if not, create it
        std::string file_path = save_path + "traj/";
        std::filesystem::path dir(file_path);
        if (!std::filesystem::exists(dir)) {
            std::filesystem::create_directories(dir);
            ROS_INFO("Created directory: %s", dir.c_str());
        }

        std::string TUM_name = file_path + file_name + ".txt";
        std::ofstream outFile(TUM_name);

        // 检查文件是否成功打开
        if (!outFile.is_open()) {
            ROS_ERROR("Failed to open file for writing.");
            return;
        }

        // 遍历路径中的所有位置，将其转换为 TUM trajectory format 并写入文件
        for(const auto& pose_stamped : path_msg->poses) {
            float x = pose_stamped.pose.position.x;
            float y = pose_stamped.pose.position.y;
            float z = pose_stamped.pose.position.z;
            float qx = pose_stamped.pose.orientation.x;
            float qy = pose_stamped.pose.orientation.y;
            float qz = pose_stamped.pose.orientation.z;
            float qw = pose_stamped.pose.orientation.w;

            // 获取时间戳
//            double secs = pose_stamped.header.stamp.toSec();
//            double secs = ros::Time::now().toSec();

            // 获取原始的时间戳（秒和纳秒）
            uint32_t secs = pose_stamped.header.stamp.sec;
            uint32_t nsecs = pose_stamped.header.stamp.nsec;
            // 为了兼容evo_traj的时间格式，我们需要将时间转换成浮点数
            // 但是保留原始的秒和纳秒形式，格式化为一个长整型或浮点数
            // 获取原始时间戳
            // double time = secs + nsecs * 1e-9;
            double time = pose_stamped.header.stamp.toSec();


            // 获取现在的时间
            // double time = ros::Time::now().toSec();

            // 使用fixed和setprecision来控制输出格式
            // setprecision指定了小数点后的数字位数，足够覆盖纳秒级精度
            outFile << std::fixed << std::setprecision(6)<< time << " " << x << " " << y << " " << z << " " << qx << " " << qy << " " << qz << " " << qw << std::endl;
        }

        outFile.close();
        ROS_INFO("Path data of %f has been saved in TUM trajectory format.", path_topic);
    }

    void pathToCloud(const nav_msgs::Path::ConstPtr& path_msg) 
    {
        //  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
         pcl::PointCloud<pcl::PointXYZ> cloud;

        // 遍历路径中的所有位置，将其添加到点云数据中
        for(const auto& pose_stamped : path_msg->poses) {
            float x = pose_stamped.pose.position.x;
            float y = pose_stamped.pose.position.y;
            float z = pose_stamped.pose.position.z;  // 如果z值不存在，可以设为0或其他合适的值
            // cloud->push_back(pcl::PointXYZ(x, y, z));
            cloud.push_back(pcl::PointXYZ(x, y, z));
        }

        // Check if the directory exists, if not, create it
        std::string file_path = save_path + "pcd/";
        std::filesystem::path dir(file_path);
        if (!std::filesystem::exists(dir)) {
            std::filesystem::create_directories(dir);
            ROS_INFO("Created directory: %s", dir.c_str());
        }

        // 保存点云数据到文件，假设你想要保存到当前目录
        std::string cloud_name = file_path + file_name + ".pcd";
        // pcl::io::savePCDFileASCII(cloud_name, *cloud);
        // ROS_INFO("Saved %d data points to path_cloud.pcd.", (int)cloud->size());

        pcl::io::savePCDFileASCII(cloud_name, cloud);
        ROS_INFO("Saved %d data points to %s.", (int)cloud.size(), cloud_name.c_str());
    }
    

public:
//    ros::NodeHandle nh;
    ros::Subscriber path_sub;
    std::string path_topic;
    std::string path_topic_cfg = "path_save/path_topic";
    std::string save_path;
    std::string file_name;
    int convt_flag; // 1 for TUM, 2 for point cloud, 3 for TUM and point cloud
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_to_tum_converter");
    ros::NodeHandle nh;

    std::string path_dir(std::string(ROOT_DIR) + "converted_data/");
    // Check if the directory exists, if not, create it
    std::filesystem::path dir(path_dir);
    if (!std::filesystem::exists(dir)) 
    {
        std::filesystem::create_directories(dir);
        ROS_INFO("Created directory: %s", dir.c_str());
    }

    std::string path_topic1 = "/path";
    // std::string file_name1 = std::string("Truth.txt");
    std::string topic_cfg1 = "path_save/path_topic1";
    std::string file_name1 = std::string("pose_10hz");

    std::string path_topic2 = "/path_global";
    std::string topic_cfg2 = "path_save/path_topic2";
    std::string file_name2 = std::string("NDT_relocalization");

    // std::string path_topic3 = "/fused_path";
    std::string path_topic3 = "fused_path";
    std::string topic_cfg3 = "path_save/path_topic3";
    std::string file_name3 = std::string("GNSS");

    // 1 for TUM, 2 for point cloud, 3 for TUM and point cloud
    PathConverter converter1(nh,path_topic1, 3,topic_cfg1, path_dir,file_name1);
//    PathConverter converter2(nh,path_topic2, 3,topic_cfg2, path_dir,file_name2);
//    PathConverter converter3(nh,path_topic3, 3,topic_cfg3, path_dir,file_name3);
    ros::spin();
    return 0;
}
