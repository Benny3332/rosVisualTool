#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <sstream>
#include <string>

class OdometryPublisher {
public:
    // 构造函数初始化ROS节点句柄和参数
    OdometryPublisher(ros::NodeHandle& nh, const std::string& odom_file, const std::string& topic_name)
     : nh_(nh), odom_file_(odom_file), count_(0) {
        // 初始化发布者
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>(topic_name, 10);
        // 从ROS参数服务器获取文件路径，默认值为"路径/到/你的/odom.txt"
        //odom_file_ = nh_.param<std::string>("odom_file", "/data/bag/gml_2024-10-15-19-38-50/pose_mid_360_tamp.txt");
    }

    // 发布里程计信息的方法
    void publishOdometry() {
        // 打开文件
        std::ifstream file(odom_file_);
        std::string line;

        // 检查文件是否成功打开
        if (!file.is_open()) {
            ROS_ERROR("Failed to open file %s", odom_file_.c_str());
            return;
        }

        // 如果文件包含头部信息，则跳过第一行
        std::getline(file, line); // Assuming first line is header.

        // 遍历文件中的每一行
        while (getline(file, line)) {
            // 使用字符串流解析每一行数据
            std::istringstream iss(line);
            double timestamp, tx, ty, tz, qx, qy, qz, qw;

            // 尝试从行中读取所有需要的数据
            if (!(iss >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw)) {
                ROS_ERROR("Failed to parse line: %s", line.c_str());
                continue;
            }

            // 创建一个新的Odometry消息
            nav_msgs::Odometry odom;
            // 设置时间戳（这里使用当前时间，如果需要使用文件中的时间戳，请取消注释下面一行）
            // odom.header.stamp = ros::Time(timestamp);
            odom.header.stamp = ros::Time::now();
            // 设置参考系ID
            odom.header.frame_id = "camera_init";
            // 设置子参考系ID
            odom.child_frame_id = "body";

            // 设置位置数据
            odom.pose.pose.position.x = tx;
            odom.pose.pose.position.y = ty;
            odom.pose.pose.position.z = tz;

            // 设置四元数姿态数据
            odom.pose.pose.orientation.x = qx;
            odom.pose.pose.orientation.y = qy;
            odom.pose.pose.orientation.z = qz;
            odom.pose.pose.orientation.w = qw;

            // 发布Odometry消息
            odom_pub_.publish(odom);
            ++count_;

            // 控制发布频率，防止过快发布
            ros::Duration(0.0666666).sleep(); // Sleep for 100ms between publications.
        }

        // 关闭文件
        file.close();
    }

private:
    ros::NodeHandle nh_; // ROS节点句柄
    ros::Publisher odom_pub_; // 发布者对象
    std::string odom_file_; // 文件路径
    int count_; // 计数器
};

int main(int argc, char** argv) {

    // 初始化ROS节点
    ros::init(argc, argv, "tum_odometry_publisher");
    ros::NodeHandle nh;
    std::string nodeNameOnly = ros::this_node::getName();
    std::string tum_file;
    std::string topic_name;
    ROS_INFO("node name: %s", nodeNameOnly.c_str());
    if(nh.hasParam(nodeNameOnly + "/tum_file")){
        nh.getParam(nodeNameOnly + "/tum_file", tum_file);
    } else{
        ROS_ERROR("No tum file path");
        return 0;
    }
    nh.getParam(nodeNameOnly + "/topic_name", topic_name);
	ROS_INFO("pubilsh tum to odometry: %s", tum_file.c_str());
    // 创建OdometryPublisher实例
    OdometryPublisher op(nh, tum_file, topic_name);

    // 调用发布函数
    op.publishOdometry();
    std::cout << "Published is done" << std::endl;
    // 进入ROS主循环
    //ros::spin();
    return 0;
}