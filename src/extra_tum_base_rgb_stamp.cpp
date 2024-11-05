#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <algorithm>
#include <string>
#include <dirent.h>
#include <sys/stat.h>

// 从这个路径下读取这样格式1724901685_314471722.png的文件名，按时间戳排序
// /media/benny/HIKSEMI/3DGS_data_for_Wang_20240915/_camera_color_image_raw/
// 然后从这个路径下读取这个txt文件，文件里有多行tum格式的数据，如：1724901680.477288 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 1.000000
// /media/benny/HIKSEMI/3DGS_data_for_Wang_20240915/pose_100HZ.txt
// 再找到与当前文件名最近的一行数据，读取位置和姿态四元数和文件名对应的时间戳输出到new_pose_15hz.txt
struct PoseData {
    double timestamp;
    std::vector<double> position_quaternion;  // 假设位置+姿态四元数总共7个值
};

std::vector<PoseData> readPoseData(const std::string& filePath) {
    std::vector<PoseData> poses;
    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filePath << std::endl;
        return poses;
    }

    std::string line;
    while (getline(file, line)) {
        std::istringstream iss(line);
        PoseData pose;

        // 读取时间戳
        if (!(iss >> pose.timestamp)) {
            continue; // 如果无法读取时间戳，则跳过此行
        }

        // 读取位置和姿态数据
        double value;
        while (iss >> value) {
            pose.position_quaternion.push_back(value);
        }

        // 如果读取到了至少一个位置/姿态值，则添加到poses中
        if (!pose.position_quaternion.empty()) {
            poses.push_back(pose);
        }
    }
    file.close();
    return poses;
}

std::vector<std::string> getPngFileNames(const std::string& dirPath) {
    std::vector<std::string> fileNames;
    DIR* dir;
    struct dirent* ent;
    if ((dir = opendir(dirPath.c_str())) != nullptr) {
        while ((ent = readdir(dir)) != nullptr) {
            std::string name(ent->d_name);
            if (name.size() > 4 && name.substr(name.size() - 4) == ".png" && name != "." && name != "..") {
                size_t underscorePos = name.find('_');
                std::string timeStr = name.substr(0, underscorePos) + "."
                + name.substr(underscorePos + 1, name.find('.png') - underscorePos - 4);
                fileNames.push_back(timeStr);
            }
        }
        closedir(dir);
    }
    return fileNames;
}

auto findClosestPose(const std::vector<PoseData>& poses, double timeStamp) {
    return std::min_element(poses.begin(), poses.end(),
        [&timeStamp](const PoseData& a, const PoseData& b) {
            return std::abs(a.timestamp - timeStamp) < std::abs(b.timestamp - timeStamp);
        });
}

int main() {
    std::cout << "this is extra RGB tum funtion" << std::endl;
    std::string file_name = "gml_2024-10-15-10-53-01";
    std::string baseDir = "/media/benny/bennyMove/data/dog_origin/";
    //图片文件夹路径
    const std::string dirPath = baseDir + file_name + "/_camera_color_image_raw";
    //pose文件路径
    const std::string poseFilePath = baseDir + file_name + "/pose_200hz.txt";
    //输出文件路径
    const std::string outputFilePath = baseDir + file_name + "/pose_mid_360.txt";
    const std::string outputTampFilePath = baseDir + file_name + "/pose_mid_360_tamp.txt";


    std::vector<PoseData> poses = readPoseData(poseFilePath);
    std::sort(poses.begin(), poses.end(), [](const PoseData& a, const PoseData& b)
    {
        return a.timestamp < b.timestamp;
    });
    std::vector<std::string> pngFiles = getPngFileNames(dirPath);
    std::sort(pngFiles.begin(), pngFiles.end(), [](const std::string& a, const std::string& b)
    {
        return std::stod(a) < std::stod(b);
    });

    std::ofstream outputFile(outputFilePath);
    if (!outputFile.is_open()) {
        std::cerr << "Failed to open output file: " << outputFilePath << std::endl;
        return 1;
    }

    std::ofstream outputTampFile(outputTampFilePath);
    if (!outputTampFile.is_open()) {
        std::cerr << "Failed to open output file: " << outputTampFilePath << std::endl;
        return 1;
    }

    auto lastIt = poses.begin();

    for (const auto& fileName : pngFiles) {

        double timeStamp = std::stod(fileName);

        // Find the closest pose data
        auto it = std::min_element(lastIt, poses.end(),
            [&timeStamp](const PoseData& a, const PoseData& b) {
                return std::abs(a.timestamp - timeStamp) < std::abs(b.timestamp - timeStamp);
            });

        lastIt = it;
        if (it != poses.end()) {
            std::ostringstream oss;
            std::ostringstream ossTamp;
            ossTamp << fileName << " ";
            for (size_t i = 0; i < it->position_quaternion.size(); ++i)
            {
                oss << it->position_quaternion[i]; // 写入位置/姿态值
                ossTamp << it->position_quaternion[i]; // 写入位置/姿态值
                if (i < it->position_quaternion.size() - 1)
                {
                    oss << " "; // 如果不是最后一个值，则添加空格
                    ossTamp << " "; // 如果不是最后一个值，则添加空格
                }
            }
            // 将oss中构建的字符串写入到输出文件中
            outputFile << oss.str();
            outputTampFile << ossTamp.str();
            outputFile << std::endl;
            outputTampFile << std::endl;
        }
    }

    outputFile.close();
    outputTampFile.close();
    return 0;
}
