//
// Created by benny on 24-10-15.
//
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <algorithm>
#include <string>
#include <dirent.h>
#include <filesystem>
#include <iomanip>
#include <sys/stat.h>

namespace fs = std::filesystem;

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

int main()
{
    std::cout << "this is Change RGB file funtion" << std::endl;
    //图片文件夹路径
    std::string file_name = "gml_2024-10-15-10-53-01";

    bool isRgb = true;

    std::string baseDir = "/media/benny/bennyMove/data/dog_origin/";

    std::string dirPath, tranPath;

    if (isRgb) {
        dirPath = baseDir + file_name + "/_camera_color_image_raw/";
        tranPath = baseDir + file_name + "/rgb/";
    } else {
        dirPath = baseDir + file_name + "/_camera_depth_image_rect_raw/";
        tranPath = baseDir + file_name + "/depth_png/";
    }
    if(!fs::exists(tranPath))
    {
        fs::create_directory(tranPath);
        std::cout << "Created directory: " << tranPath << std::endl;
    }

    std::cout << "dirPath: " << dirPath << std::endl;
    std::cout << "tranPath: " << tranPath << std::endl;
    std::vector<std::string> pngFiles = getPngFileNames(dirPath);
    std::sort(pngFiles.begin(), pngFiles.end(), [](const std::string& a, const std::string& b)
    {
        return std::stod(a) < std::stod(b);
    });
    for (int i = 0; i < pngFiles.size(); ++i)
    {
        std::string stamp = pngFiles[i];
        std::replace(stamp.begin(), stamp.end(), '.', '_');
        std::string fileName = stamp + ".png";
        // 构建原始文件路径和新文件路径
        std::string originalFilePath = dirPath + fileName;
        std::ostringstream oss;
        oss << std::setw(6) << std::setfill('0') << i;
        std::string newFileName = oss.str() + ".png";
        std::string newFilePath = tranPath + newFileName;
        // 复制文件
        std::ifstream src(originalFilePath, std::ios::binary);
        std::ofstream dst(newFilePath, std::ios::binary);
        dst << src.rdbuf();
        // 关闭文件流
        src.close();
        dst.close();
        // std::cout << newFilePath << std::endl;
    }
    return 0;
}