#pragma once
#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <vector>

struct KITTI{
    using Vector3dVector = std::vector<Eigen::Vector3d>;
    using Vector3dDoubleTuple= std::tuple<Vector3dVector, double>;
    //Variables
    const std::string data_path;
    std::vector<std::string> seq = {"00", "01", "02", "03", "04", "05", "06", "07", "08", "09", "10", "11" };

    explicit KITTI(const std::string& path):data_path(path){} 

    std::vector<double> loadTimestamps(const std::string& file_name);
    Vector3dVector loadframe(std::string& binfile);
    std::vector<Eigen::Matrix4d> loadposes(const std::string& filepath);

    std::unordered_map<std::string, std::vector<float>> read_calib_data(const std::string& calib_path);
    // void calibration( const auto poses);
};