#pragma once
#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <vector>

struct KITTI{
    using Vector3dVector = std::vector<Eigen::Vector3d>;
    using Vector3dDoubleTuple= std::tuple<Vector3dVector, double>;
    //Variables
    std::string data_path;

    explicit KITTI(std::string& path):data_path(path){} 

    std::vector<double> loadTimestamps(/*std::string file_name*/) ;
    Vector3dVector loadPoses(const std::string& binfile);
    // void read_calib_file(const std::string& calib_path);
    // void calibration( const auto poses);
};