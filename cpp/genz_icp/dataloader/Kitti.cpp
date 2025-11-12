#include "Kitti.hpp"

#include<iostream>
#include<Eigen/Core>
#include<fstream>

std::vector<double> KITTI::loadTimestamps(const std::string& timestamps_path) {
    std::vector<double> timestamps;
    std::ifstream fp(timestamps_path);
    if (!fp.is_open()) return timestamps;
    double S;
    while (fp >> S) timestamps.push_back(S);
    fp.close();
    return timestamps;
}

// Loading points from binary files 
KITTI::Vector3dVector KITTI::loadframe(std::string& binfile) {
    Vector3dVector points;
    
    // Read bin file
    std::ifstream fp(binfile, std::ios::binary);
    
    if (!fp.is_open()) {
        std::cout << "Cannot open file: " << binfile << std::endl;
        return points;
    }

    fp.seekg(0,std::ios::end);
    std::streamsize fp_size = fp.tellg();
    fp.seekg(0,std::ios::beg);
    
    // Making buffer of n size of floats
    std::vector<float> buffer(fp_size/sizeof(float));
    if(!fp.read(reinterpret_cast<char*> (buffer.data()), fp_size)){
        std::cout << "Failed to read file." << std::endl;
    }

    fp.close();

    // Number of points in each bin file aka each frame
    size_t n_points = buffer.size() / 4;
    points.reserve(n_points);

    for(size_t i = 0; i < n_points; ++i){
        points.push_back({
            static_cast<double> ( buffer[i * 4 + 0] ),
            static_cast<double> ( buffer[i * 4 + 1] ),
            static_cast<double> ( buffer[i * 4 + 2] )
        });
    }
    return points;
} 

std::vector<Eigen::Matrix4d> KITTI::loadposes(const std::string& poses_path) {
    std::vector<Eigen::Matrix4d> poses;
    std::ifstream poses_file(poses_path);
    Eigen::Matrix4d P = Eigen::Matrix4d::Identity(); 
    if (!poses_file.is_open()) {
        std::cout << "Can't open file " << poses_path <<  std::endl;
        return poses;
    }
    
    // Read each line of the file with poses
    while(poses_file >> 
        P(0, 0) >> P(0, 1) >> P(0, 2) >> P(0, 3) >>
        P(1, 0) >> P(1, 1) >> P(1, 2) >> P(1, 3) >> 
        P(2, 0) >> P(2, 1) >> P(2, 2) >> P(2, 3)){
            //Pushing each line aka 4 numbers of the T matrix into poses vector
            poses.push_back(P);
    }
    
    poses_file.close();
    return poses;
}

//Making dictionary from calibration file
std::unordered_map<std::string, std::vector<float>> KITTI::read_calib_data(const std::string& calib_path){
    std::unordered_map<std::string, std::vector<float>> calib_dict;
    std::ifstream f(calib_path);

    if(!f.is_open()){
        std::cout << "Failed to open the file" << std::endl;
    }

    std::string line;
    while (std::getline(f, line)) {
            std::istringstream iss(line);
            std::string key;
            iss >> key; // read first token (e.g., "P0:")

            if (key == "calib_time:") {
                continue;
            }

            // Remove trailing colon from key
            if (!key.empty() && key.back() == ':') {
                key.pop_back();
            }

            std::vector<float> values;
            float val;
            while (iss >> val) {
                values.push_back(val);
            }

            if (!values.empty()) {
                calib_dict[key] = values;
            }
        }

        f.close();
        return calib_dict;
    }
