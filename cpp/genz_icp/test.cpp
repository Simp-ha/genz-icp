#include "test.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <memory>
#include <sophus/se3.hpp>
#include <utility>
#include <vector>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <tuple>
#include <vector>

// GenZ-ICP
#include "genz_icp/pipeline/GenZICP.hpp"
#include "genz_icp/dataloader/Kitti.hpp"
// #include <sqlite3.h>
//#include <yaml-cpp/yaml.h>
// #include "devkit/cpp/evaluate_odometry.hpp"



int main(){
    // Object of genz algorithm 
    odometry = genz_icp::pipeline::GenZICP(config);

    //Loading each frame aka each .bin file into the datadir
    std::vector<std::string> datadir;
    for (const auto& entry : std::filesystem::directory_iterator(path_to_data)) {
        if (entry.is_regular_file() && entry.path().extension() == ".bin") {
            try {
                datadir.push_back(entry.path());
            } catch (const std::exception& e) {
                std::cout << "[ERROR] " << e.what() << std::endl;
            }
        }
    }

    // Sort out the datadir
    std::sort(datadir.begin(),datadir.end());

    // for(auto d: datadir){
        std::cout << datadir[0] << std::endl;
        auto frame = kitti.loadPoses(datadir[0]);
        for(auto f: frame) std::cout <<f<< std::endl;
            
        auto timestamps = kitti.loadTimestamps();
        // std::cout <<"Timestamps are loaded"<<std::endl;;
            
        auto [planar, non_planar] = odometry.RegisterFrame(frame, timestamps);
        // std::cout <<"HOHO THEORETICALLY END OF REGISTER FRAME"<<std::endl;
    // }
 
    // Exporting poses into est_poses.txt
    // std::string filename = "timestamps.txt";
    // std::ofstream ofs(filename);
    // if (!ofs.is_open()) {
    //     throw std::runtime_error("Failed to open file: " + filename);
    // }
    std::string filename = "est_poses.txt";
    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
        throw std::runtime_error("Failed to open file: " + filename);
    }

    // Exporting da poses 
    for(const auto &pose : odometry.poses()) {
        Eigen::Matrix4d T = pose.matrix();
        // Write as 12 values (3x4 part of the matrix)
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 4; ++col) {
                ofs << T(row, col);
                if (!(row == 2 && col == 3)) ofs << " "<<std::flush;
                // (row == 2 && col == 3) ? ofs << "\n" :  ofs << " "; //aleternative
            }
        }
        ofs << std::endl;
    }
    ofs.close();
    
    return 0;
}
