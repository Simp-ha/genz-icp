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

    //Loading each frame aka each .bin file
    auto frame = kitti.loadPoses(path_to_data);
    std::cout <<"Poses are loaded"<< std::endl;;
    
    auto timestamps = kitti.loadTimestamps();
    std::cout <<"Timestamps are loaded"<<std::endl;;
    
    auto [planar, non_planar] = odometry.RegisterFrame(frame, timestamps);
    std::cout <<"HOHO THEORETICALLY END OF REGISTER FRAME"<<std::endl;


    // for (const auto& entry : std::filesystem::directory_iterator(path_to_data)) {
    //     if (entry.is_regular_file() && entry.path().extension() == ".bin") {
    //         try {
    //             // auto data = (entry.path().string());
    //             //processBinData(data, entry.path().filename().string());
    //         } catch (const std::exception& e) {
    //             std::cerr << "[ERROR] " << e.what() << std::endl;
    //         }
    //     }
    // }
    // std::vector<Sophus::SE3d> fuckyou = odometry.poses();
//    fuckyou.emplace_back(); 
 
    // Exporting poses into est_poses.txt
    // std::string filename = "est_poses.txt";
    // std::ofstream ofs(filename);
    // if (!ofs.is_open()) {
    //     throw std::runtime_error("Failed to open file: " + filename);
    // }
    // // Exporting da poses 
    // for(const auto &pose : odometry.poses()) {
    //     Eigen::Matrix4d T = pose.matrix();
    //     // Write as 12 values (3x4 part of the matrix)
    //     for (int row = 0; row < 3; ++row) {
    //         for (int col = 0; col < 4; ++col) {
    //             ofs << T(row, col);
    //             if (!(row == 2 && col == 3)) ofs << " "<<std::flush;
    //             // (row == 2 && col == 3) ? ofs << "\n" :  ofs << " "; //aleternative
    //         }
    //     }
    //     ofs << std::endl;
    //     std::cout << "Printing out the end of the algorithm ig" << std::endl;
    // }
    // ofs.close();
    
    return 0;
}
