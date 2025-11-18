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
#include <chrono>
#include <map>

// GenZ-ICP
#include "genz_icp/pipeline/GenZICP.hpp"
#include "genz_icp/metrics/Metrics.hpp"
#include "genz_icp/dataloader/Kitti.hpp"
using Vector3dVector = std::vector<Eigen::Vector3d>;

// Apply calibration for Matrix
inline void apply_calibration(Eigen::Matrix4d& M, Eigen::Matrix4d& T) { T = M * T * M.inverse(); } 

// Apply calibration for whole pose 
inline std::vector<Eigen::Matrix4d> apply_calibration(Eigen::Matrix4d& M, std::vector<Eigen::Matrix4d>& poses) { 
    std::vector<Eigen::Matrix4d> calibrated_data;
    for(const auto& T:poses){
        calibrated_data.push_back( M * T * M.inverse());
    }
    return calibrated_data;
}

// Insert a calibration Matrix 4x4 by using the kitti read calib data
inline Eigen::Matrix4d insert_calibration_matrix (const std::string& calib_path) {
    // Dictionary calib from calib.txt
    auto calib = kitti.read_calib_data(calib_path);
    // Grubbing data with key "Tr" from calib dictionary
    Eigen::Map<const Eigen::Matrix<float,3,4,Eigen::RowMajor>> m34(calib["Tr"].data());
    Eigen::Matrix4d M = Eigen::Matrix4d::Identity(); 
    // Converting into homogenous matrix 4x4
    M.block<3,4>(0,0) = m34.cast<double>();
    return M;
}

bool eval (const std::string& result_dir, std::string& seq){
    // ground truth and result directories
    const std::string gt_dir      = dataset + "poses/" + seq;
    const std::string dataset_seq = dataset + "sequences/" + seq ; 

    auto poses_gt = kitti.loadposes(gt_dir + ".txt");
    // Already calibrated poses
    auto poses = kitti.loadposes(result_dir + "_est_poses.txt");
    
    // auto M = insert_calibration_matrix(dataset_seq + "/calib.txt");
    // auto calib_poses_gt = apply_calibration(M, poses_gt );
    // auto calib_poses = apply_calibration(M, poses);

    auto [avgT, avgR] =  genz_icp::metrics::SeqError(poses_gt, poses);
    auto [AbsR, AbsT] =  genz_icp::metrics::AbsoluteTrajectoryError(poses_gt, poses);

    std::ofstream res_file(result_dir + "_results.txt");
    
    res_file << "The Average Translation Error is :     " << avgT << std::endl;
    res_file << "The Average Rotation Error is :        " << avgR << std::endl;
    res_file << "The Absolute Translation Error is :    " << AbsT << std::endl;
    res_file << "The Absolute Rotation Error is :       " << AbsR << std::endl;

    std::cout << "The Average Translation Error is :    " << avgT << std::endl;
    std::cout << "The Average Rotation Error is :       " << avgR << std::endl;
    std::cout << "The Absolute Translation Error is :   " << AbsT << std::endl;
    std::cout << "The Absolute Rotation Error is :      " << AbsR << std::endl;

    // success
	return true;
}

bool is_empty(const std::string& filename) {
    std::ifstream file(filename);
    return file.tellg() == 0 && file.peek() == std::ifstream::traits_type::eof();
}

std::vector<std::vector<std::string>> batch_maker(const int& n, const int& batch_size, const std::vector<std::string>& paths){
    // Initial variable
    std::vector<std::vector<std::string>> batches ;
    std::vector<std::string> batch;
    for(int i = 0; i < n + 1; ++i){
        int bi = i*batch_size;
        for(int j= 0; j < batch_size; j++){
            if(paths[bi+j].empty()) break;
            batch.push_back(paths[bi+j]);
        }
        batches.push_back(batch);
        batch.clear();
    }
    return batches;
}

int main(int argc, char* argv[]){
    // First Arguments for outputs etc.
    // const std::string n_seq = "00";
    //for(int i = 2; i < 11; ++i){
    bool debug = false;
    char* p;
    int i = strtol(argv[1], &p, 10);
    std::string seq = kitti.seq[i];
    
    // Exporting poses into est_poses.txt
    if (std::filesystem::exists("./results/"+seq))
        std::cout << "Result " << seq << " directory already exists" << std::endl;
    else
        system(("mkdir -p ./results/" + seq).c_str());
    
    //Results directory with extension of sequence number for file
    const std::string result_dir = "results/" + seq;
    //Output estimated poses file
    const std::string filename = result_dir + "_est_poses.txt";
    const std::string dataset_seq = dataset + "sequences/" + seq ; 
    
    //Running the GENZ
    if(true){//std::filesystem::exists(filename) || is_empty(filename)){

        //Loading each frame aka each .bin file into the bindir
        std::vector<std::string> bindir;
        std::cout << "Loading frames" << std::endl;
        for (const auto& entry : std::filesystem::directory_iterator(dataset_seq + "/velodyne/")) {
            if (entry.is_regular_file() && entry.path().extension() == ".bin") {
                try {
                    bindir.push_back(entry.path());
                } catch (const std::exception& e) {
                    std::cout << "[ERROR] " << e.what() << std::endl;
                }
            }
            else{ 
                std::cout << "No frames there..." << std::endl ;    
                return 0;
            }
        }
        
        // Load timestamps
        auto timestamps = kitti.loadTimestamps(dataset_seq + "/times.txt");
        
        // Sort out the binary data directory
        std::sort(bindir.begin(),bindir.end());
        std::vector<std::vector<std::string>> batches;
        const int batch_size = 300;
        const int n = bindir.size()/batch_size;

        batches = batch_maker(n, batch_size, bindir);
        std::cout << "Batches made" << std::endl;
        
        // std::tuple<Vector3dVector, Vector3dVector> results[n];
        // std::vector<std::tuple<Vector3dVector, Vector3dVector>> ALLres;
        
        std::ofstream file("frame.txt");
        // Inserting the points of a sequence and the GenZ Begins
        // for(int i =0; i < n + 1; ++i ){
        //     for(int j = 0; j < batch_size; j++){

        // Dictionary-> file_registered : time_in_ms 
        std::vector<Vector3dVector> frames; 
        std::unordered_map<std::string, double> Timings;

        for(auto batch : batches){
            for(auto b : batch){
                auto frame = kitti.loadframe(b);
                std::cout << "BATCH "  << i << " " << b << std::endl;
                if(debug){
                    std::cout<<"Frame=["; 
                    for(auto f: frame) {
                        for(auto p: f) file << p << " ";
                        file << std::endl;
                    }
                }
                // Timing the RegisterFrame function
                auto t1 = std::chrono::high_resolution_clock::now();
                odometry.RegisterFrame(frame, timestamps);
                auto t2 = std::chrono::high_resolution_clock::now();
                double Dt = std::chrono::duration<double, std::milli>(t2 -t1).count();

                // Inserting into dictionary only the ones > 200 ms
                if(Dt > 200) Timings[b] = Dt;
            }
            // Exporting the results of the time
            for(auto t : Timings) std::cout << t.first << " : " << t.second << std::endl;
            getchar();
        }
        file.close();
        //Writing output seq_est_poses.txt
        std::ofstream ofs(filename);
        if (!ofs.is_open()) {
            throw std::runtime_error("Failed to open file: " + filename);
        }
        
        auto calibrate = [&](Eigen::Matrix4d& T){
            // Dictionary calib from calib.txt
            auto calib = kitti.read_calib_data(dataset_seq + "/calib.txt");
            // Grubbing data with key "Tr" from calib dictionary
            Eigen::Map<const Eigen::Matrix<float,3,4,Eigen::RowMajor>> m34(calib["Tr"].data());
            Eigen::Matrix4d M = Eigen::Matrix4d::Identity(); 
            // Converting into homogenous matrix 4x4
            M.block<3,4>(0,0) = m34.cast<double>();
            T = M*T*M.inverse();
        };

        for(const auto &pose : odometry.poses()) {
            Eigen::Matrix4d T = pose.matrix();
            std::cout << T << std::endl;
            calibrate(T);
            // Calibrate the data
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
          
    }

  //if(eval(result_dir, seq))std::cout << "Evaluation is completed" << std::endl;
    return 0;
}
