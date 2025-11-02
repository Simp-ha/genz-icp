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
#include "genz_icp/metrics/Metrics.hpp"
#include "genz_icp/dataloader/Kitti.hpp"


bool eval (const std::string& result_dir, std::string& seq){
    // ground truth and result directories

    std::string gt_dir         = "/home/dezu/Documents/DIPLO/Dataset/poses/";
    // std::string error_dir      = result_dir + "/errors";
    // std::string plot_path_dir  = result_dir + "/plot_path";
    // std::string plot_error_dir = result_dir + "/plot_error";

    // // create output directories
    // if( std::filesystem::exists(error_dir)||
    //     std::filesystem::exists(error_dir)||
    //     std::filesystem::exists(error_dir)){
    //         std::cout << "Error directories already exists"
    // }
    // system(("mkdir " + error_dir).c_str());
    // system(("mkdir " + plot_path_dir).c_str());
    // system(("mkdir " + plot_error_dir).c_str());

    const auto poses_gt = kitti.loadposes(gt_dir + seq + ".txt");
    const auto poses = kitti.loadposes(result_dir + "_est_poses.txt");
    // file name
    // char file_name[256];
    // sprintf(file_name,"%02d.txt",seq);
        
    // save + plot bird's eye view trajectories
    // savePathPlot(poses_gt,poses_result,plot_path_dir + "/" + file_name);
    // vector<int32_t> roi = computeRoi(poses_gt,poses_result);
    // plotPathPlot(plot_path_dir,roi,i);

    // // save + plot individual errors
    // char prefix[16];
    // sprintf(prefix,"%02d",i);
    // saveErrorPlots(seq_err,plot_error_dir,prefix);
    // plotErrorPlots(plot_error_dir,prefix);
    
    // // }
    
    // save + plot total errors + summary statistics
    // if (total_err.size()>0) {
    //     char prefix[16];
    //     sprintf(prefix,"avg");
    //     saveErrorPlots(total_err,plot_error_dir,prefix);
    //     plotErrorPlots(plot_error_dir,prefix);
    //     saveStats(total_err,result_dir);
    // }
    auto [avgT, avgR] =  genz_icp::metrics::SeqError(poses_gt, poses);
    auto [AbsR, AbsT] =  genz_icp::metrics::AbsoluteTrajectoryError(poses_gt, poses);
    std::ofstream res_file(result_dir + "_results.txt");
    
    res_file << "The average Translation Error is :" << avgT << std::endl;
    res_file << "The average Rotation Error is :" << avgR << std::endl;
    res_file << "The absolute Translation Error is :" << AbsT << std::endl;
    res_file << "The absolute Rotation Error is :" << AbsR << std::endl;

    std::cout << "The average Translation Error is :" << avgT << std::endl;
    std::cout << "The average Rotation Error is :" << avgR << std::endl;
    std::cout << "The absolute Translation Error is :" << AbsT << std::endl;
    std::cout << "The absolute Rotation Error is :" << AbsR << std::endl;

    // success
	return true;
}

bool is_empty(std::string& filename) {
    std::ifstream file(filename);
    return file.tellg() == 0 && file.peek() == std::ifstream::traits_type::eof();
}

int main(int argc, char* argv[]){
    // First Arguments for outputs etc.
    // const std::string n_seq = "00";
    //for(int i = 2; i < 11; ++i){
    
    
    // Exporting poses into est_poses.txt
    if (std::filesystem::exists("./results/"+seq))
        std::cout << "Result " << seq << " directory already exists" << std::endl;
    else
        system(("mkdir -p ./results/" + seq).c_str());
    
    //Results directory with extension of sequence number for file
    const std::string result_dir = "results/" + seq;
    //Output estimated poses file
    std::string filename = result_dir + "_est_poses.txt";
    
    //Running the GENZ
    if(!std::filesystem::exists(filename) || is_empty(filename)){

        //Loading each frame aka each .bin file into the bindir
        std::vector<std::string> bindir;
        for (const auto& entry : std::filesystem::directory_iterator(dataset + "sequences/" + seq + "/velodyne/")) {
            if (entry.is_regular_file() && entry.path().extension() == ".bin") {
                std::cout << "Loading frames" << std::endl;
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
        
        // Sort out the binary data directory
        std::sort(bindir.begin(),bindir.end());
        
        // Inserting the points of a sequence and the GenZ Begins
        for(auto b: bindir){
            std::cout << b << std::endl;
            auto frame = kitti.loadframe(b);
            auto corrected_frame = [&frame]() {
                constexpr double VERTICAL_ANGLE_OFFSET = (0.205 * M_PI) / 180.0;
                KITTI::Vector3dVector frame_ = frame;
                std::transform(frame_.cbegin(), frame_.cend(), frame_.begin(), [&](const auto pt) {
                    const Eigen::Vector3d rotationVector = pt.cross(Eigen::Vector3d(0., 0., 1.));
                    return Eigen::AngleAxisd(VERTICAL_ANGLE_OFFSET, rotationVector.normalized()) * pt;
                    });
                return frame_;
            };
            
            // Load timestamps
            auto timestamps = kitti.loadTimestamps(dataset + seq + "/times.txt");
            // std::cout <<"Timestamps are loaded"<<std::endl;;
            
            auto [planar, non_planar] = odometry.RegisterFrame(corrected_frame(), timestamps);
        }

        //Writing output seq_est_poses.txt
        std::ofstream ofs(filename);
        if (!ofs.is_open()) {
            throw std::runtime_error("Failed to open file: " + filename);
        }
        
        for(const auto &pose : odometry.poses()) {
            Eigen::Matrix4d T = pose.matrix();
            // Apply calibration by T_calib * poses * inv(T_calib)
            auto calibrate_poses = [](std::string& filename){
                std::ifstream f(filename);
                // std::ifstream p(result_dir + "_est_poses.txt");
                auto calib = kitti.read_calib_data(dataset + seq + "/calib.txt");
                std::vector<float> Tr ;
                std::vector Tr = calib["Tr"];
                f.close();
                return 0;
            };
            // Write as 12 values (3x4 part of the matrix)
            auto calib_data=[&](){
                calib = kitti.read_calib(dataset + seq + "/poses.txt");
                Eigen:Map<const Eigen::Matrix<float,3,4,Eigen::RowMajor>> m34(calib["Tr"].data());
                Eigen::Matrix4d M = Eigen::Matrix4d::Identity(); 
                M.block<3,4>(0,0) = m34.cast<double>();
                Eigen::Matrix4d T_calib; 
T = M * T * M.inv();
                return T;
            };
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

    if(eval(result_dir, seq))std::cout << "Evaluation is completed" << std::endl;
    return 0;
}
//}
