#include "genz_icp/pipeline/GenZICP.hpp"
#include "genz_icp/dataloader/Kitti.hpp"
// #include "devkit/cpp/evaluate_odometry.hpp"
// #include "devkit/cpp/matrix.h"


using Vector3dVector = std::vector<Eigen::Vector3d>;
// using Vector3dVectorTuple = std::tuple<Vector3dVector, double>;

// GenZ-ICP
genz_icp::pipeline::GenZICP odometry;
genz_icp::pipeline::GenZConfig config;

//Path to dataset and seq
std::string path_to_data = "/home/dezu/Documents/DIPLO/Dataset/sequences/03/velodyne/";
// std::vector<Matrix> poses;

// Initialization of the KITTI dataloader
KITTI kitti(path_to_data);
