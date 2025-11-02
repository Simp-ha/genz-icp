#include "genz_icp/pipeline/GenZICP.hpp"
#include "genz_icp/dataloader/Kitti.hpp"
#include "genz_icp/metrics/Metrics.hpp"

//Arguments
const std::string dataset = "/home/dezu/Documents/DIPLO/Dataset/";

bool o_file = false;

// GenZ-ICP
genz_icp::pipeline::GenZConfig config;
genz_icp::pipeline::GenZICP odometry = genz_icp::pipeline::GenZICP(config);

// Initialization of the KITTI dataloader
KITTI kitti(dataset);
