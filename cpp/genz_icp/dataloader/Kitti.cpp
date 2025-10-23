#include "Kitti.hpp"

#include<iostream>
#include<Eigen/Core>
#include<fstream>


  
// Vector3dVector correct_frame = [&](const std::vector<Eigen::Vector3d> &frame) {
//     constexpr double VERTICAL_ANGLE_OFFSET = (0.205 * M_PI) / 180.0;
//     std::vector<Eigen::Vector3d> frame_ = frame;
//     std::transform(frame_.cbegin(), frame_.cend(), frame_.begin(), [&](const auto pt) {
//         const Eigen::Vector3d rotationVector = pt.cross(Eigen::Vector3d(0., 0., 1.));
//         return Eigen::AngleAxisd(VERTICAL_ANGLE_OFFSET, rotationVector.normalized()) * pt;
//     });
//     return frame_;
// };
// Returns the Frame and timestamps in tuple
// Initialize of data_path and constructing the KIITI

// Variables  
std::vector<double> KITTI::loadTimestamps(/*std::string file_name*/) {
  std::vector<double> timestamps;
  FILE *fp = fopen((data_path+"times.txt").c_str(),"r");
  if (!fp) return timestamps;
  while (!feof(fp)) {
    double S ;
    timestamps.push_back(fscanf(fp, "%lf", &S));
  }
  fclose(fp);
  return timestamps;
}

KITTI::Vector3dVector KITTI::loadPoses(const std::string& binfile) {
    Vector3dVector poses;
    FILE *fp = fopen((binfile).c_str(),"r");
    if (!fp)
    return poses;
    while (!feof(fp)) {
        double a, b, c, d, e, f, g, h, i, j, k, l;
        if (fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
           &a, &b, &c, &d, &e, &f, &g, &h, &i, &j, &k, &l
          )==12) {
            poses.push_back({a,b,c});
            poses.push_back({d,e,f});
            poses.push_back({g,h,i});
            poses.push_back({j,k,l});
        }
        for(auto a : poses) {
          std::cout << "Pose is: "<< a << std::endl;
        }
    }
    fclose(fp);
    return poses;

} 

  // auto concatenate(const auto& A, const auto& B, const int n){
  //     auto C =  A.clear(); 
  //     for(int i=0; i<A.size(); )
  //         for (int j = 0; j<n; ++j){
  //             C[i][j] = 
  //         }
  // }

  //Calibrates input data according to the calib file
  // void calibrate_data(const std::string& calib_path){
    
  // }

//struct PointXYZI {
//    float x, y, z, intensity;
//};
//
//std::vector<PointXYZI> loadVelodyneBin(const std::string& filename) {
//    std::vector<PointXYZI> points;
//
//    std::ifstream ifs(filename, std::ios::binary);
//    if (!ifs) {
//        std::cout << "Failed to open: " << filename << std::endl;
//        return points;
//    }
//
//    PointXYZI point;
//    while (ifs.read(reinterpret_cast<char*>(&point), sizeof(PointXYZI))) {
//        points.push_back(point);
//    }
//
//    return points;
//}
//
//void processBinData(const std::vector<float>& data, const std::string& filename) {
//    std::cout << "[INFO] Processing: " << filename << " | Data size: " << data.size() << std::endl;
//
//    // Example: print min/max (replace with your algorithm)
//    if (!data.empty()) {
//        float min = data[0], max = data[0];
//        for (float val : data) {
//            if (val < min) min = val;
//            if (val > max) max = val;
//        }
//        std::cout << "   Min: " << min << ", Max: " << max << "\n";
//    }
//}
//