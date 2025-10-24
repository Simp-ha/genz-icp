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

KITTI::Vector3dVector KITTI::loadframe(const std::string& binfile) {
    bool output = false;

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

    // Output file
    if(output){

        std::ofstream fo("output.txt");
        for (size_t i = 0; i<n_points; ++i ){
            const auto& p = points[i];
            fo << "P" << i << " = "<< p[0] << " " << p[1] << " " << p[2] <<" "<<std::endl;
        }
        fo.close();
    }
    return points;
} 

  //Calibrates input data according to the calib file
  // void calibrate_data(const std::string& calib_path){
    
  // }