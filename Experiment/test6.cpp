//
// Created by phyorch on 26/12/18.
//

#include "Disparity.h"
#include "SimilarityMeasure.h"
#include <iostream>
#include <opencv2/core/types_c.h>
#include "StereoGC.h"
#include "ImageUtils.h"
#include "RealEquipment.h"
#include <sl/Camera.hpp>
#include <libcmaes/cmaes.h>


using namespace std;



string left_path1 = "/home/phyorch/Data/2019_01_03/2019_01_03_2/ZEDData/RGBImage/";
string left_path2 = "/home/phyorch/Data/2019_01_03/2019_01_03_4/ZEDData/RGBImage/";
string lidar_path1 = "/home/phyorch/Data/2019_01_03/2019_01_03_2/Pandar40Data/PCDDataKIT/";
string lidar_path2 = "/home/phyorch/Data/2019_01_03/2019_01_03_4/Pandar40Data/PCDDataKIT/";
string depth_map_camera_path1 = "/home/phyorch/Data/2019_01_03/2019_01_03_2/ZEDData/DepthImage/";
string depth_map_camera_path2 = "/home/phyorch/Data/2019_01_03/2019_01_03_4/ZEDData/DepthImage/";
string left_color_path = "/home/phyorch/Data/left_color.png";
string lidar_output_path = "/home/phyorch/Data/lidar.pcd";
string lidar_depth_output_path = "/home/phyorch/Data/depth_map.jpg";
string lidar_image_output_path1 = "/home/phyorch/Data/Result/OptimizationProcess/1result";
string lidar_image_output_path2 = "/home/phyorch/Data/Result/OptimizationProcess/2result";
string depth_map_camera_boader_path = "/home/phyorch/Data/camera_depth_boader.jpg";

string camera_csv_path = "/home/phyorch/Data/HistCamera.csv";
string lidar_csv_path = "/home/phyorch/Data/HistLiDAR.csv";
string test1_path = "/home/phyorch/Data/test.jpg";
string test2_path = "/home/phyorch/Data/test2.jpg";

string zed_rgb_image_path = "/home/phyorch/Data/ZEDData/RGBImage/";
string zed_depth_image_path = "/home/phyorch/Data/ZEDData/DepthImage/";
string pandar_cloud_path = "/home/phyorch/Data/Pandar40Data/PCDDataTest";

cv::Mat left_image1 = cv::imread(left_path1 + "image1546524802left.png");
cv::Mat left_image2 = cv::imread(left_path2 + "image1546525082left.png");
cv::Mat depth_map_camera1, depth_map_camera_boader1, depth_map_camera2, depth_map_camera_boader2;
cv::Mat depth_map_lidar1, depth_map_lidar_boader1, depth_map_lidar2, depth_map_lidar_boader2;
pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_part(new pcl::PointCloud<pcl::PointXYZ>);

//Intrinsic parameters for self data
cv::Mat R_self;

cv::Mat P_self;



//Extrinsic parameters variable setting
cv::Mat cam_to_lid_rotation;

cv::Mat cam_to_lid_translation;

cv::Mat lid_to_cam_rotation;

cv::Mat lid_to_cam_translation;

// kitti model
//    LiDARCalibParaKitti lidar_calib_para_kitti = {
//            T:velo_to_cam,
//            R:R_rect_00_extended,
//            P:P_rect_00,
//            imageSize:cv::Size(left_image.cols, left_image.rows)
//    };

LiDARCalibParaKittiInverse lidar_calib_para_kitti_inverse;

CameraPara camera_para;

LiDAR lidar = LiDAR(lidar_calib_para_kitti_inverse);

vector<vector<int>> region_point_set1, region_point_set2;

vector<cv::Mat> diagonal_points_set1, diagonal_points_set2;

int cnt = 0;



libcmaes::FitFunc KL_divergence = [](const double *x, const double N){


    cv::Mat rotationUpdate, translationUpdate;
    Transfer::array2MatSeperate(x, rotationUpdate, translationUpdate);
    lidar.updateParameters(rotationUpdate, translationUpdate);

    left_image1 = cv::imread(left_path1 + "image1546524802left.png");
    left_image2 = cv::imread(left_path2 + "image1546525082left.png");
    lidar.projectData(lidar_path1 + "1369.706307.pcd", depth_map_lidar1, point_cloud_part, PCD, KITTI, XYZIT, C2L, CV);
    lidar.projectData(lidar_path2 + "1677.206636.pcd", depth_map_lidar2, point_cloud_part, PCD, KITTI, XYZIT, C2L, CV);
    cv::FileStorage fs1(depth_map_camera_path1 + "depth_map1546524802.xml", cv::FileStorage::READ);
    fs1["CameraDepthMap"] >> depth_map_camera1;
    cv::FileStorage fs2(depth_map_camera_path2 + "depth_map1546525082.xml", cv::FileStorage::READ);
    fs2["CameraDepthMap"] >> depth_map_camera2;

    // LiDAR invalid region
    ImageUtils::creatMapRegion(depth_map_camera1, depth_map_camera_boader1, depth_map_camera1.rows/3, depth_map_camera1.rows/3 * 2, 50, depth_map_camera1.cols);
    ImageUtils::creatMapRegion(depth_map_lidar1, depth_map_lidar_boader1, depth_map_camera1.rows/3, depth_map_camera1.rows/3 * 2, 50, depth_map_camera1.cols);
    ImageUtils::creatMapRegion(depth_map_camera2, depth_map_camera_boader2, depth_map_camera2.rows/3, depth_map_camera2.rows/3 * 2, 50, depth_map_camera2.cols);
    ImageUtils::creatMapRegion(depth_map_lidar2, depth_map_lidar_boader2, depth_map_camera2.rows/3, depth_map_camera2.rows/3 * 2, 50, depth_map_camera2.cols);

    cv::imwrite(depth_map_camera_boader_path, depth_map_camera_boader1);
    //Unit in meter
    depth_map_camera_boader1 = depth_map_camera_boader1 / 1000;
    depth_map_camera_boader2 = depth_map_camera_boader2 / 1000;

    depth_map_camera1 = depth_map_camera1 / 1000;
    depth_map_camera2 = depth_map_camera2 / 1000;

//    cv::Size region_size, map_size;
//    region_size.width = 80;
//    region_size.height = 80;
//    if(cnt==0){
//
//        map_size.width = depth_map_camera_boader1.cols;
//        map_size.height = depth_map_camera_boader1.rows;
//        HistogramGeneration::subRegionGeneration(10, region_size, map_size, region_point_set1);
//        HistogramGeneration::subRegionGeneration(10, region_size, map_size, region_point_set2);
//    }
//    double distance1 = HistogramGeneration::mapKLDivergence(depth_map_camera_boader1, depth_map_lidar_boader1, region_point_set1, region_size);
//    double distance2 = HistogramGeneration::mapKLDivergence(depth_map_camera_boader2, depth_map_lidar_boader2, region_point_set2, region_size);
//    double kl_distance = distance1 + distance2;


    float p2p_distance1 = HistogramMeasure::point2PointDistanceTotal(depth_map_camera1, depth_map_lidar1, diagonal_points_set1);
    float p2p_distance2 = HistogramMeasure::point2PointDistanceTotal(depth_map_camera2, depth_map_lidar2, diagonal_points_set2);
    float p2p_distance = p2p_distance1 + p2p_distance2;
    cnt++;
    if(cnt%3==0){
        cout <<"iteration " << cnt << endl << "p2p distance is " << endl << p2p_distance << endl;
        cout << "calibration result is " << endl << rotationUpdate << endl << translationUpdate << endl;
        ImageUtils::colorTransfer(depth_map_lidar1, left_image1);
        ImageUtils::colorTransfer(depth_map_lidar2, left_image2);
        ImageUtils::drawRectSet(left_image1, diagonal_points_set1);
        ImageUtils::drawRectSet(left_image2, diagonal_points_set2);
        cv::imwrite(lidar_image_output_path1 + to_string(cnt/3) + "___" + to_string(p2p_distance) + ".jpg", left_image1);
        cv::imwrite(lidar_image_output_path2 + to_string(cnt/3) + "___" + to_string(p2p_distance) + ".jpg", left_image2);
    }
    return p2p_distance;
};


int main(){

//----------------------------------------------------------------------------------------------------------------------
//Intrinsic parameters
    R_self = (cv::Mat_<float>(4,4) << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1);

    P_self = (cv::Mat_<float>(3,4) << 674.213928, 0, 668.909607, 0,
            0, 674.213928, 380.501831, 0,
            0, 0, 1, 0);
//----------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------
//Calibration2
//
//    cam_to_velo_rotation = (cv::Mat_<float>(3,3) << -9.816086341230e-01, -1.508804030660e-01, 1.169597938860e-01,
//            -1.152563726088e-01, -2.001656302531e-02, -9.931340824771e-01,
//            1.521856037484e-01, -9.883493517857e-01, 2.258503135975e-03);
//    cam_to_velo_translation = (cv::Mat_<float>(3,1) << 2.757819023341e-01, 8.406759160258e-02, -9.936401615109e-02);
//    cv::transpose(cam_to_velo_rotation, velo_to_cam_rotation);
//    velo_to_cam_translation = -(velo_to_cam_rotation * cam_to_velo_translation);
//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------
//Calibration3
    cam_to_lid_rotation = (cv::Mat_<float>(3,3) << -9.995395743477e-01, -2.591763623920e-02, -1.577705437008e-02,
            1.563204680325e-02, 5.788061929611e-03, -9.998610590736e-01,
            2.600535378657e-02, -9.996473250456e-01, -5.380251254700e-03);

    cam_to_lid_translation = (cv::Mat_<float>(3,1) << -1.131729644136e-01, 9.325187939857e-02, -5.436065917896e-02);

    cv::transpose(cam_to_lid_rotation, lid_to_cam_rotation);

    lid_to_cam_translation = -(lid_to_cam_rotation * cam_to_lid_translation);
//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------
//Test1
//    lid_to_cam_rotation = (cv::Mat_<float>(3,3) << -0.96186036, -0.030997463, -0.27177888,
//    0.25852886, 0.22160418, -0.94024169,
//    0.089372441, -0.97464389, -0.20513855);
//
//    lid_to_cam_translation = (cv::Mat_<float>(3,1) << -0.33054176, 0.063405782, 0.3143903);
//----------------------------------------------------------------------------------------------------------------------
//Class initilization
    lidar_calib_para_kitti_inverse = {
            Rotation:lid_to_cam_rotation,
            Translation:lid_to_cam_translation,
            R:R_self,
            P:P_self,
            imageSize:cv::Size(left_image1.cols, left_image1.rows)
    };

    camera_para = {
            fx:984.2439,
            fy:980.8141,
            cx:690,
            cy:233.1966,
            base:0.54,
            size:cv::Size(left_image1.cols, left_image1.rows)
    };

    lidar = LiDAR(lidar_calib_para_kitti_inverse);

//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------
//Fixed comparision region
    cv::Mat region11, region12, region13, region21, region22, region23;
    region11 = (cv::Mat_<int>(2,2) << 1100, 400, 1250, 450);
    region12 = (cv::Mat_<int>(2,2) << 420, 430, 760, 460);
    region13 = (cv::Mat_<int>(2,2) << 350, 380, 460, 410);
    region21 = (cv::Mat_<int>(2,2) << 670, 390, 900, 440);
    region22 = (cv::Mat_<int>(2,2) << 280, 450, 500, 500);
    region23 = (cv::Mat_<int>(2,2) << 950, 430, 1200, 500);
    diagonal_points_set1.push_back(region11);
    diagonal_points_set1.push_back(region12);
    diagonal_points_set1.push_back(region13);
    diagonal_points_set2.push_back(region21);
    diagonal_points_set2.push_back(region22);
    diagonal_points_set2.push_back(region23);
//----------------------------------------------------------------------------------------------------------------------
//CMA-ES setting
    cnt = 0;
    int dim = 6;
    double sigma = 0.06;
    vector<double> test_vec(dim, 0);
    Transfer::mat2VectorSeperate(lid_to_cam_rotation, lid_to_cam_translation, test_vec);
    libcmaes::CMAParameters<> cma_para(test_vec, sigma);
    cma_para.set_algo(aCMAES);
    libcmaes::CMASolutions cmasols = libcmaes::cmaes<>(KL_divergence, cma_para);
    cout << "best solution: " << cmasols << endl;
    cout << "optimization took " << cmasols.elapsed_time() / 1000.0 << " seconds\n";
    return cmasols.run_status();



    /*-------------------Histogram generation---------------------*/



    //cout << "distance1:  " << distance_hist1 << endl << "distance2:  " <<distance_hist2 << endl;

    //lidar.updateParameters(rotation, translation);

    return 0;
}