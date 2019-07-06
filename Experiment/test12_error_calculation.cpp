//
// Created by phyorch on 28/02/19.
//

#include "Sensor.h"
#include "Calibration.h"
#include "StereoGC.h"
#include "ImageUtils.h"
#include "RealEquipment.h"

#include <iostream>
#include <opencv2/core/types_c.h>
#include <sl/Camera.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>

string user_name = "phyorch";
float vox_volum = 4.0;

//string data_name = "2019_01_03/2019_01_03_3";
//string image_name = "/ZEDData/RGBImage/image1546524910right.png";
//string cloud_name = "/Pandar40Data/PCDDataKIT/1520.202800.pcd";

string data_name = "2011_09_26_drive_0005_sync";
string image_name = "/image_02/data/0000000132.png";
string cloud_name = "/velodyne_points/data/0000000132.pcd";
string depth_name = "/depth/depth1.png";

string data_root = "/home/" + user_name + "/Data/";
string left_path1 = "/home/" + user_name + "/Data/" + data_name + "/ZEDData/RGBImage/";
string left_path2 = "/home/" + user_name + "/Data/" + data_name + "/ZEDData/RGBImage/";
string lidar_path1 = "/home/" + user_name + "/Data/" + data_name + "/Pandar40Data/PCDDataKIT/";
string lidar_path2 = "/home/" + user_name + "/Data/" + data_name + "/Pandar40Data/PCDDataKIT/";
string depth_map_camera_path1 = "/home/" + user_name + "/Data/" + data_name + "/ZEDData/DepthImage/";
string depth_map_camera_path2 = "/home/" + user_name + "/Data/" + data_name + "/ZEDData/DepthImage/";
string depth_map_camera_prediction_path = "/home/" + user_name + "/Data/" + data_name + "/depth/depth1.png";
string left_color_path = "/home/" + user_name + "/Data/left_color.png";
string lidar_output_path = "/home/" + user_name + "/Data/lidar.pcd";
string lidar_depth_output_path = "/home/" + user_name + "/Data/depth_map.jpg";
string lidar_image_output_path1 = "/home/" + user_name + "/Data/Result/OptimizationProcess/1result";
string lidar_image_output_path2 = "/home/" + user_name + "/Data/Result/OptimizationProcess/2result";
string depth_map_camera_boader_path = "/home/" + user_name + "/Data/camera_depth_boader.jpg";

string camera_csv_path = "/home/" + user_name + "/Data/HistCamera.csv";
string lidar_csv_path = "/home/" + user_name + "/Data/HistLiDAR.csv";
string test1_path = "/home/" + user_name + "/Data/";
string test2_path = "/home/" + user_name + "/Data/";

string zed_rgb_image_path = "/home/" + user_name + "/Data/ZEDData/RGBImage/";
string zed_depth_image_path = "/home/" + user_name + "/Data/ZEDData/DepthImage/";
string pandar_cloud_path = "/home/" + user_name + "/Data/Pandar40Data/PCDDataTest";

//cv::Mat left_image1 = cv::imread(left_path1 + "image" + image_name + "left.png");
//cv::Mat left_image2 = cv::imread(left_path2 + "image" + image_name + "left.png");
cv::Mat depth_map_camera1, depth_map_camera_boader1, depth_map_camera2, depth_map_camera_boader2;
cv::Mat depth_map_lidar1, depth_map_lidar_boader1, depth_map_lidar2, depth_map_lidar_boader2;
Eigen::Matrix4f transformation;
int main(){
    cv::Mat left_image, lid_to_cam_rotation, lid_to_cam_translation, cam_to_lid_rotation, cam_to_lid_translation, R_self, P_self, depth_map_camera, depth_map_camera_boader, depth_map_lidar_boader, depth_map_lidar;
    left_image = cv::imread(data_root + data_name + image_name);
    LiDARCalibParaKitti lidar_calib_para_kitti;
    CameraPara camera_para;
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_lidar_part(new pcl::PointCloud<pcl::PointXYZ>);



//----------------------------------------------------------------------------------------------------------------------
////2019_01_15 intrinsic calibration
//    R_self = (cv::Mat_<float>(4,4) << 1, 0, 0, 0,
//            0, 1, 0, 0,
//            0, 0, 1, 0,
//            0, 0, 0, 1);
//
//    P_self = (cv::Mat_<float>(3,4) << 674.213928, 0, 668.909607, 0,
//            0, 674.213928, 380.501831, 0,
//            0, 0, 1, 0);
//
////2019_01_15 extrinsic calibration
//    lid_to_cam_rotation = (cv::Mat_<float>(3,3) << -0.99953955, 0.015632046, 0.026005354,
//    -0.025917636, 0.0057880618, -0.99964732,
//    -0.015777055, -0.99986106, -0.0053802514);
//
//    lid_to_cam_translation = (cv::Mat_<float>(3,1) << -0.11316491, -0.057814412, 0.091160908);
//
//    cv::Mat axisx, axisy, axisz;
//
//    int degreex = 2.5;
//    int degreey = -4;
//    int degreez = 0;
//    axisx = (cv::Mat_<float>(3,3) << 1, 0.0, 0.0,
//            0.0, cos(M_PI/180 * degreex), -sin(M_PI/180 * degreex),
//            0.0, sin(M_PI/180 * degreex), cos(M_PI/180 * degreex));
//
//    axisy = (cv::Mat_<float>(3,3) << cos(M_PI/180 * degreey), 0.0, -sin(M_PI/180 * degreey),
//            0.0, 1, 0.0,
//            sin(M_PI/180 * degreey), 0.0, cos(M_PI/180 * degreey));
//
//    axisz = (cv::Mat_<float>(3,3) << cos(M_PI/180 * degreez), -sin(M_PI/180 * degreez), 0.0,
//            sin(M_PI/180 * degreez), cos(M_PI/180 * degreez), 0.0,
//            0.0, 0.0, 1.0);
//    lid_to_cam_rotation = axisx * axisy * lid_to_cam_rotation;
//
//    Transfer::cv2EigenSeperate(lid_to_cam_rotation, lid_to_cam_translation, transformation);
//----------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------
////2019_01_15 intrinsic calibration
//    R_self = (cv::Mat_<float>(4,4) << 1, 0, 0, 0,
//            0, 1, 0, 0,
//            0, 0, 1, 0,
//            0, 0, 0, 1);
//
//    P_self = (cv::Mat_<float>(3,4) << 674.213928, 0, 668.909607, 0,
//            0, 674.213928, 380.501831, 0,
//            0, 0, 1, 0);
//
////2019_01_15 extrinsic calibration
//    lid_to_cam_rotation = (cv::Mat_<float>(3,3) << 1, 0, 0,
//    0, 0, -1,
//    0, 1, 0);
//    lid_to_cam_translation = (cv::Mat_<float>(3,1) << 0.383, 0, 0);
//    transformation << 1, 0, 0, 0.383, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1;
//
////    lid_to_cam_rotation = (cv::Mat_<float>(3,3) << 0.87365133, 0.11061831, -0.47381106,
////    -0.43071392, -0.27713022, -0.85888553,
////    -0.22631583, 0.95444351, -0.19447035);
////    lid_to_cam_translation = (cv::Mat_<float>(3,1) << -0.69790131, -0.078251913, 0.27923283);
////    transformation << 0.87365133, 0.11061831, -0.47381106, -0.69790131, -0.43071392, -0.27713022, -0.85888553, -0.078251913, -0.22631583, 0.95444351, -0.19447035, 0.27923283, 0, 0, 0, 1;
//----------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------
////2011_09_26_drive_0002_sync intrinsic calibration
//    R_self = (cv::Mat_<float>(4,4) << 9.998817e-01, 1.511453e-02, -2.841595e-03, 0,
//            -1.511724e-02, 9.998853e-01, -9.338510e-04, 0,
//            2.827154e-03, 9.766976e-04, 9.999955e-01, 0,
//            0, 0, 0, 1);
//
//    P_self = (cv::Mat_<float>(3,4) << 7.215377e+02, 0.000000e+00, 6.095593e+02, 0,//4.485728e+01
//            0.000000e+00, 7.215377e+02, 1.728540e+02, 0,//2.163791e-01
//            0.000000e+00, 0.000000e+00, 1.000000e+00, 0);//2.745884e-03
//
////2011_09_26_drive_0002_sync extrinsic calibration
//    lid_to_cam_rotation = (cv::Mat_<float>(3,3) << 7.533745e-04, -9.999714e-01, -6.166020e-04,
//            1.480249e-02, 7.280733e-04, -9.998902e-01,
//            9.998621e-01, 7.523790e-03, 1.480755e-02);
//    lid_to_cam_translation = (cv::Mat_<float>(3,1) << -4.069766e-03, -7.631618e-02, -2.717806e-01);
//    cv::Mat cam0_to_cam2_rotation, cam0_to_cam2_translation;
//    cam0_to_cam2_rotation = (cv::Mat_<float>(3,3) << 9.998817e-01, 1.511453e-02, -2.841595e-03,
//            -1.511724e-02, 9.998853e-01, -9.338510e-04,
//            2.827154e-03, 9.766976e-04, 9.999955e-01);
//    cam0_to_cam2_translation = (cv::Mat_<float>(3,1) << 5.956621e-02, 2.900141e-04, 2.577209e-03);
//    lid_to_cam_rotation = cam0_to_cam2_rotation * lid_to_cam_rotation;
//    lid_to_cam_translation = cam0_to_cam2_rotation * lid_to_cam_translation + cam0_to_cam2_translation;
//
//    Transfer::cv2EigenSeperate(lid_to_cam_rotation, lid_to_cam_translation, transformation);
//    cout << lid_to_cam_rotation << endl << lid_to_cam_translation << endl << transformation;
//----------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------
////2011_09_26_drive_0048_sync intrinsic calibration
    R_self = (cv::Mat_<float>(4,4) << 9.998817e-01, 1.511453e-02, -2.841595e-03, 0,
            -1.511724e-02, 9.998853e-01, -9.338510e-04, 0,
            2.827154e-03, 9.766976e-04, 9.999955e-01, 0,
            0, 0, 0, 1);

    P_self = (cv::Mat_<float>(3,4) << 7.215377e+02, 0.000000e+00, 6.095593e+02, 0,//4.485728e+01
            0.000000e+00, 7.215377e+02, 1.728540e+02, 0,//2.163791e-01
            0.000000e+00, 0.000000e+00, 1.000000e+00, 0);//2.745884e-03

//2011_09_26_drive_0048_sync extrinsic calibration
//    lid_to_cam_rotation = (cv::Mat_<float>(3,3) << 7.533745e-04, -9.999714e-01, -6.166020e-04,
//            1.480249e-02, 7.280733e-04, -9.998902e-01,
//            9.998621e-01, 7.523790e-03, 1.480755e-02);
//    lid_to_cam_translation = (cv::Mat_<float>(3,1) << -4.069766e-03, -7.631618e-02, -2.717806e-01);

    lid_to_cam_rotation = (cv::Mat_<float>(3,3) << 0.024568876, -0.99922067, 0.03089357,
    -0.040565841, -0.031873927, -0.99866837,
    0.99887478, 0.023282936, -0.041317333);
    lid_to_cam_translation = (cv::Mat_<float>(3,1) << 0.01868224, 0.0903562, 0.4178428);

//    lid_to_cam_rotation = (cv::Mat_<float>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
//    lid_to_cam_translation = (cv::Mat_<float>(3,1) << -0.28338799, -0.82151896, 0.53261143);

    float degreex = 0;
    float degreey = 0;
    float degreez = 0;
    cv::Mat axisx, axisy, axisz;
    axisx = (cv::Mat_<float>(3,3) << 1, 0.0, 0.0,
            0.0, cos(M_PI/180 * degreex), -sin(M_PI/180 * degreex),
            0.0, sin(M_PI/180 * degreex), cos(M_PI/180 * degreex));

    axisy = (cv::Mat_<float>(3,3) << cos(M_PI/180 * degreey), 0.0, sin(M_PI/180 * degreey),
            0.0, 1, 0.0,
            -sin(M_PI/180 * degreey), 0.0, cos(M_PI/180 * degreey));

    axisz = (cv::Mat_<float>(3,3) << cos(M_PI/180 * degreez), -sin(M_PI/180 * degreez), 0.0,
            sin(M_PI/180 * degreez), cos(M_PI/180 * degreez), 0.0,
            0.0, 0.0, 1.0);
    lid_to_cam_rotation = axisz * axisy * axisx * lid_to_cam_rotation;
//    cv::Mat cam0_to_cam2_rotation, cam0_to_cam2_translation;
//    cam0_to_cam2_rotation = (cv::Mat_<float>(3,3) << 9.999758e-01, -5.267463e-03, -4.552439e-03,
//                                                     5.251945e-03, 9.999804e-01, -3.413835e-03,
//                                                     4.570332e-03, 3.389843e-03, 9.999838e-01);
//    cam0_to_cam2_translation = (cv::Mat_<float>(3,1) << 5.956621e-02, 2.900141e-04, 2.577209e-03);
//    lid_to_cam_rotation = cam0_to_cam2_rotation * lid_to_cam_rotation;
//    lid_to_cam_translation = cam0_to_cam2_rotation * lid_to_cam_translation + cam0_to_cam2_translation;

    Transfer::cv2EigenSeperate(lid_to_cam_rotation, lid_to_cam_translation, transformation);
    cout << lid_to_cam_rotation << endl << lid_to_cam_translation << endl << transformation;
//----------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------
//lidar class initialization
    lidar_calib_para_kitti = {
            Rotation:lid_to_cam_rotation,
            Translation:lid_to_cam_translation,
            R:R_self,
            P:P_self,
            imageSize:cv::Size(left_image.cols, left_image.rows)
    };
    LiDAR lidar = LiDAR(lidar_calib_para_kitti);
//----------------------------------------------------------------------------------------------------------------------




//    lid_to_cam_rotation = (cv::Mat_<float>(3,3) << 7.533745e-04, -9.999714e-01, -6.166020e-04,
//            1.480249e-02, 7.280733e-04, -9.998902e-01,
//            9.998621e-01, 7.523790e-03, 1.480755e-02);
//    lid_to_cam_translation = (cv::Mat_<float>(3,1) << -4.069766e-03, -7.631618e-02, -2.717806e-01);

    cv::Mat lid_to_cam_rotation_truth;
    cv::Mat lid_to_cam_translation_truth;
    lid_to_cam_rotation_truth = (cv::Mat_<float>(3,3) << 7.533745e-03, -9.999714e-01, -6.166020e-04,
            1.480249e-02, 7.280733e-04, -9.998902e-01,
            9.998621e-01, 7.523790e-03, 1.480755e-02);
    lid_to_cam_translation_truth = (cv::Mat_<float>(3,1) << -4.069766e-03, -7.631618e-02, -2.717806e-01);


    cv::transpose(lid_to_cam_rotation, lid_to_cam_rotation);

    cv::Mat error = lid_to_cam_rotation * lid_to_cam_rotation_truth;

    vector<float> euler_angle;
    float sy = sqrt(error.at<float>(0, 0) * error.at<float>(0, 0) + error.at<float>(1, 0) * error.at<float>(1, 0));
    float e1 = atan2(error.at<float>(2, 1), error.at<float>(2, 2)) * 180 / M_PI;
    float e2 = atan2(-error.at<float>(2, 0), sy) * 180 / M_PI;
    float e3 = atan2(error.at<float>(1, 0), error.at<float>(0, 0)) * 180 / M_PI;

    float error_rotation = abs(e1) + abs(e2) + abs(e3);
    float e4 = abs(lid_to_cam_translation.at<float>(0, 0) - lid_to_cam_translation_truth.at<float>(0, 0));
    float e5 = abs(lid_to_cam_translation.at<float>(1, 0) - lid_to_cam_translation_truth.at<float>(1, 0));
    float e6 = abs(lid_to_cam_translation.at<float>(2, 0) - lid_to_cam_translation_truth.at<float>(2, 0));

    cout << endl << "rotation error:   " << e1 << "  " << e2 << "  " << e3 << "  " << e4 << "  " << e5 << "  " << e6;


//    float sy2 = sqrt(lid_to_cam_rotation.at<float>(0, 0) * lid_to_cam_rotation.at<float>(0, 0) + lid_to_cam_rotation.at<float>(1, 0) * lid_to_cam_rotation.at<float>(1, 0));
//    float e12 = atan2(lid_to_cam_rotation.at<float>(2, 1), lid_to_cam_rotation.at<float>(2, 2)) * 180 / M_PI;
//    float e22 = atan2(-lid_to_cam_rotation.at<float>(2, 0), sy) * 180 / M_PI;
//    float e32 = atan2(lid_to_cam_rotation.at<float>(1, 0), lid_to_cam_rotation.at<float>(0, 0)) * 180 / M_PI;


    return 0;
}