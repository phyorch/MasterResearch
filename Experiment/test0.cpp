//
// Created by phyorch on 22/12/18.
//

#include "Sensor.h"
#include "Calibration.h"
#include <iostream>
#include <opencv2/core/types_c.h>
#include <opencv2/highgui/highgui.hpp>
#include "StereoGC.h"
#include "ImageUtils.h"
#include "RealEquipment.h"
//#include <matplotlib-cpp/matplotlibcpp.h>
#include <sl/Camera.hpp>
#include <libcmaes/cmaes.h>

string user_name = "phyorch";
string data_name = "2011_09_26_drive_0005_sync";
string image_name = "/image_02/data/0000000082.png";
string cloud_name = "/velodyne_points/data/0000000082.pcd";

string data_root = "/home/" + user_name + "/Data/";
string left_path1 = "/home/" + user_name + "/Data/" + data_name + "/ZEDData/RGBImage/";
string left_path2 = "/home/" + user_name + "/Data/" + data_name + "/ZEDData/RGBImage/";
string lidar_path1 = "/home/" + user_name + "/Data/" + data_name + "/Pandar40Data/PCDDataKIT/";
string lidar_path2 = "/home/" + user_name + "/Data/" + data_name + "/Pandar40Data/PCDDataKIT/";
string depth_map_camera_path1 = "/home/" + user_name + "/Data/" + data_name + "/ZEDData/DepthImage/";
string depth_map_camera_path2 = "/home/" + user_name + "/Data/" + data_name + "/ZEDData/DepthImage/";
string left_color_path = "/home/" + user_name + "/Data/left_color.png";
string lidar_output_path = "/home/" + user_name + "/Data/lidar.pcd";
string lidar_depth_output_path = "/home/" + user_name + "/Data/depth_map.jpg";
string lidar_image_output_path1 = "/home/" + user_name + "/Data/Result/OptimizationProcess/1result";
string lidar_image_output_path2 = "/home/" + user_name + "/Data/Result/OptimizationProcess/2result";
string depth_map_camera_boader_path = "/home/" + user_name + "/Data/camera_depth_boader.jpg";

string camera_csv_path = "/home/" + user_name + "/Data/HistCamera.csv";
string lidar_csv_path = "/home/" + user_name + "/Data/HistLiDAR.csv";
string test1_path = "/home/" + user_name + "/Data/CalibrationFile/t5/image.jpg";
string test1_lidar_path = "/home/" + user_name + "/Data/CalibrationFile/t5/lidar.pcd";
string test2_path = "/home/" + user_name + "/Data/00.png";
string test1_output_path = "/home/" + user_name + "/Data/test1.png";
string test2_output_path = "/home/" + user_name + "/Data/test2.png";

string zed_rgb_image_path = "/home/" + user_name + "/Data/ZEDData/RGBImage/";
string zed_depth_image_path = "/home/" + user_name + "/Data/ZEDData/DepthImage/";
string pandar_cloud_path = "/home/" + user_name + "/Data/Pandar40Data/PCDDataTest";

cv::Mat left_image = cv::imread(data_root + "image5.png");
cv::Mat left_image1 = cv::imread(left_path1 + "image" + image_name + "left.png");
cv::Mat left_image2 = cv::imread(left_path2 + "image" + image_name + "left.png");
cv::Mat test_image = cv::imread(test1_path);

cv::Mat cam_to_lid_rotation, cam_to_lid_translation, lid_to_cam_rotation, lid_to_cam_translation, R_self, P_self;
cv::Mat lid_to_cam_rotation2, lid_to_cam_translation2;
cv::Mat depth_map_camera1, depth_map_lidar1;

pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_part(new pcl::PointCloud<pcl::PointXYZ>);


void skew(cv::Mat &matOriginal, cv::Mat &matSkew) {
    CV_Assert(matOriginal.cols == 1 && matOriginal.rows == 3);
    matSkew = cv::Mat::zeros(3, 3, CV_32FC1);

    cout << "test" << endl << matOriginal << endl << matSkew << endl;
    matSkew.at<float>(0, 0) = 0.0;
    matSkew.at<float>(0, 1) = -matOriginal.at<float>(2, 0);
    matSkew.at<float>(0, 2) = matOriginal.at<float>(1, 0);

    matSkew.at<float>(1, 0) = matOriginal.at<float>(2, 0);
    matSkew.at<float>(1, 1) = 0.0;
    matSkew.at<float>(1, 2) = -matOriginal.at<float>(0, 0);

    matSkew.at<float>(2, 0) = -matOriginal.at<float>(1, 0);
    matSkew.at<float>(2, 1) = matOriginal.at<float>(0, 0);
    matSkew.at<float>(2, 2) = 0.0;
}

int main(){
    cv::Mat test1 = cv::Mat::zeros(3, 1, CV_32FC1);
    cv::Mat test2(3, 3, CV_32FC1);
    HandEyeCalibration::skew(test1, test2);
    cout << test1 << test2;
    return 0;
}