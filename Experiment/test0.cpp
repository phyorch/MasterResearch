//
// Created by phyorch on 22/12/18.
//

#include "Disparity.h"
#include "SimilarityMeasure.h"
#include <iostream>
#include <opencv2/core/types_c.h>
#include <opencv2/highgui/highgui.hpp>
#include "StereoGC.h"
#include "ImageUtils.h"
#include "RealEquipment.h"
#include <matplotlib-cpp/matplotlibcpp.h>
#include <sl/Camera.hpp>
#include <libcmaes/cmaes.h>

string user_name = "phyorch";
string data_name = "2019_01_15/2019_01_15_1";
string image_name = "1547540975";
string cloud_name = "137.508265";


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
string test1_path = "/home/" + user_name + "/Data/0.png";
string test2_path = "/home/" + user_name + "/Data/00.png";
string test1_output_path = "/home/" + user_name + "/Data/test1.png";
string test2_output_path = "/home/" + user_name + "/Data/test2.png";

string zed_rgb_image_path = "/home/" + user_name + "/Data/ZEDData/RGBImage/";
string zed_depth_image_path = "/home/" + user_name + "/Data/ZEDData/DepthImage/";
string pandar_cloud_path = "/home/" + user_name + "/Data/Pandar40Data/PCDDataTest";

cv::Mat left_image1 = cv::imread(left_path1 + "image" + image_name + "left.png");
cv::Mat left_image2 = cv::imread(left_path2 + "image" + image_name + "left.png");

cv::Mat cam_to_lid_rotation, cam_to_lid_translation, lid_to_cam_rotation, lid_to_cam_translation, R_self, P_self;

cv::Mat depth_map_camera1, depth_map_lidar1;

pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_part(new pcl::PointCloud<pcl::PointXYZ>);




int main(){



//    lid_to_cam_rotation = (cv::Mat_<float>(3,3) << 1, 0, 0,
//    0, 0, -1,
//    0, 1, 0);
//
//    lid_to_cam_translation = (cv::Mat_<float>(3,1) << 0.383, 0, 0);
    lid_to_cam_rotation = (cv::Mat_<float>(3,3) << 1.2, 0.6, 0,
            0.12, 0.12, -1,
            0.16, 1, 0.3);

    lid_to_cam_translation = (cv::Mat_<float>(3,1) << 0.2, 0, 0);

    R_self = (cv::Mat_<float>(4,4) << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1);

    P_self = (cv::Mat_<float>(3,4) << 674.213928, 0, 668.909607, 0,
            0, 674.213928, 380.501831, 0,
            0, 0, 1, 0);


    LiDARCalibParaKittiInverse lidar_calib_para_kitti_inverse = {
            Rotation:lid_to_cam_rotation,
            Translation:lid_to_cam_translation,
            R:R_self,
            P:P_self,
            imageSize:cv::Size(left_image1.cols, left_image1.rows)
    };



//    LiDAR lidar = LiDAR(lidar_calib_para_kitti_inverse);
//    lidar.projectData(lidar_path1 + cloud_name + ".pcd", depth_map_lidar1, point_cloud_part, PCD, KITTI, XYZIT, C2L, CV);
//    cv::FileStorage fs1(depth_map_camera_path1 + "depth_map" + image_name + ".xml", cv::FileStorage::READ);
//    fs1["CameraDepthMap"] >> depth_map_camera1;
//    depth_map_camera1 /= 1000;
//
//    cv::Mat camera_fragment, lidar_fragment;
//    cv::Mat region;
//    region = (cv::Mat_<int>(2,2) << 452, 398, 557, 503);
//    ImageUtils::creatMapRegion(depth_map_camera1, camera_fragment, region);
//    ImageUtils::creatMapRegion(depth_map_lidar1, lidar_fragment, region);
//    HistogramGeneration::histogramWrite(camera_csv_path, lidar_csv_path, camera_fragment, lidar_fragment);


//    for(int i=0; i<camera_fragment.rows; i++){
//        for(int j=0; j<camera_fragment.cols; j++){
//            if(camera_fragment.at<float>(i ,j)>0){
//                cout << camera_fragment.at<float>(i ,j) << "  ";
//            }
//        }
//    }
//
//    cout << endl << endl;
//
//    for(int i=0; i<lidar_fragment.rows; i++){
//        for(int j=0; j<lidar_fragment.cols; j++){
//            if(lidar_fragment.at<float>(i ,j)>0){
//                cout << lidar_fragment.at<float>(i ,j) << "  ";
//            }
//        }
//    }

//    cv::FileStorage fs1(test_path + "depth_map1547215788.xml", cv::FileStorage::READ);
//    fs1["CameraDepthMap"] >> depth_map_camera1;
//    cv::Mat test;
//    ImageUtils::creatMapRegion(depth_map_camera1, test, 374, 388, 710, 723);
//    cout << test;

    //cv::Mat test = cv::imread(left_path1 + "image" + image_name + "left.png", CV_8UC1);
    cv::Mat test = cv::imread(test1_path, CV_8UC1);
    cv::Mat test2 = cv::imread(test2_path, CV_8UC1);
    cv::Mat grad_x;
    //cv::Laplacian(test, test, CV_8U, 3, 1, 0, cv::BORDER_DEFAULT);
    cv::Laplacian(test, test, CV_8U, 3, 1, 0, cv::BORDER_DEFAULT);
    //Canny(test, test, 3, 9, 3);
    cv::Sobel(test2, grad_x, CV_16S, 1, 0, 3, 1, 1, cv::BORDER_DEFAULT);
    convertScaleAbs(grad_x, test2);
    cv::imwrite(test1_output_path, test);
    cv::imwrite(test2_output_path, test2*16);


    return 0;
}