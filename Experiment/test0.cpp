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
//#include <matplotlib-cpp/matplotlibcpp.h>
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
string test1_path = "/home/" + user_name + "/Data/CalibrationFile/t5/image.jpg";
string test1_lidar_path = "/home/" + user_name + "/Data/CalibrationFile/t5/lidar.pcd";
string test2_path = "/home/" + user_name + "/Data/00.png";
string test1_output_path = "/home/" + user_name + "/Data/test1.png";
string test2_output_path = "/home/" + user_name + "/Data/test2.png";

string zed_rgb_image_path = "/home/" + user_name + "/Data/ZEDData/RGBImage/";
string zed_depth_image_path = "/home/" + user_name + "/Data/ZEDData/DepthImage/";
string pandar_cloud_path = "/home/" + user_name + "/Data/Pandar40Data/PCDDataTest";

cv::Mat left_image1 = cv::imread(left_path1 + "image" + image_name + "left.png");
cv::Mat left_image2 = cv::imread(left_path2 + "image" + image_name + "left.png");
cv::Mat test_image = cv::imread(test1_path);

cv::Mat cam_to_lid_rotation, cam_to_lid_translation, lid_to_cam_rotation, lid_to_cam_translation, R_self, P_self;

cv::Mat depth_map_camera1, depth_map_lidar1;

pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_part(new pcl::PointCloud<pcl::PointXYZ>);




int main(){



//----------------------------------------------------------------------------------------------------------------------
//Calibrationt3
    cam_to_lid_rotation = (cv::Mat_<float>(3,3) << 7.6923790630312239e-02, -6.1243003237103522e-02, 9.9515427195463724e-01,
                                                   -9.9703450577881314e-01, -6.9465961116236818e-03, 7.6641627650121447e-02,
                                                   2.2191713457525575e-03, -9.9809871223089064e-01, -6.1595745969799609e-02);

    cam_to_lid_translation = (cv::Mat_<float>(3,1) << 1.1860577017068863e-01, 1.1305765062570572e-01, -8.0872088670730591e-02);

    cv::transpose(cam_to_lid_rotation, lid_to_cam_rotation);

    lid_to_cam_translation = -(lid_to_cam_rotation * cam_to_lid_translation);

    cout << lid_to_cam_rotation << endl << lid_to_cam_translation;
//----------------------------------------------------------------------------------------------------------------------


    R_self = (cv::Mat_<float>(4,4) << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1);


    P_self = (cv::Mat_<float>(3,4) << 6.7448272705078125e+02, 0.0, 6.7321008300781250e+02, 0.0,
                                      0.0, 6.7448272705078125e+02, 3.8029727172851562e+02, 0.0,
                                      0.000000, 0.000000, 1.000000, 0.0);

    LiDARCalibParaKittiInverse lidar_calib_para_kitti_inverse = {
            Rotation:lid_to_cam_rotation,
            Translation:lid_to_cam_translation,
            R:R_self,
            P:P_self,
            imageSize:cv::Size(test_image.cols, test_image.rows)
    };




    LiDAR lidar = LiDAR(lidar_calib_para_kitti_inverse);
    lidar.projectData(test1_lidar_path, depth_map_lidar1, point_cloud_part, XYZI, CV);
    cv::Point size(2, 2);
    cv::Mat test;
    ImageUtils::neighborDyeing(depth_map_lidar1, size, test);
    ImageUtils::colorTransfer(test, test_image, 7);
    cv::imwrite(test1_output_path, test_image);

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
    //cv::Mat test = cv::imread(test1_path, CV_8UC1);
//    for(int i=0; i<test.rows; i++){
//        for(int j=0; j<test.cols; j++){
//            if(test.at<int>(i ,j)>0){
//                int t = test.at<int>(i ,j);
//                test.at<int>(i ,j) = t;
//                cout << t << "  ";
//            }
//        }
//    }

//    cv::Mat test = cv::imread(test1_path, CV_8UC1);
//    for (int i=0; i<test.rows; i++){
//        for(int j=0; j<test.cols; j++){
//            if(test.at<int>(i, j)==0){
//                int t = test.at<int>(i, j);
//                int a = 1;
//            }
//            if(test.at<int>(i, j)>0){
//                int t = test.at<int>(i, j);
//                int a = 1;
//            }
//        }
//    }
//    cout << test;
//    cv::Mat test2 = cv::imread(test2_path, CV_8UC1);
//    cv::Mat grad_x;
//    //cv::Laplacian(test, test, CV_8U, 3, 1, 0, cv::BORDER_DEFAULT);
//    //cv::Laplacian(test, test, CV_8U, 3, 1, 0, cv::BORDER_DEFAULT);
//    //Canny(test, test, 3, 9, 3);
//    //cv::Sobel(test2, grad_x, CV_16S, 1, 0, 3, 1, 1, cv::BORDER_DEFAULT);
//    //convertScaleAbs(grad_x, test2);
//
//    cv::Point size(2, 2);
//    cv::Mat depth_map_lidar_dyed;
//    ImageUtils::neighborDyeing(test, size, depth_map_lidar_dyed);
//    cout << depth_map_lidar_dyed;
//    cv::imwrite(test1_output_path, depth_map_lidar_dyed);
    //cv::imwrite(test2_output_path, test2*16);


    return 0;
}