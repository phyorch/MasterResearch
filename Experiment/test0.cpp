//
// Created by phyorch on 22/12/18.
//

#include "Disparity.h"
#include "SimilarityMeasure.h"
#include <iostream>
#include <opencv2/core/types_c.h>
#include "StereoGC.h"
#include "ImageUtils.h"
#include "RealEquipment.h"
#include <matplotlib-cpp/matplotlibcpp.h>
#include <sl/Camera.hpp>
#include <libcmaes/cmaes.h>

string test_path = "/home/phyorch/Data/ZEDData/DepthImage/";
string left_path1 = "/home/phyorch/Data/2019_01_03/2019_01_03_2/ZEDData/RGBImage/";
string lidar_path = "/home/phyorch/Data/Pandar40Data/PCDDataTest/";
string lidar_image_output_path = "/home/phyorch/Data/depth_image.jpg";
string depth_map_camera_path1 = "/home/phyorch/Data/2019_01_03/2019_01_03_2/ZEDData/DepthImage/";
string lidar_path1 = "/home/phyorch/Data/2019_01_03/2019_01_03_2/Pandar40Data/PCDDataKIT/";
string csv_test_path = "/home/phyorch/Data/lidar_distance_test.csv";
cv::Mat left_image1 = cv::imread(left_path1 + "image1546524802left.png");


cv::Mat cam_to_lid_rotation, cam_to_lid_translation, lid_to_cam_rotation, lid_to_cam_translation, R_self, P_self;

cv::Mat depth_map_camera1, depth_map_lidar1;

pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_part(new pcl::PointCloud<pcl::PointXYZ>);




int main(){



//    cam_to_lid_rotation = (cv::Mat_<float>(3,3) << -9.995395743477e-01, -2.591763623920e-02, -1.577705437008e-02,
//            1.563204680325e-02, 5.788061929611e-03, -9.998610590736e-01,
//            2.600535378657e-02, -9.996473250456e-01, -5.380251254700e-03);
//
//    cam_to_lid_translation = (cv::Mat_<float>(3,1) << -1.131729644136e-01, 9.325187939857e-02, -5.436065917896e-02);
//    cv::transpose(cam_to_lid_rotation, lid_to_cam_rotation);
//
//    lid_to_cam_translation = -(lid_to_cam_rotation * cam_to_lid_translation);
//
//    R_self = (cv::Mat_<float>(4,4) << 1, 0, 0, 0,
//            0, 1, 0, 0,
//            0, 0, 1, 0,
//            0, 0, 0, 1);
//
//    P_self = (cv::Mat_<float>(3,4) << 674.213928, 0, 668.909607, 0,
//            0, 674.213928, 380.501831, 0,
//            0, 0, 1, 0);
//
//
//    LiDARCalibParaKittiInverse lidar_calib_para_kitti_inverse = {
//            Rotation:lid_to_cam_rotation,
//            Translation:lid_to_cam_translation,
//            R:R_self,
//            P:P_self,
//            imageSize:cv::Size(left_image1.cols, left_image1.rows)
//    };
//
//
//
//    LiDAR lidar = LiDAR(lidar_calib_para_kitti_inverse);
//    lidar.projectData(lidar_path1 + "1369.706307.pcd", depth_map_lidar1, point_cloud_part, PCD, KITTI, XYZIT, C2L, CV);
//    cv::FileStorage fs1(depth_map_camera_path1 + "depth_map1546524802.xml", cv::FileStorage::READ);
//    fs1["CameraDepthMap"] >> depth_map_camera1;
//    depth_map_camera1 /= 1000;
//
//    cv::Mat camera_fragment, lidar_fragment;
//    ImageUtils::testMapRegion(depth_map_camera1, camera_fragment, 460, 520, 345, 405);
//    ImageUtils::testMapRegion(depth_map_lidar1, lidar_fragment, 460, 520, 345, 405);
//
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

    cv::FileStorage fs1(test_path + "depth_map1547215788.xml", cv::FileStorage::READ);
    fs1["CameraDepthMap"] >> depth_map_camera1;
    cv::Mat test;
    ImageUtils::creatMapRegion(depth_map_camera1, test, 374, 388, 710, 723);
    cout << test;

    //test for the clion distribution operation


    return 0;
}