//
// Created by phyorch on 27/12/18.
//

#include "Disparity.h"
#include "SimilarityMeasure.h"
#include "StereoGC.h"
#include "ImageUtils.h"
#include "RealEquipment.h"

#include <iostream>
#include <opencv2/core/types_c.h>
#include <matplotlib-cpp/matplotlibcpp.h>
#include <sl/Camera.hpp>
#include <libcmaes/cmaes.h>
#include <pcl/visualization/pcl_visualizer.h>

string left_path = "/home/phyorch/Data/ZEDData/RGBImage/";
string lidar_path = "/home/phyorch/Data/Pandar40Data/PCDDataTest/";
string lidar_image_output_path = "/home/phyorch/Data/depth_image.jpg";
cv::Mat left_image = cv::imread(left_path + "1545286685left.png");
string test_point_cloud_path = "/home/phyorch/Data/point_cloud_test.pcd";

int main(){
    cv::Mat rotation, translation, R_self, P_self, depth_map_camera, depth_map_camera_boader, depth_map_lidar_boader, depth_map_lidar;
    LiDARCalibParaKittiInverse lidar_calib_para_kitti_inverse;
    CameraPara camera_para;
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_part(new pcl::PointCloud<pcl::PointXYZ>);

    rotation = (cv::Mat_<float>(3,3) << -0.96917289, 0.18399496, 0.16385905, -0.14308745, 0.12108093, -0.98227561, -0.20057398, -0.97544104, -0.091020979);
    translation = (cv::Mat_<float>(3,1) << 0.14438252, -0.14530572, 0.12848426);
    Eigen::Matrix4f transformation;
    transformation << 1, 0, 0, 0.14438252, 0, 1, 0, -0.14530572, 0, 0, 1, 0.12848426, 0, 0, 0, 1;
    R_self = (cv::Mat_<float>(4,4) << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1);

    P_self = (cv::Mat_<float>(3,4) << 674.213928, 0, 668.909607, 0,
            0, 674.213928, 380.501831, 0,
            0, 0, 1, 0);

    lidar_calib_para_kitti_inverse = {
            Rotation:rotation,
            Translation:translation,
            R:R_self,
            P:P_self,
            imageSize:cv::Size(left_image.cols, left_image.rows)
    };

    camera_para = {
            fx:984.2439,
            fy:980.8141,
            cx:690,
            cy:233.1966,
            base:0.54,
            size:cv::Size(left_image.cols, left_image.rows)
    };

    LiDAR lidar = LiDAR(lidar_calib_para_kitti_inverse);

    lidar.projectData(lidar_path + "125.328831.pcd", depth_map_lidar, point_cloud_part, PCD, KITTI, XYZIT, C2L, CV);
//    ImageUtils::colorTransfer(depth_map_lidar, left_image);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_sparse;
//    PointCloudAlignment::getCameraSparsePointCloud(depth_map_lidar, )
//    cv::imwrite(lidar_image_output_path, left_image);
//    pcl::transformPointCloud

    pcl::PCDWriter writer;
    // Save DoN features
    writer.write<pcl::PointXYZ>(test_point_cloud_path, *point_cloud_part, false);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (lidar_path + "125.328831.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        exit(EXIT_FAILURE);
    }



    pcl::visualization::PCLVisualizer viewer ("test");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloud, 255, 255, 255);
    viewer.addPointCloud(point_cloud_part, source_cloud_color_handler, "original_cloud");

    vector<double> theta(6, 0);
    Transfer::mat2VectorSeperate(rotation, translation, theta);
    Transfer::vector2Eigen(theta, transformation);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud2 (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (*cloud, *transformed_cloud, transformation);
    pcl::transformPointCloud (*point_cloud_part, *transformed_cloud2, transformation);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (transformed_cloud, 230, 20, 20);
    viewer.addPointCloud(transformed_cloud2, transformed_cloud_color_handler, "transformed_cloud");




    while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce ();
    }


    return 0;
}