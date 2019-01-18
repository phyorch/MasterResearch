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

string user_name = "phyorch";
string data_name = "2019_01_15/2019_01_15_1";
string image_name = "1547540975";
string cloud_name = "137.508265";
float vox_volum = 0.15;


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
string test1_path = "/home/" + user_name + "/Data/";
string test2_path = "/home/" + user_name + "/Data/";

string zed_rgb_image_path = "/home/" + user_name + "/Data/ZEDData/RGBImage/";
string zed_depth_image_path = "/home/" + user_name + "/Data/ZEDData/DepthImage/";
string pandar_cloud_path = "/home/" + user_name + "/Data/Pandar40Data/PCDDataTest";

cv::Mat left_image1 = cv::imread(left_path1 + "image" + image_name + "left.png");
cv::Mat left_image2 = cv::imread(left_path2 + "image" + image_name + "left.png");
cv::Mat depth_map_camera1, depth_map_camera_boader1, depth_map_camera2, depth_map_camera_boader2;
cv::Mat depth_map_lidar1, depth_map_lidar_boader1, depth_map_lidar2, depth_map_lidar_boader2;
Eigen::Matrix4f transformation;
int main(){
    cv::Mat lid_to_cam_rotation, lid_to_cam_translation, R_self, P_self, depth_map_camera, depth_map_camera_boader, depth_map_lidar_boader, depth_map_lidar;
    LiDARCalibParaKittiInverse lidar_calib_para_kitti_inverse;
    CameraPara camera_para;
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_lidar_part(new pcl::PointCloud<pcl::PointXYZ>);

//    rotation = (cv::Mat_<float>(3,3) << -0.96917289, 0.18399496, 0.16385905, -0.14308745, 0.12108093, -0.98227561, -0.20057398, -0.97544104, -0.091020979);
//    translation = (cv::Mat_<float>(3,1) << 0.14438252, -0.14530572, 0.12848426);
    lid_to_cam_rotation = (cv::Mat_<float>(3,3) << 0.87365133, 0.11061831, -0.47381106,
    -0.43071392, -0.27713022, -0.85888553,
    -0.22631583, 0.95444351, -0.19447035);
    lid_to_cam_translation = (cv::Mat_<float>(3,1) << -0.69790131, -0.078251913, 0.27923283);
    transformation << 0.87365133, 0.11061831, -0.47381106, -0.69790131, -0.43071392, -0.27713022, -0.85888553, -0.078251913, -0.22631583, 0.95444351, -0.19447035, 0.27923283, 0, 0, 0, 1;
//    lid_to_cam_rotation = (cv::Mat_<float>(3,3) << 1, 0, 0,
//    0, 0, -1,
//    0, 1, 0);
//    lid_to_cam_translation = (cv::Mat_<float>(3,1) << 0.383, 0, 0);
//    transformation << 1, 0, 0, 0.383, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1;

    R_self = (cv::Mat_<float>(4,4) << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1);

    P_self = (cv::Mat_<float>(3,4) << 674.213928, 0, 668.909607, 0,
            0, 674.213928, 380.501831, 0,
            0, 0, 1, 0);

    lidar_calib_para_kitti_inverse = {
            Rotation:lid_to_cam_rotation,
            Translation:lid_to_cam_translation,
            R:R_self,
            P:P_self,
            imageSize:cv::Size(left_image1.cols, left_image1.rows)
    };


    LiDAR lidar = LiDAR(lidar_calib_para_kitti_inverse);

    lidar.projectData(lidar_path1 + cloud_name + ".pcd", depth_map_lidar1, point_cloud_lidar_part, PCD, KITTI, XYZIT, C2L, CV);
    cv::FileStorage fs1(depth_map_camera_path1 + "depth_map" + image_name + ".xml", cv::FileStorage::READ);
    fs1["CameraDepthMap"] >> depth_map_camera1;
    depth_map_camera1 = depth_map_camera1 / 1000;



    //vector<double> theta(6, 0);
//    theta[0] = 2;
//    theta[3] = 0.8;
    //Transfer::vector2MatSeperate(theta, rotation, translation);
    //lidar.updateParameters(rotation, translation);
    //lidar.projectData(lidar_path1 + cloud_name + ".pcd", depth_map_lidar1, point_cloud_lidar_part, PCD, KITTI, XYZIT, C2L, CV);



//    cv::Point size(5, 10);
//    cv::Mat depth_map_lidar_dyed;
//    ImageUtils::neighborDyeing(depth_map_lidar1, size, depth_map_lidar_dyed);
//    ImageUtils::colorTransfer(depth_map_lidar1, left_image1);
//    cv::imwrite(test1_path + "test.png", left_image1);






    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_camera(new pcl::PointCloud<pcl::PointXYZ>);
    PointCloudAlignment::getCameraSparsePointCloud(depth_map_camera1, depth_map_lidar1, lidar, point_cloud_camera);
    PointCloudAlignment::pointCloudScaling(point_cloud_camera, 0.01, point_cloud_camera);



    //Transfer::vector2Eigen(theta, transformation);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_lidar_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud2 (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (*point_cloud_lidar_part, *transformed_lidar_cloud, transformation);

    float scale = PointCloudAlignment::findScaling(point_cloud_camera, transformed_lidar_cloud);
    PointCloudAlignment::pointCloudScaling(point_cloud_camera, scale, point_cloud_camera);

    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_camera_downsample (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_lidar_downsample (new pcl::PointCloud<pcl::PointXYZ> ());
    PointCloudAlignment::pointCloudDownsample(point_cloud_camera, point_cloud_camera_downsample, vox_volum);
    PointCloudAlignment::pointCloudDownsample(transformed_lidar_cloud, point_cloud_lidar_downsample, vox_volum);



    pcl::visualization::PCLVisualizer viewer ("test");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (point_cloud_lidar_downsample, 255, 255, 255);
    viewer.addPointCloud(point_cloud_lidar_downsample, source_cloud_color_handler, "transformed_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (point_cloud_camera_downsample, 230, 20, 20);
    viewer.addPointCloud(point_cloud_camera_downsample, transformed_cloud_color_handler, "camera_cloud");




    while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce ();
    }


    return 0;
}


//writer.write<pcl::PointXYZ>(test1_path + "test.pcd", *points, false);

//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    if (pcl::io::loadPCDFile<pcl::PointXYZ> (lidar_path1 + cloud_name + ".pcd", *cloud) == -1) //* load the file
//    {
//        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
//        exit(EXIT_FAILURE);
//    }