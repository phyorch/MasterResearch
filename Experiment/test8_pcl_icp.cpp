//
// Created by phyorch on 18/01/19.
//

#define HAVE_GLOG

#include <iostream>
#include <chrono>

#include "Sensor.h"
#include "Calibration.h"
#include <opencv2/core/types_c.h>
#include "StereoGC.h"
#include "ImageUtils.h"
#include "RealEquipment.h"
#include <sl/Camera.hpp>
#include <libcmaes/cmaes.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;
string user_name = "phyorch";
string data_name = "2011_09_26_drive_0005_sync";
string image_name = "/image_02/data/0000000082.png";
string cloud_name = "/velodyne_points/data/0000000082.pcd";
string cloud_name2 = "/velodyne_points/data/0000000083.pcd";
string depth_name = "/depth/depth1.png";
int feedback = 3;
float vox_volum = 0.8;
float last_distance;

//----------------------------------------------------------------------------------------------------------------------
//Notes
//1. There are several nesting method for the part of ProjectData, and the corresponding point cloud for projection
//we need to change the method determined by convenience


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
string lidar_image_output_path1 = "/home/" + user_name + "/Data/Result/OptimizationProcess/";
string lidar_image_output_path2 = "/home/" + user_name + "/Data/Result/OptimizationProcess/2result";
string depth_map_camera_boader_path = "/home/" + user_name + "/Data/camera_depth_boader.jpg";

string camera_csv_path = "/home/" + user_name + "/Data/HistCamera.csv";
string lidar_csv_path = "/home/" + user_name + "/Data/HistLiDAR.csv";
string test1_path = "/home/" + user_name + "/Data/test.jpg";
string test2_path = "/home/" + user_name + "/Data/test2.jpg";

string zed_rgb_image_path = "/home/" + user_name + "/Data/ZEDData/RGBImage/";
string zed_depth_image_path = "/home/" + user_name + "/Data/ZEDData/DepthImage/";
string pandar_cloud_path = "/home/" + user_name + "/Data/Pandar40Data/PCDDataTest";

cv::Mat left_image = cv::imread(data_root + data_name + image_name);
//cv::Mat left_image1 = cv::imread(left_path1 + "image" + image_name + "left.png");
//cv::Mat left_image2 = cv::imread(left_path2 + "image" + image_name + "left.png");
cv::Mat depth_map_camera1, depth_map_camera_boader1, depth_map_camera2, depth_map_camera_boader2;
cv::Mat depth_map_lidar1, depth_map_lidar_boader1, depth_map_lidar2, depth_map_lidar_boader2;
pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_lidar_part(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_camera_part(new pcl::PointCloud<pcl::PointXYZ>);

//Intrinsic parameters for self data
cv::Mat R_self;

cv::Mat P_self;



//Extrinsic parameters variable setting
cv::Mat cam_to_lid_rotation;

cv::Mat cam_to_lid_translation;

cv::Mat lid_to_cam_rotation;

cv::Mat lid_to_cam_translation;

Eigen::Matrix4f transformation;

// kitti model
//    LiDARCalibParaKitti lidar_calib_para_kitti = {
//            T:velo_to_cam,
//            R:R_rect_00_extended,
//            P:P_rect_00,
//            imageSize:cv::Size(left_image.cols, left_image.rows)
//    };

LiDARCalibParaKitti lidar_calib_para_kitti;

CameraPara camera_para;

LiDAR lidar = LiDAR(lidar_calib_para_kitti);

vector<vector<int>> region_point_set1, region_point_set2;

vector<cv::Mat> diagonal_points_set1, diagonal_points_set2;

pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_camera(new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_lidar_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

int cnt = 0;

int main(){

//    rotation = (cv::Mat_<float>(3,3) << -0.96917289, 0.18399496, 0.16385905, -0.14308745, 0.12108093, -0.98227561, -0.20057398, -0.97544104, -0.091020979);
//    translation = (cv::Mat_<float>(3,1) << 0.14438252, -0.14530572, 0.12848426);
//    lid_to_cam_rotation = (cv::Mat_<float>(3,3) << 0.87365133, 0.11061831, -0.47381106,
//            -0.43071392, -0.27713022, -0.85888553,
//            -0.22631583, 0.95444351, -0.19447035);
//    lid_to_cam_translation = (cv::Mat_<float>(3,1) << -0.69790131, -0.078251913, 0.27923283);

//    lid_to_cam_rotation = (cv::Mat_<float>(3,3) << 1, 0, 0,
//    0, 0, -1,
//    0, 1, 0);
//    lid_to_cam_translation = (cv::Mat_<float>(3,1) << 0.383, 0, 0);
//    transformation << 1, 0, 0, 0.383, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1;

//----------------------------------------------------------------------------------------------------------------------
//Intrinsic parameters1
    R_self = (cv::Mat_<float>(4,4) << 9.998817e-01, 1.511453e-02, -2.841595e-03, 0,
            -1.511724e-02, 9.998853e-01, -9.338510e-04, 0,
            2.827154e-03, 9.766976e-04, 9.999955e-01, 0,
            0, 0, 0, 1);

    P_self = (cv::Mat_<float>(3,4) << 7.215377e+02, 0.000000e+00, 6.095593e+02, 0,//4.485728e+01
            0.000000e+00, 7.215377e+02, 1.728540e+02, 0,//2.163791e-01
            0.000000e+00, 0.000000e+00, 1.000000e+00, 0);//2.745884e-03
//----------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------
//Calibration 2011_09_26_drive_0002_sync
//    lid_to_cam_rotation = (cv::Mat_<float>(3,3) << 0.013313008, -0.98023027, 0.19741155,
//    0.32443258, -0.1825134, -0.92813379,
//    0.94581515, 0.076402992, 0.31558883);
//    lid_to_cam_translation = (cv::Mat_<float>(3,1) << -0.26534855, 1.1770095, -0.069104768);
    lid_to_cam_rotation = (cv::Mat_<float>(3,3) << 7.533745e-03, -9.999714e-01, -6.166020e-04,
            4.480249e-02, 7.280733e-04, -9.998902e-01,
            9.998621e-01, 7.523790e-03, 1.480755e-02);
    lid_to_cam_translation = (cv::Mat_<float>(3,1) << -4.069766e-03, -7.631618e-02, -2.717806e-01);
    cv::Mat cam0_to_cam2_rotation, cam0_to_cam2_translation;
    cam0_to_cam2_rotation = (cv::Mat_<float>(3,3) << 9.998817e-01, 1.511453e-02, -2.841595e-03,
            -1.511724e-02, 9.998853e-01, -9.338510e-04,
            2.827154e-03, 9.766976e-04, 9.999955e-01);
    cam0_to_cam2_translation = (cv::Mat_<float>(3,1) << 5.956621e-02, 2.900141e-04, 2.577209e-03);
    lid_to_cam_rotation = cam0_to_cam2_rotation * lid_to_cam_rotation;
    lid_to_cam_translation = cam0_to_cam2_rotation * lid_to_cam_translation + cam0_to_cam2_translation;


    float degreex = 30;
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


    Transfer::cv2EigenSeperate(lid_to_cam_rotation, lid_to_cam_translation, transformation);
//----------------------------------------------------------------------------------------------------------------------

    lidar_calib_para_kitti = {
            Rotation:lid_to_cam_rotation,
            Translation:lid_to_cam_translation,
            R:R_self,
            P:P_self,
            imageSize:cv::Size(left_image.cols, left_image.rows)
    };

    LiDAR lidar = LiDAR(lidar_calib_para_kitti);


    left_image = cv::imread(data_root + data_name + image_name);
    depth_map_camera1 = cv::imread(data_root + data_name + "/depth/depth1.png", CV_8UC1);

    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_lidar(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI> (data_root + data_name + cloud_name, *point_cloud_lidar) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        exit(EXIT_FAILURE);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_lidar_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    PointCloudAlignment::getLiDARPointCloudXYZ(point_cloud_lidar, point_cloud_lidar_xyz);

    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_lidar2(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI> (data_root + data_name + cloud_name2, *point_cloud_lidar) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        exit(EXIT_FAILURE);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_lidar_xyz2(new pcl::PointCloud<pcl::PointXYZ>);
    PointCloudAlignment::getLiDARPointCloudXYZ(point_cloud_lidar, point_cloud_lidar_xyz2);


    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_lidar_xyz_sampled(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_lidar_xyz2_sampled(new pcl::PointCloud<pcl::PointXYZ>);
    PointCloudAlignment::pointCloudDownsample(point_cloud_lidar_xyz, point_cloud_lidar_xyz_sampled, vox_volum);
    PointCloudAlignment::pointCloudDownsample(point_cloud_lidar_xyz2, point_cloud_lidar_xyz2_sampled, vox_volum);

//    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_camera_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_lidar_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//    sor.setInputCloud (point_cloud_lidar_xyz);
//    sor.setMeanK (30);
//    sor.setStddevMulThresh (1.0);
//    sor.filter (*point_cloud_lidar_xyz);
//    sor.setInputCloud(point_cloud_lidar_xyz2);
//    sor.filter(*point_cloud_lidar_xyz2);


    pcl::visualization::PCLVisualizer viewer ("test");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (point_cloud_lidar_xyz, 255, 255, 255);
    viewer.addPointCloud(point_cloud_lidar_xyz, source_cloud_color_handler, "transformed_cloud");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (point_cloud_lidar_xyz2, 230, 20, 20);
    viewer.addPointCloud(point_cloud_lidar_xyz2, transformed_cloud_color_handler, "camera_cloud");
    while (!viewer.wasStopped ()) {
        viewer.spinOnce ();
    }
    viewer.close();




//    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_camera(new pcl::PointCloud<pcl::PointXYZ>);
//    PointCloudAlignment::getCameraPointCloud(depth_map_camera1, lidar, point_cloud_camera);
//    PointCloudAlignment::pointCloudDownsample(point_cloud_camera, point_cloud_camera, vox_volum);

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(point_cloud_lidar_xyz_sampled);
    icp.setInputTarget(point_cloud_lidar_xyz2_sampled);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
              icp.getFitnessScore() << std::endl;
//    std::cout << icp.getFinalTransformation() * transformation  << std::endl;
//    Eigen::Matrix4f final_transformation = icp.getFinalTransformation() * transformation;
//    Transfer::Eigen2MatSeperate(final_transformation, lid_to_cam_rotation, lid_to_cam_translation);
    pcl::transformPointCloud (*point_cloud_lidar_xyz, *point_cloud_lidar_xyz, icp.getFinalTransformation());

//    lidar.updateParameters(lid_to_cam_rotation, lid_to_cam_translation);
//
//    lidar.projectData(data_root + data_name + cloud_name, depth_map_lidar1, point_cloud_lidar_part, XYZI, CV);
//    cv::Point size(3, 3);
//    cv::Mat dyed;
//    ImageUtils::neighborDyeing(depth_map_lidar1, size, dyed);
//    ImageUtils::colorTransfer(dyed, left_image, 70);
//    cv::imwrite(test1_path, left_image);

    pcl::visualization::PCLVisualizer viewer2 ("test");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler2 (point_cloud_lidar_xyz, 255, 255, 255);
    viewer2.addPointCloud(point_cloud_lidar_xyz, source_cloud_color_handler2, "transformed_cloud");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler2 (point_cloud_lidar_xyz2, 230, 20, 20);
    viewer2.addPointCloud(point_cloud_lidar_xyz2, transformed_cloud_color_handler2, "camera_cloud");
    while (!viewer2.wasStopped ()) {
        viewer2.spinOnce ();
    }
    viewer2.close();


//    for(int i=0; i<50; i++){
//        left_image = cv::imread(data_root + data_name + image_name);
//
//        depth_map_camera1 = cv::imread(data_root + data_name + "/depth/depth1.png", CV_8UC1);
//        lidar.projectData(data_root + data_name + cloud_name, depth_map_lidar1, point_cloud_lidar_part, XYZI, CV);
//
//        ImageUtils::colorTransfer(depth_map_lidar1, left_image, 70);
//        if(i==0){
//            cv::imwrite(lidar_image_output_path1 + "start.png", left_image);
//        } else{
//            cv::imwrite(lidar_image_output_path1 + to_string(i) + "_" + to_string(last_distance) + ".png", left_image);
//        }
//
//        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_camera(new pcl::PointCloud<pcl::PointXYZ>);
//        PointCloudAlignment::getCameraSparsePointCloudKitti(depth_map_camera1, depth_map_lidar1, lidar, point_cloud_camera);
//
//
//        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_lidar_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
//        pcl::transformPointCloud (*point_cloud_lidar_part, *transformed_lidar_cloud, transformation);
//
//
//        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_camera_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_lidar_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//        sor.setInputCloud (point_cloud_camera);
//        sor.setMeanK (50);
//        sor.setStddevMulThresh (1.0);
//        sor.filter (*point_cloud_camera_filtered);
//        sor.setInputCloud(transformed_lidar_cloud);
//        sor.filter(*point_cloud_lidar_filtered);
//
//
//
//        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_camera_downsample (new pcl::PointCloud<pcl::PointXYZ> ());
//        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_lidar_downsample (new pcl::PointCloud<pcl::PointXYZ> ());
//        PointCloudAlignment::pointCloudDownsample(point_cloud_camera_filtered, point_cloud_camera_downsample, vox_volum);
//        PointCloudAlignment::pointCloudDownsample(point_cloud_lidar_filtered, point_cloud_lidar_downsample, vox_volum);
//
//
//        last_distance = PointCloudAlignment::chamferDistance(point_cloud_camera_downsample, point_cloud_lidar_downsample);
//
//
////        sor.setMeanK(10);
////        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_camera_filtered2(new pcl::PointCloud<pcl::PointXYZ>);
////        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_lidar_filtered2(new pcl::PointCloud<pcl::PointXYZ>);
////        sor.setInputCloud (point_cloud_camera_downsample);
////        sor.filter (*point_cloud_camera_filtered2);
////        sor.setInputCloud (point_cloud_lidar_downsample);
////        sor.filter (*point_cloud_lidar_filtered2);
//
//
////        pcl::visualization::PCLVisualizer viewer ("test");
////        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (point_cloud_camera_downsample, 255, 255, 255);
////        viewer.addPointCloud(point_cloud_camera_downsample, source_cloud_color_handler, "transformed_cloud");
////        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (point_cloud_lidar_downsample, 230, 20, 20);
////        viewer.addPointCloud(point_cloud_lidar_downsample, transformed_cloud_color_handler, "camera_cloud");
////        while (!viewer.wasStopped ()) {
////            viewer.spinOnce ();
////        }
////        viewer.close();
//
//        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//        icp.setInputSource(point_cloud_lidar_downsample);
//        icp.setInputTarget(point_cloud_camera_downsample);
//        pcl::PointCloud<pcl::PointXYZ> Final;
//        icp.align(Final);
//        std::cout << "has converged:" << icp.hasConverged() << " score: " <<
//                  icp.getFitnessScore() << std::endl;
//        std::cout << icp.getFinalTransformation() * transformation  << std::endl;
//        Eigen::Matrix4f final_transformation = icp.getFinalTransformation();
//
//        //pcl::transformPointCloud (*point_cloud_camera, *point_cloud_camera, final_transformation);
//
//
//
////        pcl::visualization::PCLVisualizer viewer2 ("test2");
////        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler2 (point_cloud_camera_downsample, 255, 255, 255);
////        viewer2.addPointCloud(point_cloud_camera_downsample, source_cloud_color_handler, "transformed_cloud");
////        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler2 (point_cloud_lidar_downsample2, 230, 20, 20);
////        viewer2.addPointCloud(point_cloud_lidar_downsample2, transformed_cloud_color_handler, "camera_cloud");
////        while (!viewer2.wasStopped ()) {
////            viewer2.spinOnce ();
////        }
////        viewer2.close();
//
//
//        transformation = icp.getFinalTransformation() * transformation;
//        Transfer::Eigen2MatSeperate(transformation, lid_to_cam_rotation, lid_to_cam_translation);
//        lidar.updateParameters(lid_to_cam_rotation, lid_to_cam_translation);
//        cout << lid_to_cam_rotation << endl << lid_to_cam_translation << endl;
//    }

    return 0;
}