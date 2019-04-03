//
// Created by phyorch on 24/02/19.
//

#include <iostream>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/filters/statistical_outlier_removal.h>

#include "Sensor.h"
#include "Calibration.h"

using namespace std;
string user_name = "phyorch";
string data_name = "2011_09_26_drive_0005_sync";
string image_name = "/image_02/data/0000000153.png";
string image_name2 = "/image_02/data/0000000152.png";
string cloud_name = "/velodyne_points/data/0000000153.pcd";
string cloud_name2 = "/velodyne_points/data/0000000152.pcd";

string data_root = "/home/" + user_name + "/Data/";
string depth_name = "/depth/0000000153.png";


int feedback = 3;
float vox_volum = 0.8;
float last_distance;
cv::Mat R_self, P_self, lid_to_cam_rotation, lid_to_cam_translation;

void bundleAdjustment (
        const vector<cv::Point3f> points_3d,
        const vector<cv::Point2f> points_2d,
        const cv::Mat& K,
        cv::Mat& R, cv::Mat& t
);

int main ()
{
    cv::Mat img_1 = cv::imread ( data_root + data_name + image_name, CV_LOAD_IMAGE_COLOR );
    cv::Mat img_2 = cv::imread ( data_root + data_name + image_name2, CV_LOAD_IMAGE_COLOR );

    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_lidar(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI> (data_root + data_name + cloud_name, *point_cloud_lidar) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        exit(EXIT_FAILURE);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_lidar_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    PointCloudAlignment::getLiDARPointCloudXYZ(point_cloud_lidar, point_cloud_lidar_xyz);

    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_lidar2(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI> (data_root + data_name + cloud_name2, *point_cloud_lidar2) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        exit(EXIT_FAILURE);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_lidar_xyz2(new pcl::PointCloud<pcl::PointXYZ>);
    PointCloudAlignment::getLiDARPointCloudXYZ(point_cloud_lidar2, point_cloud_lidar_xyz2);

    R_self = (cv::Mat_<float>(4,4) << 9.998817e-01, 1.511453e-02, -2.841595e-03, 0,
            5.251945e-03, 9.999804e-01, -3.413835e-03, 0,
            2.827154e-03, 9.766976e-04, 9.999955e-01, 0,
            0, 0, 0, 1);
    P_self = (cv::Mat_<float>(3,4) << 7.215377e+02, 0.000000e+00, 6.095593e+02, 0,//4.485728e+01
            0.000000e+00, 7.215377e+02, 1.728540e+02, 0,//2.163791e-01
            0.000000e+00, 0.000000e+00, 1.000000e+00, 0);//2.745884e-03

    lid_to_cam_rotation = (cv::Mat_<float>(3,3) << 0.055720564, -0.99817532, -0.023264745,
            -0.0065545961, 0.02293475, -0.99971545,
            0.99842489, 0.0558572, -0.005264699);
    lid_to_cam_translation = (cv::Mat_<float>(3,1) << -0.44149587, 0.47857293, -0.39773074);

    LiDARCalibParaKittiInverse lidar_calib_para_kitti_inverse;

    lidar_calib_para_kitti_inverse = {
            Rotation:lid_to_cam_rotation,
            Translation:lid_to_cam_translation,
            R:R_self,
            P:P_self,
            imageSize:cv::Size(img_1.cols, img_1.rows)
    };
    LiDAR lidar = LiDAR(lidar_calib_para_kitti_inverse);

    vector<cv::KeyPoint> keypoints_1, keypoints_2;
    vector<cv::DMatch> matches;
    HandEyeCalibration::findFeatureMatches(img_1, img_2, keypoints_1, keypoints_2, matches);
    cout<<"一共找到了"<<matches.size() <<"组匹配点"<<endl;

    vector<cv::Point3f> pts_3d;
    vector<cv::Point2f> pts_2d;

    cv::Mat K;
    K = ( cv::Mat_<double> ( 3,3 ) << 7.215377e+02, 0.000000e+00, 6.095593e+02,
            0.000000e+00, 7.215377e+02, 1.728540e+02,
            0.000000e+00, 0.000000e+00, 1.000000e+00);

    cv::Mat depth_map_camera = cv::imread(data_root + data_name + depth_name, CV_8UC1);
    HandEyeCalibration::creat3D2DPoints(lidar, depth_map_camera, keypoints_1, keypoints_2, matches, pts_3d, pts_2d);

    cout<<"3d-2d pairs: "<<pts_3d.size() <<endl;

    cv::Mat r, t;
    solvePnP ( pts_3d, pts_2d, K, cv::Mat(), r, t, false ); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
    cv::Mat R;
    cv::Rodrigues ( r, R ); // r为旋转向量形式，用Rodrigues公式转换为矩阵

    //cout<<"R="<<endl<<R<<endl;
    //cout<<"t="<<endl<<t<<endl;

    cv::Mat transformation_camera = cv::Mat::zeros(4, 4, CV_32FC1);
    Transfer::matSeperate2Mat(R, t, transformation_camera);
    cout << transformation_camera << endl;

    //cout<<"calling bundle adjustment"<<endl;

    //bundleAdjustment ( pts_3d, pts_2d, K, R, t );


    cv::Mat transformation_lidar = cv::Mat::zeros(4, 4, CV_32FC1);
    HandEyeCalibration::pointCloudRegistration(point_cloud_lidar_xyz, point_cloud_lidar_xyz2, vox_volum, transformation_lidar);
    cout << transformation_lidar << endl;

    cv::Mat transformation_camera_lidar = cv::Mat::zeros(4, 4, CV_32FC1);
    HandEyeCalibration::handEyeTsai(transformation_camera_lidar, transformation_lidar, transformation_camera);
    cout << transformation_camera_lidar;

//    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_lidar_xyz_sampled(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_lidar_xyz2_sampled(new pcl::PointCloud<pcl::PointXYZ>);
//    PointCloudAlignment::pointCloudDownsample(point_cloud_lidar_xyz, point_cloud_lidar_xyz, vox_volum);
//    PointCloudAlignment::pointCloudDownsample(point_cloud_lidar_xyz2, point_cloud_lidar_xyz2, vox_volum);
//
//    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_camera_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_lidar_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//    sor.setInputCloud (point_cloud_lidar_xyz);
//    sor.setMeanK (30);
//    sor.setStddevMulThresh (1.0);
//    sor.filter (*point_cloud_lidar_xyz);
//    sor.setInputCloud(point_cloud_lidar_xyz2);
//    sor.filter(*point_cloud_lidar_xyz2);
//
//    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//    icp.setInputSource(point_cloud_lidar_xyz);
//    icp.setInputTarget(point_cloud_lidar_xyz2);
//    pcl::PointCloud<pcl::PointXYZ> Final;
//    icp.align(Final);
//    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
//              icp.getFitnessScore() << std::endl;
//
//    pcl::transformPointCloud (*point_cloud_lidar_xyz, *point_cloud_lidar_xyz, icp.getFinalTransformation());
//    pcl::visualization::PCLVisualizer viewer ("test");
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (point_cloud_lidar_xyz, 255, 255, 255);
//    viewer.addPointCloud(point_cloud_lidar_xyz, source_cloud_color_handler, "transformed_cloud");
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (point_cloud_lidar_xyz2, 230, 20, 20);
//    viewer.addPointCloud(point_cloud_lidar_xyz2, transformed_cloud_color_handler, "camera_cloud");
//    while (!viewer.wasStopped ()) {
//        viewer.spinOnce ();
//    }
//    viewer.close();

}