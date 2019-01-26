//
// Created by phyorch on 26/12/18.
//

//#include <iostream>
//
//
//#include <opencv2/core/types_c.h>
//#include <sl/Camera.hpp>
//#include <libcmaes/cmaes.h>
//#include <pcl/filters/statistical_outlier_removal.h>
//
//
//#include "Disparity.h"
//#include "SimilarityMeasure.h"
//#include "StereoGC.h"
//#include "ImageUtils.h"
//#include "RealEquipment.h"

#include <iostream>
#include <chrono>

#include "Disparity.h"
#include "SimilarityMeasure.h"
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
string data_name = "2011_09_26_drive_0048_sync";
string image_name = "/image_02/data/0000000002.png";
string cloud_name = "/velodyne_points/data/0000000002.pcd";
string depth_name = "/depth/depth1.png";
int feedback = 3;
float vox_volum = 2.5;
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
string lidar_image_output_path1 = "/home/" + user_name + "/Data/Result/OptimizationProcess/result";
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

LiDARCalibParaKittiInverse lidar_calib_para_kitti_inverse;

CameraPara camera_para;

LiDAR lidar = LiDAR(lidar_calib_para_kitti_inverse);

vector<vector<int>> region_point_set1, region_point_set2;

vector<cv::Mat> diagonal_points_set1, diagonal_points_set2;

pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_camera(new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_lidar_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

int cnt = 0;



libcmaes::FitFunc KL_divergence = [](const double *x, const double N){


//----------------------------------------------------------------------------------------------------------------------
//Data preparation
    cv::Mat rotationUpdate, translationUpdate;
    Transfer::array2MatSeperate(x, rotationUpdate, translationUpdate);
    lidar.updateParameters(rotationUpdate, translationUpdate);

    left_image = cv::imread(data_root + data_name + image_name);
    //left_image1 = cv::imread(left_path1 + "image" + image_name + "left.png");
    //left_image2 = cv::imread(left_path2 + "image1546525082left.png");
    //cv::FileStorage fs1(depth_map_camera_path1 + "depth_map" + image_name + ".xml", cv::FileStorage::READ);
    //fs1["CameraDepthMap"] >> depth_map_camera1;
    //cv::FileStorage fs2(depth_map_camera_path2 + "depth_map1546525082.xml", cv::FileStorage::READ);
    //fs2["CameraDepthMap"] >> depth_map_camera2;
    depth_map_camera1 = cv::imread(data_root + data_name + "/depth/depth1.png", CV_8UC1);

    //lidar.projectData(lidar_path1 + cloud_name + ".pcd", depth_map_camera1, depth_map_lidar1, point_cloud_lidar_part, PCD, KITTI, XYZIT, C2L, CV);
    //lidar.projectData(lidar_path2 + "1677.206636.pcd", depth_map_lidar2, point_cloud_part, PCD, KITTI, XYZIT, C2L, CV);
    lidar.projectData(data_root + data_name + cloud_name, depth_map_lidar1, point_cloud_lidar_part, XYZI, CV);
//    pcl::PCDWriter writer;
//    writer.write<pcl::PointXYZ>("/home/phyorch/Data/test.pcd", *point_cloud_camera_part, false);
//    writer.write<pcl::PointXYZ>("/home/phyorch/Data/test2.pcd", *point_cloud_lidar_part, false);

    //depth_map_camera1 = depth_map_camera1 / 1000;
    //depth_map_camera2 = depth_map_camera2 / 1000;
//----------------------------------------------------------------------------------------------------------------------



//----------------------------------------------------------------------------------------------------------------------
//distance type
//map KL Divergence distance
//    float kl_distance1 = HistogramMeasure::mapKLDivergence(depth_map_camera1, depth_map_lidar1, diagonal_points_set1);

//p2p distance
//    float p2p_distance1 = HistogramMeasure::point2PointDistanceTotal(depth_map_camera1, depth_map_lidar1, diagonal_points_set1);
//    float p2p_distance2 = HistogramMeasure::point2PointDistanceTotal(depth_map_camera2, depth_map_lidar2, diagonal_points_set2);

//point cloud distance

//    int t = DiagnosisUtils::mapPositiveCount(depth_map_lidar1);
//    PointCloudAlignment::getCameraSparsePointCloudKitti(depth_map_camera1, depth_map_lidar1, lidar, point_cloud_camera);
//    //pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_camera2(new pcl::PointCloud<pcl::PointXYZ>);
//    //PointCloudAlignment::getCameraSparsePointCloud(depth_map_camera1, depth_map_lidar1, lidar, point_cloud_camera2);
//    //PointCloudAlignment::pointCloudScaling(point_cloud_camera, 0.3, point_cloud_camera);
//    Transfer::array2Eigen(x, transformation);
//    pcl::transformPointCloud (*point_cloud_lidar_part, *transformed_lidar_cloud, transformation);
//    //float scale = PointCloudAlignment::findScaling(point_cloud_camera, transformed_lidar_cloud);
//    //PointCloudAlignment::pointCloudScaling(point_cloud_camera, scale, point_cloud_camera);
//
//
//
//    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_camera_downsample (new pcl::PointCloud<pcl::PointXYZ> ());
//    //pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_camera_downsample2 (new pcl::PointCloud<pcl::PointXYZ> ());
//    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_lidar_downsample (new pcl::PointCloud<pcl::PointXYZ> ());
//    PointCloudAlignment::pointCloudDownsample(point_cloud_camera, point_cloud_camera_downsample, vox_volum);
//    //PointCloudAlignment::pointCloudDownsample(point_cloud_camera2, point_cloud_camera_downsample2, vox_volum);
//    PointCloudAlignment::pointCloudDownsample(transformed_lidar_cloud, point_cloud_lidar_downsample, vox_volum);


//
//    pcl::visualization::PCLVisualizer viewer2 ("test2");
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler2 (point_cloud_lidar_downsample, 255, 255, 255);
//    viewer2.addPointCloud(point_cloud_lidar_downsample, source_cloud_color_handler2, "transformed_cloud");
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler2 (point_cloud_camera_downsample, 230, 20, 20);
//    viewer2.addPointCloud(point_cloud_camera_downsample, transformed_cloud_color_handler2, "camera_cloud");
//    while (!viewer2.wasStopped ()) { // Display the visualiser until 'q' key is pressed
//        viewer2.spinOnce ();
//    }

//    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_camera_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_lidar_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//    sor.setInputCloud (point_cloud_camera_downsample);
//    sor.setMeanK (10);
//    sor.setStddevMulThresh (1.0);
//    sor.filter (*point_cloud_camera_filtered);
//    sor.setInputCloud(point_cloud_lidar_downsample);
//    sor.filter(*point_cloud_lidar_filtered);


//    pcl::visualization::PCLVisualizer viewer ("test");
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (point_cloud_camera_filtered, 230, 20, 20);
//    viewer.addPointCloud(point_cloud_camera_filtered, source_cloud_color_handler, "transformed_cloud");
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (point_cloud_lidar_filtered, 255, 255, 255);
//    viewer.addPointCloud(point_cloud_lidar_filtered, transformed_cloud_color_handler, "camera_cloud");
//    while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
//        viewer.spinOnce ();
//    }



    float cloud_distance = HistogramMeasure::point2PointDistanceTotal(depth_map_camera1, depth_map_lidar1, diagonal_points_set1);
    //float cloud_distance = PointCloudAlignment::chamferDistance(point_cloud_camera_filtered, point_cloud_lidar_filtered);
//----------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------
//final result
    last_distance = cloud_distance;
    cnt++;
    if(cnt%feedback==0){
        cout <<"iteration " << cnt << endl << "p2p distance is " << endl << last_distance << endl;
        cout << "calibration result is " << endl << rotationUpdate << endl << translationUpdate << endl;
        ImageUtils::colorTransfer(depth_map_lidar1, left_image);
        //ImageUtils::colorTransfer(depth_map_lidar2, left_image2);
        ImageUtils::drawRectSet(left_image, diagonal_points_set1);
        //ImageUtils::drawRectSet(left_image2, diagonal_points_set2);
        cv::imwrite(lidar_image_output_path1 + to_string(cnt/feedback) + "___" + to_string(last_distance) + ".png", left_image);
        //cv::imwrite(lidar_image_output_path2 + to_string(cnt/3) + "___" + to_string(p2p_distance) + ".jpg", left_image2);
    }
    return last_distance;
//----------------------------------------------------------------------------------------------------------------------
};


int main(){

//----------------------------------------------------------------------------------------------------------------------
//2019_01_15 intrinsic calibration
//    R_self = (cv::Mat_<float>(4,4) << 1, 0, 0, 0,
//            0, 1, 0, 0,
//            0, 0, 1, 0,
//            0, 0, 0, 1);
//
//    P_self = (cv::Mat_<float>(3,4) << 674.213928, 0, 668.909607, 0,
//            0, 674.213928, 380.501831, 0,
//            0, 0, 1, 0);

//2019_01_15 extrinsic calibration
//    lid_to_cam_rotation = (cv::Mat_<float>(3,3) << 1, 0, 0,
//    0, 0, -1,
//    0, 1, 0);
//    lid_to_cam_translation = (cv::Mat_<float>(3,1) << 0.383, 0, 0);
//    transformation << 1, 0, 0, 0.383, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1;

//    lid_to_cam_rotation = (cv::Mat_<float>(3,3) << 0.87365133, 0.11061831, -0.47381106,
//    -0.43071392, -0.27713022, -0.85888553,
//    -0.22631583, 0.95444351, -0.19447035);
//    lid_to_cam_translation = (cv::Mat_<float>(3,1) << -0.69790131, -0.078251913, 0.27923283);
//    transformation << 0.87365133, 0.11061831, -0.47381106, -0.69790131, -0.43071392, -0.27713022, -0.85888553, -0.078251913, -0.22631583, 0.95444351, -0.19447035, 0.27923283, 0, 0, 0, 1;
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
//2011_09_26_drive_0048_sync intrinsic calibration
    R_self = (cv::Mat_<float>(4,4) << 9.998817e-01, 1.511453e-02, -2.841595e-03, 0,
            -1.511724e-02, 9.998853e-01, -9.338510e-04, 0,
            2.827154e-03, 9.766976e-04, 9.999955e-01, 0,
            0, 0, 0, 1);

    P_self = (cv::Mat_<float>(3,4) << 7.215377e+02, 0.000000e+00, 6.095593e+02, 0,//4.485728e+01
            0.000000e+00, 7.215377e+02, 1.728540e+02, 0,//2.163791e-01
            0.000000e+00, 0.000000e+00, 1.000000e+00, 0);//2.745884e-03

//2011_09_26_drive_0048_sync extrinsic calibration
    lid_to_cam_rotation = (cv::Mat_<float>(3,3) << 7.533745e-02, -3.999714e-01, -6.166020e-02,
            1.480249e-02, 7.280733e-04, -9.098902e-01,
            9.998621e-01, 7.523790e-03, 3.480755e-02);
    lid_to_cam_translation = (cv::Mat_<float>(3,1) << -4.069766e-03, -7.631618e-02, -6.717806e-01);
    cv::Mat cam0_to_cam2_rotation, cam0_to_cam2_translation;
    cam0_to_cam2_rotation = (cv::Mat_<float>(3,3) << 9.999758e-01, -5.267463e-03, -4.552439e-03,
            5.251945e-03, 9.999804e-01, -3.413835e-03,
            4.570332e-03, 3.389843e-03, 9.999838e-01);
    cam0_to_cam2_translation = (cv::Mat_<float>(3,1) << 5.956621e-02, 2.900141e-04, 2.577209e-03);
    lid_to_cam_rotation = cam0_to_cam2_rotation * lid_to_cam_rotation;
    lid_to_cam_translation = cam0_to_cam2_rotation * lid_to_cam_translation + cam0_to_cam2_translation;

    Transfer::cv2EigenSeperate(lid_to_cam_rotation, lid_to_cam_translation, transformation);
    cout << lid_to_cam_rotation << endl << lid_to_cam_translation << endl << transformation;
//----------------------------------------------------------------------------------------------------------------------


//Class initilization
    lidar_calib_para_kitti_inverse = {
            Rotation:lid_to_cam_rotation,
            Translation:lid_to_cam_translation,
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

    lidar = LiDAR(lidar_calib_para_kitti_inverse);

//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------
//Fixed comparision region
    cv::Mat region11, region12, region13, region21, region22, region23;
    //for p2p distance
//    region11 = (cv::Mat_<int>(2,2) << 475, 450, 549, 480);
//    region12 = (cv::Mat_<int>(2,2) << 583, 421, 632, 473);
//    region13 = (cv::Mat_<int>(2,2) << 677, 419, 743, 462);
    //for KL divergence
    region11 = (cv::Mat_<int>(2,2) << 258, 162, 479, 326);
    region12 = (cv::Mat_<int>(2,2) << 475, 164, 564, 216);
    region13 = (cv::Mat_<int>(2,2) << 654, 169, 752, 241);
//    region21 = (cv::Mat_<int>(2,2) << 670, 390, 900, 440);
//    region22 = (cv::Mat_<int>(2,2) << 280, 450, 500, 500);
//    region23 = (cv::Mat_<int>(2,2) << 950, 430, 1200, 500);
    diagonal_points_set1.push_back(region11);
    diagonal_points_set1.push_back(region12);
    diagonal_points_set1.push_back(region13);
//    diagonal_points_set2.push_back(region21);
//    diagonal_points_set2.push_back(region22);
//    diagonal_points_set2.push_back(region23);
//----------------------------------------------------------------------------------------------------------------------
//CMA-ES setting
    cnt = 0;
    int dim = 6;
    double sigma = 0.06;
    int lambda = 15;
    vector<double> test_vec(dim, 0);
    Transfer::mat2VectorSeperate(lid_to_cam_rotation, lid_to_cam_translation, test_vec);
    libcmaes::CMAParameters<> cma_para(test_vec, sigma, lambda);
    int t = cma_para.lambda();
    int tt = cma_para.get_max_iter();

//    cv::FileStorage fs1(depth_map_camera_path1 + "depth_map" + image_name + ".xml", cv::FileStorage::READ);
//    fs1["CameraDepthMap"] >> depth_map_camera1;
    left_image = cv::imread(data_root + data_name + image_name);
    depth_map_camera1 = cv::imread(data_root + data_name + "/depth/depth1.png", CV_8UC1);
    lidar.projectData(data_root + data_name + cloud_name, depth_map_lidar1, point_cloud_lidar_part, XYZI, CV);
    ImageUtils::colorTransfer(depth_map_lidar1, left_image);
    ImageUtils::drawRectSet(left_image, diagonal_points_set1);
    cv::imwrite(lidar_image_output_path1 + "_start.png", left_image);

    cma_para.set_algo(BIPOP_CMAES);
    libcmaes::CMASolutions cmasols = libcmaes::cmaes<>(KL_divergence, cma_para);
    cout << "best solution: " << cmasols << endl;
    cout << "optimization took " << cmasols.elapsed_time() / 1000.0 << " seconds\n";
    return cmasols.run_status();

//    depth_map_camera1 = depth_map_camera1 /1000;
//    DiagnosisUtils::regionDiagnosis(depth_map_camera1, depth_map_lidar1, region11);
//    return 0;


    /*-------------------Histogram generation---------------------*/



    //cout << "distance1:  " << distance_hist1 << endl << "distance2:  " <<distance_hist2 << endl;

    //lidar.updateParameters(rotation, translation);

    return 0;
}

//point cloud amout balance
//if(size_lidar>=size_camera){
//float ratio = float(point_cloud_lidar_downsample->points.size()) / float(point_cloud_camera_downsample->points.size());
//float volum = vox_volum * pow(ratio, 1.0/3);
//PointCloudAlignment::pointCloudDownsample(point_cloud_lidar_downsample, point_cloud_lidar_downsample, volum);
//}
//else{
//float ratio = float(point_cloud_camera_downsample->points.size()) / float(point_cloud_lidar_downsample->points.size());
//float volum = vox_volum * pow(ratio, 1.0/3);
//PointCloudAlignment::pointCloudDownsample(point_cloud_camera_downsample, point_cloud_camera_downsample, volum);
//}