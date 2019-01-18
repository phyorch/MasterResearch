//
// Created by phyorch on 26/12/18.
//

#include "Disparity.h"
#include "SimilarityMeasure.h"
#include <iostream>
#include <opencv2/core/types_c.h>
#include "StereoGC.h"
#include "ImageUtils.h"
#include "RealEquipment.h"
#include <sl/Camera.hpp>
#include <libcmaes/cmaes.h>


using namespace std;
string user_name = "phyorch";
string data_name = "2019_01_15/2019_01_15_1";
string image_name = "1547540975";
string cloud_name = "137.508265";
int feedback = 1;
float vox_volum = 0.15;
float last_distance;

//----------------------------------------------------------------------------------------------------------------------
//Notes
//1. There are several nesting method for the part of ProjectData, and the corresponding point cloud for projection
//we need to change the method determined by convenience



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
string test1_path = "/home/" + user_name + "/Data/test.jpg";
string test2_path = "/home/" + user_name + "/Data/test2.jpg";

string zed_rgb_image_path = "/home/" + user_name + "/Data/ZEDData/RGBImage/";
string zed_depth_image_path = "/home/" + user_name + "/Data/ZEDData/DepthImage/";
string pandar_cloud_path = "/home/" + user_name + "/Data/Pandar40Data/PCDDataTest";

cv::Mat left_image1 = cv::imread(left_path1 + "image" + image_name + "left.png");
cv::Mat left_image2 = cv::imread(left_path2 + "image" + image_name + "left.png");
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

    left_image1 = cv::imread(left_path1 + "image" + image_name + "left.png");
    //left_image2 = cv::imread(left_path2 + "image1546525082left.png");
    cv::FileStorage fs1(depth_map_camera_path1 + "depth_map" + image_name + ".xml", cv::FileStorage::READ);
    fs1["CameraDepthMap"] >> depth_map_camera1;
    //cv::FileStorage fs2(depth_map_camera_path2 + "depth_map1546525082.xml", cv::FileStorage::READ);
    //fs2["CameraDepthMap"] >> depth_map_camera2;

    //lidar.projectData(lidar_path1 + cloud_name + ".pcd", depth_map_camera1, depth_map_lidar1, point_cloud_lidar_part, PCD, KITTI, XYZIT, C2L, CV);
    //lidar.projectData(lidar_path2 + "1677.206636.pcd", depth_map_lidar2, point_cloud_part, PCD, KITTI, XYZIT, C2L, CV);
    lidar.projectData(lidar_path1 + cloud_name + ".pcd", depth_map_camera1, depth_map_lidar1, point_cloud_camera_part, point_cloud_lidar_part, PCD, KITTI, XYZIT, C2L, CV);
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>("/home/phyorch/Data/test.pcd", *point_cloud_camera_part, false);
    writer.write<pcl::PointXYZ>("/home/phyorch/Data/test2.pcd", *point_cloud_lidar_part, false);

    depth_map_camera1 = depth_map_camera1 / 1000;
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

    int t = DiagnosisUtils::mapPositiveCount(depth_map_lidar1);
    PointCloudAlignment::getCameraSparsePointCloud(depth_map_camera1, depth_map_lidar1, lidar, point_cloud_camera);
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_camera2(new pcl::PointCloud<pcl::PointXYZ>);
    PointCloudAlignment::getCameraSparsePointCloud(depth_map_camera1, depth_map_lidar1, lidar, point_cloud_camera2);
    PointCloudAlignment::pointCloudScaling(point_cloud_camera, 0.3, point_cloud_camera);
    Transfer::array2Eigen(x, transformation);
    pcl::transformPointCloud (*point_cloud_lidar_part, *transformed_lidar_cloud, transformation);
    float scale = PointCloudAlignment::findScaling(point_cloud_camera, transformed_lidar_cloud);
    PointCloudAlignment::pointCloudScaling(point_cloud_camera, scale, point_cloud_camera);

    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_camera_downsample (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_camera_downsample2 (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_lidar_downsample (new pcl::PointCloud<pcl::PointXYZ> ());
    PointCloudAlignment::pointCloudDownsample(point_cloud_camera, point_cloud_camera_downsample, vox_volum);
    PointCloudAlignment::pointCloudDownsample(point_cloud_camera2, point_cloud_camera_downsample2, vox_volum);
    PointCloudAlignment::pointCloudDownsample(transformed_lidar_cloud, point_cloud_lidar_downsample, vox_volum);
    int size_camera = point_cloud_camera_downsample->points.size();
    int size_lidar = point_cloud_lidar_downsample->points.size();
    if(size_lidar>=size_camera){
        float ratio = float(point_cloud_lidar_downsample->points.size()) / float(point_cloud_camera_downsample->points.size());
        float volum = vox_volum * pow(ratio, 1.0/3);
        PointCloudAlignment::pointCloudDownsample(point_cloud_lidar_downsample, point_cloud_lidar_downsample, volum);
    }
    else{
        float ratio = float(point_cloud_camera_downsample->points.size()) / float(point_cloud_lidar_downsample->points.size());
        float volum = vox_volum * pow(ratio, 1.0/3);
        PointCloudAlignment::pointCloudDownsample(point_cloud_camera_downsample, point_cloud_camera_downsample, volum);
    }


    float cloud_distance = PointCloudAlignment::chamferDistance(point_cloud_camera_downsample, point_cloud_lidar_downsample);
//----------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------
//final result
    float distance = cloud_distance;
    cnt++;
    if(cnt%feedback==0){
        cout <<"iteration " << cnt << endl << "p2p distance is " << endl << distance << endl;
        cout << "calibration result is " << endl << rotationUpdate << endl << translationUpdate << endl;
        ImageUtils::colorTransfer(depth_map_lidar1, left_image1);
        //ImageUtils::colorTransfer(depth_map_lidar2, left_image2);
        ImageUtils::drawRectSet(left_image1, diagonal_points_set1);
        //ImageUtils::drawRectSet(left_image2, diagonal_points_set2);
        cv::imwrite(lidar_image_output_path1 + to_string(cnt/feedback) + "___" + to_string(distance) + ".jpg", left_image1);
        //cv::imwrite(lidar_image_output_path2 + to_string(cnt/3) + "___" + to_string(p2p_distance) + ".jpg", left_image2);
    }
    last_distance = distance;
    return distance;
//----------------------------------------------------------------------------------------------------------------------
};


int main(){

//----------------------------------------------------------------------------------------------------------------------
//Intrinsic parameters
    R_self = (cv::Mat_<float>(4,4) << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1);

    P_self = (cv::Mat_<float>(3,4) << 674.931946, 0, 673.048401, 0,
            0, 674.931946, 390.270477, 0,
            0, 0, 1, 0);
//----------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------
//Calibration2
//
//    cam_to_velo_rotation = (cv::Mat_<float>(3,3) << -9.816086341230e-01, -1.508804030660e-01, 1.169597938860e-01,
//            -1.152563726088e-01, -2.001656302531e-02, -9.931340824771e-01,
//            1.521856037484e-01, -9.883493517857e-01, 2.258503135975e-03);
//    cam_to_velo_translation = (cv::Mat_<float>(3,1) << 2.757819023341e-01, 8.406759160258e-02, -9.936401615109e-02);
//    cv::transpose(cam_to_velo_rotation, velo_to_cam_rotation);
//    velo_to_cam_translation = -(velo_to_cam_rotation * cam_to_velo_translation);
//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------
//Calibration3
//    cam_to_lid_rotation = (cv::Mat_<float>(3,3) << -9.995395743477e-01, -2.591763623920e-02, -1.577705437008e-02,
//            1.563204680325e-02, 5.788061929611e-03, -9.998610590736e-01,
//            2.600535378657e-02, -9.996473250456e-01, -5.380251254700e-03);
//
//    cam_to_lid_translation = (cv::Mat_<float>(3,1) << -1.131729644136e-01, 9.325187939857e-02, -5.436065917896e-02);
//
//    cv::transpose(cam_to_lid_rotation, lid_to_cam_rotation);
//
//    lid_to_cam_translation = -(lid_to_cam_rotation * cam_to_lid_translation);
//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------
//Test1
//    lid_to_cam_rotation = (cv::Mat_<float>(3,3) << 1, 0, 0,
//    0, 0, -1,
//    0, 1, 0);
//
//    lid_to_cam_translation = (cv::Mat_<float>(3,1) << 0.383, 0, 0);
//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------
//Test2
    lid_to_cam_rotation = (cv::Mat_<float>(3,3) << 1.8, 0.9, 0,
            0.12, 0.12, -1,
            -0.16, 1, 0.7);

    lid_to_cam_translation = (cv::Mat_<float>(3,1) << 0.2, 0, 0);
//----------------------------------------------------------------------------------------------------------------------
//Class initilization
    lidar_calib_para_kitti_inverse = {
            Rotation:lid_to_cam_rotation,
            Translation:lid_to_cam_translation,
            R:R_self,
            P:P_self,
            imageSize:cv::Size(left_image1.cols, left_image1.rows)
    };

    camera_para = {
            fx:984.2439,
            fy:980.8141,
            cx:690,
            cy:233.1966,
            base:0.54,
            size:cv::Size(left_image1.cols, left_image1.rows)
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
    region11 = (cv::Mat_<int>(2,2) << 111, 300, 1268, 500);
//    region12 = (cv::Mat_<int>(2,2) << 672, 325, 756, 509);
//    region13 = (cv::Mat_<int>(2,2) << 802, 481, 937, 576);
//    region21 = (cv::Mat_<int>(2,2) << 670, 390, 900, 440);
//    region22 = (cv::Mat_<int>(2,2) << 280, 450, 500, 500);
//    region23 = (cv::Mat_<int>(2,2) << 950, 430, 1200, 500);
    diagonal_points_set1.push_back(region11);
//    diagonal_points_set1.push_back(region12);
//    diagonal_points_set1.push_back(region13);
//    diagonal_points_set2.push_back(region21);
//    diagonal_points_set2.push_back(region22);
//    diagonal_points_set2.push_back(region23);
//----------------------------------------------------------------------------------------------------------------------
//CMA-ES setting
    cnt = 0;
    int dim = 6;
    double sigma = 0.06;
    vector<double> test_vec(dim, 0);
    Transfer::mat2VectorSeperate(lid_to_cam_rotation, lid_to_cam_translation, test_vec);
    libcmaes::CMAParameters<> cma_para(test_vec, sigma);

//    cv::FileStorage fs1(depth_map_camera_path1 + "depth_map" + image_name + ".xml", cv::FileStorage::READ);
//    fs1["CameraDepthMap"] >> depth_map_camera1;
//    lidar.projectData(lidar_path1 + cloud_name + ".pcd", depth_map_camera1, depth_map_lidar1, point_cloud_lidar_part, PCD, KITTI, XYZIT, C2L, CV);
//    ImageUtils::colorTransfer(depth_map_lidar1, left_image1);
//    ImageUtils::drawRectSet(left_image1, diagonal_points_set1);
//    cv::imwrite(lidar_image_output_path1 + "start.jpg", left_image1);

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