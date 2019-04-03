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
#include <time.h>
#include <random>

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
string image_name2 = "/image_02/data/0000000082.png";
string image_name3 = "/image_02/data/0000000111.png";
string cloud_name = "/velodyne_points/data/0000000082.pcd";
string cloud_name2 = "/velodyne_points/data/0000000082.pcd";
string cloud_name3 = "/velodyne_points/data/0000000111.pcd";
string depth_name = "/depth/depth1.png";


int step = 1;
int out = 0;

int time1 = int(time(0));
default_random_engine eng(time1);
uniform_int_distribution<unsigned> ur(0, 140);
float degreex = 3;//float(ur(eng)) - 70;
float degreey = 3;//float(ur(eng)) - 70;
float degreez = 0;//float(ur(eng)) - 70;

uniform_int_distribution<unsigned> ut(0, 30);
//float tx = (float(ut(eng)) - 15) / 10;
//float ty = (float(ut(eng)) - 15) / 10;
//float tz = (float(ut(eng)) - 15) / 10;
float tx = 0;
float ty = 0;
float tz = 0;

float scale_translation = 1;
float threshold1 = 45; //150 65
float threshold2 = 5.0; //3.8
float bound_rotation = 0.8;
float bound_translation = 1.0;

float cnt_end = 1;
int threshold_cnt = 0;
auto time_start = chrono::system_clock::now();
auto time_end = chrono::system_clock::now();

double sigma0 = 0.0002;
double sigma3 = 0.025;
double sigma2 = 0.0001;
double sigma1 = 0.3;
int lambda = 20;

int point_cnt = 0;
int point_cnt2 = 0;
int point_cnt3 = 0;
float vox_volum = 1.0;
float original_distance, last_distance, initial_distance;
int feedback = 3;
float p2p_distance;
float edge_distance;
float p2p_distance2;
float edge_distance2;
float p2p_distance3;
float edge_distance3;
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
string lidar_image_output_path = "/home/" + user_name + "/Data/Result/OptimizationProcess/result";
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
cv::Mat left_image2 = cv::imread(data_root + data_name + image_name2);
cv::Mat left_image3 = cv::imread(data_root + data_name + image_name3);
cv::Mat depth_map_camera, depth_map_camera2, depth_map_camera3;
cv::Mat depth_map_lidar, depth_map_lidar2, depth_map_lidar3;
cv::Mat edge_map_camera, edge_map_camera_blured, edge_map_lidar;
cv::Mat edge_map_camera2, edge_map_camera_blured2, edge_map_lidar2;
pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_lidar_part(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_lidar_part2(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_lidar_part3(new pcl::PointCloud<pcl::PointXYZ>);

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
    translationUpdate = translationUpdate * scale_translation;
    lidar.updateParameters(rotationUpdate, translationUpdate);

    left_image = cv::imread(data_root + data_name + image_name);
    left_image2 = cv::imread(data_root + data_name + image_name2);
//    left_image3 = cv::imread(data_root + data_name + image_name3);

    depth_map_camera = cv::imread(data_root + data_name + "/depth/depth1.png", CV_8UC1);
    depth_map_camera2 = cv::imread(data_root + data_name + "/depth/depth2.png", CV_8UC1);
//    depth_map_camera3 = cv::imread(data_root + data_name + "/depth/depth3.png", CV_8UC1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    lidar.projectData(data_root + data_name + cloud_name, depth_map_lidar, temp_cloud1, XYZI, CV);
    point_cloud_lidar_part = temp_cloud1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    lidar.projectData(data_root + data_name + cloud_name2, depth_map_lidar2, temp_cloud2, XYZI, CV);
    point_cloud_lidar_part2 = temp_cloud2;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud3(new pcl::PointCloud<pcl::PointXYZ>);
//    lidar.projectData(data_root + data_name + cloud_name3, depth_map_lidar3, temp_cloud3, XYZI, CV);
//    point_cloud_lidar_part3 = temp_cloud3;
//----------------------------------------------------------------------------------------------------------------------



//----------------------------------------------------------------------------------------------------------------------
//distance type
//map KL Divergence distance
//    float kl_distance1 = HistogramMeasure::mapKLDivergence(depth_map_camera1, depth_map_lidar1, diagonal_points_set1);

//p2p distance
//    float p2p_distance1 = HistogramMeasure::point2PointDistanceTotal(depth_map_camera1, depth_map_lidar1, diagonal_points_set1);
//    float p2p_distance2 = HistogramMeasure::point2PointDistanceTotal(depth_map_camera2, depth_map_lidar2, diagonal_points_set2);

    if(step==1){
        //point cloud distance
        PointCloudAlignment::getCameraSparsePointCloudKitti(depth_map_camera, depth_map_lidar, lidar, point_cloud_camera);

        Transfer::array2Eigen(x, transformation);
        pcl::transformPointCloud (*point_cloud_lidar_part, *transformed_lidar_cloud, transformation);

        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_camera_downsample (new pcl::PointCloud<pcl::PointXYZ> ());
        //pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_camera_downsample2 (new pcl::PointCloud<pcl::PointXYZ> ());
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_lidar_downsample (new pcl::PointCloud<pcl::PointXYZ> ());
        PointCloudAlignment::pointCloudDownsample(point_cloud_camera, point_cloud_camera_downsample, vox_volum);
        //PointCloudAlignment::pointCloudDownsample(point_cloud_camera2, point_cloud_camera_downsample2, vox_volum);
        PointCloudAlignment::pointCloudDownsample(transformed_lidar_cloud, point_cloud_lidar_downsample, vox_volum);


        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_camera_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_lidar_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud (point_cloud_camera_downsample);
        sor.setMeanK (10);
        sor.setStddevMulThresh (1.0);
        sor.filter (*point_cloud_camera_filtered);
        sor.setInputCloud(point_cloud_lidar_downsample);
        sor.filter(*point_cloud_lidar_filtered);

//    pcl::visualization::PCLVisualizer viewer ("test");
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (point_cloud_camera_filtered, 230, 20, 20);
//    viewer.addPointCloud(point_cloud_camera_filtered, source_cloud_color_handler, "transformed_cloud");
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (point_cloud_lidar_filtered, 255, 255, 255);
//    viewer.addPointCloud(point_cloud_lidar_filtered, transformed_cloud_color_handler, "camera_cloud");
//    while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
//        viewer.spinOnce ();
//    }

        original_distance = PointCloudAlignment::chamferDistance(point_cloud_camera_filtered, point_cloud_lidar_filtered);

        if(point_cloud_lidar_part->points.size()==0){
            last_distance = 10000;
        }
        else if(point_cloud_lidar_part->points.size()<10000){
            last_distance = original_distance + float(5500000) / float(point_cloud_lidar_part->points.size() + 1000);
        }
        else if(point_cloud_lidar_part->points.size()<21000){
            last_distance = original_distance + float(100000) / float(point_cloud_lidar_part->points.size() - 9800);
        }
        else{
            last_distance = last_distance +10;
        }
    }

    if(step==2){
        p2p_distance = HistogramMeasure::point2PointDistanceFrame(depth_map_camera, depth_map_lidar);
        p2p_distance2 = HistogramMeasure::point2PointDistanceFrame(depth_map_camera2, depth_map_lidar2);
//        p2p_distance3 = HistogramMeasure::point2PointDistanceFrame(depth_map_camera3, depth_map_lidar3);
        last_distance = p2p_distance + p2p_distance2;// + p2p_distance3;
        point_cnt = point_cloud_lidar_part->points.size();
        if(point_cnt<10000){
            last_distance = last_distance + float(1000000) / float(point_cnt + 3000) - 76;
        }
        else if(point_cnt<21000){
            last_distance = last_distance + float(10000) / float(point_cnt);
        }
        point_cnt2 = point_cloud_lidar_part2->points.size();
        if(point_cnt2<10000){
            last_distance = last_distance + float(1000000) / float(point_cnt2 + 3000) - 76;
        }
        else if(point_cnt2<21000){
            last_distance = last_distance + float(10000) / float(point_cnt2);
        }
//        point_cnt3 = point_cloud_lidar_part3->points.size();
//        if(point_cnt3<10000){
//            last_distance = last_distance + float(1000000) / float(point_cnt3 + 3000) - 76;
//        }
//        else if(point_cnt3<21000){
//            last_distance = last_distance + float(10000) / float(point_cnt3);
//        }
    }

    if(step==3){
        p2p_distance = HistogramMeasure::point2PointDistanceFrame(depth_map_camera, depth_map_lidar);
        //p2p_distance = HistogramMeasure::point2PointDistanceFrame(depth_map_camera2, depth_map_lidar2);

        if(p2p_distance>3.5){ //3.146
            p2p_distance2 = exp(p2p_distance2 - 2);
        }
        else{
            p2p_distance = 0;
        }
//        if(p2p_distance2>4){
//            p2p_distance2 = exp(p2p_distance2 - 2);
//        }

        cv::Point window_size(6, 6);
        cv::Point window_range(0, 40);
        cv::Point window_region(25, 50);

        Refinement::cameraEdgeGeneration(left_image, edge_map_camera, edge_map_camera_blured, 1, 7);
        Refinement::slideElimination2(depth_map_lidar, edge_map_lidar, window_size, window_range, 1.5, 1);
        edge_distance = -Refinement::edgeDistance(edge_map_camera_blured, edge_map_lidar, point_cnt) / 20;

//        Refinement::cameraEdgeGeneration(left_image2, edge_map_camera2, edge_map_camera_blured2, 1, 7);
//        Refinement::slideElimination2(depth_map_lidar2, edge_map_lidar2, window_size, window_range, 1.5, 1);
//        edge_distance2 = -Refinement::edgeDistance(edge_map_camera_blured2, edge_map_lidar2, point_cnt2) / 20;

        last_distance = p2p_distance + edge_distance + p2p_distance2 + edge_distance2;
    }

    for(int i=0; i<translationUpdate.rows; i++){
        if(abs(translationUpdate.at<float>(i,0))>bound_translation){
            last_distance = last_distance + pow(double(translationUpdate.at<float>(i,0)), 10.0);
        }
    }

//----------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------
//final result
    cnt++;
    if(cnt%feedback==0){
        cout <<"iteration " << cnt << endl << "distance is " << endl << last_distance << endl;

        if(step==2){
            cout << "projected point number is:   " << point_cnt << endl;
            cout << "p2p distance:   " << p2p_distance << endl;
            cout << "projected point number2 is:   " << point_cnt2 << endl;
            cout << "p2p distance2:   " << p2p_distance2 << endl;
//            cout << "projected point number3 is:   " << point_cnt3 << endl;
//            cout << "p2p distance3:   " << p2p_distance3 << endl;
        }

        if(step==3){
            cout << "projected point number is:   " << point_cnt << endl;
            cout << "projected point number2 is:   " << point_cnt2 << endl;
            cout << "p2p distance:   " << p2p_distance << "     " << "edge distance:   " << edge_distance << endl;
            cout << "p2p distance2:   " << p2p_distance2 << "     " << "edge distance2:   " << edge_distance2 << endl;
            Refinement::saveMatchResult(edge_map_camera_blured, edge_map_lidar, lidar_image_output_path, cnt/feedback);
            ImageUtils::colorTransfer(depth_map_lidar, left_image, 70);
            cv::imwrite(lidar_image_output_path + to_string(cnt/feedback) + "___" + to_string(last_distance) + ".png", left_image);

//            Refinement::saveMatchResult(edge_map_camera_blured2, edge_map_lidar2, lidar_image_output_path2, cnt/feedback);
//            ImageUtils::colorTransfer(depth_map_lidar2, left_image2, 70);
//            cv::imwrite(lidar_image_output_path2 + to_string(cnt/feedback) + "___" + to_string(last_distance) + ".png", left_image2);
        }
        else{
            ImageUtils::colorTransfer(depth_map_lidar, left_image, 70);
            cv::imwrite(lidar_image_output_path + to_string(cnt/feedback) + "___" + to_string(last_distance) + ".png", left_image);
        }
        cout << point_cloud_lidar_part->points.size();
        cout << "calibration result is " << endl << rotationUpdate << endl << translationUpdate << endl;
        //lidar.projectData(data_root + data_name + cloud_name, depth_map_lidar1, point_cloud_lidar_part, XYZI, CV);


    }

    if(step==1) {
        if (last_distance < threshold1) {
            threshold_cnt++;
        }
    }
    else if(step==2) {
        if (last_distance < threshold2) {
            threshold_cnt++;
        }
    }

    if(threshold_cnt==cnt_end){
        time_end = chrono::system_clock::now();
        auto duration = chrono::duration_cast<chrono::microseconds>(time_end - time_start);
        double time_cost = double(duration.count()) * chrono::microseconds::period::num / chrono::microseconds::period::den;
        cout << "Generation is:   " << cnt << endl << "time is:   " << time_cost << endl;
        //lidar.projectData(data_root + data_name + cloud_name, depth_map_lidar1, point_cloud_lidar_part, XYZI, CV);
        cout << "calibration result is " << endl << rotationUpdate << endl << translationUpdate << endl;
        cout <<"iteration " << cnt << endl << "originial distance is " << endl << original_distance << endl << "last distance is " << endl << last_distance << endl;
        cout << "points number is:   " << endl << point_cloud_lidar_part->points.size() << endl;
        ImageUtils::colorTransfer(depth_map_lidar, left_image, 70);
        cv::imwrite(lidar_image_output_path + "end" + "___" + to_string(last_distance) + ".png", left_image);
        if(out==1) {
            ofstream outFile1(data_root + "output.txt", ios::app);
            ofstream outFile2(data_root + "output2.txt", ios::app);
            if (step == 1) {
                outFile1 << "Rotation:   " << endl << rotationUpdate << endl << "translation:   " << endl
                         << translationUpdate << endl << "last distance:   " << last_distance << endl
                         << "Generation:   " << cnt << endl << "Time:   " << time_cost << endl << endl << endl;
                outFile1.close();
            }
            else if (step == 2) {
                outFile2 << "Rotation:   " << endl << rotationUpdate << endl << "translation:   " << endl
                         << translationUpdate << endl << "last distance:   " << last_distance << endl
                         <<"Generation:   " << cnt << endl << "Time:   " << time_cost<< endl << endl << endl;
                outFile2.close();
            }
        }
        exit(0);
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
            5.251945e-03, 9.999804e-01, -3.413835e-03, 0,
            2.827154e-03, 9.766976e-04, 9.999955e-01, 0,
            0, 0, 0, 1);

    P_self = (cv::Mat_<float>(3,4) << 7.215377e+02, 0.000000e+00, 6.095593e+02, 0,//4.485728e+01
            0.000000e+00, 7.215377e+02, 1.728540e+02, 0,//2.163791e-01
            0.000000e+00, 0.000000e+00, 1.000000e+00, 0);//2.745884e-03

//2011_09_26_drive_0048_sync extrinsic calibration
    lid_to_cam_rotation = (cv::Mat_<float>(3,3) << 7.533745e-03, -9.999714e-01, -6.166020e-04,
            1.480249e-02, 7.280733e-04, -9.998902e-01,
            9.998621e-01, 7.523790e-03, 1.480755e-02);
    lid_to_cam_translation = (cv::Mat_<float>(3,1) << -4.069766e-03 + tx, -7.631618e-02 + ty, -2.717806e-01 + tz);
//    lid_to_cam_rotation = (cv::Mat_<float>(3,3) << 0.0087676989, -0.99974012, -0.021042559,
//    0.0093115233, 0.02112408, -0.99973351,
//    0.99991822, 0.008569424, 0.009494313);
//    lid_to_cam_translation = (cv::Mat_<float>(3,1) << -0.022104684, -0.056908015, -0.22559986);

//    lid_to_cam_rotation = (cv::Mat_<float>(3,3) << 0.50745404, -0.85703075, 0.089379527,
//    0.056017749, -0.07069613, -0.9959237,
//    0.85985601, 0.51039237, 0.012133894);
//    lid_to_cam_translation = (cv::Mat_<float>(3,1) << -0.17372364, 1.4716092, 1.2500213);

    cv::Mat cam0_to_cam2_rotation, cam0_to_cam2_translation;
    cam0_to_cam2_rotation = (cv::Mat_<float>(3,3) << 9.999758e-01, -5.267463e-03, -4.552439e-03,
            5.251945e-03, 9.999804e-01, -3.413835e-03,
            4.570332e-03, 3.389843e-03, 9.999838e-01);
    cam0_to_cam2_translation = (cv::Mat_<float>(3,1) << 5.956621e-02, 2.900141e-04, 2.577209e-03);

    lid_to_cam_translation = lid_to_cam_translation / scale_translation;

    lid_to_cam_rotation = cam0_to_cam2_rotation * lid_to_cam_rotation;
    lid_to_cam_translation = cam0_to_cam2_rotation * lid_to_cam_translation + cam0_to_cam2_translation;

    if(step==1){
        cv::Mat axisx, axisy, axisz;
        axisx = (cv::Mat_<float>(3,3) << 1, 0.0, 0.0,
                0.0, cos(M_PI/180 * degreex), -sin(M_PI/180 * degreex),
                0.0, sin(M_PI/180 * degreex), cos(M_PI/180 * degreex));

        axisy = (cv::Mat_<float>(3,3) << cos(M_PI/180 * degreey), 0.0, -sin(M_PI/180 * degreey),
                0.0, 1, 0.0,
                sin(M_PI/180 * degreey), 0.0, cos(M_PI/180 * degreey));

        axisz = (cv::Mat_<float>(3,3) << cos(M_PI/180 * degreez), -sin(M_PI/180 * degreez), 0.0,
                sin(M_PI/180 * degreez), cos(M_PI/180 * degreez), 0.0,
                0.0, 0.0, 1.0);
        lid_to_cam_rotation = axisz * axisy * axisx * lid_to_cam_rotation;
    }

    Transfer::cv2EigenSeperate(lid_to_cam_rotation, lid_to_cam_translation, transformation);
    cout << lid_to_cam_rotation << endl << lid_to_cam_translation << endl << transformation;


    if(out==1) {
        ofstream outFile1(data_root + "output.txt", ios::app);
        ofstream outFile2(data_root + "output2.txt", ios::app);
        if (step == 1) {
            outFile1 << "Initial rotation:   " << endl << lid_to_cam_rotation << endl << "Initial translation:   " << endl
                     << lid_to_cam_translation << endl << endl;
            outFile1.close();
        }
        else if (step == 2) {
            outFile2 << "Initial rotation:   " << endl << lid_to_cam_rotation << endl << "translation:   " << endl
                     << lid_to_cam_translation << endl << endl;
            outFile2.close();
        }
    }

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
    //region11 = (cv::Mat_<int>(2,2) << 0, 0, 1240, 370);
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
    vector<double> test_vec(dim, 0);
    double lower_bound [dim], upper_bound [dim];
    for(int i=0; i<dim; i++){
        if(i<3){
            lower_bound[i] = -bound_rotation;
            upper_bound[i] = bound_rotation;
        }
        else{
            lower_bound[i] = -bound_translation;
            upper_bound[i] = bound_translation;
        }
    }
    Transfer::mat2VectorSeperate(lid_to_cam_rotation, lid_to_cam_translation, test_vec);
    libcmaes::CMAParameters<> cma_para;
    if(step==1){
        libcmaes::CMAParameters<> cma_para1(test_vec, sigma1, lambda);
        cma_para = cma_para1;
    }
    else if(step==2){
        libcmaes::CMAParameters<> cma_para2(test_vec, sigma2, lambda);
        cma_para = cma_para2;
    }
    else if(step==3){
        libcmaes::CMAParameters<> cma_para3(test_vec, sigma3, lambda);
        cma_para = cma_para3;
    }
//    cma_para.set_x0(lower_bound, upper_bound);
//    libcmaes::GenoPheno<libcmaes::pwqBoundStrategy> gp(lower_bound, upper_bound, dim);
    cma_para.set_fplot(data_root + "output.dat");


//    cv::FileStorage fs1(depth_map_camera_path1 + "depth_map" + image_name + ".xml", cv::FileStorage::READ);
//    fs1["CameraDepthMap"] >> depth_map_camera1;
    left_image = cv::imread(data_root + data_name + image_name);
    depth_map_camera = cv::imread(data_root + data_name + "/depth/depth1.png", CV_8UC1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    lidar.projectData(data_root + data_name + cloud_name, depth_map_lidar, temp_cloud2, XYZI, CV);
    point_cloud_lidar_part = temp_cloud2;
    cv::Point size(3, 3);
    cv::Mat dyed;
    ImageUtils::neighborDyeing(depth_map_lidar, size, dyed);
    ImageUtils::colorTransfer(dyed, left_image, 70);
    //ImageUtils::drawRectSet(left_image, diagonal_points_set1);
    cv::imwrite(lidar_image_output_path + "_start.png", left_image);


    if(step==1){
        //point cloud distance
        PointCloudAlignment::getCameraSparsePointCloudKitti(depth_map_camera, depth_map_lidar, lidar, point_cloud_camera);

        Transfer::vector2Eigen(test_vec, transformation);
        pcl::transformPointCloud (*point_cloud_lidar_part, *transformed_lidar_cloud, transformation);

        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_camera_downsample (new pcl::PointCloud<pcl::PointXYZ> ());
        //pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_camera_downsample2 (new pcl::PointCloud<pcl::PointXYZ> ());
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_lidar_downsample (new pcl::PointCloud<pcl::PointXYZ> ());
        PointCloudAlignment::pointCloudDownsample(point_cloud_camera, point_cloud_camera_downsample, vox_volum);
        //PointCloudAlignment::pointCloudDownsample(point_cloud_camera2, point_cloud_camera_downsample2, vox_volum);
        PointCloudAlignment::pointCloudDownsample(transformed_lidar_cloud, point_cloud_lidar_downsample, vox_volum);


        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_camera_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_lidar_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud (point_cloud_camera_downsample);
        sor.setMeanK (10);
        sor.setStddevMulThresh (1.0);
        sor.filter (*point_cloud_camera_filtered);
        sor.setInputCloud(point_cloud_lidar_downsample);
        sor.filter(*point_cloud_lidar_filtered);

//        pcl::visualization::PCLVisualizer viewer ("test");
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (point_cloud_camera, 230, 20, 20);
//        viewer.addPointCloud(point_cloud_camera, source_cloud_color_handler, "transformed_cloud");
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (transformed_lidar_cloud, 255, 255, 255);
//        viewer.addPointCloud(transformed_lidar_cloud, transformed_cloud_color_handler, "camera_cloud");
//        while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
//            viewer.spinOnce ();
//        }

        if(point_cloud_lidar_part->points.size()<100){
            initial_distance = 10000;
        }
        else{
            initial_distance = PointCloudAlignment::chamferDistance(point_cloud_camera_filtered, point_cloud_lidar_filtered);
        }
    }

    if(step==2){
        //last_distance = HistogramMeasure::point2PointDistanceTotal(depth_map_camera1, depth_map_lidar1, diagonal_points_set1);
        initial_distance = HistogramMeasure::point2PointDistanceFrame(depth_map_camera, depth_map_lidar);
    }

    if(step==3){
        p2p_distance = HistogramMeasure::point2PointDistanceFrame(depth_map_camera, depth_map_lidar);

        if(p2p_distance>3.146){
            p2p_distance = exp(p2p_distance - 2);
        }
        Refinement::cameraEdgeGeneration(left_image, edge_map_camera, edge_map_camera_blured, 1, 5);
        cv::Point window_size(6, 4);
        cv::Point window_range(0, 40);
        cv::Point window_region(25, 50);

        Refinement::slideElimination2(depth_map_lidar, edge_map_lidar, window_size, window_range, 1.5, 1);
        edge_distance = -Refinement::edgeDistance(edge_map_camera_blured, edge_map_lidar, point_cnt) / 20;
        initial_distance = p2p_distance + edge_distance;
    }
    cout << "initial distance is:   " << initial_distance << endl;
    p2p_distance = 0;
    edge_distance = 0;

    cma_para.set_algo(BIPOP_CMAES);
    libcmaes::CMASolutions cmasols = libcmaes::cmaes<>(KL_divergence, cma_para);
    cout << "best solution: " << cmasols << endl;
    cout << "optimization took " << cmasols.elapsed_time() / 1000.0 << " seconds\n";
    time_start = chrono::system_clock::now();
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