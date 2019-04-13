//
// Created by phyorch on 26/02/19.
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
string cloud_name = "/velodyne_points/data/0000000082.pcd";
string depth_name = "/depth/0000000082.png";

string image_name2 = "/image_02/data/0000000132.png";
string cloud_name2 = "/velodyne_points/data/0000000132.pcd";
string depth_name2 = "/depth/0000000132.png";

string image_name3 = "/image_02/data/0000000111.png";
string cloud_name3 = "/velodyne_points/data/0000000111.pcd";
string depth_name3 = "/depth/depth3.png";

string image_name4 = "/image_02/data/0000000001.png";
string cloud_name4 = "/velodyne_points/data/0000000001.pcd";
string depth_name4 = "/depth/depth4.png";

string image_name5 = "/image_02/data/0000000153.png";
string cloud_name5 = "/velodyne_points/data/0000000153.pcd";
string depth_name5 = "/depth/depth5.png";

string image_name6 = "/image_02/data/0000000100.png";
string cloud_name6 = "/velodyne_points/data/0000000100.pcd";
string depth_name6 = "/depth/depth6.png";


int step = 3;
int cnt_data = 6;
int out = 0;
int error_write = 1;

string image[6] = {image_name, image_name2, image_name3, image_name4, image_name5, image_name6};
string cloud[6] = {cloud_name, cloud_name2, cloud_name3, cloud_name4, cloud_name5, cloud_name6};
string depth[6] = {depth_name, depth_name2, depth_name3, depth_name4, depth_name5, depth_name6};

float p2p[6] = {0, 0, 0, 0, 0, 0};
float edge[6] = {0, 0, 0, 0, 0, 0};
vector<cv::Mat> map;

int time1 = int(time(0));
default_random_engine eng(time1);
uniform_int_distribution<unsigned> ur(0, 140);
float degreex = 0;//float(ur(eng)) - 70;
float degreey = 0;//float(ur(eng)) - 70;
float degreez = 0;//float(ur(eng)) - 70;

uniform_int_distribution<unsigned> ut(0, 30);
//float tx = (float(ut(eng)) - 15) / 10;
//float ty = (float(ut(eng)) - 15) / 10;
//float tz = (float(ut(eng)) - 15) / 10;
float tx = 0;
float ty = 0;
float tz = 0;

float scale_translation = 4; //4
float threshold1 = 45; //150 65
float threshold2 = 1.25;
float threshold3 = -100;
float bound_rotation = 0.8;
float bound_translation = 1.0;

float cnt_end = 1;
int threshold_cnt = 0;
auto time_start = chrono::system_clock::now();
auto time_end = chrono::system_clock::now();

ofstream out_file_error;
ofstream out_file_distance;


double sigma0 = 0.0002;
double sigma3 = 0.025;
double sigma2 = 0.08;
double sigma1 = 0.3;
int lambda = 30;

float point_cnt = 0;
float point_cnt2 = 0;
float vox_volum = 1.0;
float original_distance, last_distance, initial_distance, reference_distance;
int feedback = 8;
float p2p_distance;
float edge_distance;
float p2p_distance2;
float edge_distance2;
float p2p_distance3;
float edge_distance3;
float p2p_distance4;
float edge_distance4;
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
cv::Mat depth_map_camera, depth_map_camera_boader, depth_map_camera2, depth_map_camera_boader2;
cv::Mat depth_map_lidar, depth_map_lidar_boader, depth_map_lidar2, depth_map_lidar_boader2;
cv::Mat edge_map_camera, edge_map_camera_blured, edge_map_lidar;
cv::Mat edge_map_camera2, edge_map_camera_blured2, edge_map_lidar2;
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

cv::Mat lid_to_cam_rotation_truth;
cv::Mat lid_to_cam_translation_truth;


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


    //left_image2 = cv::imread(data_root + data_name + image_name2);


    //depth_map_camera2 = cv::imread(data_root + data_name + "/depth/depth2.png", CV_8UC1);



//    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
//    lidar.projectData(data_root + data_name + cloud_name2, depth_map_lidar2, temp_cloud2, XYZI, CV);
//    point_cloud_lidar_part = temp_cloud2;
//----------------------------------------------------------------------------------------------------------------------

    cv::Point window_size(6, 6);
    cv::Point window_range(0, 40);
    cv::Point window_region(25, 50);
    last_distance = 0;
    reference_distance = 0;
    for(int i=0; i<cnt_data; i++){
        left_image = cv::imread(data_root + data_name + image[i]);
        depth_map_camera = cv::imread(data_root + data_name + depth[i], CV_8UC1);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud1(new pcl::PointCloud<pcl::PointXYZ>);
        lidar.projectData(data_root + data_name + cloud[i], depth_map_lidar, temp_cloud1, XYZI, CV);
        //point_cloud_lidar_part = temp_cloud1;

        p2p_distance = 0;
        edge_distance = 0;
        if(i==0){
            p2p_distance = HistogramMeasure::point2PointDistanceFrame(depth_map_camera, depth_map_lidar);
        }
        if(p2p_distance>3.5){ //3.146
            p2p_distance2 = exp(p2p_distance2 - 2);
        }

        Refinement::cameraEdgeGeneration(left_image, edge_map_camera, edge_map_camera_blured, 1, 7);
        Refinement::slideElimination2(depth_map_lidar, edge_map_lidar, window_size, window_range, 1.5, 1);
        edge_distance = -Refinement::edgeDistance(edge_map_camera_blured, edge_map_lidar, point_cnt) / 20;

        last_distance = last_distance + edge_distance;
        reference_distance = reference_distance + edge_distance + p2p_distance;
        p2p[i] = p2p_distance;
        edge[i] = edge_distance;
    }

//----------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------
//final result
    cnt++;
    if(last_distance<threshold3){
        threshold_cnt++;
    }
    if(cnt%feedback==0){
        cout <<"iteration " << cnt << endl << "distance is " << endl << last_distance << endl;

        time_end = chrono::system_clock::now();
        auto duration = chrono::duration_cast<chrono::microseconds>(time_end - time_start);
        double time_cost = double(duration.count()) * chrono::microseconds::period::num / chrono::microseconds::period::den;
        cout << "time cost is:   " << time_cost << endl;
        if(error_write==1){
            float error_rotation = Refinement::errorRotation(rotationUpdate, lid_to_cam_rotation_truth);
            float error_translation = Refinement::errorTranslation(translationUpdate, lid_to_cam_translation_truth);
            Refinement::errorWrite(out_file_error, time_cost, error_rotation, error_translation);
            Refinement::distanceErrorWrite(out_file_distance, time_cost, last_distance, reference_distance, error_rotation, error_translation);
        }

        if(step==3){
            for(int i=0; i<cnt_data; i++){
                cout << "p2p distance" << to_string(i) << ":   "  << p2p[i] << "     " << "edge distance" << to_string(i) << ":   "  << edge[i] << endl;
            }
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
        cout << "calibration result is " << endl << rotationUpdate << endl << translationUpdate << endl;
        //lidar.projectData(data_root + data_name + cloud_name, depth_map_lidar1, point_cloud_lidar_part, XYZI, CV);


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
//    lid_to_cam_rotation = (cv::Mat_<float>(3,3) << 7.533745e-03, -9.999714e-01, -6.166020e-04,
//            1.480249e-02, 7.280733e-04, -9.998902e-01,
//            9.998621e-01, 7.523790e-03, 1.480755e-02);
//    lid_to_cam_translation = (cv::Mat_<float>(3,1) << -4.069766e-03 + tx, -7.631618e-02 + ty, -2.717806e-01 + tz);
    lid_to_cam_rotation = (cv::Mat_<float>(3,3) << 0.055720564, -0.99817532, -0.023264745,
    -0.0065545961, 0.02293475, -0.99971545,
    0.99842489, 0.0558572, -0.005264699);
    lid_to_cam_translation = (cv::Mat_<float>(3,1) << -0.44149587, 0.47857293, -0.39773074);
//    lid_to_cam_rotation = (cv::Mat_<float>(3,3) << 0.012126432, -0.99951559, -0.02866287,
//    0.019851571, 0.028899975, -0.99938518,
//    0.99972939, 0.011549973, 0.020192407);
//    lid_to_cam_translation = (cv::Mat_<float>(3,1) << -0.28040487, 0.14834355, -0.47335377);

    cv::Mat cam0_to_cam2_rotation, cam0_to_cam2_translation;
    cam0_to_cam2_rotation = (cv::Mat_<float>(3,3) << 9.999758e-01, -5.267463e-03, -4.552439e-03,
            5.251945e-03, 9.999804e-01, -3.413835e-03,
            4.570332e-03, 3.389843e-03, 9.999838e-01);
    cam0_to_cam2_translation = (cv::Mat_<float>(3,1) << 5.956621e-02, 2.900141e-04, 2.577209e-03);
    lid_to_cam_rotation = cam0_to_cam2_rotation * lid_to_cam_rotation;
    lid_to_cam_translation = cam0_to_cam2_rotation * lid_to_cam_translation + cam0_to_cam2_translation;

    lid_to_cam_translation = lid_to_cam_translation / scale_translation;

    lid_to_cam_rotation_truth = (cv::Mat_<float>(3,3) << 7.533745e-03, -9.999714e-01, -6.166020e-04,
            1.480249e-02, 7.280733e-04, -9.998902e-01,
            9.998621e-01, 7.523790e-03, 1.480755e-02);
    lid_to_cam_translation_truth = (cv::Mat_<float>(3,1) << -4.069766e-03, -7.631618e-02, -2.717806e-01);

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

    out_file_error.open(data_root + "error.csv", ios::out);
    out_file_error << "time" << ";" << "rotation" << ";" << "translation" << endl;
    out_file_distance.open(data_root + "distance.csv", ios::out);
    out_file_distance << "time" << ";" << "last_distance" << ";" << "reference_distance" << ";" << "error" << endl;

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