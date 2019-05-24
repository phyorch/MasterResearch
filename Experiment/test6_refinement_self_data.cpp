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

#define HAVE_GLOG

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
string data_sequence = "2011_09_26_drive_0005_sync";
string image_address = "/image_02/data/";
string cloud_address = "/velodyne_points/data/";
string depth_address = "/depth/";

string data_order[6] = {"0000000082", "0000000132", "0000000111", "0000000001", "0000000153", "0000000100"};


float p2p[6] = {0, 0, 0, 0, 0, 0};
float edge[6] = {0, 0, 0, 0, 0, 0};

int time1 = int(time(0));
default_random_engine eng(time1);

//----------------------------------------------------------------------------------------------------------------------
// Parameters for system
int cnt_data = 2; // how much data used in the optimization
int out = 0; //whether output the optimizing parameters
int error_write = 1; //whether output the error of the optimizing parameters
int feedback = 3;
int feedimage = 3;
ofstream out_file_error;
ofstream out_file_distance;
auto time_start = chrono::system_clock::now();
auto time_end = chrono::system_clock::now();
//----------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------
// Parameters for extrinsic parameters
int deviation_rotation = 0;
uniform_int_distribution<unsigned> ur(0, 140);
float degreex = 0;//float(ur(eng)) - 70;
float degreey = 0;//float(ur(eng)) - 70;
float degreez = 0;//float(ur(eng)) - 70;

int deviation_translation = 0;
uniform_int_distribution<unsigned> ut(0, 30);
//float tx = (float(ut(eng)) - 15) / 10;
//float ty = (float(ut(eng)) - 15) / 10;
//float tz = (float(ut(eng)) - 15) / 10;
float tx = 0;
float ty = 0;
float tz = 0;

float scale_translation = 4; //4
float bound_rotation = 0.8; // for rotation vector
float bound_translation = 1.0; //in meters
//----------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------
// Parameters for optimization
double sigma0 = 0.0002;
double sigma3 = 0.025;
double sigma2 = 0.08;
double sigma1 = 0.3;
int lambda = 30;
//----------------------------------------------------------------------------------------------------------------------


float cnt_end = 1;
int threshold_cnt = 0;



float point_cnt = 0;
float point_cnt2 = 0;
float vox_volum = 1.0;
float original_distance, last_distance, initial_distance, reference_distance;
float p2p_distance;
float edge_distance;
//----------------------------------------------------------------------------------------------------------------------
//Notes
//1. There are several nesting method for the part of ProjectData, and the corresponding point cloud for projection
//we need to change the method determined by convenience


string data_root = "/home/" + user_name + "/Data/";
string lidar_image_output_path = "/home/" + user_name + "/Data/Result/OptimizationProcess/result";
string lidar_image_output_path2 = "/home/" + user_name + "/Data/Result/OptimizationProcess/2result";
string depth_map_camera_boader_path = "/home/" + user_name + "/Data/camera_depth_boader.jpg";

string test1_path = "/home/" + user_name + "/Data/test1.png";
string test2_path = "/home/" + user_name + "/Data/test2.jpg";

cv::Mat left_image = cv::imread(data_root + data_sequence + image_address + data_order[0] + ".png");

cv::Mat depth_map_camera, depth_map_camera2;
cv::Mat depth_map_lidar, depth_map_lidar2;
cv::Mat edge_map_camera, edge_map_camera_blured, edge_map_lidar, edge_map_camera_lidar;
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

LiDARCalibParaKitti lidar_calib_para_kitti;

CameraPara camera_para;

LiDAR lidar = LiDAR(lidar_calib_para_kitti);

vector<vector<int>> region_point_set1, region_point_set2;

vector<cv::Mat> diagonal_points_set1, diagonal_points_set2;

pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_camera(new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_lidar_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

int cnt = 0;

cv::Mat rotationUpdate, translationUpdate;

libcmaes::FitFunc KL_divergence = [](const double *x, const double N){


//----------------------------------------------------------------------------------------------------------------------
//Data preparation

    Transfer::array2MatSeperate(x, rotationUpdate, translationUpdate);
    translationUpdate = translationUpdate * scale_translation;
    lidar.updateParameters(rotationUpdate, translationUpdate);
//----------------------------------------------------------------------------------------------------------------------

    cv::Point window_size(6, 4);
    cv::Point window_range(0, 40);
    last_distance = 0;
    reference_distance = 0;
    for(int i=0; i<cnt_data; i++){
        p2p_distance = 0;
        edge_distance = 0;
        left_image = cv::imread(data_root + data_sequence + image_address + data_order[i] + ".png");
        depth_map_camera = cv::imread(data_root + data_sequence + depth_address + data_order[i] + ".png", CV_8UC1);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud1(new pcl::PointCloud<pcl::PointXYZ>);
        lidar.projectData(data_root + data_sequence + cloud_address + data_order[i] + ".pcd", depth_map_lidar, temp_cloud1, XYZIT, CV, 0);

        if(i==0){
            p2p_distance = Refinement::point2PointDistanceFrame(depth_map_camera, depth_map_lidar);
        }
        if(p2p_distance>3.5){ //3.146
            p2p_distance = exp(p2p_distance - 2);
        }

        Refinement::cameraEdgeGeneration(left_image, edge_map_camera, edge_map_camera_blured, 1, 7);
        Refinement::slideElimination2(depth_map_lidar, edge_map_lidar, window_size, window_range, 1.5, 1);
        edge_distance = -Refinement::edgeDistance(edge_map_camera_blured, edge_map_lidar, point_cnt) / 20;

        last_distance = last_distance + edge_distance + p2p_distance;
        reference_distance = reference_distance + edge_distance;
        p2p[i] = p2p_distance;
        edge[i] = edge_distance;
    }
//----------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------
//final result
    cnt++;
    if(cnt%feedback==0){
        cout <<"iteration " << cnt << endl << "distance is " << endl << last_distance << endl;
        for(int i=0; i<cnt_data; i++){
            cout << "p2p distance" << to_string(i) << ":   "  << p2p[i] << "     " << "edge distance" << to_string(i) << ":   "  << edge[i] << endl;
        }
        cout << "calibration result is " << endl << rotationUpdate << endl << translationUpdate << endl;

        time_end = chrono::system_clock::now();
        auto duration = chrono::duration_cast<chrono::microseconds>(time_end - time_start);
        double time_cost = double(duration.count()) * chrono::microseconds::period::num / chrono::microseconds::period::den;
        cout << "time cost is:   " << time_cost << endl;
        if(error_write==1){
            cv::Point3f error_rotation, error_translation;
            Refinement::errorRotation(rotationUpdate, lid_to_cam_rotation_truth, error_rotation);
            Refinement::errorTranslation(translationUpdate, lid_to_cam_translation_truth, error_translation);
            Refinement::errorWrite(out_file_error, time_cost, error_rotation, error_translation);
        }

        if(cnt%feedimage==0){
            Refinement::saveMatchResult(edge_map_camera_blured, edge_map_lidar, lidar_image_output_path, cnt/feedimage, edge_map_camera_lidar);
            ImageUtils::colorTransfer(depth_map_lidar, left_image, 70);
            cv::imwrite(lidar_image_output_path + to_string(cnt/feedimage) + "___" + to_string(last_distance) + ".png", left_image);
        }
    }
    return last_distance;
//----------------------------------------------------------------------------------------------------------------------
};


int main(){

//----------------------------------------------------------------------------------------------------------------------
//2019_01_15 intrinsic calibration
    R_self = (cv::Mat_<float>(4,4) << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1);

    P_self = (cv::Mat_<float>(3,4) << 6.7440533447265625e+02, 0., 6.7320996093750000e+02, 0,//4.485728e+01
            0., 6.7440533447265625e+02, 3.8029739379882812e+02, 0,//2.163791e-01
            0.000000e+00, 0.000000e+00, 1.000000e+00, 0);//2.745884e-03

//2019_01_15 extrinsic calibration
    lid_to_cam_rotation = (cv::Mat_<float>(3,3) << 1, 0, 0,
    0, 0, -1,
    0, 1, 0);
    lid_to_cam_translation = (cv::Mat_<float>(3,1) << 0.383, 0, 0);
    transformation << 1, 0, 0, 0.383, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1;

    lid_to_cam_rotation = (cv::Mat_<float>(3,3) << -6.8836088830724711e-01, 1.5212676276466525e-01,
            -4.5001694725983754e-04,
            1.1326651711791161e-02, 7.0638167047866629e-02,
            -9.9743769545627137e-01,
            -1.5170517929691568e-01, -9.8583350389751967e-01,
            -7.1539088389702687e-02);
    lid_to_cam_translation = (cv::Mat_<float>(3,1) << 1.5184318319534851e-01, -7.3682146064973197e-02, -1.7484112138873420e-01);
    transformation << 0.87365133, 0.11061831, -0.47381106, -0.69790131, -0.43071392, -0.27713022, -0.85888553, -0.078251913, -0.22631583, 0.95444351, -0.19447035, 0.27923283, 0, 0, 0, 1;
//----------------------------------------------------------------------------------------------------------------------

    if(out==1) {
        ofstream outFile1(data_root + "output.txt", ios::app);
        outFile1 << "Initial rotation:   " << endl << lid_to_cam_rotation << endl << "Initial translation:   " << endl
                 << lid_to_cam_translation << endl << endl;
        outFile1.close();
    }

//----------------------------------------------------------------------------------------------------------------------


//Class initilization
    lidar_calib_para_kitti = {
            Rotation:lid_to_cam_rotation,
            Translation:lid_to_cam_translation,
            R:R_self,
            P:P_self,
            imageSize:cv::Size(left_image.cols, left_image.rows)
    };
    lidar = LiDAR(lidar_calib_para_kitti);

//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------
//Fixed comparision region if using region to calculate p2p distance
    cv::Mat region11, region12, region13, region21, region22, region23;
    //for p2p distance
    region11 = (cv::Mat_<int>(2,2) << 258, 162, 479, 326);
    region12 = (cv::Mat_<int>(2,2) << 475, 164, 564, 216);
    region13 = (cv::Mat_<int>(2,2) << 654, 169, 752, 241);
    diagonal_points_set1.push_back(region11);
    diagonal_points_set1.push_back(region12);
    diagonal_points_set1.push_back(region13);
//----------------------------------------------------------------------------------------------------------------------
//CMA-ES setting
    cnt = 0;
    int dim = 6;
    vector<double> test_vec(dim, 0);
    Transfer::mat2VectorSeperate(lid_to_cam_rotation, lid_to_cam_translation, test_vec);
    libcmaes::CMAParameters<> cma_para(test_vec, sigma3, lambda);

    //initial error
    left_image = cv::imread(data_root + data_sequence + image_address + data_order[0] + ".png");
    depth_map_camera = cv::imread(data_root + data_sequence + depth_address + data_order[0] + ".png", CV_8UC1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    lidar.projectData(data_root + data_sequence + cloud_address + data_order[0] + ".pcd", depth_map_lidar, temp_cloud2, XYZIT, CV, 0);
    point_cloud_lidar_part = temp_cloud2;
    cv::Point size(3, 3);
    cv::Mat dyed;
    ImageUtils::neighborDyeing(depth_map_lidar, size, dyed);
    ImageUtils::colorTransfer(dyed, left_image, 70);
    cv::imwrite(test1_path, left_image);
    p2p_distance = Refinement::point2PointDistanceFrame(depth_map_camera, depth_map_lidar);
    if(p2p_distance>3.146){
        p2p_distance = exp(p2p_distance - 2);
    }
    Refinement::cameraEdgeGeneration(left_image, edge_map_camera, edge_map_camera_blured, 1, 5);
    cv::Point window_size(6, 4);
    cv::Point window_range(0, 40);
    Refinement::slideElimination2(depth_map_lidar, edge_map_lidar, window_size, window_range, 1.5, 1);
    edge_distance = -Refinement::edgeDistance(edge_map_camera_blured, edge_map_lidar, point_cnt) / 20;
    initial_distance = p2p_distance + edge_distance;
    cout << "initial distance is:   " << initial_distance << endl;
    p2p_distance = 0;
    edge_distance = 0;
    out_file_error.open(data_root + "error.csv", ios::out);
    out_file_error << "time" << ";" << "pitch" << ";" << "yaw" << ";" << "roll" << ";" << "x" << ";" << "y" << ";" << "z" << endl;

    cma_para.set_algo(BIPOP_CMAES);
    libcmaes::CMASolutions cmasols = libcmaes::cmaes<>(KL_divergence, cma_para);
    cout << "best solution: " << cmasols << endl;
    cout << "optimization took " << cmasols.elapsed_time() / 1000.0 << " seconds\n";
    time_start = chrono::system_clock::now();
    return cmasols.run_status();
    return 0;
}