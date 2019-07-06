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
#include <ceres/ceres.h>

#include "Sensor.h"
#include "Calibration.h"

using namespace std;
string user_name = "phyorch";
string image_location = "2011_09_26_drive_0005_sync/image_02/data/";
string depth_location = "2011_09_26_drive_0005_sync/depth/";
string cloud_location = "2011_09_26_drive_0005_sync/velodyne_points/data/";
ofstream out_file_candidates;
string image_name = "0000000082.png";

string data_root = "/home/" + user_name + "/Data/";
string depth_name = "/depth/0000000082.png";

Eigen::Matrix3d Rotation;
Eigen::Vector3d translation;

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

    LiDARCalibParaKitti lidar_calib_para_kitti;

    cv::Mat img_1 = cv::imread ( data_root + image_location + image_name, CV_LOAD_IMAGE_COLOR );
    lidar_calib_para_kitti = {
            Rotation:lid_to_cam_rotation,
            Translation:lid_to_cam_translation,
            R:R_self,
            P:P_self,
            imageSize:cv::Size(img_1.cols, img_1.rows)
    };
    LiDAR lidar = LiDAR(lidar_calib_para_kitti);

    cv::Mat K;
    cv::Mat transformation_camera;
    cv::Mat transformation_lidar;
    K = ( cv::Mat_<float> ( 3,3 ) << 7.215377e+02, 0.000000e+00, 6.095593e+02,
            0.000000e+00, 7.215377e+02, 1.728540e+02,
            0.000000e+00, 0.000000e+00, 1.000000e+00);


    //cout<<"calling bundle adjustment"<<endl;
    //bundleAdjustment ( pts_3d, pts_2d, K, R, t );

    int begin = 82;//1
    int end = 83;//152
    int data_amount = 1;//10
    int data_interval = 1;
    int result_amount = 20;//20
    vector<Eigen::Vector3d> translations;
    srand((unsigned)time(NULL));
    out_file_candidates.open(data_root + "candidates.csv", ios::out);
    out_file_candidates << "r1" << ";" << "r2" << ";" << "r3" << ";" << "tx" << ";" << "ty" << ";" << "tz" << endl;
    for(int i=0; i<result_amount; i++){
        cout << "The " << i << "th result" << endl;
        vector<Eigen::Matrix4d> motion_camera;
        vector<Eigen::Matrix4d> motion_lidar;

        vector<string> image_list;
        vector<string> cloud_list;
        vector<string> depth_list;
        HandEyeCalibration::dataReadRandom(begin, end, data_amount, data_interval, image_location, cloud_location, depth_location, image_list, cloud_list, depth_list);

        for(int i=0; i<data_amount; i++){
            cout << "data " << i << endl;
            cv::Mat img_1 = cv::imread ( data_root + image_list[2*i], CV_LOAD_IMAGE_COLOR );
            cv::Mat img_2 = cv::imread ( data_root + image_list[2*i+1], CV_LOAD_IMAGE_COLOR );
            cv::Mat depth_map_camera = cv::imread(data_root + depth_list[i], CV_8UC1);
            vector<cv::KeyPoint> keypoints_1, keypoints_2;
            vector<cv::DMatch> matches;
            vector<cv::Point3f> pts_3d;
            vector<cv::Point2f> pts_2d;
            HandEyeCalibration::cameraRegistration(img_1, img_2, keypoints_1, keypoints_2, matches, depth_map_camera, K, lidar, transformation_camera);
//            if(matches.size()<230){
//                continue;
//            }
            cv::Mat img_out;
            cv::drawKeypoints(img_1, keypoints_1, img_out, cv::Scalar(255, 0, 0), cv::DrawMatchesFlags::DEFAULT);
            //cv::drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_out, cv::Scalar(255, 0, 0), cv::Scalar(0, 0, 255));
            cv::imwrite(data_root + "test.png", img_out);
            cout << transformation_camera << endl;


            pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_lidar(new pcl::PointCloud<pcl::PointXYZI>);
            if (pcl::io::loadPCDFile<pcl::PointXYZI> (data_root + cloud_list[2*i], *point_cloud_lidar) == -1) //* load the file
            {
                PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
                exit(EXIT_FAILURE);
            }
            pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_lidar_xyz(new pcl::PointCloud<pcl::PointXYZ>);
            PointCloudAlignment::getLiDARPointCloudXYZ(point_cloud_lidar, point_cloud_lidar_xyz);

            pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_lidar2(new pcl::PointCloud<pcl::PointXYZI>);
            if (pcl::io::loadPCDFile<pcl::PointXYZI> (data_root + cloud_list[2*i+1], *point_cloud_lidar2) == -1) //* load the file
            {
                PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
                exit(EXIT_FAILURE);
            }
            pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_lidar_xyz2(new pcl::PointCloud<pcl::PointXYZ>);
            PointCloudAlignment::getLiDARPointCloudXYZ(point_cloud_lidar2, point_cloud_lidar_xyz2);
            HandEyeCalibration::pointCloudRegistration(point_cloud_lidar_xyz, point_cloud_lidar_xyz2, vox_volum, transformation_lidar);
            cout << transformation_lidar << endl;

            Eigen::Matrix4d transformation_eigen_camera;
            Eigen::Matrix4d transformation_eigen_lidar;
            cv::cv2eigen(transformation_camera, transformation_eigen_camera);
            cv::cv2eigen(transformation_lidar, transformation_eigen_lidar);
            motion_camera.push_back(transformation_eigen_camera);
            motion_lidar.push_back(transformation_eigen_lidar);
        }
        HandEyeCalibration::handEyeOptimization(motion_camera, motion_lidar, Rotation);
        cout << "rotation" << endl << Rotation;
        HandEyeCalibration::handEyeOptimizationTranslation(motion_camera, motion_lidar, Rotation, translation);
        Eigen::Matrix3f rotationfloat = Rotation.cast<float>();
        cv::Mat rotation_mat = cv::Mat::zeros(3, 1, CV_32FC1);
        cv::eigen2cv(rotationfloat, rotation_mat);
        cout << rotation_mat << endl;
        cv::Mat rotationVector = cv::Mat::zeros(3, 1, CV_32FC1);
        cv::Rodrigues(rotation_mat, rotationVector);
        cout << rotationVector << endl;
        translations.push_back(translation);
        out_file_candidates << rotationVector.at<float>(0, 0) << ";" << rotationVector.at<float>(1, 0) << ";" << rotationVector.at<float>(2, 0) << ";" << translation[0] << ";" << translation[1] << ";" << translation[2] << endl;
    }
    return 0;
}