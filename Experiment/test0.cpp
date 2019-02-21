//
// Created by phyorch on 22/12/18.
//

#include "Disparity.h"
#include "SimilarityMeasure.h"
#include <iostream>
#include <opencv2/core/types_c.h>
#include <opencv2/highgui/highgui.hpp>
#include "StereoGC.h"
#include "ImageUtils.h"
#include "RealEquipment.h"
//#include <matplotlib-cpp/matplotlibcpp.h>
#include <sl/Camera.hpp>
#include <libcmaes/cmaes.h>

string user_name = "phyorch";
string data_name = "2011_09_26_drive_0005_sync";
string image_name = "/image_02/data/0000000082.png";
string cloud_name = "/velodyne_points/data/0000000082.pcd";

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
string lidar_image_output_path1 = "/home/" + user_name + "/Data/Result/OptimizationProcess/1result";
string lidar_image_output_path2 = "/home/" + user_name + "/Data/Result/OptimizationProcess/2result";
string depth_map_camera_boader_path = "/home/" + user_name + "/Data/camera_depth_boader.jpg";

string camera_csv_path = "/home/" + user_name + "/Data/HistCamera.csv";
string lidar_csv_path = "/home/" + user_name + "/Data/HistLiDAR.csv";
string test1_path = "/home/" + user_name + "/Data/CalibrationFile/t5/image.jpg";
string test1_lidar_path = "/home/" + user_name + "/Data/CalibrationFile/t5/lidar.pcd";
string test2_path = "/home/" + user_name + "/Data/00.png";
string test1_output_path = "/home/" + user_name + "/Data/test1.png";
string test2_output_path = "/home/" + user_name + "/Data/test2.png";

string zed_rgb_image_path = "/home/" + user_name + "/Data/ZEDData/RGBImage/";
string zed_depth_image_path = "/home/" + user_name + "/Data/ZEDData/DepthImage/";
string pandar_cloud_path = "/home/" + user_name + "/Data/Pandar40Data/PCDDataTest";

cv::Mat left_image = cv::imread(data_root + data_name + image_name);
cv::Mat left_image1 = cv::imread(left_path1 + "image" + image_name + "left.png");
cv::Mat left_image2 = cv::imread(left_path2 + "image" + image_name + "left.png");
cv::Mat test_image = cv::imread(test1_path);

cv::Mat cam_to_lid_rotation, cam_to_lid_translation, lid_to_cam_rotation, lid_to_cam_translation, R_self, P_self;
cv::Mat lid_to_cam_rotation2, lid_to_cam_translation2;
cv::Mat depth_map_camera1, depth_map_lidar1;

pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_part(new pcl::PointCloud<pcl::PointXYZ>);




int main(){


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
//    lid_to_cam_translation = (cv::Mat_<float>(3,1) << -4.069766e-03, -7.631618e-02, -2.717806e-01);
//    lid_to_cam_rotation2 = (cv::Mat_<float>(3,3) << 0.0090045128, -0.99985546, 0.014423269,
//    -0.0041264175, -0.014460885, -0.99988693,
//    0.99995095, 0.0089439778, -0.0042560343);
//    lid_to_cam_translation2 = (cv::Mat_<float>(3,1) << -4.069766e-03, -7.631618e-02, -2.717806e-01);
    lid_to_cam_rotation = (cv::Mat_<float>(3,3) << 1, 0, 0,
            0, 1, 0,
            0, 0, 1);
    lid_to_cam_translation = (cv::Mat_<float>(3,1) << -4.069766e-03, -7.631618e-02, -2.717806e-01);
    lid_to_cam_rotation2 = (cv::Mat_<float>(3,3) << 1, 0, 0,
            0, 0, -1,
            0, 1, 0);
    lid_to_cam_translation2 = (cv::Mat_<float>(3,1) << -4.069766e-03, -7.631618e-02, -2.717806e-01);
    cv::Mat cam0_to_cam2_rotation, cam0_to_cam2_translation;
    cam0_to_cam2_rotation = (cv::Mat_<float>(3,3) << 9.999758e-01, -5.267463e-03, -4.552439e-03,
            5.251945e-03, 9.999804e-01, -3.413835e-03,
            4.570332e-03, 3.389843e-03, 9.999838e-01);
    cam0_to_cam2_translation = (cv::Mat_<float>(3,1) << 5.956621e-02, 2.900141e-04, 2.577209e-03);
    lid_to_cam_rotation = cam0_to_cam2_rotation * lid_to_cam_rotation;
    lid_to_cam_translation = cam0_to_cam2_rotation * lid_to_cam_translation + cam0_to_cam2_translation;
    
//----------------------------------------------------------------------------------------------------------------------

    LiDARCalibParaKittiInverse lidar_calib_para_kitti_inverse = {
            Rotation:lid_to_cam_rotation,
            Translation:lid_to_cam_translation,
            R:R_self,
            P:P_self,
            imageSize:cv::Size(left_image.cols, left_image.rows)
    };




    LiDAR lidar = LiDAR(lidar_calib_para_kitti_inverse);
    lidar.projectData(data_root + data_name + cloud_name, depth_map_lidar1, point_cloud_part, XYZI, CV);
    cv::Point size(2, 2);
    cv::Mat test;
    //ImageUtils::neighborDyeing(depth_map_lidar1, size, test);
    //ImageUtils::colorTransfer(depth_map_lidar1, left_image, 70);
    cv::imwrite(test1_output_path, left_image);

    vector<double> theta1(6, 0);
    vector<double> theta2(6, 0);
    Transfer::mat2VectorSeperate(lid_to_cam_rotation, lid_to_cam_translation, theta1);
    Transfer::mat2VectorSeperate(lid_to_cam_rotation2, lid_to_cam_translation2, theta2);
    double norma = sqrt(theta1[0] * theta1[0] + theta1[1] * theta1[1] + theta1[2] * theta1[2]);
    double normb = sqrt(theta2[0] * theta2[0] + theta2[1] * theta2[1] + theta2[2] * theta2[2]);
    //double angle = acos((theta1[0] * theta2[0] + theta1[1] * theta2[1] + theta1[2] * theta2[2]) / norma * normb);





//    cv::Mat test2 = cv::imread(test2_path, CV_8UC1);
//    cv::Mat grad_x;
    cv::Mat output;
    //cv::medianBlur(output, output, 3);
    //cv::GaussianBlur(left_image, left_image, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);
    cv::cvtColor(left_image, output, cv::COLOR_BGR2GRAY);
    cv::Laplacian(output, output, CV_8U, 3, 1, 0, cv::BORDER_DEFAULT);
    cv::threshold(output, output, 80, 255, cv::THRESH_BINARY);
    output.convertTo(output, CV_32FC1);
    cv::Mat output2;
    cv::Mat filter = cv::getGaussianKernel(13, 2.0, CV_32F);
    Refinement::gaussianBlurModified(output, output2, 13);
    cv::Mat three = cv::Mat::zeros(output2.rows, output2.cols, CV_8UC3);
    vector<cv::Mat> channels;
    output2.convertTo(output2, CV_8UC1);
    for (int i=0;i<3;i++)
    {
        channels.push_back(output2);
    }
    cv::merge(channels,three);


    cv::Point window_size(6, 6);
    cv::Point window_range(0, 40);
    cv::Point window_region(25, 50);
    //Refinement::slideElimination2(depth_map_lidar1, window_size, window_range, window_region, 1.5);
    cout << depth_map_lidar1;

    ImageUtils::colorTransfer(depth_map_lidar1, three, 70);

    //cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
    //cv::erode(output, output, element);
    //cv::morphologyEx(output, output, cv::MORPH_OPEN, element);
    //cv::Mat label;
    //cv::Mat result(output.rows, output.cols, CV_32FC1);
    //cv::distanceTransform(output, result, label, cv::DIST_L2, 3);
    //cv::normalize(output, output, 0, 1, cv::NORM_MINMAX);
    //cout << label;

    //Canny(left_image, output, 3, 9, 7);
//    //cv::Sobel(test2, grad_x, CV_16S, 1, 0, 3, 1, 1, cv::BORDER_DEFAULT);
//    //convertScaleAbs(grad_x, test2);

    cv::imwrite(test1_output_path, three);

//    cv::Point size(2, 2);
//    cv::Mat depth_map_lidar_dyed;
//    ImageUtils::neighborDyeing(test, size, depth_map_lidar_dyed);
//    cout << depth_map_lidar_dyed;
//    cv::imwrite(test1_output_path, depth_map_lidar_dyed);
    //cv::imwrite(test2_output_path, test2*16);


    return 0;
}