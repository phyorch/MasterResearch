//
// Created by phyorch on 27/12/18.
//

#include "Sensor.h"
#include "Calibration.h"
#include "StereoGC.h"
#include "ImageUtils.h"
#include "RealEquipment.h"

#include <iostream>
#include <opencv2/core/types_c.h>
#include <sl/Camera.hpp>
//#include <libcmaes/cmaes.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>

string user_name = "phyorch";
float vox_volum = 4.0;

//string data_name = "2019_01_03/2019_01_03_3";
//string image_name = "/ZEDData/RGBImage/image1546524910right.png";
//string cloud_name = "/Pandar40Data/PCDDataKIT/1520.202800.pcd";

string data_name = "2011_09_26_drive_0005_sync";
string image_name = "/image_02/data/0000000132.png";
string cloud_name = "/velodyne_points/data/0000000132.pcd";
string depth_name = "/depth/0000000132.png";

string data_root = "/home/" + user_name + "/Data/";
string left_path1 = "/home/" + user_name + "/Data/" + data_name + "/ZEDData/RGBImage/";
string left_path2 = "/home/" + user_name + "/Data/" + data_name + "/ZEDData/RGBImage/";
string lidar_path1 = "/home/" + user_name + "/Data/" + data_name + "/Pandar40Data/PCDDataKIT/";
string lidar_path2 = "/home/" + user_name + "/Data/" + data_name + "/Pandar40Data/PCDDataKIT/";
string depth_map_camera_path1 = "/home/" + user_name + "/Data/" + data_name + "/ZEDData/DepthImage/";
string depth_map_camera_path2 = "/home/" + user_name + "/Data/" + data_name + "/ZEDData/DepthImage/";
string depth_map_camera_prediction_path = "/home/" + user_name + "/Data/" + data_name + "/depth/depth1.png";
string left_color_path = "/home/" + user_name + "/Data/left_color.png";
string lidar_output_path = "/home/" + user_name + "/Data/lidar.pcd";
string lidar_depth_output_path = "/home/" + user_name + "/Data/depth_map.jpg";
string lidar_image_output_path1 = "/home/" + user_name + "/Data/Result/OptimizationProcess/1result";
string lidar_image_output_path2 = "/home/" + user_name + "/Data/Result/OptimizationProcess/2result";
string depth_map_camera_boader_path = "/home/" + user_name + "/Data/camera_depth_boader.jpg";

string camera_csv_path = "/home/" + user_name + "/Data/HistCamera.csv";
string lidar_csv_path = "/home/" + user_name + "/Data/HistLiDAR.csv";
string test1_path = "/home/" + user_name + "/Data/test.png";
string test2_path = "/home/" + user_name + "/Data/test2.png";

string zed_rgb_image_path = "/home/" + user_name + "/Data/ZEDData/RGBImage/";
string zed_depth_image_path = "/home/" + user_name + "/Data/ZEDData/DepthImage/";
string pandar_cloud_path = "/home/" + user_name + "/Data/Pandar40Data/PCDDataTest";

//cv::Mat left_image1 = cv::imread(left_path1 + "image" + image_name + "left.png");
//cv::Mat left_image2 = cv::imread(left_path2 + "image" + image_name + "left.png");
cv::Mat depth_map_camera1, depth_map_camera_boader1, depth_map_camera2, depth_map_camera_boader2;
cv::Mat depth_map_lidar1, depth_map_lidar_boader1, depth_map_lidar2, depth_map_lidar_boader2;
Eigen::Matrix4f transformation;
int main(){
    cv::Mat left_image, lid_to_cam_rotation, lid_to_cam_translation, cam_to_lid_rotation, cam_to_lid_translation, R_self, P_self, depth_map_camera, depth_map_camera_boader, depth_map_lidar_boader, depth_map_lidar;
    left_image = cv::imread(data_root + data_name + image_name);
//    ImageUtils::creatMapRegion(left_image, left_image, 171, 547, 18, 1260);
//    cv::imwrite(data_root + data_name + image_name, left_image);
    LiDARCalibParaKittiInverse lidar_calib_para_kitti_inverse;
    CameraPara camera_para;
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_lidar_part(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_camera(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_lidar_cloud(new pcl::PointCloud<pcl::PointXYZ>);



//----------------------------------------------------------------------------------------------------------------------
////2019_01_15 intrinsic calibration
//    R_self = (cv::Mat_<float>(4,4) << 1, 0, 0, 0,
//            0, 1, 0, 0,
//            0, 0, 1, 0,
//            0, 0, 0, 1);
//
//    P_self = (cv::Mat_<float>(3,4) << 674.213928, 0, 668.909607, 0,
//            0, 674.213928, 380.501831, 0,
//            0, 0, 1, 0);
//
////2019_01_15 extrinsic calibration
//    lid_to_cam_rotation = (cv::Mat_<float>(3,3) << -0.99953955, 0.015632046, 0.026005354,
//    -0.025917636, 0.0057880618, -0.99964732,
//    -0.015777055, -0.99986106, -0.0053802514);
//
//    lid_to_cam_translation = (cv::Mat_<float>(3,1) << -0.11316491, -0.057814412, 0.091160908);
//
//    cv::Mat axisx, axisy, axisz;
//
//    int degreex = 2.5;
//    int degreey = -4;
//    int degreez = 0;
//    axisx = (cv::Mat_<float>(3,3) << 1, 0.0, 0.0,
//            0.0, cos(M_PI/180 * degreex), -sin(M_PI/180 * degreex),
//            0.0, sin(M_PI/180 * degreex), cos(M_PI/180 * degreex));
//
//    axisy = (cv::Mat_<float>(3,3) << cos(M_PI/180 * degreey), 0.0, -sin(M_PI/180 * degreey),
//            0.0, 1, 0.0,
//            sin(M_PI/180 * degreey), 0.0, cos(M_PI/180 * degreey));
//
//    axisz = (cv::Mat_<float>(3,3) << cos(M_PI/180 * degreez), -sin(M_PI/180 * degreez), 0.0,
//            sin(M_PI/180 * degreez), cos(M_PI/180 * degreez), 0.0,
//            0.0, 0.0, 1.0);
//    lid_to_cam_rotation = axisx * axisy * lid_to_cam_rotation;
//
//    Transfer::cv2EigenSeperate(lid_to_cam_rotation, lid_to_cam_translation, transformation);
//----------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------
////2019_01_15 intrinsic calibration
//    R_self = (cv::Mat_<float>(4,4) << 1, 0, 0, 0,
//            0, 1, 0, 0,
//            0, 0, 1, 0,
//            0, 0, 0, 1);
//
//    P_self = (cv::Mat_<float>(3,4) << 674.213928, 0, 668.909607, 0,
//            0, 674.213928, 380.501831, 0,
//            0, 0, 1, 0);
//
////2019_01_15 extrinsic calibration
//    lid_to_cam_rotation = (cv::Mat_<float>(3,3) << 1, 0, 0,
//    0, 0, -1,
//    0, 1, 0);
//    lid_to_cam_translation = (cv::Mat_<float>(3,1) << 0.383, 0, 0);
//    transformation << 1, 0, 0, 0.383, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1;
//
////    lid_to_cam_rotation = (cv::Mat_<float>(3,3) << 0.87365133, 0.11061831, -0.47381106,
////    -0.43071392, -0.27713022, -0.85888553,
////    -0.22631583, 0.95444351, -0.19447035);
////    lid_to_cam_translation = (cv::Mat_<float>(3,1) << -0.69790131, -0.078251913, 0.27923283);
////    transformation << 0.87365133, 0.11061831, -0.47381106, -0.69790131, -0.43071392, -0.27713022, -0.85888553, -0.078251913, -0.22631583, 0.95444351, -0.19447035, 0.27923283, 0, 0, 0, 1;
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
////2011_09_26_drive_0048_sync intrinsic calibration
    R_self = (cv::Mat_<float>(4,4) << 9.998817e-01, 1.511453e-02, -2.841595e-03, 0,
            -1.511724e-02, 9.998853e-01, -9.338510e-04, 0,
            2.827154e-03, 9.766976e-04, 9.999955e-01, 0,
            0, 0, 0, 1);

    P_self = (cv::Mat_<float>(3,4) << 7.215377e+02, 0.000000e+00, 6.095593e+02, 0,//4.485728e+01
            0.000000e+00, 7.215377e+02, 1.728540e+02, 0,//2.163791e-01
            0.000000e+00, 0.000000e+00, 1.000000e+00, 0);//2.745884e-03

//    R_self = (cv::Mat_<float>(4,4) << 1, 0, 0, 0,
//            0, 1, 0, 0,
//            0, 0, 1, 0,
//            0, 0, 0, 1);
//
//    P_self = (cv::Mat_<float>(3,4) << 6.7440533447265625e+02, 0., 6.7320996093750000e+02, 0,//4.485728e+01
//            0., 6.7440533447265625e+02, 3.8029739379882812e+02, 0,//2.163791e-01
//            0.000000e+00, 0.000000e+00, 1.000000e+00, 0);//2.745884e-03

//2011_09_26_drive_0048_sync extrinsic calibration
//    lid_to_cam_rotation = (cv::Mat_<float>(3,3) << 7.533745e-03, -9.999714e-01, -6.166020e-04,
//            1.480249e-02, 7.280733e-04, -9.998902e-01,
//            9.998621e-01, 7.523790e-03, 1.480755e-02);
//    lid_to_cam_translation = (cv::Mat_<float>(3,1) << -4.069766e-03, -7.631618e-02, -2.717806e-01);

    lid_to_cam_rotation = (cv::Mat_<float>(3,3) << 0.0087676989, -0.99974012, -0.021042559,
    0.0093115233, 0.02112408, -0.99973351,
    0.99991822, 0.008569424, 0.009494313);
    lid_to_cam_translation = (cv::Mat_<float>(3,1) << -0.022104684, -0.056908015, -0.22559986);


//    lid_to_cam_rotation = (cv::Mat_<float>(3,3) << -6.8836088830724711e-01, 1.5212676276466525e-01,
//            -4.5001694725983754e-04,
//            1.1326651711791161e-02, 7.0638167047866629e-02,
//            -9.9743769545627137e-01,
//            -1.5170517929691568e-01, -9.8583350389751967e-01,
//            -7.1539088389702687e-02);
//    lid_to_cam_translation = (cv::Mat_<float>(3,1) << 1.5184318319534851e-01, -7.3682146064973197e-02, -1.7484112138873420e-01);

    float degreex = 0;
    float degreey = 0;
    float degreez = 1.5;
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
//    cv::Mat cam0_to_cam2_rotation, cam0_to_cam2_translation;
//    cam0_to_cam2_rotation = (cv::Mat_<float>(3,3) << 9.999758e-01, -5.267463e-03, -4.552439e-03,
//                                                     5.251945e-03, 9.999804e-01, -3.413835e-03,
//                                                     4.570332e-03, 3.389843e-03, 9.999838e-01);
//    cam0_to_cam2_translation = (cv::Mat_<float>(3,1) << 5.956621e-02, 2.900141e-04, 2.577209e-03);
//    lid_to_cam_rotation = cam0_to_cam2_rotation * lid_to_cam_rotation;
//    lid_to_cam_translation = cam0_to_cam2_rotation * lid_to_cam_translation + cam0_to_cam2_translation;

    Transfer::cv2EigenSeperate(lid_to_cam_rotation, lid_to_cam_translation, transformation);
    cout << lid_to_cam_rotation << endl << lid_to_cam_translation << endl << transformation;
//----------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------
//lidar class initialization
    lidar_calib_para_kitti_inverse = {
            Rotation:lid_to_cam_rotation,
            Translation:lid_to_cam_translation,
            R:R_self,
            P:P_self,
            imageSize:cv::Size(left_image.cols, left_image.rows)
    };
    LiDAR lidar = LiDAR(lidar_calib_para_kitti_inverse);
//----------------------------------------------------------------------------------------------------------------------

    vector<float> euler_angle;
    float sy = sqrt(lid_to_cam_rotation.at<float>(0, 0) * lid_to_cam_rotation.at<float>(0, 0) + lid_to_cam_rotation.at<float>(1, 0) * lid_to_cam_rotation.at<float>(1, 0));
    float e1 = atan2(lid_to_cam_rotation.at<float>(2, 1), lid_to_cam_rotation.at<float>(2, 2)) * 180 / M_PI;
    float e2 = atan2(-lid_to_cam_rotation.at<float>(2, 0), sy) * 180 / M_PI;
    float e3 = atan2(lid_to_cam_rotation.at<float>(1, 0), lid_to_cam_rotation.at<float>(0, 0)) * 180 / M_PI;




    depth_map_camera1 = cv::imread(data_root + data_name + depth_name, CV_8UC1);
    //depth_map_camera1 = depth_map_camera1/256.0;
    lidar.projectData(data_root + data_name + cloud_name, depth_map_lidar1, point_cloud_lidar_part, XYZI, CV, 0);

    cv::Mat edge_map_camera, edge_map_camera_blured, edge_map_lidar, three;
    cv::Point window_size(6, 6);
    cv::Point window_range(0, 40);
    cv::Point size(3, 3);

    cv::Mat edge_map_lidar_dyed;
    Refinement::cameraEdgeGeneration(left_image, edge_map_camera, edge_map_camera_blured, 1, 7);
    Refinement::slideElimination2(depth_map_lidar1, edge_map_lidar, window_size, window_range, 1.5, 1);
    ImageUtils::neighborDyeing(edge_map_lidar, size, edge_map_lidar_dyed);
    Refinement::saveMatchResult(edge_map_camera_blured, edge_map_lidar_dyed, test2_path, 0, three);
    cv::imwrite(test2_path, edge_map_camera_blured);


    cv::Mat depth_map_lidar_dyed;
    ImageUtils::neighborDyeing(depth_map_lidar1, size, depth_map_lidar_dyed);
    ImageUtils::colorTransfer(depth_map_lidar_dyed, left_image, 70);
    cv::imwrite(test1_path, left_image);
    PointCloudAlignment::getCameraSparsePointCloudKitti(depth_map_camera1, depth_map_lidar1, lidar, point_cloud_camera, 171);


    int a = 1;


//    cv::Mat depth_test = cv::Mat::zeros(depth_map_camera1.rows, depth_map_camera1.cols, CV_32FC1);
//    for(int i=0; i<depth_test.rows; i++){
//        for(int j=0; j<depth_test.cols; j++){
//            if(depth_map_lidar1.at<float>(i, j) != 0){
//                depth_test.at<float>(i, j) = float(depth_map_camera1.at<uchar>(i ,j)) - depth_map_lidar1.at<float>(i, j);
//            }
//        }
//    }
//    float test = 0;
//    float cnt = 0;
//    for(int i=0; i<depth_test.rows; i++){
//        for(int j=0; j<depth_test.cols; j++){
//            if(depth_test.at<float>(i, j) != 0){
//                test = test + depth_test.at<float>(i, j);
//                cnt++;
//            }
//            }
//        }
//    float testt = test / cnt;

    //<< endl << test << endl << depth_test / test;






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









    //float scale = PointCloudAlignment::findScaling(point_cloud_camera, transformed_lidar_cloud);
    //PointCloudAlignment::pointCloudScaling(point_cloud_camera, scale, point_cloud_camera);
//
//    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_camera_downsample (new pcl::PointCloud<pcl::PointXYZ> ());
//    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_lidar_downsample (new pcl::PointCloud<pcl::PointXYZ> ());
//    PointCloudAlignment::pointCloudDownsample(point_cloud_camera, point_cloud_camera_downsample, vox_volum);
//    PointCloudAlignment::pointCloudDownsample(transformed_lidar_cloud, point_cloud_lidar_downsample, vox_volum);
//
//    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_camera_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_lidar_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//    sor.setInputCloud (point_cloud_camera_downsample);
//    sor.setMeanK (20);
//    sor.setStddevMulThresh (1.0);
//    sor.filter (*point_cloud_camera_filtered);
//    sor.setInputCloud(point_cloud_lidar_downsample);
//    sor.filter(*point_cloud_lidar_filtered);



    pcl::visualization::PCLVisualizer viewer ("test");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (point_cloud_camera, 230, 20, 20);
    viewer.addPointCloud(point_cloud_camera, source_cloud_color_handler, "transformed_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (transformed_lidar_cloud, 255, 255, 255);
    viewer.addPointCloud(transformed_lidar_cloud, transformed_cloud_color_handler, "camera_cloud");
//
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler2 (transformed_lidar_cloud2, 120, 80, 50);
//    viewer.addPointCloud(transformed_lidar_cloud2, transformed_cloud_color_handler2, "camera_cloud2");
//
    while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce ();
    }

    //float d1 = PointCloudAlignment::chamferDistance(point_cloud_camera_filtered, point_cloud_lidar_filtered);
    cout << "test";
    //cout << d1 << endl;

    return 0;
}


//writer.write<pcl::PointXYZ>(test1_path + "test.pcd", *points, false);

//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    if (pcl::io::loadPCDFile<pcl::PointXYZ> (lidar_path1 + cloud_name + ".pcd", *cloud) == -1) //* load the file
//    {
//        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
//        exit(EXIT_FAILURE);
//    }