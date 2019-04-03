//
// Created by phyorch on 17/12/18.
//

//
// Created by phyorch on 24/10/18.
//

//#include <iostream>
//#include <fstream>
//#include <string>

//#include <Eigen/Core>
//#include <Eigen/Dense>

//#include </home/phyorch/OPENCV_PROJECT/Stereo_test/src/StereoMatch.cpp>
//#include </home/phyorch/OPENCV_PROJECT/Stereo_test/src/StereoCalib.cpp>
#include "Sensor.h"
#include "Calibration.h"
#include <iostream>
#include <opencv2/core/types_c.h>
#include "StereoGC.h"
#include "ImageUtils.h"
#include "RealEquipment.h"
#include <matplotlib-cpp/matplotlibcpp.h>
#include <sl/Camera.hpp>
#include <libcmaes/cmaes.h>


using namespace std;




int main(){
    string left_path = "/home/phyorch/Data/ZEDData/RGBImage/";
    string right_path = "/home/phyorch/Data/CalibrationFile/calibration2/right.png";
    string lidar_path = "/home/phyorch/Data/Pandar40Data/PCDDataTest/";
//    string left_path = "/home/phyorch/Data/left.png";
//    string right_path = "/home/phyorch/Data/right.png";
//    string lidar_path = "/home/phyorch/Data/lidar.bin";
    string left_color_path = "/home/phyorch/Data/left_color.png";
    string lidar_output_path = "/home/phyorch/Data/lidar.pcd";
    string lidar_depth_output_path = "/home/phyorch/Data/depth_map.jpg";
    string lidar_image_output_path = "/home/phyorch/Data/depth_image.jpg";
    string depth_map_camera_path = "/home/phyorch/Data/ZEDData/DepthImage/";
    string depth_map_camera_boader_path = "/home/phyorch/Data/camera_depth_boader.jpg";

    string camera_csv_path = "/home/phyorch/Data/HistCamera.csv";
    string lidar_csv_path = "/home/phyorch/Data/HistLiDAR.csv";
    string test1_path = "/home/phyorch/Data/test.jpg";
    string test2_path = "/home/phyorch/Data/test2.jpg";

    string zed_rgb_image_path = "/home/phyorch/Data/ZEDData/RGBImage/";
    string zed_depth_image_path = "/home/phyorch/Data/ZEDData/DepthImage/";
    string pandar_cloud_path = "/home/phyorch/Data/Pandar40Data/PCDDataTest";


    cv::Mat left_image = cv::imread(left_path + "1545286685left.png");
    cv::Mat depth_map_camera, depth_map_lidar, depth_map_camera_boader, depth_map_lidar_boader;

    CameraPara camera_para = {
            fx:984.2439,
            fy:980.8141,
            cx:690,
            cy:233.1966,
            base:0.54,
            size:cv::Size(left_image.cols, left_image.rows)
    };

    SGBMPara sgbm_para = {
            PreFilterCap:63,
            SADWindowSize:9,
            MinDisparity:0,
            UniquenessRatio:10,
            SpeckleWindowSize:100,
            SpeckleRange:32,
            Disp12MaxDiff:1
    };

    BMPara bm_para = {
            BlockSize:11,
            MinDisparity:0,
            TextureThreshold:10,
            UniquenessRatio:15,
            SpeckleWindowSize:100,
            SpeckleRange:64,
            Disp12MaxDiff:1
    };

    DispFilterPara disp_filter_para = {
            Lambda:8000,
            SigmaColor:2.0
    };


    /*-------------------Camera operation---------------------*/




//    /*----------------GC method histogram test------------------*/
//    cv::Mat test = cv::imread(disp_gc_path, CV_8UC1);
//    cout << test;
//    Eigen::RowVectorXi hist;
//    vector<int> bound = HistogramGeneration::histogramCount(test, hist);
//
//    HistogramGeneration::histogramWrite(histogram_csv, hist);
//    int qa = 1;






/*-------------------LiDAR operation---------------------*/
/*-----------Parameters setting------------*/
//----------------------------------------------------------------------------------------------
//    Eigen::Matrix4f cam_to_velo;
//    cam_to_velo << -9.816086341230e-01, -1.508804030660e-01, 1.169597938860e-01, 2.757819023341e-01,
//            -1.152563726088e-01, -2.001656302531e-02, -9.931340824771e-01, 8.406759160258e-02,
//            1.521856037484e-01, -9.883493517857e-01, 2.258503135975e-03, -9.936401615109e-02,
//            0, 0, 0, 1;
//
//    //calibration2 result
//    Eigen::Matrix3f cam_to_velo_rotation;
//    cam_to_velo_rotation << -9.816086341230e-01, -1.508804030660e-01, 1.169597938860e-01,
//            -1.152563726088e-01, -2.001656302531e-02, -9.931340824771e-01,
//            1.521856037484e-01, -9.883493517857e-01, 2.258503135975e-03;
//    Eigen::Matrix3f velo_to_cam_rotation = cam_to_velo_rotation.transpose();
//    Eigen::Vector3f cam_to_velo_translation;
//    cam_to_velo_translation << 2.757819023341e-01, 8.406759160258e-02, -9.936401615109e-02;
//    Eigen::Matrix4f velo_to_cam = cam_to_velo.inverse();
//
//
//    //calibration3 result
//    Eigen::Matrix3f cam_to_velo_rotation;
//    cam_to_velo_rotation << -9.985155569330e-01, 8.235418513663e-03, 5.384106652678e-02,
//            -5.277166792839e-02, 9.845070545916e-02, -9.937417218064e-01,
//            -1.348456995580e-02, -9.951078516807e-01, -9.786996422094e-02;
//    Eigen::Matrix3f velo_to_cam_rotation = cam_to_velo_rotation.transpose();
//    Eigen::Vector3f cam_to_velo_translation;
//    cam_to_velo_translation << 2.144843661591e-01, 4.170182571745e-02, -8.966614986700e-02;
//    Eigen::Matrix4f velo_to_cam = cam_to_velo.inverse();
//
//
//    //KITTI model use this transformation
//    Eigen::Matrix4f R_rect_00_extended;
//    R_rect_00_extended << 9.985793692484e-01, -4.263103556554e-03, -5.311373889557e-02, 0,
//            4.422649746335e-03, 9.999860535090e-01, 2.886686120606e-03, 0,
//            5.310069190342e-02, -3.117488669386e-03, 9.985842967841e-01, 0,
//            0, 0, 0, 1;
//    Eigen::Matrix<float,3,4> P_rect_00;
//    P_rect_00 << 6.781688257805e+02, 0.000000000000e+00, 7.070204772949e+02, 0.000000000000e+00,
//            0.000000000000e+00, 6.781688257805e+02, 3.151121215820e+02, 0.000000000000e+00,
//            0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00, 0.000000000000e+00;
//
//
//    Eigen::Matrix4f R_self;
//    R_self << 1, 0, 0, 0,
//            0, 1, 0, 0,
//            0, 0, 1, 0,
//            0, 0, 0, 1;
//    Eigen::Matrix<float,3,4> P_self;
//    P_self << 674.213928, 0, 668.909607, 0,
//            0, 674.213928, 380.501831, 0,
//            0, 0, 1, 0;
//----------------------------------------------------------------------------------------------



//----------------------------------------------------------------------------------------------
//    //calibration2 result
//    cv::Mat cam_to_velo_rotation;
//    cam_to_velo_rotation = (cv::Mat_<float>(3,3) << -9.816086341230e-01, -1.508804030660e-01, 1.169597938860e-01,
//            -1.152563726088e-01, -2.001656302531e-02, -9.931340824771e-01,
//            1.521856037484e-01, -9.883493517857e-01, 2.258503135975e-03);
//    cv::Mat cam_to_velo_translation;
//    cam_to_velo_translation = (cv::Mat_<float>(3,1) << 2.757819023341e-01, 8.406759160258e-02, -9.936401615109e-02);
//    cv::Mat velo_to_cam_rotation;
//    cv::transpose(cam_to_velo_rotation, velo_to_cam_rotation);
//    cv::Mat velo_to_cam_translation = -velo_to_cam_rotation * cam_to_velo_translation;


    //calibration3 result
    cv::Mat cam_to_velo_rotation;
    cam_to_velo_rotation = (cv::Mat_<float>(3,3) << -9.985155569330e-01, 8.235418513663e-03, 5.384106652678e-02,
            -5.277166792839e-02, 9.845070545916e-02, -9.937417218064e-01,
            -1.348456995580e-02, -9.951078516807e-01, -9.786996422094e-02);
    cv::Mat cam_to_velo_translation;
    cam_to_velo_translation = (cv::Mat_<float>(3,1) << 2.144843661591e-01, 4.170182571745e-02, -8.966614986700e-02);
    cv::Mat velo_to_cam_rotation;
    cv::transpose(cam_to_velo_rotation, velo_to_cam_rotation);
    cv::Mat velo_to_cam_translation = velo_to_cam_rotation * cam_to_velo_translation;
    velo_to_cam_translation = - velo_to_cam_translation;
    float t1 = velo_to_cam_translation.at<float>(0,0);
    float t2 = velo_to_cam_translation.at<float>(1,0);
    float t3 = velo_to_cam_translation.at<float>(2,0);
    cout << -cam_to_velo_rotation * cam_to_velo_translation << endl;
    cout << -velo_to_cam_rotation * cam_to_velo_translation << endl;
    cout << cam_to_velo_rotation * velo_to_cam_rotation << endl;


        //calibration3 result
    Eigen::Matrix3f cam_to_velo_rotation2;
    cam_to_velo_rotation2 << -9.985155569330e-01, 8.235418513663e-03, 5.384106652678e-02,
            -5.277166792839e-02, 9.845070545916e-02, -9.937417218064e-01,
            -1.348456995580e-02, -9.951078516807e-01, -9.786996422094e-02;
    Eigen::Matrix3f velo_to_cam_rotation2 = cam_to_velo_rotation2.transpose();
    Eigen::Vector3f cam_to_velo_translation2;
    cam_to_velo_translation2 << 2.144843661591e-01, 4.170182571745e-02, -8.966614986700e-02;
    Eigen::Vector3f velo_to_cam_translation2 = - velo_to_cam_rotation2 * cam_to_velo_translation2;
//----------------------------------------------------------------------------------------------


//Intrinsic parameters for self data
    cv::Mat R_self;
    R_self = (cv::Mat_<float>(4,4) << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1);
    cv::Mat P_self;
    P_self = (cv::Mat_<float>(3,4) << 674.213928, 0, 668.909607, 0,
            0, 674.213928, 380.501831, 0,
            0, 0, 1, 0);

    Eigen::Matrix4f R_self2;
    R_self2 << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    Eigen::Matrix<float,3,4> P_self2;
    P_self2 << 674.213928, 0, 668.909607, 0,
            0, 674.213928, 380.501831, 0,
            0, 0, 1, 0;


    // kitti model
//    LiDARCalibParaKitti lidar_calib_para_kitti = {
//            T:velo_to_cam,
//            R:R_rect_00_extended,
//            P:P_rect_00,
//            imageSize:cv::Size(left_image.cols, left_image.rows)
//    };

    LiDARCalibParaKittiInverse lidar_calib_para_kitti_inverse = {
            Rotation:velo_to_cam_rotation,
            Translation:velo_to_cam_translation,
            R:R_self,
            P:P_self,
            imageSize:cv::Size(left_image.cols, left_image.rows)
    };

    LiDARCalibParaKittiInverseEigen lidar_calib_para_kitti_inverse_eigen = {
            Rotation:velo_to_cam_rotation2,
            Translation:cam_to_velo_translation2,
            R:R_self2,
            P:P_self2,
            imageSize:cv::Size(left_image.cols, left_image.rows)
    };


    /*-------------------Histogram generation---------------------*/
    LiDAR lidar = LiDAR(lidar_calib_para_kitti_inverse);
    //LiDAR lidar = LiDAR(lidar_calib_para_kitti_inverse_eigen);
    //LiDAR lidar = LiDAR(lidar_calib_para_kitti);
    lidar.projectData(lidar_path + "125.328831.pcd", depth_map_lidar, PCD, KITTI, XYZIT, C2L, CV);
    ImageUtils::colorTransfer(depth_map_lidar, left_image);
    cv::imwrite(lidar_image_output_path, left_image);

    cv::FileStorage fs(depth_map_camera_path + "1545286685.xml" , cv::FileStorage::READ);
    fs["CameraDepthMap"] >> depth_map_camera;

    // LiDAR invalid region
    ImageUtils::testMapRegion(depth_map_camera, depth_map_camera_boader, depth_map_camera.rows/3, depth_map_camera.rows/3 * 2, 50, depth_map_camera.cols);
    ImageUtils::testMapRegion(depth_map_lidar, depth_map_lidar_boader, depth_map_camera.rows/3, depth_map_camera.rows/3 * 2, 50, depth_map_camera.cols);

    cv::imwrite(depth_map_camera_boader_path, depth_map_camera_boader);
    //Unit in meter
    depth_map_camera_boader = depth_map_camera_boader / 1000;

    cv::Mat test1, test2, testc1, testl1, testc2, testl2;
    ImageUtils::testMapRegion(left_image, test1, depth_map_camera.rows/3, depth_map_camera.rows/3 * 2, 50, depth_map_camera.cols);
    ImageUtils::testMapRegion(left_image, test2, depth_map_camera.rows/3, depth_map_camera.rows/3 * 2, 50, depth_map_camera.cols);

    ImageUtils::testMapRegion(depth_map_camera_boader, testc1, 0, depth_map_camera_boader.rows, depth_map_camera_boader.cols * 5 / 9, depth_map_camera_boader.cols * 2 / 3);
    ImageUtils::testMapRegion(depth_map_lidar_boader, testl1, 0, depth_map_lidar_boader.rows, depth_map_camera_boader.cols * 5 / 9, depth_map_lidar_boader.cols * 2 / 3);
    ImageUtils::testMapRegion(depth_map_camera_boader, testc2, 0, depth_map_camera_boader.rows, depth_map_camera_boader.cols * 1 / 4, depth_map_camera_boader.cols * 7 / 16);
    ImageUtils::testMapRegion(depth_map_lidar_boader, testl2, 0, depth_map_lidar_boader.rows, depth_map_camera_boader.cols * 1 / 4, depth_map_lidar_boader.cols * 7 / 16);
    ImageUtils::testMapRegion(test1, test1, 0, test1.rows, test1.cols * 5 / 9, test1.cols * 2 / 3);
    ImageUtils::testMapRegion(test2, test2, 0, test2.rows, test2.cols * 1 / 4, test2.cols * 7 / 16);
    cv::imwrite(test1_path, test1);
    cv::imwrite(test2_path, test2);

    HistogramGeneration::testHistogramWrite(camera_csv_path, testc1);
    HistogramGeneration::testHistogramWrite(lidar_csv_path, testl1);
    vector<float> camera_hist1, lidar_hist1, camera_hist_downsampled1, camera_hist2, lidar_hist2, camera_hist_downsampled2;
    HistogramGeneration::testMap2Histogram(testc1, camera_hist1, 1, 2);
    HistogramGeneration::testMap2Histogram(testl1, lidar_hist1, 1, 2);
    HistogramGeneration::testMap2Histogram(testc2, camera_hist2, 1, 2);
    HistogramGeneration::testMap2Histogram(testl2, lidar_hist2, 1, 2);
    HistogramGeneration::testHistogramDownsampling(camera_hist1, camera_hist_downsampled1, lidar_hist1.size());
    HistogramGeneration::testHistogramDownsampling(camera_hist2, camera_hist_downsampled2, lidar_hist2.size());
    double distance_hist1 = cv::compareHist(lidar_hist1, camera_hist_downsampled1, cv::HISTCMP_KL_DIV);
    double distance_hist2 = cv::compareHist(lidar_hist2, camera_hist_downsampled2, cv::HISTCMP_KL_DIV);
    double distance = distance_hist1 + distance_hist2;
    cout << "distance1:  " << distance_hist1 << endl << "distance2:  " <<distance_hist2 << endl;
    vector<float> test_vec(6, 0);
    cv::Mat rotation, translation;
    Analysis::paramterTransfer(test_vec, rotation, translation);
    lidar.updateParameters(rotation, translation);
    int a = 1;

    return 0;
}