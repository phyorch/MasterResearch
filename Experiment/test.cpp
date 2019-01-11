//
// Created by phyorch on 24/10/18.
//

//#include <iostream>
//#include <fstream>
//#include <string>

#include "Disparity.h"
//#include <Eigen/Core>
//#include <Eigen/Dense>

//#include </home/phyorch/OPENCV_PROJECT/Stereo_test/src/StereoMatch.cpp>
//#include </home/phyorch/OPENCV_PROJECT/Stereo_test/src/StereoCalib.cpp>
#include "Disparity.h"
#include "SimilarityMeasure.h"
#include "ImageUtils.h"
#include <opencv2/core/types_c.h>
#include "StereoGC.h"

#include <iostream>
using namespace std;



int main(){
    string left_path = "/home/phyorch/Data/2011_09_26/data/2011_09_26_drive_0048_sync/image_00/data/0000000009.png";
    string right_path = "/home/phyorch/Data/2011_09_26/data/2011_09_26_drive_0048_sync/image_01/data/0000000009.png";
    string lidar_path = "/home/phyorch/Data/2011_09_26/data/2011_09_26_drive_0048_sync/velodyne_points/data/0000000009.bin";
//    string left_path = "/home/phyorch/Data/left.png";
//    string right_path = "/home/phyorch/Data/right.png";
//    string lidar_path = "/home/phyorch/Data/lidar.bin";
    string left_color_path = "/home/phyorch/Data/left_color.png";
    string lidar_output_path = "/home/phyorch/Data/lidar.pcd";
    string lidar_depth_output_path = "/home/phyorch/Data/depth_map.jpg";
    string lidar_image_output_path = "/home/phyorch/Data/depth_image.jpg";
    string left_disp_path = "/home/phyorch/Data/left_disparity.jpg";
    string right_disp_path = "/home/phyorch/Data/right_disparity.jpg";
    string filtered_disp_path = "/home/phyorch/Data/disparity_filtered.jpg";
    string depth_map_camera_path = "/home/phyorch/Data/camera_depth.jpg";
    string depth_map_camera_boader_path = "/home/phyorch/Data/camera_depth_boader.jpg";
    string depth_map_camera_boader_path2 = "/home/phyorch/Data/camera_depth_boader2.jpg";
    string depth_map_lidar_boader_path = "/home/phyorch/Data/lidar_depth_boader.jpg";
    string depth_map_lidar_boader_path2 = "/home/phyorch/Data/lidar_depth_boader2.jpg";

    string histogram_csv = "/home/phyorch/Data/hist.csv";
    string test_path = "/home/phyorch/Data/test.jpg";


    cv::Mat left_image = cv::imread(left_path, CV_8UC1);
    cv::Mat right_image = cv::imread(right_path, CV_8UC1);
    cv::Mat left_disp, right_disp, filtered_disp, depth_map_camera, depth_map_lidar;
    cv::Mat depth_map_camera_boader, depth_map_lidar_boader;

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
    /*----------------BM & SGBM method------------------*/
    StereoCamera camera = StereoCamera(sgbm_para, disp_filter_para);
    ImageUtils utils = ImageUtils();
    camera.loadCameraPara(camera_para);
    cv::Ptr<cv::StereoSGBM> sgbm = camera.sgbmMatch(left_image, right_image, left_disp);
    cv::Ptr<cv::StereoMatcher> sgbm_right = cv::ximgproc::createRightMatcher(sgbm);
    sgbm_right->compute(right_image, left_image, right_disp);
    //left_disp /= 16;
    //right_disp /= 16;
    camera.disparityFilter(left_disp, left_image, filtered_disp, sgbm, right_disp);
    filtered_disp = filtered_disp / 16;
    filtered_disp.convertTo(filtered_disp, CV_8UC1);
    cv::imwrite(left_disp_path, left_disp * 2);
    cv::imwrite(right_disp_path, right_disp * 2);
    cv::imwrite(filtered_disp_path, filtered_disp * 2);
    camera.triangulaDepthMap(filtered_disp, depth_map_camera);
    //cv::imwrite(depth_map_camera_path, depth_map_camera * 2);
    int num_disparities = sgbm->getNumDisparities();
    int min_disparity = sgbm->getMinDisparity();
    ImageUtils::disparityBoader(depth_map_camera, depth_map_camera_boader, num_disparities, min_disparity);
    cv::imwrite(depth_map_camera_boader_path, depth_map_camera_boader * 2);







    /*----------------GC method------------------*/
//    IplImage left_ipl = left_image;
//    IplImage right_ipl = right_image;
//    IplImage *left_ipl_ptr = &left_ipl;
//    IplImage *right_ipl_ptr = &right_ipl;
//    string disp_gc_path = "/home/phyorch/MasterResearch/Data/disparity_gc.jpg";
//
//    CvStereoGCState* GCState=cvCreateStereoGCState(64,1);
//    cout<<"start matching using GC"<<endl;
//    CvMat* gcdispleft=cvCreateMat(left_ipl_ptr->height, left_ipl_ptr->width,CV_16S);
//    CvMat* gcdispright=cvCreateMat(right_ipl_ptr->height,right_ipl_ptr->width,CV_16S);
//    cvFindStereoCorrespondenceGC(left_ipl_ptr, right_ipl_ptr, gcdispleft, gcdispright, GCState, 0);
//    cv::Mat left_disp_gc (gcdispleft->rows, gcdispleft->cols, gcdispleft->type, gcdispleft->data.fl);
//    cv::Mat right_disp_gc (gcdispright->rows, gcdispright->cols, gcdispright->type, gcdispright->data.fl);
//    left_disp_gc = left_disp_gc / 16;
//    left_disp_gc.convertTo(left_disp_gc, CV_8UC1);
//    //right_disp_gc = right_disp_gc / 16;
//    right_disp_gc.convertTo(right_disp_gc, CV_8UC1);
//    cout << right_disp_gc;

//    cv::imwrite(disp_gc_path, right_disp_gc);


//    CvMat* gcvdisp=cvCreateMat(left_ipl_ptr->height,left_ipl_ptr->width,CV_8U);
//    cv::Mat disp_gc;

//    cvNormalize(gcdispright,gcvdisp,0,255,CV_MINMAX);
    //cvSaveImage(disp_gc_path.c_str(), gcdispright);



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
    Eigen::Matrix4f velo_to_cam;
    velo_to_cam << 7.533745e-03, -9.999714e-01, -6.166020e-04, -4.069766e-03,
            1.480249e-02, 7.280733e-04, -9.998902e-01, -7.631618e-02,
            9.998621e-01, 7.523790e-03, 1.480755e-02, -2.717806e-01,
            0, 0, 0, 1;
    Eigen::Matrix<float,3,4> K_00;
    K_00 << 9.842439e+02, 0.000000e+00, 6.900000e+02, 0,
            0.000000e+00, 9.808141e+02, 2.331966e+02, 0,
            0.000000e+00, 0.000000e+00, 1.000000e+00, 0;
    Eigen::Matrix4f R_rect_00_extended;
    R_rect_00_extended << 9.999239e-01, 9.837760e-03, -7.445048e-03, 0,
            -9.869795e-03, 9.999421e-01, -4.278459e-03, 0,
            7.402527e-03, 4.351614e-03, 9.999631e-01, 0,
            0, 0, 0, 1;
    Eigen::Matrix<float,3,4> P_rect_00;
    P_rect_00 << 7.215377e+02, 0.000000e+00, 6.095593e+02, 0.000000e+00,
            0.000000e+00, 7.215377e+02, 1.728540e+02, 0.000000e+00,
            0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00;
    // normal model
//    LiDARCalibPara lidar_calib_para = {
//            T:velo_to_cam,
//            K:K_00,
//            imageSize:cv::Size(left_image.cols, left_image.rows)
//    };
    // kitti model
    LiDARCalibParaKitti lidar_calib_para_kitti = {
            T:velo_to_cam,
            R:R_rect_00_extended,
            P:P_rect_00,
            imageSize:cv::Size(left_image.cols, left_image.rows)
    };


    /*-----------Operation------------*/
    LiDAR lidar = LiDAR(lidar_calib_para_kitti);
    //LiDAR lidar = LiDAR(lidar_calib_para);
    lidar.convertKittiBinData(lidar_path, lidar_output_path);

    lidar.projectData(lidar_path, lidar_output_path, depth_map_lidar);
    //lidar.projectDataKitti(lidar_path, lidar_output_path, depth_map_lidar);
    //cv::Mat left_image_color = cv::imread(left_color_path);
    ImageUtils::colorTransfer(depth_map_lidar, left_image);
    cv::imwrite(lidar_image_output_path, left_image);
    ImageUtils::disparityBoader(depth_map_lidar, depth_map_lidar_boader, num_disparities, min_disparity);
    cv::imwrite(depth_map_lidar_boader_path, depth_map_lidar_boader * 4);
    //cout << depth_map_lidar_boader;
    int boader=ImageUtils::rowBoader(depth_map_lidar_boader);
    ImageUtils::disparityBoader(depth_map_camera_boader, depth_map_camera_boader, boader);
    ImageUtils::disparityBoader(depth_map_lidar_boader, depth_map_lidar_boader, boader);
    cv::imwrite(depth_map_camera_boader_path2, depth_map_camera_boader * 2);
    cv::imwrite(depth_map_lidar_boader_path2, depth_map_lidar_boader * 4);



//    /*-------------------Mutual information---------------------*/
//    //Eigen::RowVectorXi histogram;
//    //ImageUtils::histogramCount(depth_map_lidar_boader, histogram);
//    cv::Mat testc(depth_map_camera_boader, cv::Rect(220, 20, 200, 100));
//    cv::Mat testl(depth_map_lidar_boader, cv::Rect(220, 20, 200, 100));
//    //cv::imwrite(test_path, test);
//    Eigen::RowVectorXi hist_camera;
//    Eigen::RowVectorXi hist_lidar;
//    vector<int> vector_camera;
//    vector<int> vector_lidar;
//    cv::Mat depth_map_camera_corresponded;
//    //HistogramGeneration::pointCorrespond(testc, testl, depth_map_camera_corresponded);
//
//    vector<int> boundary_camera = HistogramGeneration::histogramCount(testc, hist_camera);
//    vector<int> boundary_lidar = HistogramGeneration::histogramCount(testl, hist_lidar);
//    HistogramGeneration::histogramCompletion(hist_camera, hist_lidar, boundary_camera, boundary_lidar, vector_camera, vector_lidar);
//    HistogramGeneration::histogramWrite(histogram_csv, vector_camera, vector_lidar);
//
//    cout << "For this histogram:" << endl;
//    cout << "camera:  " << endl;
//    HistogramGeneration::printHistogram(vector_camera, boundary_camera);
//    cout << "total number:  " << int(boundary_camera[2]);
//    cout << endl << "lidar:  " << endl;
//    HistogramGeneration::printHistogram(vector_lidar, boundary_lidar);
//    cout << "total number:  " << int(boundary_lidar[2]);
//
//    double dist = HistogramMeasure::cosDistance(vector_camera, vector_lidar);
//    cout << "Distance is: " << dist <<endl;





    /*-------------------Histogram analysis---------------------*/
    cv::Mat testc(depth_map_camera_boader, cv::Rect(100, 20, 50, 50));
    cv::Mat testl(depth_map_lidar_boader, cv::Rect(100, 20, 50, 50));
    //Analysis::depthAnalysis(depth_map_camera_boader, depth_map_lidar_boader, 8, histogram_csv);
    Analysis::depthDistribution(testl, histogram_csv);





    //cv::Size region_num = cv::Size(8, 4);
    //double dist = ImageUtils::imageDistance(depth_map_camera_boader, depth_map_lidar_boader, region_num);


    //cout << dist;


/* Max-pooling operation */
//    cv::Mat imgl = cv::imread("/home/phyorch/MasterResearch/Data/left.png");
//    cv::Mat imgr = cv::imread("/home/phyorch/MasterResearch/Data/right.png");
//    cv::imwrite("/home/phyorch/MasterResearch/Data/left.jpg", imgl);
//    cv::imwrite("/home/phyorch/MasterResearch/Data/right.jpg", imgr);
//    cv::Mat depth_map = cv::imread("/home/phyorch/MasterResearch/Data/depth_map.jpg");
//    cv::Mat disparaty = cv::imread("/home/phyorch/MasterResearch/Data/disparity.png", CV_8UC1);
//    int size = 3;
//    int width = depth_map.cols / size;
//    int height = depth_map.rows / size;
//    //int width = left_disp.cols /size;
//    //int height = left_disp.rows /size;
//    cv::Mat dense_map = cv::Mat::zeros(height, width, CV_8UC1);
//    cv::Mat dense_disparity = cv::Mat::zeros(height, width, CV_8UC1);
//    maxPooling(left_disp, dense_disparity, size);
//    maxPooling(depth_map, dense_map, size);
//    maxPooling(disparaty, dense_disparity, size);
//    cv::imwrite("/home/phyorch/MasterResearch/Data/dense_map.jpg", dense_map * 3);
//    cv::imwrite("/home/phyorch/MasterResearch/Data/dense_disparity.jpg", dense_disparity * 3);
//    cv::imwrite("/home/phyorch/MasterResearch/Data/dense_disparity.jpg", dense_disparity / 8);
    //cout << dense_disparity;
    return 0;
}