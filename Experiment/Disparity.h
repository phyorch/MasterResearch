//
// Created by phyorch on 13/11/18.
//

#pragma clang diagnostic push
#pragma ide diagnostic ignored "CannotResolve"
#ifndef PROJECT_TEST_H
#define PROJECT_TEST_H

#include <iostream>
#include <fstream>
#include <string>
#include "opencv2/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/ximgproc/disparity_filter.hpp"
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <boost/format.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include "hesaiLidarSDK.h"

using namespace std;

struct CameraPara{
    double fx;
    double fy;
    double cx;
    double cy;
    double base;
    cv::Size size;
};


struct BMPara{
    int BlockSize;
    int MinDisparity;
    int TextureThreshold;
    int UniquenessRatio;
    int SpeckleWindowSize;
    int SpeckleRange;
    int Disp12MaxDiff;
    int NumDisparities;
};


struct SGBMPara{
    int PreFilterCap;
    int SADWindowSize;
    int MinDisparity;
    int UniquenessRatio;
    int SpeckleWindowSize;
    int SpeckleRange;
    int Disp12MaxDiff;
    int sgbmWinSize;
    int NumDisparities;
};


struct DispFilterPara{
    double Lambda;
    double SigmaColor;
};


enum LiDARDataType {BIN, PCD};
enum LiDARDataSource {KITTI, SELF};
enum LiDARPointType {XYZIT, XYZI};
enum ExtrinsicDirection{C2L, L2C};
enum MatType{EIGEN, CV};


//struct LiDARCalibPara{
//    Eigen::Matrix4f T;
//    Eigen::Matrix<float,3,4> K;
//    cv::Size imageSize;
//};


//struct LiDARCalibParaKitti{
//    Eigen::Matrix4f T;
//    Eigen::Matrix4f R;
//    Eigen::Matrix<float,3,4> P;
//    cv::Size imageSize;
//};

//struct LiDARCalibParaKitti{
//    Eigen::Matrix4f T;
//    Eigen::Matrix4f R;
//    Eigen::Matrix<float,3,4> P;
//    cv::Size imageSize;
//};
//
struct LiDARCalibParaKittiInverseEigen{
    Eigen::Matrix3f Rotation;
    Eigen::Vector3f Translation;
    Eigen::Matrix4f R;
    Eigen::Matrix<float,3,4> P;
    cv::Size imageSize;
};

struct LiDARCalibParaKitti{
    cv::Mat T;
    cv::Mat R;
    cv::Mat P;
    cv::Size imageSize;
};

struct LiDARCalibParaKittiInverse{
    cv::Mat Rotation;
    cv::Mat Translation;
    cv::Mat R;
    cv::Mat P;
    cv::Size imageSize;
};


class StereoCamera
{
public:
    StereoCamera(BMPara bMpara, DispFilterPara dispFilterPara);
    StereoCamera(SGBMPara sGBMpara, DispFilterPara dispFilterPara);
    virtual ~StereoCamera(void);


    cv::Ptr<cv::StereoBM> bmMatch(cv::Mat leftImage, cv::Mat rightImage, cv::Mat &leftDisp);

    cv::Ptr<cv::StereoSGBM> sgbmMatch(cv::Mat leftImage, cv::Mat rightImage, cv::Mat &leftDisp);

    void disparityFilter(cv::Mat &dispMap, cv::Mat &leftImage, cv::Mat &filteredDispMap, cv::Ptr<cv::StereoBM> &bM);

    void disparityFilter(cv::Mat &dispMap, cv::Mat &leftImage, cv::Mat &filteredDispMap, cv::Ptr<cv::StereoBM> &bM, cv::Mat &rightDispMap);

    void disparityFilter(cv::Mat &dispMap, cv::Mat &leftImage, cv::Mat &filteredDispMap, cv::Ptr<cv::StereoSGBM> &sGBM);

    void disparityFilter(cv::Mat &dispMap, cv::Mat &leftImage, cv::Mat &filteredDispMap, cv::Ptr<cv::StereoSGBM> &sGBM, cv::Mat &rightDispMap);

    int loadCameraPara(CameraPara cameraPara);

    int triangulaDepthMap(cv::Mat leftDisp, cv::Mat &depthMap);


    DispFilterPara _dispFilterPara;
private:
    bool cameraParaLoaded;
    CameraPara _cameraPara;
    SGBMPara _sGBMPara;
    BMPara _bMPara;

    void initCameraPara(BMPara bmpara, DispFilterPara dispFilterPara);

    void initCameraPara(SGBMPara sgbmpara, DispFilterPara dispFilterPara);

};


class LiDAR{
public:

    LiDAR(LiDARCalibParaKitti lidarCalibParaKitti);

    LiDAR(LiDARCalibParaKittiInverse lidarCalibParaKittiInverse);

    LiDAR(LiDARCalibParaKittiInverseEigen &lidarCalibParaKittiInverseEigen);

    virtual ~LiDAR(void);

    void convertKittiBinData(string &inFile, string &outFile); //, vector *pointCloud

    void projectPointKitti(cv::Mat &depthMap, pandar_pointcloud::PointXYZIT &point);

    void projectPointKittiSeperate(cv::Mat &depthMapLiDAR, pandar_pointcloud::PointXYZIT &point, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPart);
    
    void projectPointKittiSeperate(cv::Mat &depthMapCamera, cv::Mat &depthMapLiDAR, pandar_pointcloud::PointXYZIT &point, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPart);

    void projectPointKittiSeperate(cv::Mat &depthMapCamera, cv::Mat &depthMapLiDAR, pandar_pointcloud::PointXYZIT &point, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudCamera, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudLiDAR);

    void projectPointKittiSeperateEigen(cv::Mat &depthMap, pandar_pointcloud::PointXYZIT &point);

    void projectPointKittiEigen(cv::Mat &depthMap, pandar_pointcloud::PointXYZIT &point);

    void projectData(string inFile, cv::Mat &depthMapCamera, cv::Mat &depthMapLiDAR, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloudPart, LiDARDataType dataType = BIN, LiDARDataSource dataSource = KITTI, LiDARPointType poinyType = XYZI, ExtrinsicDirection liDARDirection = L2C, MatType matType = EIGEN);

    void projectData(string inFile, cv::Mat &depthMapCamera, cv::Mat &depthMapLiDAR, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloudCamera, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloudLiDAR, LiDARDataType dataType = BIN, LiDARDataSource dataSource = KITTI, LiDARPointType poinyType = XYZI, ExtrinsicDirection liDARDirection = L2C, MatType matType = EIGEN);

    void updateParameters(cv::Mat &rotation, cv::Mat &translation);

    void projectPointInverse(cv::Point2f &point, float depth, pcl::PointXYZ &point3d);


private:
    LiDARCalibParaKitti _liDARCalibParaKitti;
    LiDARCalibParaKittiInverse _liDARCalibParaKittiInverse;
    LiDARCalibParaKittiInverseEigen _liDARCalibParaKittiInverseEigen;

    void initLiDARCalibParaKitti(LiDARCalibParaKitti liDARCalibParaKitti);

    void initLiDARCalibParaKittiInverse(LiDARCalibParaKittiInverse liDARCalibParaKittiInverse);

    void initLiDARCalibParaKittiInverseEigen(LiDARCalibParaKittiInverseEigen &liDARCalibParaKittiInverseEigen);
};



#endif //PROJECT_TEST_H

#pragma clang diagnostic pop