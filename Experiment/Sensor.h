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

class PointCloudAlignment;

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
enum LiDARPointType {XYZIT, XYZI, XYZ};
enum ExtrinsicDirection{C2L, L2C};
enum MatType{EIGEN, CV};

struct LiDARCalibParaKitti{
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

    PointCloudAlignment *pointCloudAlignment;

    LiDAR(LiDARCalibParaKitti lidarCalibParaKitti);

    virtual ~LiDAR(void);

    static void convertKittiBinData(string &inFile, string &outFile); //, vector *pointCloud

    void projectPointKittiSeperate(cv::Mat &depthMapLiDAR, pcl::PointXYZ &point, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPart);

    void projectPointKittiSeperate(cv::Mat &depthMapLiDAR, pcl::PointXYZI &point, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPart);

    void projectPointKittiSeperate(cv::Mat &depthMapLiDAR, pandar_pointcloud::PointXYZIT &point, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPart, int colDeviation = 0);

    void projectData(string inFile, cv::Mat &depthMapLiDAR, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloudPart, LiDARPointType poinyType = XYZI, float downsampleVox = 0, int colDeviation = 0);

    void updateParameters(cv::Mat &rotation, cv::Mat &translation);

    void projectPointInverse(cv::Point2f &point, float depth, pcl::PointXYZ &point3d, int colDeviation = 0);

    void projectPointInverse(cv::Point2f &point2d, float depth, cv::Point3f &point3d);

    void projectPointInverseKitti(cv::Point2f &point, int depth, pcl::PointXYZ &point3d);


private:
    LiDARCalibParaKitti _liDARCalibParaKitti;

    void initLiDARCalibParaKitti(LiDARCalibParaKitti liDARCalibParaKitti);
};



#endif //PROJECT_TEST_H

#pragma clang diagnostic pop