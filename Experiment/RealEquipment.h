//
// Created by phyorch on 13/12/18.
//

#ifndef PROJECT_REALEQUIPMENT_H
#define PROJECT_REALEQUIPMENT_H

// ZED include
#include <sl/Camera.hpp>
#include "hesaiLidarSDK.h"

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

using namespace std;

class ZEDCamera{
public:
    ZEDCamera(sl::Camera &camera);
    virtual ~ZEDCamera(void);

    void getDepthMap(sl::Mat &pointCloud, cv::Mat &depthMap);

    void getDepthMapAndPointCloud(sl::Mat &pointCloud, cv::Mat &depthMap, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloudPCL);

    void getDepthMapView(cv::Mat &depthMap, cv::Mat &outputMap);

    void nanComplete(cv::Mat &depthMap);

    cv::Mat slMat2cvMat(sl::Mat& input);

    void getOneDepthFrame(sl::Camera &camera, HesaiLidarSDK &liDAR, cv::Mat &depthMapCamera, string &outFileCameraDepthMap, string outFileCameraImage = " ", string outFileDepthView = " ");


private:
    sl::Resolution _mapSize;


};

class PandarLiDAR{
public:
    PandarLiDAR();
    virtual ~PandarLiDAR(void);

    void getXYZtxt(pcl::PointCloud<pandar_pointcloud::PointXYZIT>::Ptr &cloud, string &inPath, string &outPath);

    void getXYZtxtST(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, string &inPath, string &outPath);

    void XYZIT2XYZI(pcl::PointCloud<pandar_pointcloud::PointXYZIT>::Ptr &cloudXYZIT, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloudXYZI, string &inPath);

};


#endif //PROJECT_REALEQUIPMENT_H
