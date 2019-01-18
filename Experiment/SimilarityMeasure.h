//
// Created by phyorch on 07/12/18.
//

#ifndef PROJECT_SIMILARITYMEASURE_H
#define PROJECT_SIMILARITYMEASURE_H

#include <iostream>
#include <fstream>
#include <string>
#include <limits>
#include<stdlib.h>
#include <time.h>

#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include "opencv2/core.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/ximgproc/disparity_filter.hpp"
#include "Disparity.h"
#include <boost/format.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <sl/Camera.hpp>


using namespace std;

struct SubRegion{
    int RowStart;
    int RowEnd;
    int ColumStart;
    int ColumEnd;
};


class HistogramGeneration{
public:

    static void pointCorrespond(cv::Mat &cameraDepthMap, cv::Mat &liDARDepthMap, cv::Mat &imageCorresponded);

    static vector<int> histogramCount(cv::Mat &image, Eigen::RowVectorXi &histogram);

    static void histogramCount(cv::Mat &image, cv::Mat &imageRef, Eigen::RowVectorXi &histogram);

    static int histogramBoundary(vector<int> &boundaryCamera, vector<int> &boundaryLiDAR);

    static void histogramCompletion(Eigen::RowVectorXi &histCamera, Eigen::RowVectorXi &histLiDAR, vector<int> &boundaryCamera,
                                    vector<int> &boundaryLiDAR, vector<int> &vectorCamera, vector<int> &vectorLiDAR);

    static void histogramCompletion(Eigen::RowVectorXi &histogram, vector<int> &vecTor, vector<int> &boundary, int maxBound);

    static void histogramCountCorrespond(cv::Mat &imageCamera, cv::Mat &imageLiDAR, Eigen::RowVectorXi &histogram);

    static void printHistogram(Eigen::RowVectorXi &histogram, uchar maxVal, uchar minVal);

    static void printHistogram(vector<int> &histogram, vector<int> &boundary);

    static double histogramDistance(cv::Mat &region_camera, cv::Mat &region_lidar);

    static void histogramWrite(string csvPath, vector<int> &vectorCamera, vector<int> &vectorLiDAR);

    static void histogramWrite(string csvPath, Eigen::RowVectorXi &vecTor);

    static void histogramWrite(string csvPath, vector<int> &vecTor);




    static void histogramWrite(string csvPath, cv::Mat &map);

    static void histogramWrite(string csvPathCamera, string csvPathLiDAR, cv::Mat &mapCamera, cv::Mat &mapLiDAR);

    static void map2Histogram(cv::Mat &map, vector<float> &histogram, int truncationBegin, int truncationEnd);

    static void histogramDownsampling(vector<float> &histogram, vector<float> &histogramDownsampled, int DownsamplingSize);

};

class HistogramMeasure{
public:

    static void vectorToHist(vector<int> &vectorVariable, Eigen::RowVectorXi &histogramVariable);

    static double cosDistance(vector<int> &vectorCamera, vector<int> &vectorLiDAR);

    static bool detectMinusValue(vector<float> &histogram);

    static double mapKLDivergence(cv::Mat &mapCamera, cv::Mat &mapLiDAR, vector<cv::Mat> &diagonalPointsSet);

    static float point2PointDistance(cv::Mat &mapCamera, cv::Mat &mapLiDAR);

    static float point2PointDistanceTotal(cv::Mat &mapCamera, cv::Mat &mapLiDAR, vector<cv::Mat> &diagonalPointsSet);

    static float pointCloudDistance(cv::Mat &mapCamera, cv::Mat &mapLiDAR);

};

class Transfer{
public:

    static void vector2MatSeperate(vector<double> &theta, cv::Mat &rotation, cv::Mat &translation);

    static void array2MatSeperate(const double *theta, cv::Mat &rotation, cv::Mat &translation);

    static void mat2VectorSeperate(cv::Mat &rotation, cv::Mat &translation, vector<double> &theta);

    static void array2Eigen(const double *theta, Eigen::Matrix4f &transformation);

    static void vector2Eigen(vector<double> &theta, Eigen::Matrix4f &transformation);

    static void depthAnalysis(cv::Mat &iamgeCamera, cv::Mat &imageLiDAR, int depth, string csvPath);

    static void depthDistribution(cv::Mat &imageLiDAR, string csvPath);
};

class PointCloudAlignment{
public:

    static void getCameraSparsePointCloud(cv::Mat &depthMapCamera, cv::Mat &depthMapLiDAR, LiDAR &lidar, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloudSparse);

    static void pointCloudDownsample(pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloudDownsampled, float gridSize);

    static float findScaling(pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloudCamera, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloudLiDAR);

    static void pointCloudScaling(pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud, float scale, pcl::PointCloud<pcl::PointXYZ>::Ptr &transformedCloud);

    static float chamferDistanceElem(pcl::PointXYZ &point, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud);

    static float chamferDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloudCamera, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloudLiDAR);
};


#endif //PROJECT_SIMILARITYMEASURE_H
