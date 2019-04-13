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
#include <random>

#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include "opencv2/core.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/ximgproc/disparity_filter.hpp"
#include "Sensor.h"
#include <boost/format.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
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

    static float point2PointDistanceKitti(cv::Mat &mapCamera, cv::Mat &mapLiDAR);

    static float point2PointDistanceTotal(cv::Mat &mapCamera, cv::Mat &mapLiDAR, vector<cv::Mat> &diagonalPointsSet);

    static float point2PointDistanceFrame(cv::Mat &mapCamera, cv::Mat &mapLiDAR);

    static float pointCloudDistance(cv::Mat &mapCamera, cv::Mat &mapLiDAR);

};

class Transfer{
public:

    static void vector2MatSeperate(vector<double> &theta, cv::Mat &rotation, cv::Mat &translation);

    static void array2MatSeperate(const double *theta, cv::Mat &rotation, cv::Mat &translation);

    static void mat2VectorSeperate(cv::Mat &rotation, cv::Mat &translation, vector<double> &theta);

    static void array2Eigen(const double *theta, Eigen::Matrix4f &transformation);

    static void cv2EigenSeperate(cv::Mat &rotation, cv::Mat &translation, Eigen::Matrix4f &transformation);

    static void matSeperate2Mat(cv::Mat &rotation, cv::Mat &translation, cv::Mat &transformation);

    static void Eigen2MatSeperate(Eigen::Matrix4f &transformation, cv::Mat &rotation, cv::Mat &translation);

    static void vector2Eigen(vector<double> &theta, Eigen::Matrix4f &transformation);

    static void depthAnalysis(cv::Mat &iamgeCamera, cv::Mat &imageLiDAR, int depth, string csvPath);

    static void depthDistribution(cv::Mat &imageLiDAR, string csvPath);

};

class PointCloudAlignment{
public:

    static void getCameraPointCloud(cv::Mat &depthMapCamera, LiDAR &lidar, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloudCamera);
    
    static void getLiDARPointCloudXYZ(pcl::PointCloud<pcl::PointXYZI>::Ptr &pointCloudXYZI, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloudXYZ);

    static void getCameraSparsePointCloudKitti(cv::Mat &depthMapCamera, cv::Mat &depthMapLiDAR, LiDAR &lidar, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloudSparse);

    static void pointCloudDownsample(pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloudDownsampled, float gridSize);

    static void pointCloudDownsample(pcl::PointCloud<pcl::PointXYZI>::Ptr &pointCloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &pointCloudDownsampled, float gridSize);

    static float findScaling(pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloudCamera, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloudLiDAR);

    static void pointCloudScaling(pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud, float scale, pcl::PointCloud<pcl::PointXYZ>::Ptr &transformedCloud);

    static float chamferDistanceElem(pcl::PointXYZ &point, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud);

    static float chamferDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloudCamera, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloudLiDAR);
};

class HandEyeCalibration {
public:

    static string zfill(int dataNum);

    static void imageRead(int begin, int end, string dataRoot, vector<string> &dataList);

    static void cloudRead(int begin, int end, string dataRoot, vector<string> &dataList);

    static void depthRead(int begin, int end, string dataRoot, vector<string> &dataList);

    static void findFeatureMatches(cv::Mat &image1, cv::Mat &image2, vector<cv::KeyPoint> &keyPoints1, vector<cv::KeyPoint> &keyPoints2, vector<cv::DMatch> &matches);

    static void creat3D2DPoints(LiDAR &lidar, cv::Mat &depthMapCamera, vector<cv::KeyPoint> &keyPoints1, vector<cv::KeyPoint> &keyPoints2,
                                vector<cv::DMatch> &matches, vector<cv::Point3f> &points3d, vector<cv::Point2f> &points2d);

    static void cameraRegistration(cv::Mat &image1, cv::Mat &image2, vector<cv::KeyPoint> &keyPoints1, vector<cv::KeyPoint> &keyPoints2, vector<cv::DMatch> &matches,
                                   cv::Mat &depthMapCamera, cv::Mat &cameraMatrix, LiDAR &liDAR, cv::Mat &transformationCamera);

    static void pointCloudRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud2, float voxVolum, cv::Mat &transformation);

    static void skew(cv::Mat &matOriginal, cv::Mat &matSkew);

    static void handEyeTsai(cv::Mat &transformationCameraLiDAR, cv::Mat &transformationLiDAR, cv::Mat &transformationCamera);

    static void handEyeTsai(cv::Mat &transformationCameraLiDAR, vector<cv::Mat> transformationLiDAR, vector<cv::Mat> transformationCamera);
};

class Refinement{
public:

    static bool validRegion(cv::Mat &depthMapLiDAR, cv::Point &point, cv::Point &region, int threshold);

    static void slideElimination(cv::Mat &depthMapLiDAR, cv::Point &slideWindowSize, cv::Point &slideWindowRange, cv::Point &slideWindowRegion, int elimiThreshold);

    static void slideElimination2(cv::Mat &depthMapLiDAR, cv::Mat &edgeMapLiDAR, cv::Point &slideWindowSize, cv::Point &slideWindowRange, float elimiThreshold, int setOne = 0);

    static void gaussianBlurModified(cv::Mat &edgeMap, cv::Mat &edgeMapBlured, int filterSize);

    static void gaussianBlurModified(cv::Mat &edgeMap, cv::Mat &filter, cv::Mat &edgeMapBlured);

    static void cameraEdgeGeneration(cv::Mat &imageCamera, cv::Mat &edgeMapCamera, cv::Mat &edgeMapCameraBlured, int blur = 0, int blurSize = 0);

    static float edgeDistance(cv::Mat &edgeMapCamera, cv::Mat &edgeMapLiDAR, int cnt);

    static void saveMatchResult(cv::Mat &edgeMapCamera, cv::Mat &edgeMapLiDAR, string savePath, int number);

    static float errorRotation(cv::Mat &rotationResult, cv::Mat &rotationTruth);

    static float errorTranslation(cv::Mat &translationResult, cv::Mat &translationTruth);

    static void errorWrite(ofstream &outFile, float time, float errorRotation, float errorTranslation);

    static void distanceWrite(ofstream &outFile, float time, float lastDistance, float referenceDistance);

    static void distanceErrorWrite(ofstream &outFile, float time, float lastDistance, float referenceDistance, float errorRotation, float errorTranslation);
};


#endif //PROJECT_SIMILARITYMEASURE_H
