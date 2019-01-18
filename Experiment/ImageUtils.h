//
// Created by phyorch on 19/12/18.
//

#ifndef PROJECT_IMAGEUTILS_H
#define PROJECT_IMAGEUTILS_H

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


class ImageUtils
{
public:

    static void colorTransfer(cv::Mat &depthMap, cv::Mat &depthImage);

    static void neighborDyeingElem(cv::Mat &depthMap, cv::Point &point, float pxValue, cv::Point &size, cv::Mat &depthMapDyed);

    static void neighborDyeing(cv::Mat &depthMap, cv::Point &size, cv::Mat &depthMapDyed);

    static void disparityBoader(cv::Mat &image, cv::Mat &imageBoard, int &numDisparities, int &minDisparity);

    static void disparityBoader(cv::Mat &image, cv::Mat &imageBoard, int &boader);

    static void creatMapRegion(cv::Mat &map, cv::Mat &mapRegion, int rowsBegin, int rowsEnd, int colsBegin, int colsEnd);

    static void creatMapRegion(cv::Mat &map, cv::Mat &mapRegion, cv::Mat &diagonalPoints);

    static void creatMapRegionSet(cv::Mat &map, vector<cv::Mat> &mapRegionSet, vector<cv::Mat> &diagonalPointsSet);

    static void drawRect(cv::Mat &map, cv::Mat &diagonalPoints);

    static void drawRectSet(cv::Mat &map, vector<cv::Mat> &diagonalPointsSet);

    static int maxPooling(cv::Mat &sparse, cv::Mat &dense, int size);

    template<typename T> static void maxMat(cv::Mat &image, T &max){
        if (image.empty() || image.rows == 0)
        {
            cerr << "Empty input for maxMat";
            exit(EXIT_FAILURE);
        }
        max = 0;
        for(int i=0; i<image.rows; i++){
            for(int j=0; j<image.cols; j++){
                if(image.at<T>(i,j) > max){
                    max = image.at<T>(i,j);
                }
            }
        }
    }

    template<typename T> static void minMat(cv::Mat &image, T &min){
        if (image.empty() || image.rows == 0)
        {
            cerr << "Empty input for minMat";
            exit(EXIT_FAILURE);
        }
        min = 200;
        for(int i=0; i<image.rows; i++){
            for(int j=0; j<image.cols; j++){
                //uchar test = image.at<uchar>(i,j);
                if(image.at<T>(i,j) < min && image.at<T>(i,j) > 0){
                    min = image.at<T>(i,j);
                }
            }
        }
    }

//----------------------------------------------------------------------------------------------------------------------
//This part is used to analyze the histogram manually, without opencv histogram compare function
    static void pointCorrespond(cv::Mat &cameraDepthMap, cv::Mat &liDARDepthMap, cv::Mat &imageCorresponded);

    static void histogramCount(cv::Mat &image, Eigen::RowVectorXi &histogram);

    static void histogramCount(cv::Mat &image, cv::Mat &imageRef, Eigen::RowVectorXi &histogram);

    static void histogramCountCorrespond(cv::Mat &imageCamera, cv::Mat &imageLiDAR, Eigen::RowVectorXi &histogram);

    static double histogramDistance(cv::Mat &region_camera, cv::Mat &region_lidar);

    static void printHistogram(Eigen::RowVectorXi &histogram, uchar maxVal, uchar minVal);

    static double imageDistance(cv::Mat &cameraDepthMap, cv::Mat &liDARDepthMap, cv::Size &regionSize);
//----------------------------------------------------------------------------------------------------------------------
};


class DiagnosisUtils
{
public:

    static void distanceCorrespond(cv::Mat &mapCamera, cv::Mat &mapLiDAR, float rangeMin, float rangeMax);

    static void distanceCorrespondWrite(cv::Mat &mapCamera, cv::Mat &mapLiDAR, float rangeMin, float rangeMax, string outputPath);

    static void regionDiagnosis(cv::Mat &cameraMap, cv::Mat &liDARMap, cv::Mat &region);

    static int mapPositiveCount(cv::Mat &map);

};


#endif //PROJECT_IMAGEUTILS_H
