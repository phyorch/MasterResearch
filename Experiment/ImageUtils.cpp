//
// Created by phyorch on 19/12/18.
//

#include "ImageUtils.h"


void ImageUtils::colorTransfer(cv::Mat &depthMap, cv::Mat &depthImage) {
    if (depthMap.empty() || depthMap.rows == 0 || depthMap.rows != depthImage.rows || depthMap.cols != depthImage.cols){
        cerr << "Invalid input image for color transfer" << endl;
        exit(EXIT_FAILURE);
    }
    float maxVal, minVal;
    ImageUtils::maxMat<float>(depthMap, maxVal);
    ImageUtils::minMat<float>(depthMap, minVal);
    for (int y = 0; y<depthMap.rows; y++)
    {
        for (int x = 0; x<depthMap.cols; x++)
        {
            if(depthMap.at<float>(y, x)>0){
                uchar r, g, b;
                float color = (depthMap.at<float>(y, x) - minVal) / (20 - minVal) * 255;
                if(color>255){
                    color = 255;
                }
                r = uchar(255 - color);
                g = color < 128 ? uchar(color * 2) : uchar((255 - color) * 2);
                b = uchar(color);

                depthImage.at<cv::Vec3b>(y, x) = cv::Vec3b(b, g, r);
            }
        }
    }
}



void ImageUtils::disparityBoader(cv::Mat &image, cv::Mat &imageBoard, int &numDisparities, int &minDisparity) {
    if (image.empty() || image.rows == 0)
    {
        cerr << "Empty input";
        exit(EXIT_FAILURE);
    }
    imageBoard = image.colRange(numDisparities + minDisparity, image.cols);
}

void ImageUtils::disparityBoader(cv::Mat &image, cv::Mat &imageBoard, int &boader) {
    if (image.empty() || image.rows == 0)
    {
        cerr << "Empty input";
        exit(EXIT_FAILURE);
    }
    imageBoard = image.rowRange(boader, image.rows);
}

void ImageUtils::creatMapRegion(cv::Mat &map, cv::Mat &mapRegion, int rowsBegin, int rowsEnd, int colsBegin, int colsEnd) {
    if (map.rows == 0 || map.cols == 0)
    {
        cerr << "Invalid input for map in testMapBoarder";
        exit(EXIT_FAILURE);
    }
    mapRegion = map.rowRange(rowsBegin, rowsEnd);
    mapRegion = mapRegion.colRange(colsBegin, colsEnd);
}

void ImageUtils::creatMapRegion(cv::Mat &map, cv::Mat &mapRegion, cv::Mat &diagonalPoints) {
    if (map.rows == 0 || map.cols == 0)
    {
        cerr << "Invalid input for map in testMapBoarder";
        exit(EXIT_FAILURE);
    }
    if(diagonalPoints.rows != 2 && diagonalPoints.cols != 2){
        cerr << "Invalid 2x2 diagonal points input for the region";
        exit(EXIT_FAILURE);
    }
    mapRegion = map.rowRange(diagonalPoints.at<int>(0,1), diagonalPoints.at<int>(1,1));
    mapRegion = mapRegion.colRange(diagonalPoints.at<int>(0,0), diagonalPoints.at<int>(1,0));
}

void ImageUtils::creatMapRegionSet(cv::Mat &map, vector<cv::Mat> &mapRegionSet, vector<cv::Mat> &diagonalPointsSet) {
    for(int i=0; i<diagonalPointsSet.size(); i++){
        cv::Mat diagonalPoints = diagonalPointsSet[i];
        cv::Mat mapRegion;
        creatMapRegion(map, mapRegion, diagonalPoints);
        mapRegionSet.push_back(mapRegion);
    }
}

void ImageUtils::drawRect(cv::Mat &map, cv::Mat &diagonalPoints) {
    if (map.rows == 0 || map.cols == 0)
    {
        cerr << "Invalid input for map in testMapBoarder";
        exit(EXIT_FAILURE);
    }
    if(diagonalPoints.rows != 2 && diagonalPoints.cols != 2){
        cerr << "Invalid 2x2 diagonal points input for the region";
        exit(EXIT_FAILURE);
    }
    cv::Point p1 = cv::Point(diagonalPoints.at<int>(0,0), diagonalPoints.at<int>(0,1));
    cv::Point p2 = cv::Point(diagonalPoints.at<int>(1,0), diagonalPoints.at<int>(1,1));
    cv::rectangle(map, p1, p2, cv::Scalar(0,255,255));
}

void ImageUtils::drawRectSet(cv::Mat &map, vector<cv::Mat> &diagonalPointsSet) {
    for(int i=0; i<diagonalPointsSet.size(); i++){
        cv::Mat diagonalPoints = diagonalPointsSet[i];
        drawRect(map, diagonalPoints);
    }
}

int ImageUtils::maxPooling(cv::Mat &sparse, cv::Mat &dense, int size) {
    int width = sparse.cols / size;
    int height = sparse.rows / size;
    //cv::Mat dense_map = cv::Mat::zeros(height, width, CV_8UC1);
    for(int i=0; i<width; i++){
        for(int j=0; j<height; j++){
            //uchar test = maxMat(sparse(cv::Rect(i*size, j*size, size, size-1)));
            //dense.at<uchar>(i,j) = maxMat(sparse(cv::Rect(j*size, i*size, size, size)));
        }
    }
    return 1;
}

void ImageUtils::pointCorrespond(cv::Mat &cameraDepthMap, cv::Mat &liDARDepthMap, cv::Mat &imageCorresponded) {
    if (cameraDepthMap.size() != liDARDepthMap.size())
    {
        cerr << "Invalid input for point correspondence";
        exit(EXIT_FAILURE);
    }
    for(int i=0; i<cameraDepthMap.rows; i++){
        for(int j=0; j<cameraDepthMap.cols; j++){
            if(liDARDepthMap.at<uchar>(i, j) == 0){
                cameraDepthMap.at<uchar>(i, j) = 0;
            }
        }
    }
}

void ImageUtils::histogramCount(cv::Mat &image, Eigen::RowVectorXi &histogram) {
//    uchar min_val = minMat(image);
//    uchar max_val = maxMat(image);
//    histogram = Eigen::RowVectorXi::Zero(max_val - min_val +1);
//    for(int i=0; i<image.rows; i++){
//        for(int j=0; j<image.cols; j++){
//            uchar val = image.at<uchar>(i,j);
//            if (val>0){
//                histogram[val - min_val] = histogram[val - min_val] + 1;
//            }
//        }
//    }
//    printHistogram(histogram, max_val, min_val);
}

void ImageUtils::histogramCount(cv::Mat &image, cv::Mat &imageRef, Eigen::RowVectorXi &histogram) {
//    uchar min_val = minMat(imageRef);
//    uchar max_val = maxMat(imageRef);
//    histogram = Eigen::RowVectorXi::Zero(max_val - min_val +1);
//    for(int i=0; i<image.rows; i++){
//        for(int j=0; j<image.cols; j++){
//            uchar val = image.at<uchar>(i,j);
//            if (val>min_val && val<max_val){
//                histogram[val - min_val] = histogram[val - min_val] + 1;
//            }
//        }
//    }
}

void ImageUtils::histogramCountCorrespond(cv::Mat &imageCamera, cv::Mat &imageLiDAR, Eigen::RowVectorXi &histogram) {
//    cv::Mat imageCameraCorresponded = imageCamera;
//    pointCorrespond(imageCamera, imageLiDAR, imageCameraCorresponded);
//    uchar min_val = minMat(imageCameraCorresponded);
//    uchar max_val = maxMat(imageCameraCorresponded);
//    histogram = Eigen::RowVectorXi::Zero(max_val - min_val +1);
//    for(int i=0; i<imageCameraCorresponded.rows; i++){
//        for(int j=0; j<imageCameraCorresponded.cols; j++){
//            uchar val = imageCameraCorresponded.at<uchar>(i,j);
//            if (val>0){
//                histogram[val - min_val] = histogram[val - min_val] + 1;
//            }
//        }
//    }
//    printHistogram(histogram, max_val, min_val);
}

void ImageUtils::printHistogram(Eigen::RowVectorXi &histogram, uchar maxVal, uchar minVal) {
    cout << "min value:   " << int(minVal) << endl;
    cout << "max value:   " << int(maxVal) << endl;
    cout << "histogram:   " << histogram << endl;
}

double ImageUtils::histogramDistance(cv::Mat &region_camera, cv::Mat &region_lidar) {
    Eigen::RowVectorXi histogram_camera;
    Eigen::RowVectorXi histogram_lidar;
    cout << "camera:  " << endl;
    histogramCount(region_camera, histogram_camera);
    cout << "lidar:  " << endl;
    histogramCount(region_lidar, histogram_lidar);
    //double cos = double(histogram_camera.dot(histogram_lidar)) / (histogram_camera.norm() * histogram_lidar.norm());
    //cout << "cos distance:   " << cos << endl;
    //return cos;
    return 0;
}

double ImageUtils::imageDistance(cv::Mat &cameraDepthMap, cv::Mat &liDARDepthMap, cv::Size &regionNum) {
    double dist = 0;
    Eigen::RowVectorXi camera_hist;
    Eigen::RowVectorXi lidar_hist;
    cv::Size region_size;
    region_size.width = cameraDepthMap.cols / regionNum.width;
    region_size.height = cameraDepthMap.rows / regionNum.height;
    for(int i=0; i<regionNum.width; i++){
        for(int j=0; j<regionNum.height; j++){
            cv::Mat camera_region (cameraDepthMap, cv::Rect(i * region_size.width, j * region_size.height, region_size.width, region_size.height));
            cv::Mat lidar_region (liDARDepthMap, cv::Rect(i * region_size.width, j * region_size.height, region_size.width, region_size.height));
            cout << "For " << i << "," << j <<endl;
            dist += histogramDistance(camera_region, lidar_region);
        }
    }
    return dist;
}

void DiagnosisUtils::distanceCorrespond(cv::Mat &mapCamera, cv::Mat &mapLiDAR, float rangeMin, float rangeMax) {
    for(int i=0; i<mapCamera.rows; i++){
        for(int j=0; j<mapCamera.cols; j++){
            if(mapCamera.at<float>(i, j) > rangeMin && mapCamera.at<float>(i, j) < rangeMax && mapLiDAR.at<float>(i, j) !=0){
                cout << i << "  " << j << "  " << mapLiDAR.at<float>(i ,j) << endl;
            }
        }
    }
}

void DiagnosisUtils::distanceCorrespondWrite(cv::Mat &mapCamera, cv::Mat &mapLiDAR, float rangeMin, float rangeMax,
                                             string outputPath) {
    ofstream outFile;
    outFile.open(outputPath, ios::out);
    outFile << "LiDAR" << endl;

    for(int i=0; i<mapCamera.rows; i++){
        for(int j=0; j<mapCamera.cols; j++){
            if(mapCamera.at<float>(i, j) > rangeMin && mapCamera.at<float>(i, j) < rangeMax && mapLiDAR.at<float>(i, j) !=0){
                outFile << mapLiDAR.at<float>(i ,j) << endl;
            }
        }
    }
    outFile.close();
}