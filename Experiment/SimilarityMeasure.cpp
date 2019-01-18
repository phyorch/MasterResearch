//
// Created by phyorch on 07/12/18.
//

#include "SimilarityMeasure.h"
#include "Disparity.h"
#include "ImageUtils.h"

void HistogramGeneration::pointCorrespond(cv::Mat &cameraDepthMap, cv::Mat &liDARDepthMap, cv::Mat &imageCorresponded) {
    if (cameraDepthMap.size() != liDARDepthMap.size())
    {
        cerr << "Invalid input for point correspondence";
        exit(EXIT_FAILURE);
    }
    imageCorresponded = cv::Mat::zeros(cameraDepthMap.rows, cameraDepthMap.cols, CV_8UC1);
    for(int i=0; i<cameraDepthMap.rows; i++){
        for(int j=0; j<cameraDepthMap.cols; j++){
            if(liDARDepthMap.at<uchar>(i, j) != 0){
                imageCorresponded.at<uchar>(i, j) = cameraDepthMap.at<uchar>(i, j);
            }
        }
    }
}

vector<int> HistogramGeneration::histogramCount(cv::Mat &image, Eigen::RowVectorXi &histogram) {
//    int minVal = ImageUtils::minMat(image);
//    int maxVal = ImageUtils::maxMat(image);
//    int total = 0;
//    histogram = Eigen::RowVectorXi::Zero(maxVal - minVal +1);
//    for(int i=0; i<image.rows; i++){
//        for(int j=0; j<image.cols; j++){
//            uchar val = image.at<uchar>(i,j);
//            if (val>0){
//                total++;
//                histogram[val - minVal] = histogram[val - minVal] + 1;
//            }
//        }
//    }
//    //printHistogram(histogram, maxVal, minVal);
//    vector<int> boundary;
//    boundary.push_back(minVal);
//    boundary.push_back(maxVal);
//    boundary.push_back(total);
//    return boundary;
}

void HistogramGeneration::histogramCount(cv::Mat &image, cv::Mat &imageRef, Eigen::RowVectorXi &histogram) {
//    uchar min_val = ImageUtils::minMat(imageRef);
//    uchar max_val = ImageUtils::maxMat(imageRef);
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

int HistogramGeneration::histogramBoundary(vector<int> &boundaryCamera, vector<int> &boundaryLiDAR) {
    int maxCamera = boundaryCamera[1];
    int maxLiDAR = boundaryLiDAR[1];
    if(maxCamera>maxLiDAR){
        return maxCamera;
    }
    else{
        return maxLiDAR;
    }
}

void HistogramGeneration::histogramCompletion(Eigen::RowVectorXi &histCamera, Eigen::RowVectorXi &histLiDAR, vector<int> &boundaryCamera,
                                         vector<int> &boundaryLiDAR, vector<int> &vectorCamera, vector<int> &vectorLiDAR) {
    int minCamera = boundaryCamera[0];
    int maxCamera = boundaryCamera[1];
    int minLiDAR = boundaryLiDAR[0];
    int maxLiDAR = boundaryLiDAR[1];
    for(int i=0; i<maxCamera; i++){
        if(i<minCamera-1){
            vectorCamera.push_back(0);
        }
        else{
            vectorCamera.push_back(histCamera[i-minCamera+1]);
        }
    }
    for(int i=0; i<maxLiDAR; i++){
        if(i<minLiDAR-1){
            vectorLiDAR.push_back(0);
        }
        else{
            vectorLiDAR.push_back(histLiDAR[i-minLiDAR+1]);
        }
    }
    if(maxCamera<maxLiDAR){
        for(int i=0; i<maxLiDAR-maxCamera; i++){
            vectorCamera.push_back(0);
        }
    }
    if(maxCamera>maxLiDAR){
        for(int i=0; i<maxCamera-maxLiDAR; i++){
            vectorLiDAR.push_back(0);
        }
    }
}

void HistogramGeneration::histogramCompletion(Eigen::RowVectorXi &histogram, vector<int> &vecTor, vector<int> &boundary, int maxBound){
    int minVal = boundary[0];
    int maxVal = boundary[1];
    for(int i=0; i<maxVal; i++){
        if(i<minVal-1){
            vecTor.push_back(0);
        }
        else{
            vecTor.push_back(histogram[i-minVal+1]);
        }
    }
    for(int i=maxVal; i<maxBound; i++){
        vecTor.push_back(0);
    }
}


void HistogramGeneration::histogramCountCorrespond(cv::Mat &imageCamera, cv::Mat &imageLiDAR, Eigen::RowVectorXi &histogram) {
//    cv::Mat imageCameraCorresponded = imageCamera;
//    pointCorrespond(imageCamera, imageLiDAR, imageCameraCorresponded);
//    uchar min_val = ImageUtils::minMat(imageCameraCorresponded);
//    uchar max_val = ImageUtils::maxMat(imageCameraCorresponded);
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

void HistogramGeneration::printHistogram(Eigen::RowVectorXi &histogram, uchar maxVal, uchar minVal) {
    cout << "min value:   " << int(minVal) << endl;
    cout << "max value:   " << int(maxVal) << endl;
    cout << "histogram:   " << histogram << endl;
}

void HistogramGeneration::printHistogram(vector<int> &histogram, vector<int> &boundary) {
    cout << "min value:   " << int(boundary[0]) << endl;
    cout << "max value:   " << int(boundary[1]) << endl;
    for(int i=0; i<histogram.size(); i++){
        cout << int(histogram[i]) << "  ";
    }
}

double HistogramGeneration::histogramDistance(cv::Mat &imageCamera, cv::Mat &imageLiDAR) {
    Eigen::RowVectorXi histogramCamera;
    Eigen::RowVectorXi histogramLiDAR;
    vector<int> vectorCamera;
    vector<int> vectorLiDAR;

    vector<int> boundaryCamera = histogramCount(imageCamera, histogramCamera);
    vector<int> boundaryLiDAR = histogramCount(imageLiDAR, histogramLiDAR);
    histogramCompletion(histogramCamera, histogramLiDAR, boundaryCamera, boundaryLiDAR, vectorCamera, vectorLiDAR);
    cout << "camera:  " << endl;
    printHistogram(vectorCamera, boundaryCamera);
    cout << endl << "lidar:  " << endl;
    printHistogram(vectorLiDAR, boundaryLiDAR);

    //double cos = double(histogram_camera.dot(histogram_lidar)) / (histogram_camera.norm() * histogram_lidar.norm());
    //cout << "cos distance:   " << cos << endl;
    //return cos;
    return 0;
}

void HistogramGeneration::histogramWrite(string csvPath, vector<int> &vectorCamera, vector<int> &vectorLiDAR) {
    if (vectorCamera.size() != vectorLiDAR.size())
    {
        cerr << "Invalid input for csv output";
        exit(EXIT_FAILURE);
    }
    ofstream out_file;
    out_file.open(csvPath, ios::out);
    out_file << "Camera" << "," << "LiDAR" << endl;

    for(int i=0; i<vectorCamera.size(); i++) {
        out_file << int(vectorCamera[i]) << "," << int(vectorLiDAR[i]) << endl;
    }
    out_file.close();
}

void HistogramGeneration::histogramWrite(string csvPath, Eigen::RowVectorXi &vecTor) {
    ofstream outFile;
    outFile.open(csvPath, ios::out);
    outFile << "Camera" << endl;

    for(int i=0; i<vecTor.size(); i++) {
        outFile << int(vecTor[i]) << endl;
    }
    outFile.close();
}

void HistogramGeneration::histogramWrite(string csvPath, vector<int> &vecTor){
    ofstream outFile;
    outFile.open(csvPath, ios::out);
    outFile << "Camera" << endl;

    for(int i=0; i<vecTor.size(); i++) {
        outFile << int(vecTor[i]) << endl;
    }
    outFile.close();
}

void HistogramGeneration::histogramWrite(string csvPath, cv::Mat &map) {
    ofstream outFile;
    outFile.open(csvPath, ios::out);
    outFile << "Element" << endl;

    for(int i=0; i<map.rows; i++){
        for(int j=0; j<map.cols; j++){
            if(map.at<float>(i, j)>0 && map.at<float>(i, j)<8){
                outFile << map.at<float>(i, j) << endl;
            }
        }
    }
}

void HistogramGeneration::histogramWrite(string csvPathCamera, string csvPathLiDAR, cv::Mat &mapCamera, cv::Mat &mapLiDAR) {
    ofstream outFileCamera;
    outFileCamera.open(csvPathCamera, ios::out);
    outFileCamera << "Element" << endl;

    ofstream outFileLiDAR;
    outFileLiDAR.open(csvPathLiDAR, ios::out);
    outFileLiDAR << "Element" << endl;

    for(int i=0; i<mapCamera.rows; i++){
        for(int j=0; j<mapCamera.cols; j++){
            if(mapLiDAR.at<float>(i, j)>0 && mapCamera.at<float>(i, j)<8){
                outFileCamera << mapCamera.at<float>(i, j) << endl;
                outFileLiDAR << mapLiDAR.at<float>(i, j) << endl;
            }
        }
    }
}

void HistogramGeneration::map2Histogram(cv::Mat &map, vector<float> &histogram, int truncationBegin, int truncationEnd) {
    if (map.rows==0 || map.cols==0)
    {
        cerr << "Invalid input for HistogramGeneration::testMap2Histogram";
        exit(EXIT_FAILURE);
    }

    for(int i=0; i<map.rows; i++){
        for(int j=0; j<map.cols; j++){
            if(map.at<float>(i, j) > 0){
                histogram.push_back(map.at<float>(i, j));
            }
        }
    }
//    if(histogram.size()<50){
//        cerr << "The amount of points are too small to generate histogram in HistogramGeneration::testMap2Histogram";
//        exit(EXIT_FAILURE);
//    }
    sort(histogram.begin(), histogram.end());
    histogram.erase(histogram.begin(), histogram.begin()+truncationBegin*histogram.size()/100);
    histogram.erase(histogram.end()-truncationEnd*histogram.size()/100, histogram.end());
}

void HistogramGeneration::histogramDownsampling(vector<float> &histogram, vector<float> &histogramDownsampled, int DownsamplingSize) {
//    if(histogram.size()<50){
//        cerr << "Histogram is too short in HistogramGeneration::testHistogramDownsampling";
//        exit(EXIT_FAILURE);
//    }
    random_shuffle(histogram.begin(), histogram.end());
    //vector<float> sample;
    histogramDownsampled.assign(histogram.begin(), histogram.begin()+DownsamplingSize);
    //histogramDownsampled = sample;
}

bool HistogramMeasure::detectMinusValue(vector<float> &histogram) {
    for(int i=0; i<histogram.size(); i++){
        if(histogram[i]<0){
            return false;
        }
    }
    return true;
}

double HistogramMeasure::mapKLDivergence(cv::Mat &mapCamera, cv::Mat &mapLiDAR, vector<cv::Mat> &diagonalPointsSet) {
    double distance = 0;
    for(int i=0; i<diagonalPointsSet.size(); i++){
        cv::Mat mapCameraRegion, mapLiDARRegion;
        ImageUtils::creatMapRegion(mapCamera, mapCameraRegion, diagonalPointsSet[i]);
        ImageUtils::creatMapRegion(mapLiDAR, mapLiDARRegion, diagonalPointsSet[i]);
        vector<float> cameraHist, liDARHist, cameraHistDownsampled;
        cout << mapLiDARRegion << endl;
        HistogramGeneration::map2Histogram(mapCameraRegion, cameraHist, 1, 2);
        HistogramGeneration::map2Histogram(mapLiDARRegion, liDARHist, 1, 2);
//        if(cameraHist.size()<regionSize.width*regionSize.height/3){
//            continue;
//        }
        HistogramGeneration::histogramDownsampling(cameraHist, cameraHistDownsampled, liDARHist.size());
        bool minusValue;
        minusValue = detectMinusValue(liDARHist);
        if(minusValue == false){
            continue;
        }
        try{
            distance += cv::compareHist(liDARHist, cameraHistDownsampled, cv::HISTCMP_KL_DIV) / liDARHist.size();
        }
        catch (cv::Exception){
            cout << "test";
        }

    }
    return distance;
}

float HistogramMeasure::point2PointDistance(cv::Mat &mapCamera, cv::Mat &mapLiDAR) {
    float distance = 0;
    int cnt = 0;
    for(int i=0; i<mapCamera.rows; i++){
        for(int j=0; j<mapCamera.cols; j++){
            if(mapLiDAR.at<float>(i, j) > 0 && mapCamera.at<float>(i, j) != INFINITY){
                cnt++;
                distance += abs(mapCamera.at<float>(i, j) - mapLiDAR.at<float>(i, j));
            }
        }
    }
    if(cnt<50){
        distance = 10000;
    }
    distance /= cnt;
//    if(isnan(distance)){
//        distance = 10000;
//    }
    return distance;
}

float HistogramMeasure::point2PointDistanceTotal(cv::Mat &mapCamera, cv::Mat &mapLiDAR,
                                                    vector<cv::Mat> &diagonalPointsSet) {
    float p2pDistance = 0;
    for(int i=0; i<diagonalPointsSet.size(); i++){
        cv::Mat mapCameraRegion, mapLiDARRegion;
        ImageUtils::creatMapRegion(mapCamera, mapCameraRegion, diagonalPointsSet[i]);
        ImageUtils::creatMapRegion(mapLiDAR, mapLiDARRegion, diagonalPointsSet[i]);
        p2pDistance += point2PointDistance(mapCameraRegion, mapLiDARRegion);
    }
}

float HistogramMeasure::pointCloudDistance(cv::Mat &mapCamera, cv::Mat &mapLiDAR) {
    float pcDistance = 0;

    return pcDistance;
}

void HistogramMeasure::vectorToHist(vector<int> &vectorVariable, Eigen::RowVectorXi &histogramVariable) {
    histogramVariable = Eigen::RowVectorXi::Zero(vectorVariable.size());
    for(int i=0; i<vectorVariable.size(); i++){
        histogramVariable[i] = vectorVariable[i];
    }
}

double HistogramMeasure::cosDistance(vector<int> &vectorCamera, vector<int> &vectorLiDAR) {
    Eigen::RowVectorXi histogramCamera;
    Eigen::RowVectorXi histogramLiDAR;
    vectorToHist(vectorCamera, histogramCamera);
    vectorToHist(vectorLiDAR, histogramLiDAR);
    double cos = double(histogramCamera.dot(histogramLiDAR)) / (histogramCamera.norm() * histogramLiDAR.norm());
    return cos;
}

void Transfer::vector2MatSeperate(vector<double> &theta, cv::Mat &rotation, cv::Mat &translation) {
    if(theta.size()!=6){
        cerr << "The input theta is not 6 degrees in Analysis::paramterTransfer";
        exit(EXIT_FAILURE);
    }
    vector<float> rotationVector(theta.begin(), theta.begin()+3);
    vector<float> translationVector(theta.begin()+3, theta.end());
    rotation = cv::Mat::zeros(3, 3, CV_32FC1);
    translation = cv::Mat::zeros(3, 1, CV_32FC1);
    cv::Rodrigues(rotationVector, rotation);
    //rotation = rotation.t();
    translation.at<float>(0, 0) = translationVector[0];
    translation.at<float>(1, 0) = translationVector[1];
    translation.at<float>(2, 0) = translationVector[2];
}

void Transfer::array2MatSeperate(const double *theta, cv::Mat &rotation, cv::Mat &translation) {
    const double *rotationVector = theta;
    const double *translationVector = theta+3;
    rotation = cv::Mat::zeros(3, 3, CV_32FC1);
    translation = cv::Mat::zeros(3, 1, CV_32FC1);
    cv::Mat rotationVectorMat;
    rotationVectorMat = (cv::Mat_<float>(3,1) << rotationVector[0], rotationVector[1], rotationVector[2]);
    cv::Rodrigues(rotationVectorMat, rotation);
    //rotation = rotation.t();
    translation.at<float>(0, 0) = translationVector[0];
    translation.at<float>(1, 0) = translationVector[1];
    translation.at<float>(2, 0) = translationVector[2];
}

void Transfer::mat2VectorSeperate(cv::Mat &rotation, cv::Mat &translation, vector<double> &theta) {
    cv::Mat rotationVector = cv::Mat::zeros(3, 1, CV_32FC1);
    cv::Rodrigues(rotation, rotationVector);
    theta[0] = rotationVector.at<float>(0,0);
    theta[1] = rotationVector.at<float>(1,0);
    theta[2] = rotationVector.at<float>(2,0);
    theta[3] = translation.at<float>(0,0);
    theta[4] = translation.at<float>(1,0);
    theta[5] = translation.at<float>(2,0);
}

void Transfer::array2Eigen(const double *theta, Eigen::Matrix4f &transformation) {
    cv::Mat rotation, translation;
    array2MatSeperate(theta, rotation, translation);
    cv::Mat transformationMat = cv::Mat::zeros(4, 4, CV_32FC1);
    rotation.copyTo(transformationMat(cv::Rect(0, 0, 3, 3)));
    translation.copyTo(transformationMat(cv::Rect(3, 0, 1, 3)));
    transformationMat.at<float>(3, 0) = 0;
    transformationMat.at<float>(3, 1) = 0;
    transformationMat.at<float>(3, 2) = 0;
    transformationMat.at<float>(3, 3) = 1;
    cv::cv2eigen(transformationMat, transformation);
}

void Transfer::vector2Eigen(vector<double> &theta, Eigen::Matrix4f &transformation) {
    cv::Mat rotation, translation;
    vector2MatSeperate(theta, rotation, translation);
    cv::Mat transformationMat = cv::Mat::zeros(4, 4, CV_32FC1);
    rotation.copyTo(transformationMat(cv::Rect(0, 0, 3, 3)));
    translation.copyTo(transformationMat(cv::Rect(3, 0, 1, 3)));
    transformationMat.at<float>(3, 0) = 0;
    transformationMat.at<float>(3, 1) = 0;
    transformationMat.at<float>(3, 2) = 0;
    transformationMat.at<float>(3, 3) = 1;
    cv::cv2eigen(transformationMat, transformation);
}

void Transfer::depthAnalysis(cv::Mat &imageCamera, cv::Mat &imageLiDAR, int depth, string csvPath) {
    cout << "Now, analysis the same LiDAR depth for camera points" << endl;
    for(int i=0; i< imageLiDAR.rows; i++){
        for(int j=0; j<imageLiDAR.cols; j++){
            int val = int(imageLiDAR.at<uchar>(i, j));
            if(val != depth){
                imageCamera.at<uchar>(i, j) = uchar(0);
            }
        }
    }
    Eigen::RowVectorXi histogram;
    vector<int> bound = HistogramGeneration::histogramCount(imageCamera, histogram);
    int maxBound = bound[1];
    vector<int> vecTor;
    HistogramGeneration::histogramCompletion(histogram, vecTor, bound, maxBound);
    HistogramGeneration::histogramWrite(csvPath, vecTor);

}

void Transfer::depthDistribution(cv::Mat &imageLiDAR, string csvPath) {
    cout << "Now, analysis the LiDAR point cloud depth distribution";
    Eigen::RowVectorXi histogram;
    vector<int> bound = HistogramGeneration::histogramCount(imageLiDAR, histogram);
    int maxBound = bound[1];
    vector<int> vecTor;
    HistogramGeneration::histogramCompletion(histogram, vecTor, bound, maxBound);
    HistogramGeneration::histogramWrite(csvPath, vecTor);
}

void PointCloudAlignment::getCameraSparsePointCloud(cv::Mat &depthMapCamera, cv::Mat &depthMapLiDAR, LiDAR &lidar, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloudSparse) {
    for(int i=0; i<depthMapCamera.rows; i++){
        for(int j=0; j<depthMapCamera.cols; j++){
            if(depthMapLiDAR.at<float>(i ,j) > 0){
                cv::Point2f point(j, i);
                pcl::PointXYZ point3d;
                lidar.projectPointInverse(point, depthMapCamera.at<float>(i ,j), point3d);
                pointCloudSparse->push_back(point3d);
            }
        }
    }
}

void PointCloudAlignment::pointCloudDownsample(pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud,
                                               pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloudDownsampled, float gridSize) {
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(pointCloud);
    filter.setLeafSize(gridSize, gridSize, gridSize);  //0.01f, 0.01f, 0.01f
    filter.filter(*pointCloudDownsampled);
}

float PointCloudAlignment::findScaling(pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloudCamera,
                                       pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloudLiDAR) {
    Eigen::Vector4f centroidLiDAR;
    pcl::compute3DCentroid(*pointCloudLiDAR, centroidLiDAR);
    float centralLiDAR = (centroidLiDAR[0] + centroidLiDAR[1] + centroidLiDAR[2]) / 3;
    Eigen::Vector4f centroidCamera;
    pcl::compute3DCentroid(*pointCloudCamera, centroidCamera);
    float centralCamera = (centroidCamera[0] + centroidCamera[1] + centroidCamera[2]) / 3;
    float scale = centralLiDAR / centralCamera;
    return scale;
}

void PointCloudAlignment::pointCloudScaling(pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud, float scale, pcl::PointCloud<pcl::PointXYZ>::Ptr &transformedCloud) {
    Eigen::Matrix4f transformation;
    transformation << scale, 0, 0, 0, 0, scale, 0, 0, 0, 0, scale, 0, 0, 0, 0, 1;
    //pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (*pointCloud, *transformedCloud, transformation);
}

float PointCloudAlignment::chamferDistanceElem(pcl::PointXYZ &point, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud) {
    float minVal = INFINITY;
    for(int i=0; i<pointCloud->points.size(); i++){
        float distance = (pointCloud->points[i].x - point.x) * (pointCloud->points[i].x - point.x) + (pointCloud->points[i].y - point.y) * (pointCloud->points[i].y - point.y) + (pointCloud->points[i].z - point.z) * (pointCloud->points[i].z - point.z);
        if(distance<minVal){
            minVal = distance;
        }
    }
    return minVal;
}

float PointCloudAlignment::chamferDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloudCamera, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloudLiDAR) {
    float distance;
    for(int i=0; i<pointCloudCamera->points.size(); i++){
        distance += chamferDistanceElem(pointCloudCamera->points[i], pointCloudLiDAR);
    }
    for(int i=0; i<pointCloudLiDAR->points.size(); i++){
        distance +=chamferDistanceElem(pointCloudLiDAR->points[i], pointCloudCamera);
    }
    distance = distance / (pointCloudLiDAR->points.size() + pointCloudCamera->points.size());
    cout << pointCloudCamera->points.size() << endl << pointCloudLiDAR->points.size() << endl;
    return distance;
}
