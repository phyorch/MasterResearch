//
// Created by phyorch on 07/12/18.
//

#include "Calibration.h"
#include "Sensor.h"
#include "ImageUtils.h"

//void HistogramGeneration::map2Histogram(cv::Mat &map, vector<float> &histogram, int truncationBegin, int truncationEnd) {
//    if (map.rows==0 || map.cols==0)
//    {
//        cerr << "Invalid input for HistogramGeneration::testMap2Histogram";
//        exit(EXIT_FAILURE);
//    }
//
//    for(int i=0; i<map.rows; i++){
//        for(int j=0; j<map.cols; j++){
//            if(map.at<float>(i, j) > 0){
//                histogram.push_back(map.at<float>(i, j));
//            }
//        }
//    }
////    if(histogram.size()<50){
////        cerr << "The amount of points are too small to generate histogram in HistogramGeneration::testMap2Histogram";
////        exit(EXIT_FAILURE);
////    }
//    sort(histogram.begin(), histogram.end());
//    histogram.erase(histogram.begin(), histogram.begin()+truncationBegin*histogram.size()/100);
//    histogram.erase(histogram.end()-truncationEnd*histogram.size()/100, histogram.end());
//}

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

void Transfer::cv2EigenSeperate(cv::Mat &rotation, cv::Mat &translation, Eigen::Matrix4f &transformation) {
    cv::Mat transformationMat = cv::Mat::zeros(4, 4, CV_32FC1);
    rotation.copyTo(transformationMat(cv::Rect(0, 0, 3, 3)));
    translation.copyTo(transformationMat(cv::Rect(3, 0, 1, 3)));
    transformationMat.at<float>(3, 0) = 0;
    transformationMat.at<float>(3, 1) = 0;
    transformationMat.at<float>(3, 2) = 0;
    transformationMat.at<float>(3, 3) = 1;
    cv::cv2eigen(transformationMat, transformation);
}

void Transfer::matSeperate2Mat(cv::Mat &rotation, cv::Mat &translation, cv::Mat &transformation) {
    transformation = cv::Mat::zeros(4, 4, CV_32FC1);
    rotation.copyTo(transformation(cv::Rect(0, 0, 3, 3)));
    translation.copyTo(transformation(cv::Rect(3, 0, 1, 3)));
    transformation.at<float>(3, 0) = 0;
    transformation.at<float>(3, 1) = 0;
    transformation.at<float>(3, 2) = 0;
    transformation.at<float>(3, 3) = 1;
}

void Transfer::Eigen2MatSeperate(Eigen::Matrix4f &transformation, cv::Mat &rotation, cv::Mat &translation) {
    for(int i=0; i<3; i++){
        for(int j=0; j<4; j++){
            if(j<3){
                rotation.at<float>(i ,j) = transformation(i, j);
            }
            if(j==3){
                translation.at<float>(i, 0) = transformation(i, j);
            }
        }
    }
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

Eigen::Matrix3d Transfer::axisRot2R(double rx, double ry, double rz) {
    Eigen::Matrix4d R,rotx,roty,rotz;
    double sinv,cosv;
    sinv=sin(rx),cosv=cos(rx);
    rotx<<1,0,0,0,0,cosv,-sinv,0,0,sinv,cosv,0,0,0,0,1;
    sinv=sin(ry);cosv=cos(ry);
    roty<<cosv,0,sinv,0,0,1,0,0,-sinv,0,cosv,0,0,0,0,1;
    sinv=sin(rz);cosv=cos(rz);
    rotz<<cosv,-sinv,0,0,sinv,cosv,0,0,0,0,1,0,0,0,0,1;
    R=rotx*roty*rotz;
    Eigen::Matrix3d retMat=R.block(0,0,3,3);
    return retMat;
}

void Transfer::R2axisRot(Eigen::Matrix3d R, double &rx, double &ry, double &rz) {
    ry=asin(R(0,2));
    rx=-atan2(R(1,2),R(2,2));
    rz=-atan2(R(0,1),R(0,0));
}

void Transfer::mat2axis_angle(Eigen::Matrix3d m, Eigen::Vector3d &retv, double &angle) {
    double x, y, z;
    double r = sqrt((m(2, 1) - m(1, 2))*(m(2, 1) - m(1, 2)) + (m(0, 2) - m(2, 0))*(m(0, 2) - m(2, 0)) + (m(1, 0) - m(0, 1))*(m(1, 0) - m(0, 1)));
    x = (m(2, 1) - m(1, 2)) / r;
    y = (m(0, 2) - m(2, 0)) / r;
    z = (m(1, 0) - m(0, 1)) / r;
    Eigen::Vector3d t;
    t << x, y, z;
    retv = t;
    angle = acos((m(0, 0) + m(1, 1) + m(2, 2) - 1) / 2);
}

void matVec2EigenVec(vector<cv::Mat> &matVec, vector<Eigen::Matrix4d> &eigenVec) {
    for(int i=0; i<matVec.size(); i++){
        Eigen::Matrix4f eigenElemf;
        cv::cv2eigen(matVec.at(i), eigenElemf);
        Eigen::Matrix4d eigenElem = eigenElemf.cast<double>();
        eigenVec.push_back(eigenElem);
    }
}

void PointCloudAlignment::getCameraPointCloud(cv::Mat &depthMapCamera, LiDAR &lidar, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloudCamera) {
    for(int i=0; i<depthMapCamera.rows; i++){
        for(int j=0; j<depthMapCamera.cols; j++){
            cv::Point2f point(j, i);
            pcl::PointXYZ point3d;
            int val = int(depthMapCamera.at<uchar>(i ,j));
            lidar.projectPointInverse(point, val, point3d);
            pointCloudCamera->push_back(point3d);
        }
    }
}

void PointCloudAlignment::getLiDARPointCloudXYZ(pcl::PointCloud<pcl::PointXYZI>::Ptr &pointCloudXYZI,
                                                  pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloudXYZ) {
    for(int i=0; i<pointCloudXYZI->points.size(); i++){
        pcl::PointXYZ point3d;
        point3d.x = pointCloudXYZI->points[i].x;
        point3d.y = pointCloudXYZI->points[i].y;
        point3d.z = pointCloudXYZI->points[i].z;
        pointCloudXYZ->push_back(point3d);
    }
}

void PointCloudAlignment::getCameraSparsePointCloudKitti(cv::Mat &depthMapCamera, cv::Mat &depthMapLiDAR, LiDAR &lidar, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloudSparse, int colDeviation) {
    for(int i=0; i<depthMapCamera.rows; i++){
        for(int j=0; j<depthMapCamera.cols; j++){
            if(depthMapLiDAR.at<float>(i ,j) > 0){
                int val = int(depthMapCamera.at<uchar>(i ,j));
                cv::Point2f point(j, i);
                pcl::PointXYZ point3d;
                lidar.projectPointInverse(point, val, point3d, colDeviation);
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

void PointCloudAlignment::pointCloudDownsample(pcl::PointCloud<pcl::PointXYZI>::Ptr &pointCloud,
                                               pcl::PointCloud<pcl::PointXYZI>::Ptr &pointCloudDownsampled, float gridSize) {
    pcl::VoxelGrid<pcl::PointXYZI> filter;
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
    float distance = 0;
    for(int i=0; i<pointCloudCamera->points.size(); i++){
        distance += chamferDistanceElem(pointCloudCamera->points[i], pointCloudLiDAR);
    }
    for(int i=0; i<pointCloudLiDAR->points.size(); i++){
        distance +=chamferDistanceElem(pointCloudLiDAR->points[i], pointCloudCamera);
    }
    if(distance>0){
        distance = distance / (pointCloudLiDAR->points.size() + pointCloudCamera->points.size());
    }
    cout << pointCloudCamera->points.size() << endl << pointCloudLiDAR->points.size() << endl;
    return distance;
}

string HandEyeCalibration::zfill(int dataNum) {
    int digit = log10(dataNum) + 1;
    string dataName = to_string(dataNum);
    for(int i=0; i<10 - digit; i++){
        dataName = "0" + dataName;
    }
    return dataName;
}

void HandEyeCalibration::imageRead(int begin, int end, string dataType, vector<string> &dataList) {
    if(begin<=end){
        for(int i=0; i<=end-begin; i++){
            string dataName = dataType + zfill(begin + i) + ".png";
            if(i==0 || i==end-begin){
                dataList.push_back(dataName);
            }
            else{
                dataList.push_back(dataName);
                dataList.push_back(dataName);
            }
        }
    }
    else{
        for(int i=0; i<=begin-end; i++){
            string dataName = dataType + zfill(begin - i) + ".png";
            if(i==0 || i==begin-end){
                dataList.push_back(dataName);
            }
            else{
                dataList.push_back(dataName);
                dataList.push_back(dataName);
            }
        }
    }
}

void HandEyeCalibration::dataReadRandom(int begin, int end, int dataAmount, string imageLocation, string cloudLocation, string depthLocation,
                                        vector<string> &imageList, vector<string> &cloudList, vector<string> &depthList) {
    vector<int> sequence;
    for(int i=0; i<end-begin; i++){
        int elem = begin+i;
        sequence.push_back(elem);
    }
    random_shuffle(sequence.begin(), sequence.end());
    vector<int> site(sequence.begin(), sequence.begin() + dataAmount);
    for(int i=0; i<site.size(); i++){
        string dataName1 = imageLocation + zfill(site[i]) + ".png";
        string dataName2 = imageLocation + zfill(site[i]+1) + ".png";
        imageList.push_back(dataName1);
        imageList.push_back(dataName2);
    }
    for(int i=0; i<site.size(); i++){
        string dataName1 = cloudLocation + zfill(site[i]) + ".pcd";
        string dataName2 = cloudLocation + zfill(site[i]+1) + ".pcd";
        cloudList.push_back(dataName1);
        cloudList.push_back(dataName2);
    }
    for(int i=0; i<site.size(); i++) {
        string dataName1 = depthLocation + zfill(site[i]) + ".png";
        depthList.push_back(dataName1);
    }
}

void HandEyeCalibration::cloudRead(int begin, int end, string dataType, vector<string> &dataList) {
    if(begin<=end){
        for(int i=0; i<=end-begin; i++){
            string dataName = dataType + zfill(begin + i) + ".pcd";
            if(i==0 || i==end-begin){
                dataList.push_back(dataName);
            }
            else{
                dataList.push_back(dataName);
                dataList.push_back(dataName);
            }
        }
    }
    else{
        for(int i=0; i<=begin-end; i++){
            string dataName = dataType + zfill(begin - i) + ".pcd";
            if(i==0 || i==begin-end){
                dataList.push_back(dataName);
            }
            else{
                dataList.push_back(dataName);
                dataList.push_back(dataName);
            }
        }
    }
}

void HandEyeCalibration::depthRead(int begin, int end, string dataRoot, vector<string> &dataList) {
    if(begin<=end){
        for(int i=0; i<end-begin; i++){
            string dataName = dataRoot + zfill(begin + i) + ".png";
            dataList.push_back(dataName);
        }
    }
    else{
        for(int i=0; i<begin-end; i++){
            string dataName = dataRoot + zfill(begin - i) + ".png";
            dataList.push_back(dataName);
        }
    }
}

void HandEyeCalibration::findFeatureMatches(cv::Mat &image1, cv::Mat &image2,
                                            vector<cv::KeyPoint> &keyPoints1,
                                            vector<cv::KeyPoint> &keyPoints2, vector<cv::DMatch> &matches) {
    cv::Mat descriptors_1, descriptors_2;
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher  = cv::DescriptorMatcher::create ( "BruteForce-Hamming" );
    detector->detect ( image1,keyPoints1 );
    detector->detect ( image2,keyPoints2 );

    descriptor->compute ( image1, keyPoints1, descriptors_1 );
    descriptor->compute ( image2, keyPoints2, descriptors_2 );

    vector<cv::DMatch> match;
    // BFMatcher matcher ( NORM_HAMMING );
    matcher->match( descriptors_1, descriptors_2, match );

    //-- 第四步:匹配点对筛选
    double min_dist=10000, max_dist=0;

    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = match[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( match[i].distance <= max ( 2*min_dist, 30.0 ) )
        {
            matches.push_back ( match[i] );
        }
    }
}

void HandEyeCalibration::creat3D2DPoints(LiDAR &lidar, cv::Mat &depthMapCamera, vector<cv::KeyPoint> &keyPoints1, vector<cv::KeyPoint> &keyPoints2,
                                         vector<cv::DMatch> &matches, vector<cv::Point3f> &points3d, vector<cv::Point2f> &points2d) {
    for ( cv::DMatch m:matches )
    {
        float depth = float(depthMapCamera.at<uchar>(int(keyPoints1[m.queryIdx].pt.y), int(keyPoints1[m.queryIdx].pt.x)));
        if ( depth < 5 || depth > 50 )   // bad depth
            continue;
        cv::Point3f point_3d;
        lidar.projectPointInverse(keyPoints1[m.queryIdx].pt, depth, point_3d);
        points3d.push_back (point_3d);
        points2d.push_back (keyPoints2[m.trainIdx].pt);
    }
}

void HandEyeCalibration::cameraRegistration(cv::Mat &image1, cv::Mat &image2, vector<cv::KeyPoint> &keyPoints1, vector<cv::KeyPoint> &keyPoints2, vector<cv::DMatch> &matches,
                                            cv::Mat &depthMapCamera, cv::Mat &cameraMatrix, LiDAR &liDAR, cv::Mat &transformationCamera) {
    findFeatureMatches(image1, image2, keyPoints1, keyPoints2, matches);
    cout<<"Find "<<matches.size() <<"matched points"<<endl;
    vector<cv::Point3f> pts_3d;
    vector<cv::Point2f> pts_2d;

    creat3D2DPoints(liDAR, depthMapCamera, keyPoints1, keyPoints2, matches, pts_3d, pts_2d);

    cv::Mat r, t;
    solvePnP ( pts_3d, pts_2d, cameraMatrix, cv::Mat(), r, t, false, cv::SOLVEPNP_ITERATIVE); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
    cv::Mat R;
    cv::Rodrigues ( r, R ); // r为旋转向量形式，用Rodrigues公式转换为矩阵

    transformationCamera = cv::Mat::zeros(4, 4, CV_32FC1);
    Transfer::matSeperate2Mat(R, t, transformationCamera);
}



void HandEyeCalibration::pointCloudRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud1,
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud2,
                                                float voxVolum, cv::Mat &transformation) {
    PointCloudAlignment::pointCloudDownsample(pointCloud1, pointCloud1, voxVolum);
    PointCloudAlignment::pointCloudDownsample(pointCloud2, pointCloud2, voxVolum);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (pointCloud1);
    sor.setMeanK (30);
    sor.setStddevMulThresh (1.0);
    sor.filter (*pointCloud1);
    sor.setInputCloud(pointCloud2);
    sor.filter(*pointCloud2);

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(pointCloud1);
    icp.setInputTarget(pointCloud2);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << endl;
    transformation = cv::Mat::zeros(4, 4, CV_32FC1);
    cv::eigen2cv(icp.getFinalTransformation(), transformation);
}

void HandEyeCalibration::skew(cv::Mat &matOriginal, cv::Mat &matSkew)
{
    CV_Assert(matOriginal.cols == 1 && matOriginal.rows == 3);
    //matSkew = cv::Mat::zeros(3, 3, CV_32FC1);

    matSkew.at<float>(0, 0) = 0.0;
    matSkew.at<float>(0, 1) = -matOriginal.at<float>(2, 0);
    matSkew.at<float>(0, 2) = matOriginal.at<float>(1, 0);

    matSkew.at<float>(1, 0) = matOriginal.at<float>(2, 0);
    matSkew.at<float>(1, 1) = 0.0;
    matSkew.at<float>(1, 2) = -matOriginal.at<float>(0, 0);

    matSkew.at<float>(2, 0) = -matOriginal.at<float>(1, 0);
    matSkew.at<float>(2, 1) = matOriginal.at<float>(0, 0);
    matSkew.at<float>(2, 2) = 0.0;
}

void HandEyeCalibration::handEyeTsai(cv::Mat &transformationCameraLiDAR, cv::Mat &transformationLiDAR,
                                     cv::Mat &transformationCamera) {
    cv::Mat Rgij = cv::Mat::zeros(3, 3, CV_32FC1);
    cv::Mat Rcij = cv::Mat::zeros(3, 3, CV_32FC1);
    cv::Mat Tgij = cv::Mat::zeros(3, 1, CV_32FC1);
    cv::Mat Tcij = cv::Mat::zeros(3, 1, CV_32FC1);

    transformationLiDAR(cv::Rect(0, 0, 3, 3)).copyTo(Rgij);
    transformationCamera(cv::Rect(0, 0, 3, 3)).copyTo(Rcij);
    transformationLiDAR(cv::Rect(3, 0, 1, 3)).copyTo(Tgij);
    transformationCamera(cv::Rect(3, 0, 1, 3)).copyTo(Tcij);

    cout << "Rgij Rcij" << "  "<< Rgij << endl << Rcij << endl;

    cv::Mat rgij = cv::Mat::zeros(3, 1, CV_32FC1);
    cv::Mat rcij = cv::Mat::zeros(3, 1, CV_32FC1);

    cv::Rodrigues(Rgij, rgij);
    cv::Rodrigues(Rcij, rcij);

    float theta_gij = norm(rgij);
    float theta_cij = norm(rcij);

    cout << "rgij  rcij" << "   " << rgij << endl << rcij <<theta_gij << endl << theta_cij << endl;

    cv::Mat rngij = cv::Mat::zeros(3, 1, CV_32FC1);
    cv::Mat rncij = cv::Mat::zeros(3, 1, CV_32FC1);

    rngij = rgij / theta_gij;
    rncij = rcij / theta_cij;

    cout << "rngij  rncij" << "   " << rngij << endl << rncij << endl;



    cv::Mat Pgij = cv::Mat::zeros(3, 1, CV_32FC1);
    cv::Mat Pcij = cv::Mat::zeros(3, 1, CV_32FC1);
    cv::Mat Pgcij = cv::Mat::zeros(3, 1, CV_32FC1);

    Pgij = 2 * sin(theta_gij / 2)*rngij;
    Pcij = 2 * sin(theta_cij / 2)*rncij;
    Pgcij = Pgij + Pcij;

    cout << "Pgij Pcij Pgcij" << "  " << Pgij << endl << Pcij << endl << Pgcij << endl;

    cv::Mat tempA = cv::Mat::zeros(3, 3, CV_32FC1);
    cv::Mat tempb = cv::Mat::zeros(3, 1, CV_32FC1);

    HandEyeCalibration::skew(Pgcij, tempA);
    tempb = Pcij - Pgij;

    cout << "tempA tempb" << "   " << tempA << endl << tempb << endl;

    cv::Mat pinA = cv::Mat::zeros(3, 1, CV_32FC1);
    cv::invert(tempA, pinA, cv::DECOMP_SVD);

    cv::Mat Pcg_prime = cv::Mat::zeros(3, 1, CV_32FC1);
    Pcg_prime = pinA * tempb;

    cout << "Pcg_prime" << "   " << Pcg_prime << endl;

    cv::Mat Pcg = cv::Mat::zeros(3, 1, CV_32FC1);
    cv::Mat PcgTrs = cv::Mat::zeros(1, 3, CV_32FC1);
    Pcg = 2 * Pcg_prime / sqrt(1 + cv::norm(Pcg_prime) * cv::norm(Pcg_prime));

    cout << "Pcg" << "   " << Pcg << endl;

    PcgTrs = Pcg.t();

    cv::Mat sPcg = cv::Mat::zeros(3, 3, CV_32FC1);
    HandEyeCalibration::skew(Pcg, sPcg);

    cv::Mat Rcg = cv::Mat::zeros(3, 3, CV_32FC1);
    cv::Mat eyeM = cv::Mat::eye(3, 3, CV_32FC1);
    cv::Mat Rcg1 = (1 - cv::norm(Pcg) * cv::norm(Pcg) / 2) * eyeM;
    cout << "Rcg1  " << Rcg1 << endl;

    float test = cv::norm(Pcg)*cv::norm(Pcg);
    cout << "   " <<test << endl;
    cv::Mat Rcg2 = 0.5 * (Pcg * PcgTrs + sqrt(4 - cv::norm(Pcg)*cv::norm(Pcg))* sPcg);

    Rcg = (1 - cv::norm(Pcg) * cv::norm(Pcg) / 2) * eyeM + 0.5 * (Pcg * PcgTrs + sqrt(4 - cv::norm(Pcg)*cv::norm(Pcg))* sPcg);

    cout << "Rcg" << "   " << Rcg << endl;



    cv::Mat tempAA = cv::Mat::zeros(3, 3, CV_32FC1);
    cv::Mat tempbb = cv::Mat::zeros(3, 1, CV_32FC1);

//    cv::Mat AA;
//    cv::Mat bb;
    cv::Mat pinAA = cv::Mat::zeros(3, 1, CV_32FC1);

    cv::Mat Tcg = cv::Mat::zeros(3, 1, CV_32FC1);


    tempAA = Rgij - eyeM;
    cout << "tempAA  " << tempAA << endl;
    tempbb = Rcg * Tcij - Tgij;
    cout << "tempbb  " << tempbb << endl;

    cv::invert(tempAA, pinAA);//, cv::DECOMP_SVD);
    cout << "pinAA  " << pinAA << endl;
    Tcg = pinAA * tempbb;

    Rcg.copyTo(transformationCameraLiDAR(cv::Rect(0, 0, 3, 3)));
    Tcg.copyTo(transformationCameraLiDAR(cv::Rect(3, 0, 1, 3)));
    transformationCameraLiDAR.at<float>(3, 0) = 0.0;
    transformationCameraLiDAR.at<float>(3, 1) = 0.0;
    transformationCameraLiDAR.at<float>(3, 2) = 0.0;
    transformationCameraLiDAR.at<float>(3, 3) = 1.0;
}

void HandEyeCalibration::handEyeTsai(cv::Mat &transformationCameraLiDAR, vector<cv::Mat> transformationLiDAR,
                                     vector<cv::Mat> transformationCamera) {

    cv::Mat Rgij = cv::Mat::zeros(3, 3, CV_32FC1);
    cv::Mat Rcij = cv::Mat::zeros(3, 3, CV_32FC1);
    cv::Mat Tgij = cv::Mat::zeros(3, 1, CV_32FC1);
    cv::Mat Tcij = cv::Mat::zeros(3, 1, CV_32FC1);

    cv::Mat rgij = cv::Mat::zeros(3, 1, CV_32FC1);
    cv::Mat rcij = cv::Mat::zeros(3, 1, CV_32FC1);

    float theta_gij;
    float theta_cij;

    cv::Mat rngij = cv::Mat::zeros(3, 1, CV_32FC1);
    cv::Mat rncij = cv::Mat::zeros(3, 1, CV_32FC1);

    cv::Mat Pgij = cv::Mat::zeros(3, 1, CV_32FC1);
    cv::Mat Pcij = cv::Mat::zeros(3, 1, CV_32FC1);
    cv::Mat Pgcij = cv::Mat::zeros(3, 1, CV_32FC1);

    cv::Mat tempA = cv::Mat::zeros(3, 3, CV_32FC1);
    cv::Mat tempb = cv::Mat::zeros(3, 1, CV_32FC1);

    cv::Mat A;
    cv::Mat b;
    cv::Mat pinA;

    cv::Mat Pcg_prime = cv::Mat::zeros(3, 1, CV_32FC1);

    cv::Mat Pcg = cv::Mat::zeros(3, 1, CV_32FC1);
    cv::Mat PcgTrs = cv::Mat::zeros(1, 3, CV_32FC1);

    cv::Mat Rcg = cv::Mat::zeros(3, 3, CV_32FC1);
    cv::Mat eyeM = cv::Mat::eye(3, 3, CV_32FC1);

    for(int i=0; i<transformationCamera.size(); i++){
        transformationLiDAR[i](cv::Rect(0, 0, 3, 3)).copyTo(Rgij);
        transformationCamera[i](cv::Rect(0, 0, 3, 3)).copyTo(Rcij);
        transformationLiDAR[i](cv::Rect(3, 0, 1, 3)).copyTo(Tgij);
        transformationCamera[i](cv::Rect(3, 0, 1, 3)).copyTo(Tcij);
        cout << "Rgij Rcij" << "  "<< Rgij << endl << Rcij << endl;

        cv::Rodrigues(Rgij, rgij);
        cv::Rodrigues(Rcij, rcij);

        theta_gij = norm(rgij);
        theta_cij = norm(rcij);
        cout << "rgij  rcij" << "   " << rgij << endl << rcij <<theta_gij << endl << theta_cij << endl;

        rngij = rgij / theta_gij;
        rncij = rcij / theta_cij;
        cout << "rngij  rncij" << "   " << rngij << endl << rncij << endl;

        Pgij = 2 * sin(theta_gij / 2)*rngij;
        Pcij = 2 * sin(theta_cij / 2)*rncij;
        Pgcij = Pgij + Pcij;
        cout << "Pgij Pcij Pgcij" << "  " << Pgij << endl << Pcij << endl << Pgcij << endl;

        HandEyeCalibration::skew(Pgcij, tempA);
        tempb = Pcij - Pgij;
        cout << "tempA tempb" << "   " << tempA << endl << tempb << endl;
        A.push_back(tempA);
        b.push_back(tempb);
    }


    cv::invert(A, pinA, cv::DECOMP_SVD);

    Pcg_prime = pinA * b;
    cout << "Pcg_prime" << "   " << Pcg_prime << endl;

    Pcg = 2 * Pcg_prime / sqrt(1 + cv::norm(Pcg_prime) * cv::norm(Pcg_prime));
    cout << "Pcg" << "   " << Pcg << endl;
    PcgTrs = Pcg.t();

    cv::Mat sPcg = cv::Mat::zeros(3, 3, CV_32FC1);
    HandEyeCalibration::skew(Pcg, sPcg);


//    cv::Mat Rcg1 = (1 - cv::norm(Pcg) * cv::norm(Pcg) / 2) * eyeM;
//    cout << "Rcg1  " << Rcg1 << endl;
//
//    float test = cv::norm(Pcg)*cv::norm(Pcg);
//    cout << "   " <<test << endl;
//    cv::Mat Rcg2 = 0.5 * (Pcg * PcgTrs + sqrt(4 - cv::norm(Pcg)*cv::norm(Pcg))* sPcg);

    Rcg = (1 - cv::norm(Pcg) * cv::norm(Pcg) / 2) * eyeM + 0.5 * (Pcg * PcgTrs + sqrt(4 - cv::norm(Pcg)*cv::norm(Pcg))* sPcg);

    cout << "Rcg" << "   " << Rcg << endl;



//    cv::Mat tempAA = cv::Mat::zeros(3, 3, CV_32FC1);
//    cv::Mat tempbb = cv::Mat::zeros(3, 1, CV_32FC1);
//
////    cv::Mat AA;
////    cv::Mat bb;
//    cv::Mat pinAA = cv::Mat::zeros(3, 1, CV_32FC1);
//
//    cv::Mat Tcg = cv::Mat::zeros(3, 1, CV_32FC1);
//
//
//    tempAA = Rgij - eyeM;
//    cout << "tempAA  " << tempAA << endl;
//    tempbb = Rcg * Tcij - Tgij;
//    cout << "tempbb  " << tempbb << endl;
//
//    cv::invert(tempAA, pinAA);//, cv::DECOMP_SVD);
//    cout << "pinAA  " << pinAA << endl;
//    Tcg = pinAA * tempbb;
//
//    Rcg.copyTo(transformationCameraLiDAR(cv::Rect(0, 0, 3, 3)));
//    Tcg.copyTo(transformationCameraLiDAR(cv::Rect(3, 0, 1, 3)));
//    transformationCameraLiDAR.at<float>(3, 0) = 0.0;
//    transformationCameraLiDAR.at<float>(3, 1) = 0.0;
//    transformationCameraLiDAR.at<float>(3, 2) = 0.0;
//    transformationCameraLiDAR.at<float>(3, 3) = 1.0;
}

void HandEyeCalibration::handEyeOptimization(vector<Eigen::Matrix4d> pepdMat, vector<Eigen::Matrix4d> holdMat,
                                             Eigen::Matrix3d &R) {
    Eigen::MatrixXd KA(3, pepdMat.size()), KB(3, pepdMat.size());
    struct axisAlignCostFunc {
    public:
        axisAlignCostFunc(Eigen::Vector3d& p3d_, Eigen::Vector3d& eyeDirec_)
        {
            p3d = p3d_;
            eyeDirec = eyeDirec_;
        };
        bool operator()(const double* parameters, double* residual) const {
            double rx = parameters[0];
            double ry = parameters[1];
            double rz = parameters[2];

            Eigen::Matrix3d R = Transfer::axisRot2R(rx, ry, rz);
            Eigen::Vector3d plot = (R.transpose()*p3d).normalized();

            residual[0] = eyeDirec.cross(plot).norm();
            return true;
        }
    private:
        Eigen::Vector3d p3d;
        Eigen::Vector3d eyeDirec;

    };

    struct rotErrCostFunc {
    public:
        rotErrCostFunc(Eigen::Matrix3d& RA_, Eigen::Matrix3d& RB_)
        {
            RA = RA_;
            RB = RB_;
        };
        bool operator()(const double* parameters, double* residual) const {
            double rx = parameters[0];
            double ry = parameters[1];
            double rz = parameters[2];

            Eigen::Matrix3d R = Transfer::axisRot2R(rx, ry, rz);
            Eigen::Matrix3d err = RA*R-R*RB;

            residual[0] = err.norm();
            return true;
        }
    private:
        Eigen::Matrix3d RA;
        Eigen::Matrix3d RB;

    };
    double opt[3];
    ceres::Problem problem;
    for (int i = 0;i<pepdMat.size();i++) {
        double angle1,angle2;
        cout << pepdMat.at(i) << endl;
        cout << holdMat.at(i) << endl;
        Eigen::Vector3d pepax;
        Transfer::mat2axis_angle(pepdMat.at(i).block(0,0,3,3), pepax, angle1);
        Eigen::Vector3d holax;
        Transfer::mat2axis_angle(holdMat.at(i).block(0, 0, 3, 3), holax, angle2);
        KA.col(i) = pepax;
        KB.col(i) = holax;
        Eigen::Matrix3d RA = pepdMat.at(i).block(0, 0, 3, 3);
        Eigen::Matrix3d RB = holdMat.at(i).block(0, 0, 3, 3);
        rotErrCostFunc* p= new rotErrCostFunc(RA, RB);
        ceres::CostFunction* c = new ceres::NumericDiffCostFunction<rotErrCostFunc, ceres::CENTRAL, 1, 3>(
                p);
        problem.AddResidualBlock(c,new ceres::HuberLoss(1.0e-2), opt);
    }
    Eigen::MatrixXd KBKA = KB*KA.transpose();
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(KBKA, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix3d Hm;
    Eigen::Matrix3d uvt = svd.matrixU()*svd.matrixV().transpose();
    Hm << 1, 0, 0,
            0, 1, 0,
            0, 0, uvt.determinant();
    R = svd.matrixV()*Hm*svd.matrixU().transpose();
    Transfer::R2axisRot(R, opt[0], opt[1], opt[2]);
    ceres::Solver::Options options;
    options.max_num_iterations = 1e4;
    options.function_tolerance = 1e-6;
    options.parameter_tolerance = 1e-6;
    options.linear_solver_type = ceres::DENSE_QR;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    R = Transfer::axisRot2R(opt[0], opt[1], opt[2]);
}

void HandEyeCalibration::handEyeOptimizationTranslation(vector<Eigen::Matrix4d> pepdMat,
                                                        vector<Eigen::Matrix4d> holdMat, Eigen::Matrix3d &R,
                                                        Eigen::Vector3d &t) {
    Eigen::Vector3d translation;
    translation << 0, 0, 0;
    Eigen::Matrix3d identity;
    identity = Eigen::Matrix3d::Zero();
    for(int i=0; i<pepdMat.size(); i++){
        Eigen::Matrix3d RA = pepdMat.at(i).block(0, 0, 3, 3);
        Eigen::Vector3d tA = pepdMat.at(i).block(0, 3, 3, 1);
        Eigen::Matrix3d RB = holdMat.at(i).block(0, 0, 3, 3);
        Eigen::Vector3d tB = holdMat.at(i).block(0, 3, 3, 1);
        translation = (RA - identity).inverse() * (R * tB - tA);
        t = t + translation;
    }
    t[0] = t[0] / pepdMat.size();
    t[1] = t[1] / pepdMat.size();
    t[2] = t[2] / pepdMat.size();
    double t1 = t[0];
    double t2 = t[1];
    double t3 = t[2];
    int a = 1;
}

float Refinement::point2PointDistanceKitti(cv::Mat &mapCamera, cv::Mat &mapLiDAR) {
    float distance = 0;
    int cnt = 0;
    for(int i=0; i<mapCamera.rows; i++){
        for(int j=0; j<mapCamera.cols; j++){
            if(mapLiDAR.at<float>(i, j) > 0){
                cnt++;
                distance += abs(float(mapCamera.at<uchar>(i, j)) - mapLiDAR.at<float>(i, j));
            }
        }
    }

    if(cnt<50){
        distance = 1000;
    }
    else{
        distance /= cnt;
    }
//    if(isnan(distance)){
//        distance = 10000;
//    }
    return distance;
}

float Refinement::point2PointDistanceTotal(cv::Mat &mapCamera, cv::Mat &mapLiDAR,
                                                 vector<cv::Mat> &diagonalPointsSet) {
    float p2pDistance = 0;
    for(int i=0; i<diagonalPointsSet.size(); i++){
        cv::Mat mapCameraRegion, mapLiDARRegion;
        ImageUtils::creatMapRegion(mapCamera, mapCameraRegion, diagonalPointsSet[i]);
        ImageUtils::creatMapRegion(mapLiDAR, mapLiDARRegion, diagonalPointsSet[i]);
        p2pDistance += point2PointDistanceKitti(mapCameraRegion, mapLiDARRegion);
    }
}

float Refinement::point2PointDistanceFrame(cv::Mat &mapCamera, cv::Mat &mapLiDAR) {
    float distance = 0;
    int cnt = 0;
    for(int i=0; i<mapCamera.rows; i++){
        for(int j=0; j<mapCamera.cols; j++){
            if(mapLiDAR.at<float>(i, j) > 0){
                cnt++;
                distance += abs(float(mapCamera.at<uchar>(i, j)) - mapLiDAR.at<float>(i, j));
            }
        }
    }

    if(cnt<5000){
        distance = 1000;
    }
    else{
        distance /= cnt;
    }
    return distance;
}

//For point, x : rows, y : cols
//For region, x : height, y : length
bool Refinement::validRegion(cv::Mat &depthMapLiDAR, cv::Point &point, cv::Point &region, int threshold) {
    cv::Mat refRegion;
    ImageUtils::creatMapRegion(depthMapLiDAR, refRegion, point.x - region.x, point.x + region.x, point.y - region.y, point.y + region.y);
    int cnt = 0;
    for(int i=0; i<refRegion.rows; i++){
        for(int j=0; j<refRegion.cols; j++){
            if(refRegion.at<float>(i, j)>0){
                cnt++;
            }
        }
    }
    if(cnt>threshold){
        return false;
    }
    return true;
}

//For slideWindowSize, x : rows, y : cols
//For slideWindowRegion, x : rows, y : cols
void Refinement::slideElimination(cv::Mat &depthMapLiDAR, cv::Point &slideWindowSize, cv::Point &slideWindowRange, cv::Point &slideWindowRegion, int elimiThreshold) {
    for(int i=0; i<depthMapLiDAR.rows; i++){
        for(int j=0; j<depthMapLiDAR.cols; j++){
            if(depthMapLiDAR.at<float>(i, j) > slideWindowRange.x && depthMapLiDAR.at<float>(i, j) < slideWindowRange.y && i>slideWindowRegion.x &&
               i<depthMapLiDAR.rows-slideWindowRegion.x && j>slideWindowRegion.y && j<depthMapLiDAR.cols-slideWindowRegion.y){
                cv::Mat window;
                ImageUtils::creatMapRegion(depthMapLiDAR, window, i-slideWindowSize.x/2, i+slideWindowSize.x/2, j-slideWindowSize.y/2, j+slideWindowSize.y/2);
                float maxVal, minVal;
                ImageUtils::maxMat<float>(window, maxVal);
                ImageUtils::minMat<float>(window, minVal);
                if(depthMapLiDAR.at<float>(i, j)>30 && depthMapLiDAR.at<float>(i, j)==minVal){
                    cv::Point point(i, j);
                    if(validRegion(depthMapLiDAR, point, slideWindowRegion, 200)){
                        continue;
                    }
                }
                if(maxVal-minVal<elimiThreshold){
                    depthMapLiDAR.at<float>(i, j) = 0;
                }
            }
            else{
                depthMapLiDAR.at<float>(i, j) = 0;
            }
        }
    }
}

void Refinement::slideElimination2(cv::Mat &depthMapLiDAR, cv::Mat &edgeMapLiDAR, cv::Point &slideWindowSize, cv::Point &slideWindowRange,
                                   float elimiThreshold, int setOne) {
    edgeMapLiDAR = depthMapLiDAR.clone();
    for(int i=0; i<depthMapLiDAR.rows; i++){
        for(int j=0; j<depthMapLiDAR.cols; j++){
            if(depthMapLiDAR.at<float>(i, j) > slideWindowRange.x && depthMapLiDAR.at<float>(i, j) < slideWindowRange.y && i>slideWindowSize.x &&
               i<depthMapLiDAR.rows-slideWindowSize.x && j>slideWindowSize.y && j<depthMapLiDAR.cols-slideWindowSize.y){
                cv::Mat window;
                ImageUtils::creatMapRegion(depthMapLiDAR, window, i-slideWindowSize.x/2, i+slideWindowSize.x/2, j-slideWindowSize.y/2, j+slideWindowSize.y/2);
                float maxVal, minVal;
                ImageUtils::maxMat<float>(window, maxVal);
                ImageUtils::minMat<float>(window, minVal);
//                if(depthMapLiDAR.at<float>(i, j)>30 && depthMapLiDAR.at<float>(i, j)==minVal){
//                    cv::Point point(i, j);
//                    if(validRegion(depthMapLiDAR, point, slideWindowRegion, 200)){
//                        continue;
//                    }
//                }
                if(maxVal-minVal<elimiThreshold || depthMapLiDAR.at<float>(i, j)>(minVal + (maxVal-minVal)/50)){ //
                    edgeMapLiDAR.at<float>(i, j) = 0;
                }
                else if(setOne == 1){
                    edgeMapLiDAR.at<float>(i, j) = 1;
                }
            }
            else{
                edgeMapLiDAR.at<float>(i, j) = 0;
            }
        }
    }
}

void Refinement::gaussianBlurModified(cv::Mat &edgeMap, cv::Mat &edgeMapBlured, int filterSize) {
    edgeMapBlured = edgeMap.clone();
    cv::Mat filter;
    if(filterSize==3){
        filter = (cv::Mat_<float>(3,3) << 0.055, 0.110, 0.055,
        0.110, 0.221, 0.110,
        0.055, 0.110, 0.055);
    }
    else if(filterSize==5){
        filter = (cv::Mat_<float>(5,5) << 0.003765, 0.015019, 0.023792, 0.015019, 0.003765,
        0.015019, 0.059912, 0.094907, 0.059912, 0.015019,
        0.023792, 0.094907, 0.150342, 0.094907, 0.023792,
        0.015019, 0.059912, 0.094907, 0.059912, 0.015019,
        0.003765, 0.015019, 0.023792, 0.015019, 0.003765);
    }
    else if(filterSize==7){ //sigma=2
        filter = (cv::Mat_<float>(7,7) << 0.000036,	0.000363, 0.001446, 0.002291, 0.001446, 0.000363, 0.000036,
        0.000363, 0.003676, 0.014662, 0.023226, 0.014662, 0.003676, 0.000363,
        0.001446, 0.014662, 0.058488, 0.092651, 0.058488, 0.014662, 0.001446,
        0.002291, 0.023226, 0.092651, 0.146768, 0.092651, 0.023226, 0.002291,
        0.001446, 0.014662, 0.058488, 0.092651, 0.058488, 0.014662, 0.001446,
        0.000363, 0.003676, 0.014662, 0.023226, 0.014662, 0.003676, 0.000363,
        0.000036, 0.000363, 0.001446, 0.002291, 0.001446, 0.000363, 0.000036);
    }
    else if(filterSize==9){
        filter = (cv::Mat_<float>(9,9) << 0.000814, 0.001918, 0.003538, 0.005108, 0.005774, 0.005108, 0.003538, 0.001918, 0.000814,
        0.001918, 0.00452, 0.008338, 0.012038, 0.013605, 0.012038, 0.008338, 0.00452, 0.001918,
        0.003538, 0.008338, 0.015378, 0.022203, 0.025094, 0.022203, 0.015378, 0.008338, 0.003538,
        0.005108, 0.012038, 0.022203, 0.032057, 0.036231, 0.032057, 0.022203, 0.012038, 0.005108,
        0.005774, 0.013605, 0.025094, 0.036231, 0.04095, 0.036231, 0.025094, 0.013605, 0.005774,
        0.005108, 0.012038, 0.022203, 0.032057, 0.036231, 0.032057, 0.022203, 0.012038, 0.005108,
        0.003538, 0.008338, 0.015378, 0.022203, 0.025094, 0.022203, 0.015378, 0.008338, 0.003538,
        0.001918, 0.00452, 0.008338, 0.012038, 0.013605, 0.012038, 0.008338, 0.00452, 0.001918,
        0.000814, 0.001918, 0.003538, 0.005108, 0.005774, 0.005108, 0.003538, 0.001918, 0.000814);

    }
    else if(filterSize==13){ //sigma=2
        filter = (cv::Mat_<float>(13,13) << 0.000006, 0.000022, 0.000067, 0.000158, 0.000291, 0.000421, 0.000476, 0.000421, 0.000291, 0.000158, 0.000067, 0.000022, 0.000006,
        0.000022, 0.000086, 0.000258, 0.000608, 0.001121, 0.001618, 0.001829, 0.001618, 0.001121, 0.000608, 0.000258, 0.000086, 0.000022,
        0.000067, 0.000258, 0.000777, 0.00183, 0.003375, 0.004873, 0.005508, 0.004873, 0.003375, 0.00183, 0.000777, 0.000258, 0.000067,
        0.000158, 0.000608, 0.00183, 0.004312, 0.007953, 0.011483, 0.012978, 0.011483, 0.007953, 0.004312, 0.00183, 0.000608, 0.000158,
        0.000291, 0.001121, 0.003375, 0.007953, 0.014669, 0.021179, 0.023938, 0.021179, 0.014669, 0.007953, 0.003375, 0.001121, 0.000291,
        0.000421, 0.001618, 0.004873, 0.011483, 0.021179, 0.030579, 0.034561, 0.030579, 0.021179, 0.011483, 0.004873, 0.001618, 0.000421,
        0.000476, 0.001829, 0.005508, 0.012978, 0.023938, 0.034561, 0.039062, 0.034561, 0.023938, 0.012978, 0.005508, 0.001829, 0.000476,
        0.000421, 0.001618, 0.004873, 0.011483, 0.021179, 0.030579, 0.034561, 0.030579, 0.021179, 0.011483, 0.004873, 0.001618, 0.000421,
        0.000291, 0.001121, 0.003375, 0.007953, 0.014669, 0.021179, 0.023938, 0.021179, 0.014669, 0.007953, 0.003375, 0.001121, 0.000291,
        0.000158, 0.000608, 0.00183, 0.004312, 0.007953, 0.011483, 0.012978, 0.011483, 0.007953, 0.004312, 0.00183, 0.000608, 0.000158,
        0.000067, 0.000258, 0.000777, 0.00183, 0.003375, 0.004873, 0.005508, 0.004873, 0.003375, 0.00183, 0.000777, 0.000258, 0.000067,
        0.000022, 0.000086, 0.000258, 0.000608, 0.001121, 0.001618, 0.001829, 0.001618, 0.001121, 0.000608, 0.000258, 0.000086, 0.000022,
        0.000006, 0.000022, 0.000067, 0.000158, 0.000291, 0.000421, 0.000476, 0.000421, 0.000291, 0.000158, 0.000067, 0.000022, 0.000006);

    }
    for(int i=filterSize; i<edgeMap.rows-filterSize; i++){
        for(int j=filterSize; j<edgeMap.cols-filterSize; j++){
            if(edgeMap.at<float>(i, j)==0){
                cv::Mat window;
                ImageUtils::creatMapRegion(edgeMap, window, i-filterSize/2, i+filterSize/2+1, j-filterSize/2, j+filterSize/2+1);
//                cv::Scalar test = cv::sum(window.mul(filter));
//                if(test[0]>0){
//                    int a = 1;
//                }
                edgeMapBlured.at<float>(i, j) = cv::sum(window.mul(filter))[0];
            }
        }
    }
}

void Refinement::gaussianBlurModified(cv::Mat &edgeMap, cv::Mat &filter, cv::Mat &edgeMapBlured) {
    edgeMapBlured = edgeMap.clone();
    for(int i=filter.rows; i<edgeMap.rows-filter.rows; i++){
        for(int j=filter.cols; j<edgeMap.cols-filter.cols; j++){
            if(edgeMap.at<float>(i, j)==0){
                cv::Mat window;
                ImageUtils::creatMapRegion(edgeMap, window, i-filter.rows/2, i+filter.rows/2+1, j-filter.cols/2, j+filter.cols/2+1);
//                cv::Scalar test = cv::sum(window.mul(filter));
//                if(test[0]>0){
//                    int a = 1;
//                }
                edgeMapBlured.at<float>(i, j) = cv::sum(window.mul(filter))[0];
            }
        }
    }
}

void Refinement::cameraEdgeGeneration(cv::Mat &imageCamera, cv::Mat &edgeMapCamera, cv::Mat &edgeMapCameraBlured, int blur, int blurSize) {
    cv::cvtColor(imageCamera, edgeMapCamera, cv::COLOR_BGR2GRAY);
    cv::Laplacian(edgeMapCamera, edgeMapCamera, CV_8U, 3, 1, 0, cv::BORDER_DEFAULT);
    cv::threshold(edgeMapCamera, edgeMapCamera, 80, 255, cv::THRESH_BINARY);
    edgeMapCamera.convertTo(edgeMapCamera, CV_32FC1);
    if(blur==1){
        Refinement::gaussianBlurModified(edgeMapCamera, edgeMapCameraBlured, blurSize);
    }
}

float Refinement::edgeDistance(cv::Mat &edgeMapCamera, cv::Mat &edgeMapLiDAR, int cnt) {
    float distance = cv::sum(edgeMapCamera.mul(edgeMapLiDAR))[0];
    cnt = cv::sum(edgeMapLiDAR)[0];
    return distance / float(cnt);
}

void Refinement::saveMatchResult(cv::Mat &edgeMapCamera, cv::Mat &edgeMapLiDAR, string savePath, int number, cv::Mat &three) {
    three = cv::Mat::zeros(edgeMapCamera.rows, edgeMapCamera.cols, CV_8UC3);
    vector<cv::Mat> channels;
    edgeMapCamera.convertTo(edgeMapCamera, CV_8UC1);
    for (int i=0;i<3;i++)
    {
        channels.push_back(edgeMapCamera);
    }
    cv::merge(channels,three);
    ImageUtils::colorTransfer(edgeMapLiDAR, three, 70);
    //cv::imwrite(savePath + "___" + "match" + to_string(number) + ".png", three);
}

float Refinement::errorRotation(cv::Mat &rotationResult, cv::Mat &rotationTruth) {
    cv::transpose(rotationResult, rotationResult);
    cv::Mat error = rotationResult * rotationTruth;
    float sy = sqrt(error.at<float>(0, 0) * error.at<float>(0, 0) + error.at<float>(1, 0) * error.at<float>(1, 0));
    float e1 = abs(atan2(error.at<float>(2, 1), error.at<float>(2, 2)) * 180 / M_PI);
    float e2 = abs(atan2(-error.at<float>(2, 0), sy) * 180 / M_PI);
    float e3 = abs(atan2(error.at<float>(1, 0), error.at<float>(0, 0)) * 180 / M_PI);
    float e = (e1 + e2 + e3) / 3;
    return e;
}

float Refinement::errorTranslation(cv::Mat &translationResult, cv::Mat &translationTruth) {
    float error_translation = abs(translationResult.at<float>(0, 0) - translationTruth.at<float>(0, 0)) +
                              abs(translationResult.at<float>(1, 0) - translationTruth.at<float>(1, 0)) +
                              abs(translationResult.at<float>(2, 0) - translationTruth.at<float>(2, 0));
    error_translation = error_translation / 3;
    return error_translation;
}

void Refinement::errorRotation(cv::Mat &rotationResult, cv::Mat &rotationTruth, cv::Point3f &errorRotation) {
    cv::transpose(rotationResult, rotationResult);
    cv::Mat error = rotationResult * rotationTruth;
    vector<float> euler_angle;
    float sy = sqrt(error.at<float>(0, 0) * error.at<float>(0, 0) + error.at<float>(1, 0) * error.at<float>(1, 0));
    float e1 = abs(atan2(error.at<float>(2, 1), error.at<float>(2, 2)) * 180 / M_PI);
    float e2 = abs(atan2(-error.at<float>(2, 0), sy) * 180 / M_PI);
    float e3 = abs(atan2(error.at<float>(1, 0), error.at<float>(0, 0)) * 180 / M_PI);
    errorRotation.x = e1;
    errorRotation.y = e2;
    errorRotation.z = e3;
}

void Refinement::errorTranslation(cv::Mat &translationResult, cv::Mat &translationTruth,
                                  cv::Point3f &errorTranslation) {
    errorTranslation.x = abs(translationResult.at<float>(0, 0) - translationTruth.at<float>(0, 0));
    errorTranslation.y = abs(translationResult.at<float>(1, 0) - translationTruth.at<float>(1, 0));
    errorTranslation.z = abs(translationResult.at<float>(2, 0) - translationTruth.at<float>(2, 0));
}

void Refinement::errorWrite(ofstream &outFile, float time, float errorRotation, float errorTranslation) {
//    ofstream outFile;
//    outFile.open(csvPath, ios::out);
//    outFile << "time" << "error" << endl;
    if(!isnan(errorRotation) && !isnan(errorTranslation)){
        outFile << time << ";" << errorRotation << ";" << errorTranslation << endl;
    }
}

void Refinement::errorWrite(ofstream &outFile, float time, cv::Point3f &errorRotation, cv::Point3f &errorTranslation) {
    outFile << time << ";" << errorRotation.x << ";" << errorRotation.y << ";" << errorRotation.z << ";" << errorTranslation.x << ";" << errorTranslation.y << ";" << errorTranslation.z << endl;
}

void Refinement::distanceErrorWrite(ofstream &outFile, float time, float lastDistance, float referenceDistance, float errorRotation, float errorTranslation) {
    if(!isnan(lastDistance) && !isnan(referenceDistance) && !isnan(errorRotation) && !isnan(errorTranslation)){
        outFile << time << ";" << lastDistance << ";" << referenceDistance << ";" << errorRotation << ";" << errorTranslation << endl;
    }
}