////
//// Created by phyorch on 13/11/18.
////
//
#include "Sensor.h"
#include "Calibration.h"


StereoCamera::StereoCamera(BMPara bMPara, DispFilterPara dispFilterPara) {
    initCameraPara(bMPara, dispFilterPara);
}
StereoCamera::StereoCamera(SGBMPara sGBMPara, DispFilterPara dispFilterPara) {
    initCameraPara(sGBMPara, dispFilterPara);
};
StereoCamera::~StereoCamera() {}



void StereoCamera::initCameraPara(BMPara bMPara, DispFilterPara dispFilterPara) {
    _bMPara = bMPara;
    _dispFilterPara = dispFilterPara;
}

void StereoCamera::initCameraPara(SGBMPara sgbmpara, DispFilterPara dispFilterPara) {
    _sGBMPara = sgbmpara;
    _dispFilterPara = dispFilterPara;
}

cv::Ptr<cv::StereoBM> StereoCamera::bmMatch(cv::Mat leftImage, cv::Mat rightImage, cv::Mat &leftDisp) {

    //输入检查
    if (leftImage.empty() || rightImage.empty())
    {
        leftDisp = cv::Scalar(0);
        cout << "Empty input" << endl;
        exit(EXIT_FAILURE);
    }
    if (leftImage.cols == 0 || leftImage.rows == 0)
    {
        cout << "Empty input" << endl;
        exit(EXIT_FAILURE);
    }

    cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create();
    cv::Mat disp;
    _bMPara.NumDisparities = ((leftImage.cols / 8) + 15) & -16;
    bm->setBlockSize(_bMPara.BlockSize);
    bm->setMinDisparity(_bMPara.MinDisparity);
    bm->setNumDisparities(_bMPara.NumDisparities);
    bm->setTextureThreshold(_bMPara.TextureThreshold);
    bm->setUniquenessRatio(_bMPara.UniquenessRatio);
    bm->setSpeckleWindowSize(_bMPara.SpeckleWindowSize);
    bm->setSpeckleRange(_bMPara.SpeckleRange);
    bm->setDisp12MaxDiff(_bMPara.Disp12MaxDiff);
    bm->compute(leftImage, rightImage, leftDisp);
    return bm;
}


cv::Ptr<cv::StereoSGBM> StereoCamera::sgbmMatch(cv::Mat leftImage, cv::Mat rightImage, cv::Mat &leftDisp) {

    // 输入检查
    if (leftImage.empty() || rightImage.empty())
    {
        leftDisp = cv::Scalar(0);
        cerr << "Empty input" << endl;
        exit(EXIT_FAILURE);
    }
    if (leftImage.cols == 0 || leftImage.rows == 0)
    {
        cerr << "Empty input" << endl;
        exit(EXIT_FAILURE);
    }

    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create();
    int cn = leftImage.channels();
    int SADWindowSize = _sGBMPara.SADWindowSize;
    _sGBMPara.sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
    _sGBMPara.NumDisparities = ((leftImage.cols / 8) + 15) & -16;
    sgbm->setPreFilterCap(_sGBMPara.PreFilterCap);
    sgbm->setBlockSize(_sGBMPara.sgbmWinSize);
    sgbm->setP1(8 * cn*_sGBMPara.sgbmWinSize*_sGBMPara.sgbmWinSize);
    sgbm->setP2(32 * cn*_sGBMPara.sgbmWinSize*_sGBMPara.sgbmWinSize);
    sgbm->setMinDisparity(_sGBMPara.MinDisparity);
    sgbm->setNumDisparities(_sGBMPara.NumDisparities);
    sgbm->setUniquenessRatio(_sGBMPara.UniquenessRatio);
    sgbm->setSpeckleWindowSize(_sGBMPara.SpeckleWindowSize);
    sgbm->setSpeckleRange(_sGBMPara.SpeckleRange);
    sgbm->setDisp12MaxDiff(_sGBMPara.Disp12MaxDiff);
    sgbm->compute(leftImage, rightImage, leftDisp);
    return sgbm;
}

void StereoCamera::disparityFilter(cv::Mat &dispMap, cv::Mat &leftImage, cv::Mat &filteredDispMap, cv::Ptr<cv::StereoBM> &bM) {
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter = cv::ximgproc::createDisparityWLSFilter(bM);
    wls_filter->setLambda(_dispFilterPara.Lambda);
    wls_filter->setSigmaColor(_dispFilterPara.SigmaColor);
    wls_filter->filter(dispMap, leftImage, filteredDispMap);
}

void StereoCamera::disparityFilter(cv::Mat &dispMap, cv::Mat &leftImage, cv::Mat &filteredDispMap, cv::Ptr<cv::StereoBM> &bM, cv::Mat &rightDispMap) {
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter = cv::ximgproc::createDisparityWLSFilter(bM);
    wls_filter->setLambda(_dispFilterPara.Lambda);
    wls_filter->setSigmaColor(_dispFilterPara.SigmaColor);
    wls_filter->filter(dispMap, leftImage, filteredDispMap, rightDispMap);
}

void StereoCamera::disparityFilter(cv::Mat &dispMap, cv::Mat &leftImage, cv::Mat &filteredDispMap, cv::Ptr<cv::StereoSGBM> &sGBM) {
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter = cv::ximgproc::createDisparityWLSFilter(sGBM);

    wls_filter->setLambda(_dispFilterPara.Lambda);
    wls_filter->setSigmaColor(_dispFilterPara.SigmaColor);
    wls_filter->filter(dispMap, leftImage, filteredDispMap);
}

void StereoCamera::disparityFilter(cv::Mat &dispMap, cv::Mat &leftImage, cv::Mat &filteredDispMap, cv::Ptr<cv::StereoSGBM> &sGBM, cv::Mat &rightDispMap) {
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter = cv::ximgproc::createDisparityWLSFilter(sGBM);
    wls_filter->setLambda(_dispFilterPara.Lambda);
    wls_filter->setSigmaColor(_dispFilterPara.SigmaColor);
    wls_filter->filter(dispMap, leftImage, filteredDispMap, rightDispMap);
}

int StereoCamera::loadCameraPara(CameraPara cameraPara) {
    _cameraPara = cameraPara;
    return 1;
}

int StereoCamera::triangulaDepthMap(cv::Mat leftDisp, cv::Mat &depthMap) {
    depthMap = cv::Mat::zeros(leftDisp.rows, leftDisp.cols, CV_8UC1);
    for (int i=0; i<depthMap.rows; i++){
        for (int j=0; j<depthMap.cols; j++){
                if(leftDisp.at<uchar>(i,j)!=-1 || leftDisp.at<uchar>(i,j)==0) {
                    uchar test = leftDisp.at<uchar>(i, j);
                    depthMap.at<uchar>(i, j) = _cameraPara.base * _cameraPara.fx / leftDisp.at<uchar>(i, j);
                }
        }
    }
}



LiDAR::LiDAR(LiDARCalibParaKitti lidarCalibParaKitti) {
    initLiDARCalibParaKitti(lidarCalibParaKitti);
}

LiDAR::LiDAR(LiDARCalibParaKittiInverse lidarCalibParaKittiInverse) {
    initLiDARCalibParaKittiInverse(lidarCalibParaKittiInverse);
}

LiDAR::LiDAR(LiDARCalibParaKittiInverseEigen &lidarCalibParaKittiInverseEigen) {
    initLiDARCalibParaKittiInverseEigen(lidarCalibParaKittiInverseEigen);
}

LiDAR::~LiDAR() {}


void LiDAR::initLiDARCalibParaKitti(LiDARCalibParaKitti liDARCalibParaKitti) {
    _liDARCalibParaKitti = liDARCalibParaKitti;
}

void LiDAR::initLiDARCalibParaKittiInverse(LiDARCalibParaKittiInverse liDARCalibParaKittiInverse) {
    _liDARCalibParaKittiInverse = liDARCalibParaKittiInverse;
}

void LiDAR::initLiDARCalibParaKittiInverseEigen(LiDARCalibParaKittiInverseEigen &liDARCalibParaKittiInverseEigen) {
    _liDARCalibParaKittiInverseEigen = liDARCalibParaKittiInverseEigen;
}

void LiDAR::convertKittiBinData(string &inFile, string &outFile) {   //, vector *pointCloud
    std::fstream input(inFile.c_str(), std::ios::in | std::ios::binary);
    if (!input.good()) {
        cerr << "Could not read file: " << inFile << std::endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, std::ios::beg);

    pcl::PointCloud<pcl::PointXYZI>::Ptr points(new pcl::PointCloud<pcl::PointXYZI>);

    int i;
    for (i = 0; input.good() && !input.eof(); i++) {
        pcl::PointXYZI point;
        input.read((char *) &point.x, 3 * sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));
        points->push_back(point);
    }
    input.close();
    //pointCloud = &(points->points);

    std::cout << "Read KTTI point cloud with " << i << " points, writing to " << outFile << std::endl;
    pcl::PCDWriter writer;

    // Save DoN features
    writer.write<pcl::PointXYZI>(outFile, *points, false);
}

void LiDAR::projectPointKitti(cv::Mat &depthMap, pandar_pointcloud::PointXYZIT &point) {
    cv::Mat p;
    p = (cv::Mat_<float>(4,1) << point.x, point.y, point.z, 1);
    cv::Mat p_projected = _liDARCalibParaKitti.P * _liDARCalibParaKitti.R * _liDARCalibParaKitti.T * p;
    int u = int(p_projected.at<float>(1,1) / p_projected.at<float>(3,1));
    int v = int(p_projected.at<float>(2,1) / p_projected.at<float>(3,1));
    if(0<=u && 0<=v && u<depthMap.cols && v<depthMap.rows){  // && point.x>2
        //cout << "Point " << test << "is a valid point." << v << " " << u << endl;
        depthMap.at<float>(v,u) = point.x;    //sqrt(point.x*point.x + point.y*point.y);  //sqrt(point.x*point.x + point.y*point.y)
    }
}

void LiDAR::projectPointKittiSeperate(cv::Mat &depthMapLiDAR, pcl::PointXYZ &point, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPart) {
    cv::Mat p;
    p = (cv::Mat_<float>(3,1) << point.x, point.y, point.z);
    cv::Mat  p_ = _liDARCalibParaKittiInverse.Rotation * p + _liDARCalibParaKittiInverse.Translation;
    cv::Mat p__;
    p__ = (cv::Mat_<float>(4,1) << p_.at<float>(0,0), p_.at<float>(1,0), p_.at<float>(2,0), 1);
    cv::Mat p_projected = _liDARCalibParaKittiInverse.P * _liDARCalibParaKittiInverse.R * p__;
    int u = int(p_projected.at<float>(0,0) / p_projected.at<float>(2,0));
    int v = int(p_projected.at<float>(1,0) / p_projected.at<float>(2,0));
    if(0<=u && 0<=v && u<depthMapLiDAR.cols && v<depthMapLiDAR.rows && point.x>2){  // && point.x>2 to delete the point too close to the car, 50<=u to delete the points in region that has no disparity
        //cout << "Point " << test << "is a valid point." << v << " " << u << endl;
        depthMapLiDAR.at<float>(v,u) = point.y;    //sqrt(point.x*point.x + point.y*point.y);  //sqrt(point.x*point.x + point.y*point.y)
//        pcl::PointXYZ pointxyz;
//        pointxyz.x = point.x;
//        pointxyz.y = point.y;
//        pointxyz.z = point.z;
        pointCloudPart->push_back(pcl::PointXYZ(point.x, point.y, point.z));
    }
}

void LiDAR::projectPointKittiSeperate(cv::Mat &depthMapLiDAR, pcl::PointXYZI &point, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPart) {
    cv::Mat p;
    p = (cv::Mat_<float>(3,1) << point.x, point.y, point.z);
    cv::Mat  p_ = _liDARCalibParaKittiInverse.Rotation * p + _liDARCalibParaKittiInverse.Translation;
    cv::Mat p__;
    p__ = (cv::Mat_<float>(4,1) << p_.at<float>(0,0), p_.at<float>(1,0), p_.at<float>(2,0), 1);
    cv::Mat p_projected = _liDARCalibParaKittiInverse.P * _liDARCalibParaKittiInverse.R * p__;
    int u = int(p_projected.at<float>(0,0) / p_projected.at<float>(2,0));
    int v = int(p_projected.at<float>(1,0) / p_projected.at<float>(2,0));
    if(0<=u && 0<=v && u<depthMapLiDAR.cols && v<depthMapLiDAR.rows && point.x>2){  // && point.x>2 to delete the point too close to the car, 50<=u to delete the points in region that has no disparity
        //cout << "Point " << test << "is a valid point." << v << " " << u << endl;
        depthMapLiDAR.at<float>(v,u) = point.x;    //sqrt(point.x*point.x + point.y*point.y);  //sqrt(point.x*point.x + point.y*point.y)
//        pcl::PointXYZ pointxyz;
//        pointxyz.x = point.x;
//        pointxyz.y = point.y;
//        pointxyz.z = point.z;
        pointCloudPart->push_back(pcl::PointXYZ(point.x, point.y, point.z));
    }
}

void LiDAR::projectPointKittiSeperate(cv::Mat &depthMapLiDAR, pandar_pointcloud::PointXYZIT &point, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPart) {
    cv::Mat p;
    p = (cv::Mat_<float>(3,1) << point.x, point.y, point.z);
    cv::Mat  p_ = _liDARCalibParaKittiInverse.Rotation * p + _liDARCalibParaKittiInverse.Translation;
    cv::Mat p__;
    p__ = (cv::Mat_<float>(4,1) << p_.at<float>(0,0), p_.at<float>(1,0), p_.at<float>(2,0), 1);
    cv::Mat p_projected = _liDARCalibParaKittiInverse.P * _liDARCalibParaKittiInverse.R * p__;
    int u = int(p_projected.at<float>(0,0) / p_projected.at<float>(2,0));
    int v = int(p_projected.at<float>(1,0) / p_projected.at<float>(2,0));
    if(50<=u && 0<=v && u<depthMapLiDAR.cols && v<depthMapLiDAR.rows && point.y<0){  // && point.x>2 to delete the point too close to the car, 50<=u to delete the points in region that has no disparity
        //cout << "Point " << test << "is a valid point." << v << " " << u << endl;
        depthMapLiDAR.at<float>(v,u) = -point.y;    //sqrt(point.x*point.x + point.y*point.y);  //sqrt(point.x*point.x + point.y*point.y)
//        pcl::PointXYZ pointxyz;
//        pointxyz.x = point.x;
//        pointxyz.y = point.y;
//        pointxyz.z = point.z;
        pointCloudPart->push_back(pcl::PointXYZ(point.x, point.y, point.z));
    }
}

void LiDAR::projectPointKittiSeperate(cv::Mat &depthMapCamera, cv::Mat &depthMapLiDAR, pandar_pointcloud::PointXYZIT &point, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPart) {
    cv::Mat p;
    p = (cv::Mat_<float>(3,1) << point.x, point.y, point.z);
    cv::Mat  p_ = _liDARCalibParaKittiInverse.Rotation * p + _liDARCalibParaKittiInverse.Translation;
    cv::Mat p__;
    p__ = (cv::Mat_<float>(4,1) << p_.at<float>(0,0), p_.at<float>(1,0), p_.at<float>(2,0), 1);
    cv::Mat p_projected = _liDARCalibParaKittiInverse.P * _liDARCalibParaKittiInverse.R * p__;
    int u = int(p_projected.at<float>(0,0) / p_projected.at<float>(2,0));
    int v = int(p_projected.at<float>(1,0) / p_projected.at<float>(2,0));
    if(50<=u && 0<=v && u<depthMapLiDAR.cols && v<depthMapLiDAR.rows && point.y>0 && !isnan(depthMapCamera.at<float>(v,u)) && depthMapCamera.at<float>(v,u) != INFINITY){  // && point.x>2 to delete the point too close to the car, 50<=u to delete the points in region that has no disparity
        //cout << "Point " << test << "is a valid point." << v << " " << u << endl;
        depthMapLiDAR.at<float>(v,u) = point.y;    //sqrt(point.x*point.x + point.y*point.y);  //sqrt(point.x*point.x + point.y*point.y)
//        pcl::PointXYZ pointxyz;
//        pointxyz.x = point.x;
//        pointxyz.y = point.y;
//        pointxyz.z = point.z;
        pointCloudPart->push_back(pcl::PointXYZ(point.x, point.y, point.z));
    }
}

void LiDAR::projectPointKittiSeperate(cv::Mat &depthMapCamera, cv::Mat &depthMapLiDAR, pandar_pointcloud::PointXYZIT &point, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudCamera, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudLiDAR) {
    cv::Mat p;
    p = (cv::Mat_<float>(3,1) << point.x, point.y, point.z);
    cv::Mat  p_ = _liDARCalibParaKittiInverse.Rotation * p + _liDARCalibParaKittiInverse.Translation;
    cv::Mat p__;
    p__ = (cv::Mat_<float>(4,1) << p_.at<float>(0,0), p_.at<float>(1,0), p_.at<float>(2,0), 1);
    cv::Mat p_projected = _liDARCalibParaKittiInverse.P * _liDARCalibParaKittiInverse.R * p__;
    int u = int(p_projected.at<float>(0,0) / p_projected.at<float>(2,0));
    int v = int(p_projected.at<float>(1,0) / p_projected.at<float>(2,0));
    if(50<=u && 0<=v && u<depthMapLiDAR.cols && v<depthMapLiDAR.rows && point.y>0 && !isnan(depthMapCamera.at<float>(v,u)) && depthMapCamera.at<float>(v,u) != INFINITY){  // && point.x>2 to delete the point too close to the car, 50<=u to delete the points in region that has no disparity

        depthMapLiDAR.at<float>(v,u) = point.y;
        pointCloudLiDAR->push_back(pcl::PointXYZ(point.x, point.y, point.z));

        cv::Point2f point(u, v);
        pcl::PointXYZ point3d;
        projectPointInverse(point, depthMapCamera.at<float>(v ,u), point3d);
        point3d.x = point3d.x /1000;
        point3d.y = point3d.y /1000;
        point3d.z = point3d.z /1000;
        pointCloudCamera->push_back(point3d);
    }
}

void LiDAR::projectPointKittiSeperateEigen(cv::Mat &depthMap, pandar_pointcloud::PointXYZIT &point) {
    Eigen::Vector3f p(point.x, point.y, point.z);
    Eigen::Vector3f p_ = _liDARCalibParaKittiInverseEigen.Rotation * p - _liDARCalibParaKittiInverseEigen.Rotation * _liDARCalibParaKittiInverseEigen.Translation;
    Eigen::Vector4f p__(p_[0], p_[1], p_[2], 1);
    Eigen::Vector4f p___ = _liDARCalibParaKittiInverseEigen.R * p__;
    Eigen::Vector3f p_projected = _liDARCalibParaKittiInverseEigen.P * p___;
    int u = int(p_projected[0] / p_projected[2]);
    int v = int(p_projected[1] / p_projected[2]);
    if(0<=u && 0<=v && u<depthMap.cols && v<depthMap.rows){  // && point.x>2
        //cout << "Point " << test << "is a valid point." << v << " " << u << endl;
        depthMap.at<float>(v,u) = -point.y;    //sqrt(point.x*point.x + point.y*point.y);  //sqrt(point.x*point.x + point.y*point.y)
    }
}

void LiDAR::projectPointKittiEigen(cv::Mat &depthMap, pandar_pointcloud::PointXYZIT &point) {
//    Eigen::Vector4f p(point.x, point.y, point.z, 1);
//    Eigen::Vector4f p_ = _liDARCalibParaKitti.T * p;
//    Eigen::Vector4f p__ = _liDARCalibParaKitti.R * p_;
//    Eigen::Vector3f p_projected = _liDARCalibParaKitti.P * p__;
//    int u = int(p_projected[0] / p_projected[2]);
//    int v = int(p_projected[1] / p_projected[2]);
//    if(0<=u && 0<=v && u<depthMap.cols && v<depthMap.rows){  // && point.x>2
//        //cout << "Point " << test << "is a valid point." << v << " " << u << endl;
//        depthMap.at<float>(v,u) = point.x;    //sqrt(point.x*point.x + point.y*point.y);  //sqrt(point.x*point.x + point.y*point.y)
//    }
}

void LiDAR::projectData(string inFile, cv::Mat &depthMapLiDAR, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloudPart, LiDARPointType pointType, MatType matType, float downsampleVox) {

    if(matType==CV){
        depthMapLiDAR = cv::Mat::zeros(_liDARCalibParaKittiInverse.imageSize, CV_32FC1);
    }
    if(matType==EIGEN) {
        depthMapLiDAR = cv::Mat::zeros(_liDARCalibParaKittiInverseEigen.imageSize, CV_32FC1);
    }

    if(pointType==XYZ){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (inFile, *cloud) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read file in LiDAR::projectData\n");
            exit(EXIT_FAILURE);
        }
        if(downsampleVox != 0){
            pointCloudAlignment->pointCloudDownsample(cloud, cloud, downsampleVox);
        }
        for(int i=0; i<cloud->points.size(); i++){
            if(matType==CV){
                projectPointKittiSeperate(depthMapLiDAR, cloud->points[i], pointCloudPart);
            }
        }
    }
    if(pointType==XYZI){
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        if (pcl::io::loadPCDFile<pcl::PointXYZI> (inFile, *cloud) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
            exit(EXIT_FAILURE);
        }
        if(downsampleVox != 0){
            pointCloudAlignment->pointCloudDownsample(cloud, cloud, downsampleVox);
        }
        for(int i=0; i<cloud->points.size(); i++){
            if(matType==CV){
                projectPointKittiSeperate(depthMapLiDAR, cloud->points[i], pointCloudPart);
            }
        }
    }
    if(pointType==XYZIT){
        pcl::PointCloud<pandar_pointcloud::PointXYZIT>::Ptr cloud(new pcl::PointCloud<pandar_pointcloud::PointXYZIT>);
        if (pcl::io::loadPCDFile<pandar_pointcloud::PointXYZIT> (inFile, *cloud) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read file in LiDAR::projectData\n");
            exit(EXIT_FAILURE);
        }

        for(int i=0; i<cloud->points.size(); i++){
            if(matType==CV){
                projectPointKittiSeperate(depthMapLiDAR, cloud->points[i], pointCloudPart);
            }
            if(matType==EIGEN){
                projectPointKittiSeperateEigen(depthMapLiDAR, cloud->points[i]);
            }
        }
    }
}

void LiDAR::projectData(string inFile, cv::Mat &depthMapCamera, cv::Mat &depthMapLiDAR, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloudPart, LiDARDataType dataType, LiDARDataSource dataSource, LiDARPointType pointType, ExtrinsicDirection liDARDirection, MatType matType) {

    if(matType==CV){
        depthMapLiDAR = cv::Mat::zeros(_liDARCalibParaKittiInverse.imageSize, CV_32FC1);
    }
    if(matType==EIGEN){
        depthMapLiDAR = cv::Mat::zeros(_liDARCalibParaKittiInverseEigen.imageSize, CV_32FC1);
    }

    if(pointType==XYZ){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (inFile, *cloud) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read file in LiDAR::projectData\n");
            exit(EXIT_FAILURE);
        }
        int i;
        for(i=0; i<cloud->points.size(); i++){
            if(matType==CV){
                projectPointKittiSeperate(depthMapLiDAR, cloud->points[i], pointCloudPart);
            }
        }
    }
    if(pointType==XYZI){
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        if (pcl::io::loadPCDFile<pcl::PointXYZI> (inFile, *cloud) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
            exit(EXIT_FAILURE);
        }
        for(int i=0; i<cloud->points.size(); i++){
            if(matType==CV){
                projectPointKittiSeperate(depthMapLiDAR, cloud->points[i], pointCloudPart);
            }
        }
    }
    if(pointType==XYZIT){
        pcl::PointCloud<pandar_pointcloud::PointXYZIT>::Ptr cloud(new pcl::PointCloud<pandar_pointcloud::PointXYZIT>);
        if (pcl::io::loadPCDFile<pandar_pointcloud::PointXYZIT> (inFile, *cloud) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read file in LiDAR::projectData\n");
            exit(EXIT_FAILURE);
        }
        int i;
        for(i=0; i<cloud->points.size(); i++){
            if(matType==CV){
                projectPointKittiSeperate(depthMapCamera, depthMapLiDAR, cloud->points[i], pointCloudPart);
            }
            if(matType==EIGEN){
                projectPointKittiSeperateEigen(depthMapLiDAR, cloud->points[i]);
            }
        }
    }
}

void LiDAR::projectData(string inFile, cv::Mat &depthMapCamera, cv::Mat &depthMapLiDAR,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloudCamera,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloudLiDAR, LiDARDataType dataType,
                        LiDARDataSource dataSource, LiDARPointType poinyType, ExtrinsicDirection liDARDirection,
                        MatType matType) {
    if(matType==CV){
        depthMapLiDAR = cv::Mat::zeros(_liDARCalibParaKittiInverse.imageSize, CV_32FC1);
    }
    if(matType==EIGEN){
        depthMapLiDAR = cv::Mat::zeros(_liDARCalibParaKittiInverseEigen.imageSize, CV_32FC1);
    }

    pcl::PointCloud<pandar_pointcloud::PointXYZIT>::Ptr cloud(new pcl::PointCloud<pandar_pointcloud::PointXYZIT>);
    if (pcl::io::loadPCDFile<pandar_pointcloud::PointXYZIT> (inFile, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file in LiDAR::projectData\n");
        exit(EXIT_FAILURE);
    }
    for(int i=0; i<cloud->points.size(); i++){
        projectPointKittiSeperate(depthMapCamera, depthMapLiDAR, cloud->points[i], pointCloudCamera, pointCloudLiDAR);
    }
}

void LiDAR::updateParameters(cv::Mat &rotation, cv::Mat &translation) {
        _liDARCalibParaKittiInverse.Rotation = rotation;
        _liDARCalibParaKittiInverse.Translation = translation;
}

void LiDAR::projectPointInverse(cv::Point2f &point, float depth, pcl::PointXYZ &point3d) {
    point3d.x = (point.x * depth - _liDARCalibParaKittiInverse.P.at<float>(0, 2) * depth) / _liDARCalibParaKittiInverse.P.at<float>(0, 0);
    point3d.y = (point.y * depth - _liDARCalibParaKittiInverse.P.at<float>(1, 2) * depth) / _liDARCalibParaKittiInverse.P.at<float>(1, 1);
    point3d.z = depth;
}

void LiDAR::projectPointInverse(cv::Point2f &point2d, float depth, cv::Point3f &point3d) {
    point3d.x = (point2d.x * depth - _liDARCalibParaKittiInverse.P.at<float>(0, 2) * depth) / _liDARCalibParaKittiInverse.P.at<float>(0, 0);
    point3d.y = (point2d.y * depth - _liDARCalibParaKittiInverse.P.at<float>(1, 2) * depth) / _liDARCalibParaKittiInverse.P.at<float>(1, 1);
    point3d.z = depth;
}

void LiDAR::projectPointInverseKitti(cv::Point2f &point, int depth, pcl::PointXYZ &point3d) {
    point3d.z = 0;//-(point.y * depth - _liDARCalibParaKittiInverse.P.at<float>(1, 2) * depth) / _liDARCalibParaKittiInverse.P.at<float>(1, 1);
    point3d.y = 0;//-(point.x * depth - _liDARCalibParaKittiInverse.P.at<float>(0, 2) * depth) / _liDARCalibParaKittiInverse.P.at<float>(0, 0);
    point3d.x = depth;
}