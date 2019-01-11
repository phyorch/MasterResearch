//
// Created by phyorch on 13/12/18.
//

#include "RealEquipment.h"

ZEDCamera::ZEDCamera(sl::Camera &camera) {
    _mapSize = camera.getResolution();
}

ZEDCamera::~ZEDCamera() {}

void ZEDCamera::getDepthMap(sl::Mat &pointCloud, cv::Mat &depthMap) {
    depthMap = cv::Mat::zeros(_mapSize.height, _mapSize.width, CV_32FC1);
    for(int i=0; i<_mapSize.height; i++){
        for(int j=0; j<_mapSize.width; j++){
            sl::float4 point3D;
            pointCloud.getValue(j, i, &point3D);
            depthMap.at<float>(i, j) = point3D.z;
            cout << point3D.z;
        }
    }
}

void ZEDCamera::getDepthMapAndPointCloud(sl::Mat &pointCloud, cv::Mat &depthMap, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloudPCL) {
    depthMap = cv::Mat::zeros(_mapSize.height, _mapSize.width, CV_32FC1);
    for(int i=0; i<_mapSize.height; i++){
        for(int j=0; j<_mapSize.width; j++){
            sl::float4 point3D;
            pointCloud.getValue(j, i, &point3D);
            depthMap.at<float>(i, j) = point3D.z;
            if(isnan(point3D.x) || isnan(point3D.y) || isnan(point3D.z)){
                continue;
            }
            if(point3D.x ==INFINITY || point3D.y ==INFINITY || point3D.z ==INFINITY){
                continue;
            }
            pcl::PointXYZ point;
            point.x = point3D.x;
            point.y = point3D.y;
            point.z = point3D.z;
            pointCloudPCL->push_back(point);
        }
    }
}

void ZEDCamera::getDepthMapView(cv::Mat &depthMap, cv::Mat &outputMap) {
    outputMap = cv::Mat::zeros(depthMap.rows, depthMap.cols, depthMap.type());
    cv::normalize(depthMap, outputMap, 0, 255, cv::NORM_MINMAX);
    outputMap.convertTo(outputMap, CV_8UC1);
}

void ZEDCamera::nanComplete(cv::Mat &depthMap) {
    for(int i=0; i<depthMap.rows; i++){
        for(int j=0; j<depthMap.cols; j++){
            if(!(depthMap.at<float>(i, j)>0)){
                depthMap.at<float>(i, j) = 0;
            }
        }
    }
}

cv::Mat slMat2cvMat(sl::Mat& input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case sl::MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
        case sl::MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
        case sl::MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
        case sl::MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
        case sl::MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
        case sl::MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
        case sl::MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
        case sl::MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
        default: break;
    }

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM_CPU));
}

void ZEDCamera::getOneDepthFrame(sl::Camera &camera, HesaiLidarSDK &liDAR,  cv::Mat &depthMapCamera, string &outFileCameraMap, string outFileCameraImage, string outFileDepthView) {
    sl::Resolution map_size = camera.getResolution();
    sl::Mat leftImage, pointCloud, depthMap, depthView;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPCL(new pcl::PointCloud<pcl::PointXYZ>);

    // Capture new images until 'q' is pressed
    char key = ' ';
    int i = 0;
    time_t timeStart = time(nullptr);
    time_t timeDepthMap;
    while (key != 'q') {

        // Check that grab() is successful
        if (camera.grab() == sl::SUCCESS) {
            camera.retrieveImage(leftImage, sl::VIEW_LEFT);
            // Display image with OpenCV
            cv::imshow("VIEW", cv::Mat((int) leftImage.getHeight(), (int) leftImage.getWidth(), CV_8UC4, leftImage.getPtr<sl::uchar1>(sl::MEM_CPU)));
            //depth map
            timeDepthMap = time(nullptr);
            if(timeDepthMap-timeStart>3){

//                //camera.retrieveMeasure(depthView, sl::MEASURE_DEPTH);
//                camera.retrieveMeasure(pointCloud,sl::MEASURE_XYZRGBA);
//                //cv::Mat depthViewMat = cv::Mat((int) depthView.getHeight(), (int) depthView.getWidth(), CV_8UC4, depthView.getPtr<sl::uchar1>(sl::MEM_CPU));
//                getDepthMapAndPointCloud(pointCloud, depthMapCamera, pointCloudPCL);
//                nanComplete(depthMapCamera);
                camera.retrieveMeasure(depthMap, sl::MEASURE_DEPTH);
                depthMapCamera = slMat2cvMat(depthMap);
                string timestamp = to_string(timeDepthMap);
                cv::FileStorage fs(outFileCameraMap + timestamp + ".xml" , cv::FileStorage::WRITE);
                fs << "CameraDepthMap" << depthMapCamera;
                //liDAR.start();
                if(outFileCameraImage != " "){
                    sl::Mat imageLeft, imageRight;
                    camera.retrieveImage(imageLeft, sl::VIEW_LEFT);
                    camera.retrieveImage(imageRight, sl::VIEW_RIGHT);
                    cv::imwrite(outFileCameraImage + timestamp + "left.png", cv::Mat((int) imageLeft.getHeight(), (int) imageLeft.getWidth(), CV_8UC4, imageLeft.getPtr<sl::uchar1>(sl::MEM_CPU)));
                    cv::imwrite(outFileCameraImage + timestamp + "right.png", cv::Mat((int) imageRight.getHeight(), (int) imageRight.getWidth(), CV_8UC4, imageRight.getPtr<sl::uchar1>(sl::MEM_CPU)));
                }
                if(outFileDepthView != " "){
                    //cv::Mat depthMapView;
                    //getDepthMapView(depthMapCamera, depthMapView);
                    sl::Mat depthMapView;
                    camera.retrieveImage(depthMapView, sl::VIEW_DEPTH);
                    cv::imwrite(outFileDepthView + timestamp + ".png", cv::Mat((int) depthMapView.getHeight(), (int) depthMapView.getWidth(), CV_8UC4, depthMapView.getPtr<sl::uchar1>(sl::MEM_CPU)));
                }
            }
            key = cv::waitKey(5);

            // Change camera settings with keyboard
            //updateCameraSettings(key, zed);
        } else
            key = cv::waitKey(5);
        i++;
    }
}

PandarLiDAR::PandarLiDAR() {}

PandarLiDAR::~PandarLiDAR(void) {}

void PandarLiDAR::getXYZtxt(pcl::PointCloud<pandar_pointcloud::PointXYZIT>::Ptr &cloud, string &inPath, string &outPath) {
    if (pcl::io::loadPCDFile<pandar_pointcloud::PointXYZIT> (inPath, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        exit(EXIT_FAILURE);
    }
    ofstream outFile(outPath, ios::out);
    int i;
    for(i=0; i<cloud->points.size(); i++){
        outFile << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << endl;
    }
    outFile.close();
}