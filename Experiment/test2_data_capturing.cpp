//
// Created by phyorch on 11/12/18.
//
#include <iostream>
#include <string.h>
#include <time.h>
#include <ctime>

// ZED include
#include <sl/Camera.hpp>

// OpenCV include (for display)
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <boost/format.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include "Disparity.h"
#include "RealEquipment.h"
#include "hesaiLidarSDK.h"

using namespace std;
string user_name = "phyorch";
string data_name = "2019_01_15/2019_01_15_1";


void gpsCallback(int timestamp)
{
    printf("gps: %d" , timestamp);
}

void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp)
{
    printf("lidar: time %lf , points %d\n", timestamp , cld->points.size());
    pcl::PCDWriter writer;
    string outFile = "/home/" + user_name + "/Data/" + data_name + "/Pandar40Data/PCDDataKIT/" + to_string(timestamp) + ".pcd";
    writer.write<pandar_pointcloud::PointXYZIT>(outFile, *cld, false);
}

int main(){

    Eigen::Matrix3f intrinsic_para;
    intrinsic_para << 674.02594, 0, 668.906494, 0, 674.02594, 380.506592, 0, 0, 1;


    // Create a ZED Camera object
    sl::Camera zed;

    sl::InitParameters init_parameters;
    init_parameters.depth_mode = sl::DEPTH_MODE_ULTRA ; // Set the depth mode to ULTRA
    init_parameters.coordinate_units = sl::UNIT_MILLIMETER;
    init_parameters.depth_minimum_distance = 1 ; // Set the minimum depth perception distance to 15cm


    // Open the camera
    sl::ERROR_CODE err = zed.open(init_parameters);
    zed.setDepthMaxRangeValue(20); // Set the maximum depth perception distance to 40m
    if (err != sl::SUCCESS) {
        cout << toString(err) << endl;
        zed.close();
        return 1; // Quit if an error occurred
    }

    // Print help in console
    //printHelp();

    // Print camera information
    printf("ZED Model                 : %s\n", toString(zed.getCameraInformation().camera_model).c_str());
    printf("ZED Serial Number         : %d\n", zed.getCameraInformation().serial_number);
    printf("ZED Firmware              : %d\n", zed.getCameraInformation().firmware_version);
    printf("ZED Camera Resolution     : %dx%d\n", (int) zed.getResolution().width, (int) zed.getResolution().height);
    printf("ZED Camera FPS            : %d\n", (int) zed.getCameraFPS());

    // Create a Mat to store images
    sl::Mat zed_imagel, zed_imager;
    sl::Mat depth_for_display;
    cv::Mat depth_map;
    cv::Mat depth_map_output;
    string depth_path = "/home/"+ user_name + "/Data/" + data_name + "/ZEDData/DepthImage/depth_map";
    string depth_test_path = "/home/"+ user_name + "/Data/" + data_name + "/depth_test.png";
    string depth_view_path = "/home/"+ user_name + "/Data/" + data_name + "/ZEDData/DepthImage/depth_view";
    string image_path = "/home/"+ user_name + "/Data/" + data_name + "/ZEDData/RGBImage/image";
    //string imager_path = "/home/ubuntu/Data/ZEDData/RGBImage/imager";
    sl::Resolution map_size = zed.getResolution();
    ZEDCamera zed_camera = ZEDCamera(zed);
    HesaiLidarSDK psdk(
            8080				/* lidar data port */,
            8308				/* gps data port */,
            std::string("/home/"+ user_name + "/Data/" + data_name + "/Pandar40Data/Pandar40_Correction.csv")	/* calibration file of lidar */,
            lidarCallback 			/* point cloud data call back */,
            gpsCallback 			/* gps data callback */,
            HESAI_LIDAR_RAW_DATA_STRCUT_SINGLE_RETURN/* Return Mode: Single Return data structure */,
            40				/* laser counter */,
            HESAI_LIDAR_PCL_DATA_TYPE_REDUCED/* pcl data alignment */
    );
    sl::Mat point_cloud;
    sl::CameraInformation zed_test = zed.getCameraInformation();
    // Capture new images until 'q' is pressed
    char key = ' ';
    zed_camera.getOneDepthFrame(zed, psdk, depth_map, depth_path, image_path, depth_view_path);
    sl::RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = sl::SENSING_MODE_STANDARD;



    if (zed.grab(runtime_parameters) == sl::SUCCESS) {
        while (key != 'q') {

            // Check that grab() is successful
            if (zed.grab() == sl::SUCCESS) {
                // Retrieve left image
                zed.retrieveImage(zed_imagel, sl::VIEW_LEFT);
                zed.retrieveImage(zed_imager, sl::VIEW_RIGHT);

                zed.retrieveImage(depth_for_display, sl::VIEW_DEPTH);

                // Display image with OpenCV
                //cv::imshow("VIEW", cv::Mat((int) zed_imagel.getHeight(), (int) zed_imagel.getWidth(), CV_8UC4, zed_imagel.getPtr<sl::uchar1>(sl::MEM_CPU)));
                //cv::imshow("VIEWr", cv::Mat((int) zed_imager.getHeight(), (int) zed_imager.getWidth(), CV_8UC4, zed_imager.getPtr<sl::uchar1>(sl::MEM_CPU)));
                cv::imshow("VIEWd", cv::Mat((int) depth_for_display.getHeight(), (int) depth_for_display.getWidth(), CV_8UC4, depth_for_display.getPtr<sl::uchar1>(sl::MEM_CPU)));

                //RGB image
                if(key=='w'){
                    zed.retrieveImage(depth_for_display, sl::VIEW_DEPTH);
                    cv::imwrite(depth_test_path, cv::Mat((int) depth_for_display.getHeight(), (int) depth_for_display.getWidth(), CV_8UC4, depth_for_display.getPtr<sl::uchar1>(sl::MEM_CPU)));
                    return 0;
                }

                key = cv::waitKey(5);

                // Change camera settings with keyboard
                //updateCameraSettings(key, zed);
            } else
                key = cv::waitKey(5);
        }
    }


    // Exit
    zed.close();
    return EXIT_SUCCESS;
}
