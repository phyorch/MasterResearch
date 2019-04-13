//
// Created by phyorch on 14/12/18.
//

#include "RealEquipment.h"
#include "Calibration.h"
#include "Sensor.h"
#include <boost/format.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <iostream>
using namespace std;

int main(){
    string path = "/home/phyorch/Data/2011_09_26_drive_0005_sync/velodyne_points/data/"; //0000000083.bin";
    string out_path;
    string in_path;
//    PandarLiDAR lidar = PandarLiDAR();
//    pcl::PointCloud<pandar_pointcloud::PointXYZIT>::Ptr point_cloud (new pcl::PointCloud<pandar_pointcloud::PointXYZIT>);
//    lidar.getXYZtxt(point_cloud, in_path, out_path);
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//    lidar.getXYZtxtST(point_cloud, in_path, out_path);
    
    for(int i=0; i<153; i++){
        in_path = path + HandEyeCalibration::zfill(i+1)+ ".bin";
        out_path = path + HandEyeCalibration::zfill(i+1)+ ".pcd";
        LiDAR::convertKittiBinData(in_path, out_path);
    }
    return 0;
}