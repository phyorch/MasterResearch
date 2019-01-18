//
// Created by phyorch on 14/12/18.
//

#include "RealEquipment.h"

#include <boost/format.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <iostream>
using namespace std;

int main(){
    string in_path = "/home/phyorch/Data/rs_1547523719642061.pcd";
    string out_path = "/home/phyorch/Data/lidar.txt";

    PandarLiDAR lidar = PandarLiDAR();
//    pcl::PointCloud<pandar_pointcloud::PointXYZIT>::Ptr point_cloud (new pcl::PointCloud<pandar_pointcloud::PointXYZIT>);
//    lidar.getXYZtxt(point_cloud, in_path, out_path);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    lidar.getXYZtxtST(point_cloud, in_path, out_path);
    return 0;
}