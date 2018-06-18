/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

int main(int argc, char **argv) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr fullCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
            new pcl::PointCloud<pcl::PointXYZRGB>());

    string fname = "output.pcd";

    cout<< "Load the pcd file : " << fname << "...\n";
    if( pcl::io::loadPCDFile<pcl::PointXYZRGB>( fname, *fullCloud) == -1 ){
        PCL_ERROR( "Couldn't read pcd file\n");
        return (-1);
    }
    cout << "Finished Load.\n";

    pcl::visualization::CloudViewer viewer("Test Pcd Viewer");

    viewer.showCloud( fullCloud );

    while( ! viewer.wasStopped()) {
        ;
    }

    return 0;
}
