#ifndef ROS_VIEWER_H
#define ROS_VIEWER_H

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nodelet/nodelet.h>
#include <iostream>
#include <mutex>
#include "include/System.h"

namespace My_Viewer {

    struct rawData {
        cv::Mat im;
        cv::Mat depth;
        cv::Mat mTcw;
        double timestamp;
        int id;
    };

    class ros_viewer {

    public:
        ros_viewer(){};
        ros_viewer(const std::string &strSettingPath, ORB_SLAM2::System *pSLAM);

        void addKfToQueue(const cv::Mat im, const cv::Mat depthmap, const double timestamp, const cv::Mat mTcw,
                          const unsigned int id);

        bool ifNewKFs();

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr createPointCloud(const rawData rawimg, int step = 1);

        cv::Mat coordinateTransform_view(cv::Mat mTcw);

        void updateAllPose();

        void publshAllPointCloud();

        void saveAllPointCloud( string fname );

        // Main function
        void Run();

        std::mutex mMutexROSViewer;

        bool viewer_stop;
    private:
        //Calibration matrix
        cv::Mat mK;
        cv::Mat mDistCoef;
        float fx_;
        float fy_;
        float cx_;
        float cy_;
        float invfx;
        float invfy;
        cv::Mat mMapx;
        cv::Mat mMapy;
        cv::Mat mK_new;

        ORB_SLAM2::System *mpSLAM;

        std::vector<rawData> rawImages; //global raw images to be used when a loop is closed

        ros::Publisher pub_pointCloudFull;

        int mNewKFcount;
        std::mutex mMutexNewKFcount;

    };

} // namespace
#endif // ROS_VIEWER_H
