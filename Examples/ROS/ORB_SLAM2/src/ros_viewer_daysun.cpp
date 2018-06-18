#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <eigen3/Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include "octomap_ros/Id_PointCloud2.h"
#include "octomap_ros/loopId_PointCloud2.h"

#include "ros_viewer_daysun.h"

using namespace std;

namespace My_Viewer {

    ros_viewer_daysun::ros_viewer_daysun(const string &strSettingPath) {
        ros::NodeHandle nh_;
        pub_pointCloud = nh_.advertise<octomap_ros::Id_PointCloud2>("ORB_SLAM/pointcloud2", 1);
        pub_pointCloudLocalUpdate = nh_.advertise<octomap_ros::Id_PointCloud2>("ORB_SLAM/pointcloudlocalup2", 1);
        pub_pointCloudFull = nh_.advertise<sensor_msgs::PointCloud2>("ORB_SLAM/pointcloudfull2", 1);
        //这是闭环完成之后的整体的点云
        pub_pointCloudupdated = nh_.advertise<octomap_ros::loopId_PointCloud2>("ORB_SLAM/pointcloudup2", 1);

        //Check settings file
        cv::FileStorage fSettings(strSettingPath.c_str(), cv::FileStorage::READ);
        if (!fSettings.isOpened()) {
            cerr << "Failed to open settings file at: " << strSettingPath << endl;
            exit(-1);
        }

        // Load camera parameters from settings file
        float fx = fSettings["Camera.fx"];
        float fy = fSettings["Camera.fy"];
        float cx = fSettings["Camera.cx"];
        float cy = fSettings["Camera.cy"];

        cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
        K.at<float>(0, 0) = fx;
        K.at<float>(1, 1) = fy;
        K.at<float>(0, 2) = cx;
        K.at<float>(1, 2) = cy;
        K.copyTo(mK);

        fx_ = fx;
        fy_ = fy;
        cx_ = cx;
        cy_ = cy;
        invfx = 1.0f / fx;
        invfy = 1.0f / fy;

        cv::Mat DistCoef(4, 1, CV_32F);
        DistCoef.at<float>(0) = fSettings["Camera.k1"];
        DistCoef.at<float>(1) = fSettings["Camera.k2"];
        DistCoef.at<float>(2) = fSettings["Camera.p1"];
        DistCoef.at<float>(3) = fSettings["Camera.p2"];
        const float k3 = fSettings["Camera.k3"];
        if (k3 != 0) {
            DistCoef.resize(5);
            DistCoef.at<float>(4) = k3;
        }
        DistCoef.copyTo(mDistCoef);

        cout << endl << "Camera Parameters read by ros viewer." << endl;
        cv::initUndistortRectifyMap(mK, mDistCoef, cv::Mat(), mK_new, cv::Size(640, 480), CV_32FC1, mMapx, mMapy);

        fullCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
        mbNeedUpdateKFs = false;
        mbLocalNeedUpdateKFs = false;
    }

    void ros_viewer_daysun::addKfToQueue(const cv::Mat im, const cv::Mat depthmap, const double timestamp,
                                  const cv::Mat mTcw, const unsigned int id) {
        rawData temp;
        temp.im = im.clone();
        temp.depth = depthmap.clone();
        temp.mTcw = mTcw.clone();
        temp.timestamp = timestamp;
        temp.id = id;

        unique_lock<mutex> lock(mMutexROSViewer);
        rawImages_queue.push_back(temp);
        rawImages.push_back(temp);
    }

//daysun
    void ros_viewer_daysun::addLocalupdate(const std::map<double, cv::Mat> kfposes) {
        localUpdateKFPose = kfposes;
        mbLocalNeedUpdateKFs = true;
    }

    void ros_viewer_daysun::addUpdatedKF(const std::map<double, cv::Mat> kfposes) {
        updatedKFposes = kfposes;
        mbNeedUpdateKFs = true;
    }

    void ros_viewer_daysun::updateFullPointCloud() {
        fullCloud = NULL;
        fullCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
//  cout<<"update full pointcloud rawimage size:"<< rawImages.size()<<endl;
        for (unsigned int i = 0; i < rawImages.size(); i++) {
            // update kf poses, check timestamp
//          cout << "image id: " << i << ", pose: " << rawImages[i].mTcw << endl;
            for (map<double, cv::Mat>::iterator mit = updatedKFposes.begin(), mend = updatedKFposes.end();
                 mit != mend; mit++) {
                double mtime = mit->first;
                if (rawImages[i].timestamp == mtime) {
                    rawImages[i].mTcw = mit->second.clone();
                    break;
                }
            }
//    cout << "updated pose: " << rawImages[i].mTcw << endl;
            // recreate point cloud
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
            cloud = createPointCloud(rawImages[i], 8);

            if (pub_pointCloudupdated.getNumSubscribers()) {
                octomap_ros::loopId_PointCloud2 my_msg;
                pcl::toROSMsg(*cloud, my_msg.msg);
                my_msg.msg.header.frame_id = "world";
                my_msg.msg.header.stamp = ros::Time(rawImages[rawImages.size() - 1].timestamp);
                my_msg.kf_id = rawImages[i].id;
                my_msg.loop_id = ros_viewer_daysun::getLoopId();
                cout << "global update kfid:" << rawImages[i].id << "loop_id:" << my_msg.loop_id << endl;
                pub_pointCloudupdated.publish(my_msg);
            }

//    *fullCloud += *cloud;
//    cout << "updated pose: " << rawImages[i].mTcw << endl;
        }
    }

    void ros_viewer_daysun::Run() {
        ros::NodeHandle nh;
        while (nh.ok()) {
            while (rawImages_queue.size()) {
                rawData temp = rawImages_queue[0];

                unique_lock<mutex> lock(mMutexROSViewer);
                rawImages_queue.erase(rawImages_queue.begin());
                lock.unlock();

                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
                cloud = createPointCloud(temp, 8);

                /// publish point cloud
                ///add mnId
                if (pub_pointCloud.getNumSubscribers()) {
                    octomap_ros::Id_PointCloud2 my_msg;
                    pcl::toROSMsg(*cloud, my_msg.msg);
                    my_msg.msg.header.frame_id = "world";
                    my_msg.msg.header.stamp = ros::Time(temp.timestamp);
                    my_msg.kf_id = temp.id;
//        cout<<"initial send kfid:"<<temp.id<<endl;
                    pub_pointCloud.publish(my_msg);
                }

                // publish full point cloud
                // simply ignoring the dynamic changes of pointcloud during SLAM
//      static unsigned int nfullcloud = 0;
//      *fullCloud += *cloud;

//      if (pub_pointCloudFull.getNumSubscribers()){
//        sensor_msgs::PointCloud2Ptr msgf(new sensor_msgs::PointCloud2());
//        pcl::toROSMsg(*fullCloud, *msgf);
//        nfullcloud++;
//        msgf->header.frame_id = "world";//
//        msgf->header.stamp = ros::Time(temp.timestamp);
//        pub_pointCloudFull.publish(msgf);
//      }
            }

            //daysun
            //publish pointCloud after the local optimization
            if (mbLocalNeedUpdateKFs) {
                for (size_t t = 0; t < updateLocalId.size(); t++) {
                    // update kf poses, check timestamp
                    int i = updateLocalId[t] - 1;
                    if (i < 0) continue;
                    rawData temp = rawImages[i];
                    for (map<double, cv::Mat>::iterator mit = localUpdateKFPose.begin(), mend = localUpdateKFPose.end();
                         mit != mend; mit++) {
                        double mtime = mit->first;
                        if (rawImages[i].timestamp == mtime) {
                            rawImages[i].mTcw = mit->second.clone();
                            break;
                        }
                    }
                    // recreate point cloud
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
                    cloud = createPointCloud(rawImages[i], 8);

                    if (pub_pointCloudLocalUpdate.getNumSubscribers()) {
                        octomap_ros::Id_PointCloud2 my_msg;
                        pcl::toROSMsg(*cloud, my_msg.msg);
                        my_msg.msg.header.frame_id = "world";
                        my_msg.msg.header.stamp = ros::Time(rawImages[rawImages.size() - 1].timestamp);
                        my_msg.kf_id = i;
//            cout<<"change kfid:"<<(i+1)<<endl;
                        pub_pointCloudLocalUpdate.publish(my_msg);
                    }
                }
                mbLocalNeedUpdateKFs = false;
                updateLocalId.clear();
            }

            // if a loop is closed, re-create the full point cloud
            if (mbNeedUpdateKFs) {
                updateFullPointCloud();
                mbNeedUpdateKFs = false;
            }

            usleep(3000);
        }
    }

//transform image+depth-->pointCloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ros_viewer_daysun::createPointCloud(const rawData rawimg, int step) {
        cv::Mat myrgb = rawimg.im;
        cv::Mat depth = rawimg.depth;
        cv::Mat mTcw = rawimg.mTcw;

        assert(depth.type() == CV_16UC1);
        assert(depth.cols == myrgb.cols);
        assert(depth.rows == myrgb.rows[1]);

        switch (myrgb.type()) {
            case CV_8UC1:
                cv::cvtColor(myrgb, myrgb, CV_GRAY2BGR);
                break;
            case CV_8UC3:
                break;
            default:
                assert(false);
        }

        float bad_point = std::numeric_limits<float>::quiet_NaN();

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr res(new pcl::PointCloud<pcl::PointXYZRGB>());

        int widthstep = step;
        res->width = std::ceil((double) depth.cols / (double) widthstep);
        res->height = std::ceil((double) depth.rows / (double) step);
        res->resize(res->width * res->height);

//    FileStorage fs("depthpc.txt", FileStorage::WRITE);
//    fs << "depth" << depth;
        bool bgr = false;

//  ros::Time tB = ros::Time::now();
        cv::Mat mRcw = mTcw.rowRange(0, 3).colRange(0, 3);
        cv::Mat mRwc(3, 3, CV_32FC1);
        mRwc = mRcw.t();
        cv::Mat mtcw = mTcw.rowRange(0, 3).col(3);
        cv::Mat mOw(3, 1, CV_32FC1);
        mOw = -mRwc * mtcw;
        Eigen::Matrix3f Rwc;
        Eigen::Vector3f Ow;
        cv::cv2eigen(mRwc, Rwc);
        cv::cv2eigen(mOw, Ow);

        pcl::PointCloud<pcl::PointXYZRGB>::iterator pc_iter = res->begin();
        for (int y = 0; y < depth.rows; y += step) {
            const uint16_t *depthPtr = depth.ptr<uint16_t>(y);
            const cv::Vec3b *rgbPtr = myrgb.ptr<cv::Vec3b>(y);

            for (int x = 0; x < depth.cols; x += widthstep) {
                const uint16_t &d = depthPtr[x];
                pcl::PointXYZRGB &pt = *pc_iter++;

                if ((d > 0) && (d < 5000)) {
                    // reproject to 3D. DistCoef not considered yet.
                    // TODO: use proper undistort map from opencv to speed up
//        cv::Mat mat(1,2,CV_32F);
//        mat.at<float>(0,0)=(float)x;
//        mat.at<float>(0,1)=(float)y;
//        mat=mat.reshape(2);
//        cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
                    const float u = x;//mMapx.at<float>(x,y);// not this remap matrix
                    const float v = y;//mMapy.at<float>(x,y);
                    const float xx = (u - cx_) * d * invfx * 0.001;
                    const float yy = (v - cy_) * d * invfy * 0.001;

                    Eigen::Vector3f x3Dc(xx, yy, d * 0.001);
                    Eigen::Vector3f p = Rwc * x3Dc + Ow;

                    pt.x = p(0);
                    pt.y = p(1);
                    pt.z = p(2);

                    const cv::Vec3b &col = rgbPtr[x];
                    if (bgr) {
                        pt.b = col[0];
                        pt.g = col[1];
                        pt.r = col[2];
                    } else {
                        pt.r = col[0];
                        pt.g = col[1];
                        pt.b = col[2];
                    }
                } else {
                    pt.x = pt.y = pt.z = bad_point;
                }
            }
        }
//  ros::Duration bTcreate = ros::Time::now() - tB;
//  std::cout << "time cost bTcreate interations: " << bTcreate.toSec() << std::endl;
        return res;
    }

} // namespace





