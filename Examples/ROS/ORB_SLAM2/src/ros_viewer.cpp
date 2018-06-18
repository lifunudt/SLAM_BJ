#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <eigen3/Eigen/Core>
#include <opencv2/core/eigen.hpp>

#include "ros_viewer.h"

using namespace std;

namespace My_Viewer {

    ros_viewer::ros_viewer(const string &strSettingPath, ORB_SLAM2::System *pSLAM) : mpSLAM(pSLAM) {

        ros::NodeHandle nh_;

        pub_pointCloudFull = nh_.advertise<sensor_msgs::PointCloud2>("ORB_SLAM/pointcloudfull2", 1);

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

        mNewKFcount = 0;
        viewer_stop = false;
    }

    void ros_viewer::addKfToQueue(const cv::Mat im, const cv::Mat depthmap, const double timestamp,
                                  const cv::Mat mTcw, const unsigned int id) {
        rawData temp;
        temp.im = im.clone();
        temp.depth = depthmap.clone();
        temp.mTcw = mTcw.clone();
        temp.timestamp = timestamp;
        temp.id = id;

        {
            unique_lock<mutex> lock(mMutexROSViewer);
            rawImages.push_back(temp);
        }
        {
            unique_lock<mutex> lock1(mMutexNewKFcount);
            mNewKFcount++;
        }
    }

    bool ros_viewer::ifNewKFs() {

        unique_lock<mutex> lock1(mMutexNewKFcount);
        return ( mNewKFcount > 0);

    }

    // the world coordinates in ORB-SLAM was set to be the first frame coordinates
    cv::Mat ros_viewer::coordinateTransform_view(cv::Mat mTcw) {
        // rotate to world coordinates
        float rot[3][3] = {{0, -1, 0},
                           {0, 0,  -1},
                           {1, 0,  0}};
        float trans[3] = {0., 0., 0.5};
        cv::Mat mR1w = cv::Mat(3, 3, CV_32F, rot);
        cv::Mat mtw1 = cv::Mat(3, 1, CV_32F, trans);

        cv::Mat mRc1 = mTcw.rowRange(0, 3).colRange(0, 3);
        cv::Mat mtc1 = mTcw.rowRange(0, 3).col(3);
        cv::Mat mt1c = -mRc1.t() * mtc1;
        cv::Mat mRcw = mRc1 * mR1w;
        cv::Mat mtcw = -mRc1 * mt1c - mRcw * mtw1;

        cv::Mat mTcwr = cv::Mat::eye(4, 4, CV_32F);
        mRcw.copyTo(mTcwr.rowRange(0, 3).colRange(0, 3));
        mtcw.copyTo(mTcwr.rowRange(0, 3).col(3));

        return mTcwr.clone();
    }

    void ros_viewer::updateAllPose() {

        std::map<int, cv::Mat> mID2Pose = mpSLAM->getAllKFPose();

        {
            unique_lock<mutex> lock(mMutexROSViewer);

            for (std::vector<rawData>::iterator mit = rawImages.begin(); mit != rawImages.end(); mit++) {

                int tmpId = (*mit).id;

                if (mID2Pose.find(tmpId) != mID2Pose.end()) {
                    (*mit).mTcw = this->coordinateTransform_view(mID2Pose[tmpId]).clone();
                }
            }

        }

    }

    void ros_viewer::publshAllPointCloud() {

        if (rawImages.size() <= 0)
            return;

        updateAllPose();

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr fullCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
                new pcl::PointCloud<pcl::PointXYZRGB>());

        double mtimestamp = 0;

        for (size_t i = 0; i < rawImages.size(); i++) {

            rawData temp = rawImages[i];

            // create the pointcloud of the keyframe images
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

            cloud = createPointCloud(temp, 8);

            *fullCloud += *cloud;

            mtimestamp = temp.timestamp;

        }

        if (pub_pointCloudFull.getNumSubscribers()) {

            sensor_msgs::PointCloud2Ptr msgf(new sensor_msgs::PointCloud2());

            // translate the pcl type to ros message
            pcl::toROSMsg(*fullCloud, *msgf);

            msgf->header.frame_id = "world";

            msgf->header.stamp = ros::Time(mtimestamp);

            // pulish the pcl
            pub_pointCloudFull.publish(msgf);
            cout << "Publish the point Cloud. " << endl;
        }

        // set the tmp pcl free
        fullCloud.reset();

    }

    void ros_viewer::saveAllPointCloud(string fname) {

        if (rawImages.size() <= 0) {
            cout << "No pointcloud at all.\n";
            return;
        }

        cout << "Start saving pointcloud to the file :" << fname << endl;

        updateAllPose();

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr fullCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
                new pcl::PointCloud<pcl::PointXYZRGB>());

        for (size_t i = 0; i < rawImages.size(); i++) {

            rawData temp = rawImages[i];

            // create the pointcloud of the keyframe images
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

            cloud = createPointCloud(temp, 8);

            *fullCloud += *cloud;

        }

        pcl::io::savePCDFileASCII(fname, *fullCloud);

        cout << "Finshed saving !!" << endl;
        fullCloud.reset();

    }

    void ros_viewer::Run() {

        ros::NodeHandle nh;

        // Saving policy. if 10s have no new KF, need to save pcl to file

        string fname = "output.pcd";
        time_t start_t, end_t;

        time( &start_t );

        bool iffirst = true;
        while (nh.ok()) {

            if ( this->ifNewKFs() ) {

                {
                    unique_lock<mutex> lock(mMutexNewKFcount);
                    mNewKFcount = 0;
                    time(&start_t);
                    iffirst = false;
                }

                publshAllPointCloud();

            }

            time( &end_t );
            double delt_t = difftime( end_t , start_t ) ;
            cout << delt_t << endl;
            if( delt_t > 15.0 && !iffirst ) {
                this->saveAllPointCloud( fname );
                start_t = clock();
                break;
            }

            usleep(3000000);

            if (viewer_stop)
                break;
        }

    }

    //transform image+depth-->pointCloud

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ros_viewer::createPointCloud(const rawData rawimg, int step) {
        cv::Mat myrgb = rawimg.im;
        cv::Mat depth = rawimg.depth;
        cv::Mat mTcw = rawimg.mTcw;

        assert(depth.type() == CV_16UC1);
        assert(depth.cols == myrgb.cols);
        assert(depth.rows == myrgb.rows);

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

        //FileStorage fs("depthpc.txt", FileStorage::WRITE);
        //fs << "depth" << depth;
        bool bgr = false;

        //ros::Time tB = ros::Time::now();
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
                    //cv::Mat mat(1, 2, CV_32F);
                    //mat.at<float>(0, 0) = (float) x;
                    //mat.at<float>(0, 1) = (float) y;
                    //mat = mat.reshape(2);
                    //cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
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





