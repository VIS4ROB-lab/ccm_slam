/**
* This file is part of CCM-SLAM.
*
* Copyright (C): Patrik Schmuck <pschmuck at ethz dot ch> (ETH Zurich)
* For more information see <https://github.com/patriksc/CCM-SLAM>
*
* CCM-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* CCM-SLAM is based in the monocular version of ORB-SLAM2 by Raúl Mur-Artal.
* CCM-SLAM partially re-uses modules of ORB-SLAM2 in modified or unmodified condition.
* For more information see <https://github.com/raulmur/ORB_SLAM2>.
*
* CCM-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with CCM-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef CSLAM_VIEWER_H_
#define CSLAM_VIEWER_H_

#define FEATCOL 0,255,0 //BGR->Green
#define MUL 2.0 //factor by that the KF at current agent position is scaled

//C++
#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>

//ROS
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <ros/publisher.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>

//CSLAM
#include <cslam/config.h>
#include <cslam/Tracking.h>
#include <cslam/Map.h>
#include <cslam/CentralControl.h>

//Msgs
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;
using namespace estd;

namespace cslam{

//forward decs
class Tracking;
//----------

class Viewer
{
public:
    typedef boost::shared_ptr<Viewer> viewptr;
    typedef boost::shared_ptr<Tracking> trackptr;
    typedef boost::shared_ptr<Map> mapptr;
    typedef boost::shared_ptr<MapPoint> mpptr;
    typedef boost::shared_ptr<KeyFrame> kfptr;
    typedef boost::shared_ptr<CentralControl> ccptr;

    struct VisBundle
    {
        map<idpair,kfptr> mmpKFs;
        map<idpair,mpptr> mmpMPs;
        size_t mNativeId;
        size_t mMaxKfId;
        size_t mMaxKfIdUnique;
        set<idpair> msCurKFIds;
    };

public:
    Viewer(mapptr pMap, ccptr pCC);

    //Main function
    void RunClient();
    void RunServer();

    //Frame Drawing
    void SetTracker(trackptr pTracker){mpTracker = pTracker;}
    void UpdateAndDrawFrame();

    //Map Drawing
    void DrawMap(mapptr pMap);

    //Clearing
    void ClearCovGraph(size_t MapId);

    //Reset
    void RequestReset();

private:

    //ROS
    ros::NodeHandle mNh;

    boost::shared_ptr<image_transport::ImageTransport> mpIT;
    image_transport::Publisher mPubIm;

    //Infrastructure
    ccptr mpCC;
    mapptr mpMap;

    //+++++++++++++++++++++++++++++++++
    //Frame Drawing
    //+++++++++++++++++++++++++++++++++
    bool mbDrawFrame;
    trackptr mpTracker;

    bool DrawFrameTrue(){unique_lock<mutex> lockFrame(mMutexFrameDraw); return mbDrawFrame;}
    void UpdateAndDraw();
    void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);
    void UpdateFrame(); // Update info from the last processed frame.
    cv::Mat DrawFrame(); // Draw last processed frame.

    // Info of the frame to be drawn
    cv::Mat mIm;
    std::vector<mpptr> mvpFrameMPs;
    int N;
    vector<cv::KeyPoint> mvCurrentKeys;
    vector<bool> mvbMap, mvbVO;
    bool mbOnlyTracking;
    int mnTracked, mnTrackedVO;
    vector<cv::KeyPoint> mvIniKeys;
    vector<int> mvIniMatches;
    int mState;

    //+++++++++++++++++++++++++++++++++

    //+++++++++++++++++++++++++++++++++
    //Map Visualization
    //+++++++++++++++++++++++++++++++++


    //Publisher
    ros::Publisher mPubMarker0;
    ros::Publisher mPubPcl0;

    ros::Publisher mPubMarker1;
    ros::Publisher mPubPcl1;

    ros::Publisher mPubMarker2;
    ros::Publisher mPubPcl2;

    ros::Publisher mPubMarker3;
    ros::Publisher mPubPcl3;

    ros::Publisher mPubLocalMPs;

    //Data
    map<string,VisBundle> mmVisData;
    bool CheckVisData(){unique_lock<mutex> lock(mMutexDrawMap);return !mmVisData.empty();}

    VisBundle mCurVisBundle;
    string msCurFrame;
    string mSysType;

    set<size_t> msBlockedMaps;
    vector<int> mvNumPoints;

    //publisher methods
    void PubFramePoseAsFrustum();
    void PubKeyFramesAsFrusta();
    void PubMapPointsAsCloud();
    void PubCovGraph();
    void PubTrajectories();
    void PubLocalMPs();

    void ClearMarkers(size_t MapId);
    void ClearMapPoints(size_t n, size_t ClientId);

    //+++++++++++++++++++++++++++++++++

    //Reset
    void ResetIfRequested();
    bool mbResetRequested;

    //Mutexes
    mutex mMutexFrameDraw;
    mutex mMutexDrawMap;
    mutex mMutexReset;

    pcl::PointXYZRGB CreateMP(cv::Mat p3D, size_t nClientId);
};

} //end namespace

#endif
