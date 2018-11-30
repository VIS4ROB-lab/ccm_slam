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

#ifndef CSLAM_MAPMATCHER_H_
#define CSLAM_MAPMATCHER_H_

//C++
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <sstream>

//ROS
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

//CSLAM
#include <cslam/config.h>
#include <cslam/estd.h>
#include <cslam/Datatypes.h>
#include <cslam/Database.h>
#include <cslam/Map.h>
#include <cslam/KeyFrame.h>
#include <cslam/ORBVocabulary.h>
#include <cslam/ORBmatcher.h>
#include <cslam/Sim3Solver.h>
#include <cslam/Converter.h>
#include <cslam/Optimizer.h>
#include <cslam/MapMerger.h>

using namespace std;
using namespace estd;

namespace cslam{

//forward decs
class KeyFrameDatabase;
class MapMerger;
class Map;
//-------------

struct MapMatchHit
{
public:
    typedef boost::shared_ptr<KeyFrame> kfptr;
    typedef boost::shared_ptr<MapPoint> mpptr;
public:
    MapMatchHit(kfptr pKFCurr = nullptr, kfptr pKFMatch = nullptr, g2o::Sim3 g2oScw = g2o::Sim3(), std::vector<mpptr> vpLoopMapPoints = std::vector<mpptr>(), std::vector<mpptr> vpCurrentMatchedPoints = std::vector<mpptr>())
        : mpKFCurr(pKFCurr), mpKFMatch(pKFMatch), mg2oScw(g2oScw),
          mvpLoopMapPoints(vpLoopMapPoints), mvpCurrentMatchedPoints(vpCurrentMatchedPoints) {}
    kfptr mpKFCurr;
    kfptr mpKFMatch;
    g2o::Sim3 mg2oScw;
    std::vector<mpptr> mvpLoopMapPoints;
    std::vector<mpptr> mvpCurrentMatchedPoints;
};

class MapMatcher : public boost::enable_shared_from_this<MapMatcher>
{
public:
    typedef boost::shared_ptr<MapMatcher> matchptr;
    typedef boost::shared_ptr<MapMerger> mergeptr;
    typedef boost::shared_ptr<KeyFrameDatabase> dbptr;
    typedef boost::shared_ptr<KeyFrame> kfptr;
    typedef boost::shared_ptr<MapPoint> mpptr;
    typedef boost::shared_ptr<Map> mapptr;

    typedef pair<set<kfptr>,int> ConsistentGroup;
public:
    MapMatcher(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, dbptr pDB, vocptr pVoc, mapptr pMap0, mapptr pMap1, mapptr pMap2, mapptr pMap3);

    void Run();
    void InsertKF(kfptr pKF);
    void EraseKFs(vector<kfptr> vpKFs);
    int GetNumKFsinQueue();

private:
    bool CheckKfQueue();
    bool DetectLoop();
    bool ComputeSim3();
    void CorrectLoop();
    void PublishLoopEdges();
    void ClearLoopEdges();

    //Note: No need for a reset function for the MapMatcher

    ros::NodeHandle mNh;
    ros::NodeHandle mNhPrivate;

    ros::Publisher mPubMarker;

    visualization_msgs::Marker mMapMatchEdgeMsg;
    tf::TransformListener mTfListen;

    dbptr mpKFDB;
    vocptr mpVoc;
    map<size_t,mapptr> mmpMaps;
    set<mapptr> mspMaps;

    mapptr mpMap0;
    mapptr mpMap1;
    mapptr mpMap2;
    mapptr mpMap3;

    mergeptr mpMapMerger;

    std::list<kfptr> mlKfInQueue;
    std::mutex mMutexKfInQueue;

    // Loop detector parameters
    kfptr mpCurrentKF;
    kfptr mpMatchedKF;
    mapptr mpCurrMap;

    float mnCovisibilityConsistencyTh;
    std::map<mapptr,std::vector<ConsistentGroup>> mmvConsistentGroups;
    std::vector<kfptr> mvpEnoughConsistentCandidates;
    std::vector<kfptr> mvpCurrentConnectedKFs;
    std::vector<mpptr> mvpCurrentMatchedPoints;
    std::vector<mpptr> mvpLoopMapPoints;
    cv::Mat mScw;
    g2o::Sim3 mg2oScw;

    long unsigned int mLastLoopKFid;

    cv::Mat mMatchMatrix;
    std::map<mapptr,std::map<mapptr,vector<MapMatchHit>>> mFoundMatches;

    // Fix scale in the stereo/RGB-D case - reamnant from ORB-SLAM, always set to false
    bool mbFixScale;
};

} //end namespace

#endif
