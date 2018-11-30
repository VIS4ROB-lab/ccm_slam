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

#ifndef CSLAM_MAPMERGER_H_
#define CSLAM_MAPMERGER_H_

//C++
#include <boost/shared_ptr.hpp>
#include <mutex>
#include <opencv2/opencv.hpp>

//ROS
#include <ros/ros.h>
#include <ros/publisher.h>

//CSLAM
#include <cslam/config.h>
#include <cslam/estd.h>
#include <cslam/Converter.h>
#include <cslam/Datatypes.h>
#include <cslam/CentralControl.h>
#include <cslam/Map.h>
#include <cslam/KeyFrame.h>
#include <cslam/Optimizer.h>
#include <cslam/MapMatcher.h>
#include <cslam/ClientHandler.h>
#include <cslam/MapPoint.h>

//THIRDPARTY
#include "thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

using namespace std;
using namespace estd;

namespace cslam{

//forward decs
class MapMatcher;
class ClientHandler;
class Map;
class CentralControl;
class KeyFrame;
class MapPoint;
struct MapMatchHit;
//-----------------

class MapMerger
{
public:
    typedef boost::shared_ptr<MapMerger> mergeptr;
    typedef boost::shared_ptr<MapMatcher> matchptr;
    typedef boost::shared_ptr<ClientHandler> chptr;
    typedef boost::shared_ptr<KeyFrame> kfptr;
    typedef boost::shared_ptr<MapPoint> mpptr;
    typedef boost::shared_ptr<Map> mapptr;
    typedef boost::shared_ptr<CentralControl> ccptr;
public:
    typedef pair<set<kfptr>,int> ConsistentGroup;
    typedef map<kfptr,g2o::Sim3,std::less<kfptr>,
        Eigen::aligned_allocator<std::pair<const kfptr, g2o::Sim3> > > KeyFrameAndPose;
public:
    MapMerger(matchptr pMatcher);
    mapptr MergeMaps(mapptr pMapCurr, mapptr pMapMatch, vector<MapMatchHit> vMatchHits);

    bool isBusy();
private:
    void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap, std::vector<mpptr> vpLoopMapPoints);
    void SetBusy();
    void SetIdle();

    matchptr mpMatcher;

    bool bIsBusy;
    mutex mMutexBusy;

    std::vector<kfptr> mvpCurrentConnectedKFs;

    void RunGBA(idpair nLoopKf, mapptr pFusedMap);
};

} //end namespace

#endif
