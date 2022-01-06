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

#ifndef CSLAM_MAPPOINT_H_
#define CSLAM_MAPPOINT_H_

//C++
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <opencv2/opencv.hpp>
#include <mutex>

//CSLAM
#include <cslam/config.h>
#include <cslam/estd.h>
#include <cslam/Datatypes.h>
#include <cslam/ORBVocabulary.h>
#include <cslam/Converter.h>
#include <cslam/KeyFrame.h>
#include <cslam/Communicator.h>
#include <cslam/Map.h>
#include <cslam/Frame.h>

//Thirdparty
#include <thirdparty/DBoW2/DBoW2/BowVector.h>
#include <thirdparty/DBoW2/DBoW2/FeatureVector.h>
#include "thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

//Msgs
#include <ccmslam_msgs/MP.h>
#include <ccmslam_msgs/MPred.h>
#include <ccmslam_msgs/Map.h>

//SERIALIZATION
#include "../../thirdparty/cereal/cereal.hpp"
#include "../../thirdparty/cereal/types/memory.hpp"
#include "../../thirdparty/cereal/types/utility.hpp"
#include "../../thirdparty/cereal/types/vector.hpp"
#include "../../thirdparty/cereal/types/polymorphic.hpp"
#include "../../thirdparty/cereal/types/concepts/pair_associative_container.hpp"
#include "../../thirdparty/cereal/types/base_class.hpp"
#include "../../thirdparty/cereal/archives/binary.hpp"
#include "../../thirdparty/cereal/archives/binary.hpp"
#include "../../thirdparty/cereal/access.hpp"

using namespace std;
using namespace estd;

namespace cslam{

//forward decs
class KeyFrame;
class Map;
class Communicator;
//-------------

class MapPoint : public boost::enable_shared_from_this<MapPoint>
{
public:

    typedef boost::shared_ptr<MapPoint> mpptr;
    typedef boost::shared_ptr<KeyFrame> kfptr;
    typedef boost::shared_ptr<Map> mapptr;
    typedef boost::shared_ptr<Communicator> commptr;
    typedef boost::shared_ptr<Frame> frameptr;
    typedef boost::shared_ptr<CentralControl> ccptr;
public:
    //---constructor---
    MapPoint(const cv::Mat &Pos, kfptr pRefKF, mapptr pMap, size_t ClientId, commptr pComm, eSystemState SysState, size_t UniqueId);
    MapPoint(ccmslam_msgs::MP *pMsg, mapptr pMap, commptr pComm, eSystemState SysState, size_t UniqueId = defid, g2o::Sim3 g2oS_wm_wc = g2o::Sim3()); //constructor for messages
    MapPoint(mapptr pMap, commptr pComm, eSystemState SysState, size_t UniqueId);  // constructor for save/load

    void EstablishInitialConnectionsServer(); //this is necessary, because we cannot use shared_from_this() in constructor
    void EstablishInitialConnectionsClient(); //this is necessary, because we cannot use shared_from_this() in constructor

    //---communication---
    void ReduceMessage(ccmslam_msgs::MP *pMsgFull, ccmslam_msgs::MPred *pMsgRed);
    void ConvertToMessage(ccmslam_msgs::Map &msgMap, kfptr pRefKf, g2o::Sim3 mg2oS_wcurmap_wclientmap = g2o::Sim3(), bool bForceUpdateMsg = false);
    void UpdateFromMessage(ccmslam_msgs::MP *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap = g2o::Sim3());
    void UpdateFromMessage(ccmslam_msgs::MPred *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap = g2o::Sim3());
    void WriteMembersFromMessage(ccmslam_msgs::MP *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap);
    bool SetPoseFromMessage(ccmslam_msgs::MP *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap);
    bool SetPoseFromMessage(ccmslam_msgs::MPred *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap);

    void SendMe();
    void MarkInOutBuffer() {unique_lock<mutex> lock(mMutexOut); mbInOutBuffer = true;}
    void UnMarkInOutBuffer() {unique_lock<mutex> lock(mMutexOut); mbInOutBuffer = false;}
    void EraseInOutBuffer();
    bool IsInOutBuffer() {unique_lock<mutex> lock(mMutexOut); return mbInOutBuffer;}
    void Ack(){unique_lock<mutex> lock(mMutexOut); mbAck = true;}
    bool AckSet(){unique_lock<mutex> lock(mMutexOut); return mbAck;}
    bool IsSent(){unique_lock<mutex> lock(mMutexOut); return mbSentOnce;}
    void SetSendFull();
    bool CanBeForgotten();

    //---set/get pointers---
    void ReplaceAndLock(mpptr pMP);
    void ReplaceMap(mapptr pNewMap);
    void AddCommPtr(commptr pComm){unique_lock<mutex> lockMap(mMutexOut); mspComm.insert(pComm);}
    set<commptr> GetCommPtrs(){unique_lock<mutex> lockMap(mMutexOut); return mspComm;}
    size_t GetMaxObsKFId(){unique_lock<mutex> lock(mMutexFeatures); return mMaxObsKFId;}

    //---visualization---
    std::string GetId();

    void SetMultiUse(){mbMultiUse = true;SendMe();}
    bool mbMultiUse;
    bool mbFromServer;
    bool mbUpdatedByServer;

    void SetWorldPos(const cv::Mat &Pos, bool bLock, bool bIgnorePosMutex = false);
    void SetWorldPosFromRef(const cv::Mat &Pos, bool bLock, kfptr pRefKf);
    cv::Mat GetWorldPos();
    bool IsPosLocked(){unique_lock<mutex> lock(mMutexPos); return mbPoseLock;}

    cv::Mat GetNormal();
    kfptr GetReferenceKeyFrame();
    void SetReferenceKeyFrame(kfptr pKF);

    std::map<kfptr,size_t> GetObservations();
    int Observations();

    void AddObservation(kfptr pKF,size_t idx, bool bLock = false);
    void EraseObservation(kfptr pKF, bool bLock = false,bool bSuppressMapAction = false);
    bool IsObsLocked(kfptr pKF);

    int GetIndexInKeyFrame(kfptr pKF, bool bIgnoreMutex = false);
    bool IsInKeyFrame(kfptr pKF);

    void SetBadFlag(bool bSuppressMapAction = false);
    bool isBad() {unique_lock<mutex> lock(mMutexFeatures); unique_lock<mutex> lock2(mMutexPos); return mbBad;}
    bool IsEmpty() {unique_lock<mutex> lock(mMutexFeatures); unique_lock<mutex> lock2(mMutexPos); return mbIsEmpty;}
    bool mbOmitSending;

    void Replace(mpptr pMP, bool bLock = false);
    mpptr GetReplaced();
    bool mbDoNotReplace;

    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }

    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth();

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();

    int PredictScale(const float &currentDist, kfptr pKF);
    int PredictScale(const float &currentDist, frameptr pF);

    void RemapObservationId(kfptr pKF, const size_t &idx);

    //---save/load

    friend class cereal::access;                                                                                                // Serialization

    template<class Archive>
    void save(Archive &archive) const {
        // pre-process data
        this->SaveData();
        // save
        archive(mId,
//                mUniqueId,
//                mFirstKfId,
                nObs,
                mWorldPos,
                mRefKfId,
                mmObservations_minimal
                );
    }

    template<class Archive>
    void load(Archive &archive) {
        // pre-process data
        mmObservations_minimal.clear();
        // load
        archive(mId,
//                mUniqueId,
//                mFirstKfId,
                nObs,
                mWorldPos,
                mRefKfId,
                mmObservations_minimal
                );
    }

public:
    static long unsigned int nNextId;
    idpair mId;
    size_t mUniqueId;
    idpair mFirstKfId = defpair;
    idpair mFirstFrame;
    int nObs;

    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    bool mbTrackInView;
    int mnTrackScaleLevel;
    float mTrackViewCos;
    idpair mTrackReferenceForFrame;
    idpair mLastFrameSeen;

    // Variables used by local mapping
    idpair mBALocalForKF;
    idpair mFuseCandidateForKF;
    size_t mInsertedWithKF;

    // Variables used by loop closing
    idpair mLoopPointForKF_LC;
    idpair mCorrectedByKF_LC;
    size_t mCorrectedReference_LC;
    bool mbLoopCorrected;

    // Variables used by map matching
    idpair mLoopPointForKF_MM;
    idpair mCorrectedByKF_MM;
    size_t mCorrectedReference_MM;

    idpair mBAGlobalForKF;

    cv::Mat mPosGBA;

    //---mutexes---
    static mutex mGlobalMutex;

    //---save/load
    mutable std::map<idpair, size_t> mmObservations_minimal;
    mutable idpair mRefKfId = defpair;
    void SaveData() const;

protected:
    //---communication---
    bool mbInOutBuffer;
    bool mbSentOnce;
    bool mbSendFull;
    set<size_t> msuSentToClient;
    bool mbAck;

    //---infrastructure---
    mapptr mpMap;
    set<commptr> mspComm;
    eSystemState mSysState;

     // Position in absolute coordinates
     cv::Mat mWorldPos;
     cv::Mat mRefPos;

     bool mbPoseLock;
     bool mbPoseChanged;

     // Keyframes observing the point and associated index in keyframe
     std::map<kfptr,size_t> mObservations;
     std::map<kfptr,bool> mObservationsLock;
     size_t mMaxObsKFId;

     // Mean viewing direction
     cv::Mat mNormalVector;

     // Best descriptor to fast matching
     cv::Mat mDescriptor;

     // Reference KeyFrame
     kfptr mpRefKF;

     // Tracking counters
     int mnVisible;
     int mnFound;

     // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad;
     mpptr mpReplaced;
     bool mbIsEmpty;

     // Scale invariance distances
     float mfMinDistance;
     float mfMaxDistance;

     //---mutexes---
     std::mutex mMutexOut;
     std::mutex mMutexPos;
     std::mutex mMutexFeatures;
};

} //end namespace

#endif
