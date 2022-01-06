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

#ifndef CSLAM_KEYFRAME_H_
#define CSLAM_KEYFRAME_H_

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
#include <cslam/MapPoint.h>
#include <cslam/Database.h>
#include <cslam/Map.h>
#include <cslam/Communicator.h>
#include <cslam/Frame.h>

//Thirdparty
#include <thirdparty/DBoW2/DBoW2/BowVector.h>
#include <thirdparty/DBoW2/DBoW2/FeatureVector.h>
#include "thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

//Msgs
#include <ccmslam_msgs/KF.h>
#include <ccmslam_msgs/KFred.h>
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
class Communicator;
class Frame;
class MapPoint;
class Map;
class KeyFrameDatabase;
//------------

class KeyFrame : public boost::enable_shared_from_this<KeyFrame>
{
public:
    typedef boost::shared_ptr<KeyFrame> kfptr;
    typedef boost::shared_ptr<MapPoint> mpptr;
    typedef boost::shared_ptr<Map> mapptr;
    typedef boost::shared_ptr<KeyFrameDatabase> dbptr;
    typedef boost::shared_ptr<Communicator> commptr;
    typedef boost::shared_ptr<CentralControl> ccptr;
public:
    //---constructor---
    KeyFrame(Frame &F, mapptr pMap, dbptr pKFDB, commptr pComm, eSystemState SysState, size_t UniqueId);
    KeyFrame(ccmslam_msgs::KF* pMsg, vocptr pVoc, mapptr pMap, dbptr pKFDB, commptr pComm, eSystemState SysState,
                       size_t UniqueId = defid, g2o::Sim3 mg2oS_wcurmap_wclientmap = g2o::Sim3()); //constructor for message input
    KeyFrame(vocptr pVoc, mapptr pMap, dbptr pKFDB, commptr pComm, eSystemState SysState, size_t UniqueId); // constructor for save/load

    void EstablishInitialConnectionsServer(); //this is necessary, because we cannot use shared_from_this() in constructor
    void EstablishInitialConnectionsClient(); //this is necessary, because we cannot use shared_from_this() in constructor

    //---communication---
    void ReduceMessage(ccmslam_msgs::KF *pMsgFull, ccmslam_msgs::KFred *pMsgRed);
    void ConvertToMessage(ccmslam_msgs::Map &msgMap, g2o::Sim3 mg2oS_wcurmap_wclientmap = g2o::Sim3(), kfptr pRefKf = nullptr, bool bForceUpdateMsg = false);
    void UpdateFromMessage(ccmslam_msgs::KF *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap = g2o::Sim3());
    void UpdateFromMessage(ccmslam_msgs::KFred *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap = g2o::Sim3());
    void WriteMembersFromMessage(ccmslam_msgs::KF *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap);
    bool SetPoseFromMessage(ccmslam_msgs::KF *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap);
    bool SetPoseFromMessage(ccmslam_msgs::KFred *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap);

    void SendMe();
    void MarkInOutBuffer() {unique_lock<mutex> lock(mMutexOut); mbInOutBuffer = true;}
    void UnMarkInOutBuffer() {unique_lock<mutex> lock(mMutexOut); mbInOutBuffer = false;}
    bool IsInOutBuffer() {unique_lock<mutex> lock(mMutexOut); return mbInOutBuffer;}
    void AddInformedClient(size_t ClientId){unique_lock<mutex> lock(mMutexOut); msuSentToClient.insert(ClientId);}
    bool SentToClient(size_t ClientId){unique_lock<mutex> lock(mMutexOut); return msuSentToClient.count(ClientId);}
    void Ack(){unique_lock<mutex> lock(mMutexOut); mbAck = true;}
    bool AckSet(){unique_lock<mutex> lock(mMutexOut); return mbAck;}
    bool IsSent(){unique_lock<mutex> lock(mMutexOut); return mbSentOnce;}
    void SetSendFull();
    bool CanBeForgotten();

    //---set/get pointers---
    void ReplaceMap(mapptr pNewMap);
    mapptr GetMapptr() {return mpMap;}
    void AddCommPtr(commptr pComm){unique_lock<mutex> lockMap(mMutexOut); mspComm.insert(pComm);}
    set<commptr> GetCommPtrs(){unique_lock<mutex> lockMap(mMutexOut); return mspComm;}

    //---visualization---
    bool mbFromServer;
    bool mbUpdatedByServer;
    std::string GetId();

    // Pose functions
    void SetPose(const cv::Mat &Tcw, bool bLock, bool bIgnorePoseMutex = false);
    void SetPoseFromTcp(const cv::Mat &Tcp_, bool bLock, kfptr pParent);
    cv::Mat GetPose();
    cv::Mat GetPoseInverse();
    cv::Mat GetCameraCenter();
    cv::Mat GetStereoCenter();
    cv::Mat GetRotation();
    cv::Mat GetTranslation();
    cv::Mat GetTcp();
    bool IsPoseLocked(){unique_lock<mutex> lock(mMutexPose); return mbPoseLock;}

    // Bag of Words Representation
    void ComputeBoW();

    // Covisibility graph functions
    void AddConnection(kfptr pKF, const int &weight);
    void EraseConnection(kfptr pKF);
    void UpdateConnections(bool bIgnoreMutex = false); //if called from map, we need IgnoreMutes=true if we then want to get get a KF from the map
    void UpdateBestCovisibles();
    std::set<kfptr> GetConnectedKeyFrames();
    std::vector<kfptr > GetVectorCovisibleKeyFrames();
    std::vector<kfptr> GetBestCovisibilityKeyFrames(const int &N);
    std::vector<kfptr> GetCovisiblesByWeight(const int &w);
    int GetWeight(kfptr pKF);

    // Spanning tree functions
    void AddChild(kfptr pKF);
    void EraseChild(kfptr pKF, bool bIgnoreMutex = false);
    void ChangeParent(kfptr pKF);
    std::set<kfptr> GetChilds();
    kfptr GetParent(bool bIgnorePoseMutex = false);
    bool hasChild(kfptr pKF);

    // Loop Edges
    void AddLoopEdge(kfptr pKF);
    std::set<kfptr> GetLoopEdges();

    // MapPoint observation functions
    void AddMapPoint(mpptr pMP, const size_t &idx, bool bLock = false);
    void EraseMapPointMatch(const size_t &idx, bool bLock = false);
    void EraseMapPointMatch(mpptr pMP, bool bLock = false);
    void ReplaceMapPointMatch(const size_t &idx, mpptr pMP, bool bLock = false, bool bOverrideLock = false);
    std::set<mpptr> GetMapPoints();
    std::vector<mpptr> GetMapPointMatches();
    int TrackedMapPoints(const int &minObs);
    mpptr GetMapPoint(const size_t &idx);
    bool IsMpLocked(const size_t &idx);
    void RemapMapPointMatch(mpptr pMP, const size_t &idx_now, const size_t &idx_new);

    // KeyPoint functions
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r) const;

    // Image
    bool IsInImage(const float &x, const float &y) const;

    // Enable/Disable bad flag changes
    void SetNotErase();
    void SetErase();

    // Set/check bad / empty flag
    void SetBadFlag(bool bSuppressMapAction = false, bool bNoParent = false);
    bool isBad() {unique_lock<mutex> lock(mMutexConnections); return mbBad;}
    bool IsEmpty() {unique_lock<mutex> lock(mMutexConnections); return mbIsEmpty;}
    bool mbOmitSending;

    // Compute Scene Depth (q=2 median). Used in monocular.
    float ComputeSceneMedianDepth(const int q);

    static bool weightComp( int a, int b){
        return a>b;
    }

    static bool compKFstamp(kfptr pKF1, kfptr pKF2) {
      return  pKF1->mTimeStamp > pKF2->mTimeStamp;
    }

    //---save/load

    friend class cereal::access;                                                                                                // Serialization

    template<class Archive>
    void save(Archive &archive) const {
        // pre-process data
        this->SaveData();
        // save
        archive(mdServerTimestamp, mTimeStamp, mdInsertStamp,
                mFrameId, mId,
//                mUniqueId,
                mVisId,
                mnGridCols, mnGridRows, mfGridElementWidthInv, mfGridElementHeightInv,
                fx, fy, cx, cy, invfx, invfy,
                N,
//                mvKeys, mvKeysUn,
//                mKeysAsCvMat,
                mKeysUnAsCvMat,
                mDescriptors,
                mTcp,
                mnScaleLevels, mfScaleFactor, mfLogScaleFactor,
                mvScaleFactors, mvLevelSigma2, mvInvLevelSigma2,
                mnMinX, mnMinY, mnMaxX, mnMaxY, mK,
                mT_SC,
                mbSentOnce,
                Tcw, Twc, Ow, mdPoseTime, Cw,
                mmMapPoints_minimal,
//                mParentId,
                mHalfBaseline
                );
    }

    template<class Archive>
    void load(Archive &archive) {
        // pre-process data
        mmMapPoints_minimal.clear();
        mvLoopEdges_minimal.clear();
        // load
        archive(mdServerTimestamp, mTimeStamp, mdInsertStamp,
                mFrameId, mId,
//                mUniqueId,
                mVisId,
                mnGridCols, mnGridRows, mfGridElementWidthInv, mfGridElementHeightInv,
                fx, fy, cx, cy, invfx, invfy,
                N,
//                mvKeys, mvKeysUn,
//                mKeysAsCvMat,
                mKeysUnAsCvMat,
                mDescriptors,
                mTcp,
                mnScaleLevels, mfScaleFactor, mfLogScaleFactor,
                mvScaleFactors, mvLevelSigma2, mvInvLevelSigma2,
                mnMinX, mnMinY, mnMaxX, mnMaxY, mK,
                mT_SC,
                mbSentOnce,
                Tcw, Twc, Ow, mdPoseTime, Cw,
                mmMapPoints_minimal,
//                mParentId,
                mHalfBaseline
                );
    }

    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:
    //---environment---
    double mdServerTimestamp;
    /*const*/ double mTimeStamp;
    double mdInsertStamp;

    //---IDs---
    static long unsigned int nNextId;
    idpair mFrameId;
    idpair mId;
    size_t mUniqueId;
    size_t mVisId;

    // Grid (to speed up feature matching)
    int mnGridCols;
    int mnGridRows;
    float mfGridElementWidthInv;
    float mfGridElementHeightInv;

    // Variables used by the tracking
    idpair mTrackReferenceForFrame;
    idpair mFuseTargetForKF;

    // Variables used by the local mapping
    idpair mBALocalForKF;
    idpair mBAFixedForKF;

    // Variables used by the keyframe database
    idpair mLoopQuery;
    idpair mMatchQuery;
    int mnLoopWords;
    float mLoopScore;
    idpair mRelocQuery;
    int mnRelocWords;
    float mRelocScore;

    // Variables used by loop closing
    cv::Mat mTcwGBA;
    cv::Mat mTcwBefGBA;
    idpair mBAGlobalForKF;
    bool mbLoopCorrected;

    // Variables used by map merging
    idpair mCorrected_MM;

    // Calibration parameters
    float fx, fy, cx, cy, invfx, invfy;

    // Number of KeyPoints
    int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    std::vector<cv::KeyPoint> mvKeys;
    std::vector<cv::KeyPoint> mvKeysUn;
    cv::Mat mDescriptors;

    //BoW
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // Pose relative to parent (this is computed when bad flag is activated)
    cv::Mat mTcp;

    // Scale
    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    std::vector<float> mvScaleFactors;
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;

    // Image bounds and calibration
    int mnMinX;
    int mnMinY;
    int mnMaxX;
    int mnMaxY;
    cv::Mat mK;

    // Transformation to body frame (for KF write-out
    Eigen::Matrix4d mT_SC;

    //---save/load
    mutable std::map<int, idpair> mmMapPoints_minimal;
//    mutable idpair mParentId = defpair;
    mutable vector<idpair> mvLoopEdges_minimal;
//    mutable cv::Mat mKeysAsCvMat; // distorted Keys not used on server side
    mutable cv::Mat mKeysUnAsCvMat;
    void SaveData() const;
    void ProcessAfterLoad(map<idpair, idpair> saved_kf_ids_to_sys_ids);

    // The following variables need to be accessed trough a mutex to be thread safe.
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
    dbptr mpKeyFrameDB;
    vocptr mpORBvocabulary;

    // SE3 Pose and camera center
    cv::Mat Tcw;
    cv::Mat Twc;
    cv::Mat Ow;
    bool mbPoseLock;
    bool mbPoseChanged;
    double mdPoseTime;
    cv::Mat Cw;

    // MapPoints associated to keypoints
    std::vector<mpptr> mvpMapPoints;
    std::vector<bool> mvbMapPointsLock;

    // Grid over the image to speed up feature matching
    void AssignFeaturesToGrid();
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);
    std::vector< std::vector <std::vector<size_t> > > mGrid;

    std::map<kfptr,int> mConnectedKeyFrameWeights;
    std::vector<kfptr> mvpOrderedConnectedKeyFrames;
    std::vector<int> mvOrderedWeights;

    // Spanning Tree and Loop Edges
    bool mbFirstConnection;
    kfptr mpParent;
    std::set<kfptr> mspChildrens;
    std::set<kfptr> mspLoopEdges;

    // Bad flags
    bool mbNotErase;
    bool mbToBeErased;
    bool mbBad;
    bool mbIsEmpty;

    float mHalfBaseline; // Only for visualization

    //---mutexes---
    std::mutex mMutexPose;
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;
    std::mutex mMutexOut;
    std::mutex mMapMutex;
};

} //end namespace

namespace cereal {

//save and load function for Eigen::Matrix type

    template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    inline
    typename std::enable_if<traits::is_output_serializable<BinaryData<_Scalar>, Archive>::value, void>::type
    save(Archive& ar, const Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& matrix) {
        const std::int32_t rows = static_cast<std::int32_t>(matrix.rows());
        const std::int32_t cols = static_cast<std::int32_t>(matrix.cols());
        ar(rows);
        ar(cols);
        ar(binary_data(matrix.data(), rows * cols * sizeof(_Scalar)));
    }

    template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    inline
    typename std::enable_if<traits::is_input_serializable<BinaryData<_Scalar>, Archive>::value, void>::type
    load(Archive& ar, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& matrix) {
        std::int32_t rows;
        std::int32_t cols;
        ar(rows);
        ar(cols);

        matrix.resize(rows, cols);

        ar(binary_data(matrix.data(), static_cast<std::size_t>(rows * cols * sizeof(_Scalar))));
    }

//save and load function for cv::Mat type
    template<class Archive>
    inline
    void save(Archive& ar, const cv::Mat& mat) {
        int rows, cols, type;
        bool continuous;

        rows = mat.rows;
        cols = mat.cols;
        type = mat.type();
        continuous = mat.isContinuous();

        ar & rows & cols & type & continuous;

        if (continuous) {
            const int data_size = rows * cols * static_cast<int>(mat.elemSize());
            auto mat_data = cereal::binary_data(mat.ptr(), data_size);
            ar & mat_data;
        }
        else {
            const int row_size = cols * static_cast<int>(mat.elemSize());
            for (int i = 0; i < rows; i++) {
                auto row_data = cereal::binary_data(mat.ptr(i), row_size);
                ar & row_data;
            }
        }
    }

    template<class Archive>
    void load(Archive& ar, cv::Mat& mat) {
        int rows, cols, type;
        bool continuous;

        ar & rows & cols & type & continuous;

        if (continuous) {
            mat.create(rows, cols, type);
            const int data_size = rows * cols * static_cast<int>(mat.elemSize());
            auto mat_data = cereal::binary_data(mat.ptr(), data_size);
            ar & mat_data;
        }
        else {
            mat.create(rows, cols, type);
            const int row_size = cols * static_cast<int>(mat.elemSize());
            for (int i = 0; i < rows; i++) {
                auto row_data = cereal::binary_data(mat.ptr(i), row_size);
                ar & row_data;
            }
        }
    }

} /* namespace cereal */

#endif
