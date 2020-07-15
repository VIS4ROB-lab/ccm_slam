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

#include <cslam/MapPoint.h>

namespace cslam {

long unsigned int MapPoint::nNextId=0;
mutex MapPoint::mGlobalMutex;

MapPoint::MapPoint(mapptr pMap, commptr pComm, eSystemState SysState, size_t UniqueId)
    : nObs(0),mpReplaced(nullptr),mpMap(pMap),
      mbIsEmpty(false), mbPoseLock(false),mbPoseChanged(false), mbSentOnce(true),mbInOutBuffer(false),
      mLoopPointForKF_LC(defpair), mCorrectedByKF_LC(defpair),mCorrectedReference_LC(defid),
      mLoopPointForKF_MM(defpair), mCorrectedByKF_MM(defpair),mCorrectedReference_MM(defid),
      mBAGlobalForKF(defpair),mBALocalForKF(defpair),
      mFuseCandidateForKF(defpair),
      mSysState(SysState),mbDoNotReplace(false),mbOmitSending(false),
      mbLoopCorrected(false),mbMultiUse(false),
      mbBad(false),mbAck(true),mbUpdatedByServer(false),mInsertedWithKF(-1),mMaxObsKFId(0),mbSendFull(false)
{
    mId = defpair;
    mUniqueId = UniqueId;
    mspComm.insert(pComm);
}

MapPoint::MapPoint(const cv::Mat &Pos, kfptr pRefKF, mapptr pMap, size_t ClientId, commptr pComm, eSystemState SysState, size_t UniqueId)
    : mFirstKfId(pRefKF->mId), mFirstFrame(pRefKF->mFrameId), mUniqueId(UniqueId),
      nObs(0), mTrackReferenceForFrame(defpair),mLastFrameSeen(defpair),
      mpReplaced(nullptr), mfMinDistance(0), mfMaxDistance(0),
      mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
      mpMap(pMap),
      mbIsEmpty(false), mbPoseLock(false),mbPoseChanged(false), mbSentOnce(false),mbInOutBuffer(false),
      mLoopPointForKF_LC(defpair), mCorrectedByKF_LC(defpair),mCorrectedReference_LC(-1),
      mLoopPointForKF_MM(defpair), mCorrectedByKF_MM(defpair),mCorrectedReference_MM(-1),
      mBAGlobalForKF(defpair),mBALocalForKF(defpair),
      mFuseCandidateForKF(defpair),
      mSysState(SysState),mbDoNotReplace(false),mbOmitSending(false),
      mbLoopCorrected(false),mbMultiUse(false),
      mbAck(false),mbFromServer(false),mbUpdatedByServer(false),mInsertedWithKF(-1),
      mMaxObsKFId(0),mbSendFull(true)
{
    mspComm.insert(pComm);

    Pos.copyTo(mWorldPos);

    if(mpRefKF)
    {
        cv::Mat T_cref_w =  mpRefKF->GetPose();

        cv::Mat Rcw = T_cref_w.rowRange(0,3).colRange(0,3);
        cv::Mat tcw = T_cref_w.rowRange(0,3).col(3);

        mRefPos = Rcw * mWorldPos + tcw;
    }
    else
    {
        cout << "\033[1;31m!!!!! FATAL ERROR !!!!!\033[0m " << __func__ << __LINE__ << ": MP has no parent" << endl;
        throw infrastructure_ex();
    }

    mNormalVector = cv::Mat::zeros(3,1,CV_32F);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    while(!mpMap->LockPointCreation()){usleep(params::timings::miLockSleep);}
    mId=make_pair(nNextId++,ClientId);
    mpMap->UnLockPointCreation();

    if(mUniqueId == 0 && !mId.first == 0) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m MapPoint::MapPoint(..): mUniqueId not set" << endl;
}

MapPoint::MapPoint(ccmslam_msgs::MP *pMsg, mapptr pMap, commptr pComm, eSystemState SysState, size_t UniqueId, g2o::Sim3 g2oS_wm_wc)
    : nObs(0),mpReplaced(nullptr),mpMap(pMap),
      mbIsEmpty(false), mbPoseLock(false),mbPoseChanged(false), mbSentOnce(true),mbInOutBuffer(false),
      mLoopPointForKF_LC(defpair), mCorrectedByKF_LC(defpair),mCorrectedReference_LC(defid),
      mLoopPointForKF_MM(defpair), mCorrectedByKF_MM(defpair),mCorrectedReference_MM(defid),
      mBAGlobalForKF(defpair),mBALocalForKF(defpair),
      mFuseCandidateForKF(defpair),
      mSysState(SysState),mbDoNotReplace(false),mbOmitSending(false),
      mbLoopCorrected(false),mbMultiUse(false),
      mbBad(false),mbAck(true),mbUpdatedByServer(false),mInsertedWithKF(-1),mMaxObsKFId(0),mbSendFull(false)
{
    if(pMsg->mbBad)
    {
         cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << "Incoming KF message: mbBad == true" << endl;
         throw infrastructure_ex();
    }

    mbOmitSending = true;

    if(mSysState == eSystemState::SERVER)
    {
        mUniqueId = UniqueId;
        mbFromServer = false;
    }
    else if(mSysState == eSystemState::CLIENT)
    {
        mUniqueId = pMsg->mUniqueId;
        mbFromServer = true;
    }
    else
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << " System Type Not Set" << endl;
        throw infrastructure_ex();
    }

    mspComm.insert(pComm);

    this->WriteMembersFromMessage(pMsg,g2oS_wm_wc);

    mbOmitSending = false;

    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // Call EstablishInitialConnectionsServer()
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
}

void MapPoint::EstablishInitialConnectionsServer()
{
    //this is necessary, because we cannot use shared_from_this() in constructor

    #ifdef LOGGING
    ccptr pCC;
    if(this->mSysState == SERVER)
    {
        pCC = mpMap->GetCCPtr(this->mId.second);
        pCC->mpLogger->SetMP(__LINE__,this->mId.second);
    }
    #endif

    {
        #ifdef TRACELOCK
        pCC->mpLogger->SetMP(__LINE__,this->mId.second);
        #endif

        unique_lock<mutex> lock1(mMutexFeatures);

        #ifdef TRACELOCK
        pCC->mpLogger->SetMP(__LINE__,this->mId.second);
        #endif

        for(map<kfptr,size_t>::iterator mit = mObservations.begin();mit != mObservations.end();)
        {
            kfptr pKFi = mit->first;
            size_t idx = mit->second;

            if(pKFi)
            {
                #ifdef TRACELOCK
                pCC->mpLogger->SetMP(__LINE__,this->mId.second);
                #endif

                mpptr pMP = pKFi->GetMapPoint(idx);

                #ifdef TRACELOCK
                pCC->mpLogger->SetMP(__LINE__,this->mId.second);
                #endif

                if(pMP)
                {
                    //there is already an associated MP to this feature

                    #ifdef TRACELOCK
                    pCC->mpLogger->SetMP(__LINE__,this->mId.second);
                    #endif

                    if(shared_from_this()->mId == pMP->mId)
                    {
                        //But it's this MP, and we're calling the constructor -- thats inconsistent
                        cout << "\033[1;31m!!!!! FATAL ERROR !!!!!\033[0m " << __func__ << __LINE__ << "Constructing MP & is already associated to KF" << endl;
                        cout << "KF-ID: " << pKFi->mId.first << "|" << pKFi->mId.second << " --- this MP-ID: " << this->mId.first << "|" << this->mId.second << " --- comp. MP-ID: " << pMP->mId.first << "|" << pMP->mId.second << endl;
                        throw infrastructure_ex();
                    }
                    else if(!pKFi->IsMpLocked(idx))
                    {
                        pKFi->ReplaceMapPointMatch(idx,shared_from_this(),false);
                        pMP->EraseObservation(pKFi);
                    }
                    else
                    {
                        //its locked - cannot replace
                        //delete KF association
                        mit = mObservations.erase(mit);
                        nObs--;
                        continue;
                    }

                    #ifdef TRACELOCK
                    pCC->mpLogger->SetMP(__LINE__,this->mId.second);
                    #endif
                }
                else
                {
                    //nothing associated to this feature, we can add
                    pKFi->AddMapPoint(shared_from_this(),idx,false);
                }
            }
            else
            {
                cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << " pKFi is NULLPTR" << endl;
            }

            ++mit;
        }

        #ifdef TRACELOCK
        pCC->mpLogger->SetMP(__LINE__,this->mId.second);
        #endif

        if(nObs == 0)
        {
            mbBad = true;
            #ifdef LOGGING
            pCC->mpLogger->SetMP(__LINE__,this->mId.second);
            #endif
            return;
        }

        mpRefKF = mObservations.begin()->first; //should not be bad, we checked that when we added the KF

        if(mpRefKF->isBad())
        {
            cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << ": mpRefKF is BAD" << endl;
        }
    }

    #ifdef TRACELOCK
    pCC->mpLogger->SetMP(__LINE__,this->mId.second);
    #endif

    this->UpdateNormalAndDepth();

    #ifdef LOGGING
    pCC->mpLogger->SetMP(__LINE__,this->mId.second);
    #endif
}

void MapPoint::EstablishInitialConnectionsClient()
{
    {
        unique_lock<mutex> lock1(mMutexFeatures);

        for(map<kfptr,size_t>::iterator mit = mObservations.begin();mit != mObservations.end();)
        {
            kfptr pKFi = mit->first;
            size_t idx = mit->second;

            if(pKFi)
            {
                mpptr pMP = pKFi->GetMapPoint(idx);
                if(pMP)
                {
                    //there is already an associated MP to this feature -- replace or discard? ToDo: find best solution

                    if(shared_from_this()->mId == pMP->mId)
                    {
                        //But it's this MP, and we're calling the constructor -- thats inconsistent
                        cout << "\033[1;31m!!!!! FATAL ERROR !!!!!\033[0m " << __func__ << __LINE__ << "Constructing MP & is already associated to KF" << endl;
                        cout << "KF-ID: " << pKFi->mId.first << "|" << pKFi->mId.second << " --- this MP-ID: " << this->mId.first << "|" << this->mId.second << " --- comp. MP-ID: " << pMP->mId.first << "|" << pMP->mId.second << endl;
                        cout << "(pMP = shared_from_this()): " << (int)(pMP == shared_from_this()) << std::endl;

                        if(pMP = shared_from_this())
                        {
                            ++mit;
                            continue;
                        }
                        else
                            mbBad = true;
                            return;
                    }

                        //delete KF association
                        mit = mObservations.erase(mit);
                        nObs--;

                        continue;
                }
                else
                {
                    //nothing assocaited to this feature, we can add
                    pKFi->AddMapPoint(shared_from_this(),idx,false);
                }
            }
            else
            {
                cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << " pKFi is NULLPTR" << endl;
            }

            ++mit;
        }

        if(nObs == 0)
        {
            mbBad = true; //we need to do that, since w/o Obs, no ref ptr can exist
            return;
        }

        mpRefKF = mObservations.begin()->first; //should not be bad, we checked taht when we added the KF

        if(mpRefKF->isBad())
        {
            cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << ": mpRefKF is BAD" << endl;
        }
    }

    this->UpdateNormalAndDepth();

    mMaxObsKFId = 0;
    for(std::map<kfptr,size_t>::iterator mit = mObservations.begin();mit!=mObservations.end();++mit)
    {
        kfptr pKF = mit->first;
        if(pKF->mId.first > mMaxObsKFId && pKF->mId.second == mpMap->mMapId)
            mMaxObsKFId = pKF->mId.first;
    }
}

void MapPoint::SetWorldPos(const cv::Mat &Pos, bool bLock, bool bIgnorePosMutex)
{
    if(mbPoseLock && mSysState == eSystemState::CLIENT)
        return;

    {
        unique_lock<mutex> lock2(mGlobalMutex);
        if(!bIgnorePosMutex)
            unique_lock<mutex> lock(mMutexPos);

        Pos.copyTo(mWorldPos);

        if(bLock)
        {
            mbPoseLock = true;
        }
    }

    //unique_lock<mutex> lockMap(mMutexOut); //no need for mMutexOut, since ConvertToMsg needs Pos mutex
    if(!mbOmitSending && this->IsSent())
    {
        mbPoseChanged = true;
        if(mSysState == CLIENT)
            SendMe();
    }
}

void MapPoint::SetWorldPosFromRef(const Mat &Pos, bool bLock, kfptr pRefKf)
{
//    unique_lock<mutex> lock2(mGlobalMutex); //would cause deadlock when called from "UpdateFromMessage"
//    unique_lock<mutex> lock(mMutexPos);  //would cause deadlock when called from "UpdateFromMessage"

    Pos.copyTo(mRefPos);

    if(pRefKf)
    {
        cv::Mat T_w_cref =  pRefKf->GetPoseInverse();

        cv::Mat Rwc = T_w_cref.rowRange(0,3).colRange(0,3);
        cv::Mat twc = T_w_cref.rowRange(0,3).col(3);

        mWorldPos = Rwc * mRefPos + twc;
    }
    else
    {
        cout << "\033[1;31m!!!!! FATAL ERROR !!!!!\033[0m " << __func__ << __LINE__ << ": MP has no parent" << endl;
        throw infrastructure_ex();
    }

    if(bLock)
    {
        mbPoseLock = true;
    }
}

cv::Mat MapPoint::GetWorldPos()
{
    unique_lock<mutex> lock(mMutexPos);
    return mWorldPos.clone();
}

cv::Mat MapPoint::GetNormal()
{
    unique_lock<mutex> lock(mMutexPos);
    return mNormalVector.clone();
}

MapPoint::kfptr MapPoint::GetReferenceKeyFrame()
{
     unique_lock<mutex> lock(mMutexFeatures);
     return mpRefKF;
}

void MapPoint::SetReferenceKeyFrame(kfptr pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mpRefKF = pKF;
}

void MapPoint::AddObservation(kfptr pKF, size_t idx, bool bLock)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return;

    if(mObservationsLock.count(pKF) && mSysState == eSystemState::CLIENT)
        return;

    mObservations[pKF]=idx;

    nObs++;

    if(mSysState == eSystemState::CLIENT)
    {
        if(pKF->mId.first > mMaxObsKFId && pKF->mId.second == mpMap->mMapId)
            mMaxObsKFId = pKF->mId.first;
    }

    if(bLock)
    {
        mObservationsLock[pKF] = true;
    }
}

void MapPoint::EraseObservation(kfptr pKF, bool bLock, bool bSuppressMapAction)
{
    if(mObservationsLock.count(pKF) && mSysState == eSystemState::CLIENT)
        return;

    bool bBad=false;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF))
        {
            nObs--;

            mObservations.erase(pKF);

            if(mSysState == eSystemState::CLIENT)
            {
                if(pKF->mId.first == mMaxObsKFId && pKF->mId.second == mpMap->mMapId)
                {
                    mMaxObsKFId = 0;
                    for(std::map<kfptr,size_t>::iterator mit = mObservations.begin();mit!=mObservations.end();++mit)
                    {
                        if(pKF->mId.first > mMaxObsKFId && pKF->mId.second == mpMap->mMapId)
                            mMaxObsKFId = pKF->mId.first;
                    }
                }
            }

            if(bLock)
            {
                mObservationsLock[pKF] = true;
            }

            if(mpRefKF==pKF)
            {
                if(nObs > 0)
                {
                    mpRefKF = nullptr;
                    std::map<kfptr,size_t>::const_iterator mitRef = mObservations.begin();
                    while(!mpRefKF)
                    {
                        if(mitRef == mObservations.end()) break;
                        if(!(mitRef->first->isBad()))
                            mpRefKF=mitRef->first;
                        else
                            ++mitRef;
                    }
                }
                else
                    mpRefKF=nullptr;
            }

            // If only 2 observations or less, discard point
            if(nObs<=2)
                bBad=true;
        }
    }

    if(bBad)
        SetBadFlag(bSuppressMapAction);

    if(!mpRefKF)
    {
        if(!mbBad)
        {
            SetBadFlag(bSuppressMapAction);
        }
    }
}

map<MapPoint::kfptr, size_t> MapPoint::GetObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
}

int MapPoint::Observations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}

void MapPoint::SetBadFlag(bool bSuppressMapAction)
{
    #ifdef LOGGING
    ccptr pCC;
    if(this->mSysState == SERVER)
    {
        pCC = mpMap->GetCCPtr(this->mId.second);
        pCC->mpLogger->SetMP(__LINE__,this->mId.second);
    }
    #endif

    {
        if(mbBad)
        {
            #ifdef LOGGING
            if(this->mSysState == SERVER)
                pCC->mpLogger->SetMP(__LINE__,this->mId.second);
            #endif
            return;
        }
    }

    map<kfptr,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad=true;
        obs = mObservations;
        mObservations.clear();
    }

    for(map<kfptr,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        kfptr pKF = mit->first;
        pKF->EraseMapPointMatch(mit->second);
    }

    if(!this->IsSent())
    {
        this->EraseInOutBuffer();
    }

    if(!bSuppressMapAction)
        mpMap->EraseMapPoint(this->shared_from_this());
    else
        mpMap->mspMPsToErase.insert(this->shared_from_this());

    #ifdef LOGGING
    if(this->mSysState == SERVER)
        pCC->mpLogger->SetMP(__LINE__,this->mId.second);
    #endif
}

MapPoint::mpptr MapPoint::GetReplaced()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mpReplaced;
}

void MapPoint::Replace(mpptr pMP, bool bLock)
{
    if(pMP->mId==this->mId)
        return;

    int nvisible, nfound;
    map<kfptr,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        obs=mObservations;
        mObservations.clear();
        mbBad=true;
        nvisible = mnVisible;
        nfound = mnFound;
        mpReplaced = pMP;
    }

    for(map<kfptr,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        kfptr pKF = mit->first;

        if(!pMP->IsInKeyFrame(pKF))
        {
            pKF->ReplaceMapPointMatch(mit->second, pMP,bLock);
            pMP->AddObservation(pKF,mit->second,bLock);
        }
        else
        {
            //MP already has connection to this KF
            if(mit->second == pMP->GetIndexInKeyFrame(pKF))
            {
                //ids match -- replace old mappoint with new one
                pKF->EraseMapPointMatch(mit->second,bLock);
                pKF->AddMapPoint(pMP,mit->second,bLock);
            }
            else if(pMP->GetIndexInKeyFrame(pKF) >= 0)
            {
                //feat-ids of this MP and replacement MP do not match

                //MP knows feat-id in KF, but KF does not know MP -- fix by taking the id with the better descriptor distance
                pKF->EraseMapPointMatch(mit->second,bLock);

                vector<mpptr> mvpMPs = pKF->GetMapPointMatches();
                vector<mpptr>::iterator vit = std::find(mvpMPs.begin(),mvpMPs.end(),pMP);
                int id = vit - mvpMPs.begin();

                if(id == pMP->GetIndexInKeyFrame(pKF))
                {
                    //pMP thinks it's associated to feat-id x, KF thinks pMP is asscociated to feat-id x -- so just erase connection to THIS (already done)
                }
                else if(vit == mvpMPs.end())
                {
                    //MP knows KF, but KF does not know MP
                    //remap to new id or keep old one?

                    const cv::Mat dMP = pMP->GetDescriptor();

                    const cv::Mat &dKF_pMP_id = pKF->mDescriptors.row(pMP->GetIndexInKeyFrame(pKF));
                    const cv::Mat &dKF_this_id = pKF->mDescriptors.row(mit->second);

                    double dist_pMP_id = ORBmatcher::DescriptorDistance(dMP,dKF_pMP_id);
                    double dist_this_id = ORBmatcher::DescriptorDistance(dMP,dKF_this_id);

                    if(dist_pMP_id <= dist_this_id)
                    {
                        pKF->AddMapPoint(pMP,pMP->GetIndexInKeyFrame(pKF),false);
                        //MP already has correct feat-id
                    }
                    else
                    {
                        pKF->AddMapPoint(pMP,mit->second,bLock);
                        pMP->EraseObservation(pKF,bLock);
                        pMP->AddObservation(pKF,mit->second,bLock);
                    }
                }
                else
                {
                    //MP thinks it's associated to feat-id x, KF thinks MP is asscociated to feat-id y
                    cout << "\033[1;33m!!! WARN !!!\033[0m " << __func__ << ":" << __LINE__ << " id mismatch MP " << pMP->mId.first << endl;
                }
            }
            else
            {
                cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << ":" << __LINE__ << " id mismatch" << endl;
                throw infrastructure_ex();
            }
        }
    }
    pMP->IncreaseFound(nfound);
    pMP->IncreaseVisible(nvisible);
    pMP->ComputeDistinctiveDescriptors();

    mpMap->EraseMapPoint(this->shared_from_this());
}

void MapPoint::ReplaceAndLock(mpptr pMP)
{
    if(pMP->mId==this->mId)
        return;

    int nvisible, nfound;
    map<kfptr,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        obs=mObservations;
        mObservations.clear();
        mbBad=true;
        nvisible = mnVisible;
        nfound = mnFound;
        mpReplaced = pMP;
    }

    for(map<kfptr,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        kfptr pKF = mit->first;

        if(pKF->IsEmpty()) continue;

        if(!pMP->IsInKeyFrame(pKF))
        {
            pKF->ReplaceMapPointMatch(mit->second, pMP,true);
            pMP->AddObservation(pKF,mit->second,true);
        }
        else
        {
            pKF->EraseMapPointMatch(mit->second,true);
        }
    }
    pMP->IncreaseFound(nfound);
    pMP->IncreaseVisible(nvisible);
    pMP->ComputeDistinctiveDescriptors();

    mpMap->EraseMapPoint(this->shared_from_this());
}

void MapPoint::IncreaseVisible(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnVisible+=n;
}

void MapPoint::IncreaseFound(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnFound+=n;
}

float MapPoint::GetFoundRatio()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return static_cast<float>(mnFound)/mnVisible;
}

cv::Mat MapPoint::GetDescriptor()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mDescriptor.clone();
}

int MapPoint::GetIndexInKeyFrame(kfptr pKF, bool bIgnoreMutex)
{
    if(!bIgnoreMutex)
        unique_lock<mutex> lock(mMutexFeatures);

    if(mObservations.count(pKF))
        return mObservations[pKF];
    else
        return -1;
}

bool MapPoint::IsInKeyFrame(kfptr pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return (mObservations.count(pKF));
}

bool MapPoint::IsObsLocked(kfptr pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);

    map<kfptr,bool>::iterator mit = mObservationsLock.find(pKF);

    if(mit == mObservationsLock.end())
    {
        return false;
    }
    else
    {
        return mit->second;
    }
}

void MapPoint::UpdateNormalAndDepth()
{
    map<kfptr,size_t> observations;
    kfptr pRefKF;
    cv::Mat Pos;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        if(mbBad)
            return;
        observations=mObservations;
        pRefKF=mpRefKF;
        Pos = mWorldPos.clone();
    }

    if(observations.empty())
        return;

    cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
    int n=0;
    for(map<kfptr,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        kfptr pKF = mit->first;

        if(pKF->isBad()) continue;

        cv::Mat Owi = pKF->GetCameraCenter();
        cv::Mat normali = mWorldPos - Owi;
        normal = normal + normali/cv::norm(normali);
        n++;
    }

    cv::Mat PC = Pos - pRefKF->GetCameraCenter();
    const float dist = cv::norm(PC);
    const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
    const float levelScaleFactor =  pRefKF->mvScaleFactors[level];
    const int nLevels = pRefKF->mnScaleLevels;

    {
        unique_lock<mutex> lock3(mMutexPos);
        mfMaxDistance = dist*levelScaleFactor;
        mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1];
        mNormalVector = normal/n;
    }
}

float MapPoint::GetMinDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 0.8f*mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 1.2f*mfMaxDistance;
}

int MapPoint::PredictScale(const float &currentDist, kfptr pKF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pKF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pKF->mnScaleLevels)
        nScale = pKF->mnScaleLevels-1;

    return nScale;
}

int MapPoint::PredictScale(const float &currentDist, frameptr pF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pF->mnScaleLevels)
        nScale = pF->mnScaleLevels-1;

    return nScale;
}

void MapPoint::SaveData() const {

    mmObservations_minimal.clear();

    for(auto p : mObservations) {
        if(p.first != nullptr){
            mmObservations_minimal.insert(std::make_pair(p.first->mId,p.second));
        }
    }

    if(mpRefKF)
        mRefKfId = mpRefKF->mId;
    else
        std::cout << COUTWARN << "MP " << mId.first << "|" << mId.second << ": no Ref KF" << std::endl;
}

void MapPoint::SendMe()
{
    if(mspComm.empty())
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m MapPoint::SendMe(): no Comm ptrs" << endl;
        cout << "bad?: " << (mbBad == true) << endl;
        return;
    }

    if(this->IsSent() && !this->IsInOutBuffer())
    {
        for(set<commptr>::const_iterator sit = mspComm.begin();sit!=mspComm.end();++sit)
        {
            commptr pComm = *sit;
            pComm->PassMptoComm(this->shared_from_this());
        }
    }
}

void MapPoint::SetSendFull()
{
    {
        unique_lock<mutex> lock(mMutexOut);
        mbSendFull = true;
    }

    this->SendMe();
}

void MapPoint::ReplaceMap(mapptr pNewMap)
{
//    unique_lock<mutex> lockMap(mMapMutex);

    unique_lock<mutex> lockFeat(mMutexFeatures,defer_lock);
    unique_lock<mutex> lockPos(mMutexPos,defer_lock);
    unique_lock<mutex> lockOut(mMutexOut,defer_lock);

    lock(lockFeat,lockPos,lockOut);

    mpMap = pNewMap;
}

void MapPoint::ComputeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    map<kfptr,size_t> observations;

    {
        unique_lock<mutex> lock1(mMutexFeatures);
        if(mbBad)
            return;
        observations=mObservations;
    }

    if(observations.empty())
        return;

    vDescriptors.reserve(observations.size());

    for(map<kfptr,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        kfptr pKF = mit->first;

        if(!pKF->isBad())
            vDescriptors.push_back(pKF->mDescriptors.row(mit->second));
    }

    if(vDescriptors.empty())
        return;

    // Compute distances between them
    const size_t N = vDescriptors.size();

    float Distances[N][N];
    for(size_t i=0;i<N;i++)
    {
        Distances[i][i]=0;
        for(size_t j=i+1;j<N;j++)
        {
            int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
            Distances[i][j]=distij;
            Distances[j][i]=distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i=0;i<N;i++)
    {
        vector<int> vDists(Distances[i],Distances[i]+N);
        sort(vDists.begin(),vDists.end());
        int median = vDists[0.5*(N-1)];

        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        unique_lock<mutex> lock(mMutexFeatures);
        mDescriptor = vDescriptors[BestIdx].clone();
    }
}

void MapPoint::ReduceMessage(ccmslam_msgs::MP *pMsgFull, ccmslam_msgs::MPred *pMsgRed)
{
    pMsgRed->mnId = pMsgFull->mnId;
    pMsgRed->mClientId = pMsgFull->mClientId;
    pMsgRed->mUniqueId = pMsgFull->mUniqueId;
    pMsgRed->mPosPred = pMsgFull->mPosPred;
    pMsgRed->mPosPar = pMsgFull->mPosPar;
    pMsgRed->mbNormalAndDepthChanged = pMsgFull->mbNormalAndDepthChanged;
    pMsgRed->mpPredKFId = pMsgFull->mpPredKFId;
    pMsgRed->mpPredKFClientId = pMsgFull->mpPredKFClientId;
    pMsgRed->mpParKFId = pMsgFull->mpParKFId;
    pMsgRed->mpParKFClientId = pMsgFull->mpParKFClientId;
    pMsgRed->mbBad = pMsgFull->mbBad;
    pMsgRed->mbMultiUse = pMsgFull->mbMultiUse;
}

void MapPoint::ConvertToMessage(ccmslam_msgs::Map &msgMap, kfptr pRefKf, g2o::Sim3 mg2oS_wcurmap_wclientmap, bool bForceUpdateMsg)
{
    unique_lock<mutex> lockOut(mMutexOut);

    if((mbSendFull || mSysState == eSystemState::SERVER) && !bForceUpdateMsg)
    {
        ccmslam_msgs::MP Msg;

        unique_lock<mutex> lockFeat(mMutexFeatures,defer_lock);
        unique_lock<mutex> lockPos(mMutexPos,defer_lock);

        lock(lockFeat,lockPos);

        Msg.mpPredKFId = static_cast<uint16_t>(pRefKf->mId.first);
        Msg.mpPredKFClientId = static_cast<uint8_t>(pRefKf->mId.second);

        if(mSysState == eSystemState::SERVER)
        {
            cv::Mat Tpw = pRefKf->GetPose();
            cv::Mat Rpw = Tpw.rowRange(0,3).colRange(0,3);
            cv::Mat tpw = Tpw.rowRange(0,3).col(3);

            cv::Mat P3D_p = Rpw * mWorldPos + tpw;

            float s = static_cast<double>(mg2oS_wcurmap_wclientmap.inverse().scale());
            s = 1/s;

            P3D_p *=(1./s);

            Converter::CvMatToMsgArrayFixedSize<ccmslam_msgs::MP::_mPosPred_type,float>(P3D_p,Msg.mPosPred);

            Msg.mpParKFId = KFRANGE;
            Msg.mpParKFClientId = MAPRANGE;

            Msg.bSentOnce = mbSentOnce;

            ccptr pCC = mpMap->GetCCPtr(mId.second);
            if(pCC->mbOptimized)
                Msg.mbPoseChanged = true;
            else
                Msg.mbPoseChanged = mbPoseChanged;

            Msg.mbServerBA = false;
        }
        else
        {
            cv::Mat Rcw = pRefKf->GetPose().rowRange(0,3).colRange(0,3);
            cv::Mat tcw = pRefKf->GetPose().rowRange(0,3).col(3);

            cv::Mat RefPos = Rcw * mWorldPos + tcw;

            Converter::CvMatToMsgArrayFixedSize<ccmslam_msgs::MP::_mPosPred_type,float>(RefPos,Msg.mPosPred);

            if(mpRefKF)
            {
                Msg.mpParKFId = static_cast<uint16_t>(mpRefKF->mId.first);
                Msg.mpParKFClientId = static_cast<uint8_t>(mpRefKF->mId.second);

                cv::Mat Rparcw = mpRefKF->GetPose().rowRange(0,3).colRange(0,3);
                cv::Mat tparcw = mpRefKF->GetPose().rowRange(0,3).col(3);

                cv::Mat ParPos = Rparcw * mWorldPos + tparcw;

                Converter::CvMatToMsgArrayFixedSize<ccmslam_msgs::MP::_mPosPar_type,float>(ParPos,Msg.mPosPar);
            }
            else
            {
                Msg.mpParKFId = KFRANGE;
                Msg.mpParKFClientId = MAPRANGE;
                cout << "\033[1;33m!!! WARN !!!\033[0m " << __func__ << ":" << __LINE__ << " no Parent" << endl;
            }

            Msg.mbAck = mbAck;

            Msg.bSentOnce = mbSentOnce;
            mbSentOnce = true;
            mbPoseChanged = false;
            mbSendFull = false;

            Msg.mbServerBA = false;
        }

        //Normal Vector

        Converter::CvMatToMsgArrayFixedSize<ccmslam_msgs::MP::_mNormalVector_type,float>(mNormalVector,Msg.mNormalVector);

        //Descriptor

        Converter::CvMatToMsgArrayFixedSize<ccmslam_msgs::MP::_mDescriptor_type,uint8_t>(mDescriptor,Msg.mDescriptor);

        //Observation

        for(std::map<kfptr,size_t>::const_iterator mit=mObservations.begin();mit!=mObservations.end();++mit)
        {
            if(!mit->first->isBad())
            {
                Msg.mObservations_KFIDs.push_back(static_cast<uint16_t>(mit->first->mId.first));
                Msg.mObservations_KFClientIDs.push_back(static_cast<uint8_t>(mit->first->mId.second));
                Msg.mObservations_n.push_back(static_cast<uint16_t>(mit->second));
            }
        }

        Msg.mfMinDistance = mfMinDistance;
        Msg.mfMaxDistance = mfMaxDistance;

        Msg.mbBad = mbBad;
        Msg.mnId = static_cast<uint32_t>(mId.first);
        Msg.mClientId = static_cast<uint8_t>(mId.second);
        Msg.mUniqueId = mUniqueId;

        Msg.mnFirstKFid = static_cast<uint16_t>(mFirstKfId.first);
        Msg.mnFirstKfClientId = static_cast<uint8_t>(mFirstKfId.second);

        Msg.mbMultiUse = mbMultiUse;

        msgMap.MapPoints.push_back(Msg);
    }
    else
    {
        if(mSysState == eSystemState::SERVER)
        {
            cout << "\033[1;31m!!!!! FATAL !!!!!\033[0m " << __func__ << ":"  << __LINE__ << " must no be used by server" << endl;
            throw infrastructure_ex();
        }

        ccmslam_msgs::MPred Msg;

        unique_lock<mutex> lockPos(mMutexPos);

        if(mbPoseChanged || this->mbMultiUse) // this part should only be called on client. System will send MP without pose change to server to inform it about a "MultiUse-Event"
        {
            Msg.mpPredKFId = static_cast<uint16_t>(pRefKf->mId.first);
            Msg.mpPredKFClientId = static_cast<uint8_t>(pRefKf->mId.second);

            cv::Mat Rcw = pRefKf->GetPose().rowRange(0,3).colRange(0,3);
            cv::Mat tcw = pRefKf->GetPose().rowRange(0,3).col(3);

            cv::Mat RefPos = Rcw * mWorldPos + tcw;

            Converter::CvMatToMsgArrayFixedSize<ccmslam_msgs::MP::_mPosPred_type,float>(RefPos,Msg.mPosPred);

            if(mpRefKF)
            {
                Msg.mpParKFId = static_cast<uint16_t>(mpRefKF->mId.first);
                Msg.mpParKFClientId = static_cast<uint8_t>(mpRefKF->mId.second);

                cv::Mat Rparcw = mpRefKF->GetPose().rowRange(0,3).colRange(0,3);
                cv::Mat tparcw = mpRefKF->GetPose().rowRange(0,3).col(3);

                cv::Mat ParPos = Rparcw * mWorldPos + tparcw;

                Converter::CvMatToMsgArrayFixedSize<ccmslam_msgs::MP::_mPosPar_type,float>(ParPos,Msg.mPosPar);
            }
            else
            {
                Msg.mpParKFId = KFRANGE;
                Msg.mpParKFClientId = MAPRANGE;
                cout << "\033[1;33m!!! WARN !!!\033[0m " << __func__ << ":" << __LINE__ << " no Parent" << endl;
            }

            Msg.mbServerBA = false;

            mbPoseChanged = false;

            Msg.mbBad = mbBad;
            Msg.mnId = static_cast<uint32_t>(mId.first);
            Msg.mClientId = static_cast<uint8_t>(mId.second);
            Msg.mUniqueId = mUniqueId;
            Msg.mbMultiUse = mbMultiUse;
            Msg.mbAck = mbAck;

            msgMap.MPUpdates.push_back(Msg);
        }
    }
}

void MapPoint::UpdateFromMessage(ccmslam_msgs::MP *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap)
{
    //if MP already exists, we reduce it to a Update-Msg to use common interfaces

    ccmslam_msgs::MPred *pMsgRed = new ccmslam_msgs::MPred();

    this->ReduceMessage(pMsg,pMsgRed);

    this->UpdateFromMessage(pMsgRed,mg2oS_wcurmap_wclientmap);

    delete pMsgRed;
}

void MapPoint::UpdateFromMessage(ccmslam_msgs::MPred *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap)
{
    mbOmitSending = true;

    unique_lock<mutex> lockOut(mMutexOut);
    unique_lock<mutex> lock(mMutexPos);

    if(mSysState == eSystemState::CLIENT)
    {
        bool bSetPos = this->SetPoseFromMessage(pMsg,mg2oS_wcurmap_wclientmap);

        if(!bSetPos)
        {
            mbOmitSending = false;
            return;
        }

        mbPoseChanged = false;
        mbUpdatedByServer = true;
    }
    else if(mSysState == eSystemState::SERVER)
    {
        if(!mbPoseLock)
        {
            bool bSetPos = this->SetPoseFromMessage(pMsg,mg2oS_wcurmap_wclientmap);

            if(!bSetPos)
            {
                mbOmitSending = false;
                return;
            }
        }
    }

    if(!mbMultiUse)
        mbMultiUse = pMsg->mbMultiUse;

    mbOmitSending = false;
}

void MapPoint::WriteMembersFromMessage(ccmslam_msgs::MP *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap)
{
    mbSentOnce=true;

    mId = make_pair(pMsg->mnId,pMsg->mClientId);
    mFirstKfId = make_pair(pMsg->mnFirstKFid,pMsg->mnFirstKfClientId);

    for(int idx=0;idx<pMsg->mObservations_KFIDs.size();++idx)
    {
        kfptr pKFi = mpMap->GetKfPtr(pMsg->mObservations_KFIDs[idx],pMsg->mObservations_KFClientIDs[idx]);

        if(pKFi && !pKFi->isBad())
        {
            mObservations[pKFi]=pMsg->mObservations_n[idx];
            ++nObs;
        }
    }

    if(nObs == 0)
    {
        mbBad = true;
        return;
    }

    mNormalVector = cv::Mat(3,1,5);
    Converter::MsgArrayFixedSizeToCvMat<ccmslam_msgs::MP::_mNormalVector_type,float>(mNormalVector,pMsg->mNormalVector);
    mDescriptor = cv::Mat(1,32,0);
    Converter::MsgArrayFixedSizeToCvMat<ccmslam_msgs::MP::_mDescriptor_type,uint8_t>(mDescriptor,pMsg->mDescriptor);

    mnFound = nObs;
    mnVisible = nObs;

    mbMultiUse = pMsg->mbMultiUse;

    mWorldPos = cv::Mat(3,1,5);
    mRefPos = cv::Mat(3,1,5);

    if(mSysState == eSystemState::CLIENT)
    {
        bool bSetPos = this->SetPoseFromMessage(pMsg,mg2oS_wcurmap_wclientmap);

        if(!bSetPos)
        {
            mObservations.clear();
            mbBad = true;
            return;
        }
    }
    else if(mSysState == eSystemState::SERVER)
    {
        bool bSetPos = this->SetPoseFromMessage(pMsg,mg2oS_wcurmap_wclientmap);

        if(!bSetPos)
        {
            mObservations.clear();
            mbBad = true;
            return;
        }
    }
}

bool MapPoint::SetPoseFromMessage(ccmslam_msgs::MP *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap)
{
    ccmslam_msgs::MPred *pMsgRed = new ccmslam_msgs::MPred();

    this->ReduceMessage(pMsg,pMsgRed);

    bool bReturn = this->SetPoseFromMessage(pMsgRed,mg2oS_wcurmap_wclientmap);

    delete pMsgRed;

    return bReturn;
}

bool MapPoint::SetPoseFromMessage(ccmslam_msgs::MPred *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap)
{
    if(mSysState == eSystemState::CLIENT)
    {
        idpair MsgPredId = make_pair(pMsg->mpPredKFId,pMsg->mpPredKFClientId);
        kfptr pPredKf = mpMap->GetKfPtr(MsgPredId); //message sent pose relative to this KF

        if(pPredKf && !pPredKf->isBad())
        {
            cv::Mat Twp = pPredKf->GetPoseInverse();
            cv::Mat Rwp = Twp.rowRange(0,3).colRange(0,3);
            cv::Mat twp = Twp.rowRange(0,3).col(3);

            cv::Mat P3D_p = cv::Mat(3,1,5); //relative to reference
            Converter::MsgArrayFixedSizeToCvMat<ccmslam_msgs::MP::_mPosPred_type,float>(P3D_p,pMsg->mPosPred);

            cv::Mat P3D_w = Rwp * P3D_p + twp;

            this->SetWorldPos(P3D_w,false,true);
        }
        else
        {
            //so there is really no pointer available -- we have to ignore this message
            return false;
        }
    }
    else if(mSysState == eSystemState::SERVER)
    {
        cv::Mat P3D_ref = cv::Mat(3,1,5); //in world client

        kfptr pRef = mpMap->GetKfPtr(pMsg->mpPredKFId,pMsg->mpPredKFClientId);

        if(pRef)
        {
            Converter::MsgArrayFixedSizeToCvMat<ccmslam_msgs::MP::_mPosPred_type,float>(P3D_ref,pMsg->mPosPred);
        }

        if(!pRef)
        {
            pRef = mpMap->GetKfPtr(pMsg->mpParKFId,pMsg->mpParKFClientId);

            if(pRef)
            {
                Converter::MsgArrayFixedSizeToCvMat<ccmslam_msgs::MP::_mPosPar_type,float>(P3D_ref,pMsg->mPosPar);
            }
        }

        if(!pRef)
        {
            pRef = mpMap->GetErasedKfPtr(pMsg->mpPredKFId,pMsg->mpPredKFClientId);

            if(pRef)
            {
                Converter::MsgArrayFixedSizeToCvMat<ccmslam_msgs::MP::_mPosPred_type,float>(P3D_ref,pMsg->mPosPred);
            }
        }

        if(!pRef)
        {
            pRef = mpMap->GetErasedKfPtr(pMsg->mpParKFId,pMsg->mpParKFClientId);

            if(pRef)
            {
                Converter::MsgArrayFixedSizeToCvMat<ccmslam_msgs::MP::_mPosPar_type,float>(P3D_ref,pMsg->mPosPar);
            }
        }

        if(!pRef)
        {
            //so there is really no pointer available -- we have to ignore this message
            return false;
        }

        float s = static_cast<double>(mg2oS_wcurmap_wclientmap.scale());
        s = 1/s;

        P3D_ref *=(1./s);

        if(!pRef->isBad())
        {
            cv::Mat Twp =  pRef->GetPoseInverse();

            cv::Mat Rwp = Twp.rowRange(0,3).colRange(0,3);
            cv::Mat twp = Twp.rowRange(0,3).col(3);

            cv::Mat P3D_w = Rwp * P3D_ref + twp;
            P3D_w.copyTo(mWorldPos);
        }
        else
        {
            kfptr pRefRef = pRef->GetParent();

            if(!pRefRef)
            {
                cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << ":"  << __LINE__ << " parent is bad or not existing" << endl;
                cout << "this: " << this->mId.first << "|" << this->mId.second << endl;
                cout << "pPred: " << pRef->mId.first << "|" << pRef->mId.second << endl;
                cout << "!pPredPred" << endl;
                throw infrastructure_ex();
            }

            cv::Mat T_p_pp = pRef->mTcp;

            while(pRefRef->isBad())
            {
                T_p_pp = T_p_pp * pRefRef->mTcp;

                pRefRef = pRefRef->GetParent();

                if(!pRefRef)
                {
                    cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << ":"  << __LINE__ << " parent is bad or not existing" << endl;
                    cout << "this: " << this->mId.first << "|" << this->mId.second << endl;
                    cout << "pPred: " << pRef->mId.first << "|" << pRef->mId.second << endl;
                    cout << "!pPredPred" << endl;
                    throw infrastructure_ex();
                }
            }

            cv::Mat T_pp_w = pRefRef->GetPose();
            cv::Mat Tpw = T_p_pp * T_pp_w;
            cv::Mat Twp = Tpw.inv();

            cv::Mat Rwp = Twp.rowRange(0,3).colRange(0,3);
            cv::Mat twp = Twp.rowRange(0,3).col(3);

            cv::Mat P3D_w = Rwp * P3D_ref + twp;
            P3D_w.copyTo(mWorldPos);
        }
    }

    return true;
}

void MapPoint::EraseInOutBuffer()
{
//    unique_lock<mutex> lockMap(mMutexOut); //is only called by "MapPoint::SetBadFlag()", which already locks this mutex

    if(mspComm.empty())
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m MapPoint::EraseInOutBuffer(...): no Comm ptrs" << endl;
        throw infrastructure_ex();
    }

    if(this->IsInOutBuffer())
    {
        for(set<commptr>::const_iterator sit = mspComm.begin();sit!=mspComm.end();++sit)
        {
            commptr pComm = *sit;
            pComm->DeleteMpFromBuffer(shared_from_this());
            mbInOutBuffer = false;
        }
    }
}

bool MapPoint::CanBeForgotten()
{
    unique_lock<mutex> lock(mMutexOut);

    if(mbSentOnce && mbAck && !mbInOutBuffer)
        return true;
    else
        return false;
}

void MapPoint::RemapObservationId(kfptr pKF, const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);

    mObservations[pKF] = idx;
}

std::string MapPoint::GetId()
{
    std::stringstream kfid;
    kfid << mId.first << "|" << mId.second;
    return kfid.str();
}

} //end ns
