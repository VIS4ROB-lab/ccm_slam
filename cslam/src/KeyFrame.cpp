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

#include <cslam/KeyFrame.h>

#define NEWSENDING

namespace cslam {

size_t KeyFrame::nNextId=0;

KeyFrame::KeyFrame(vocptr pVoc, mapptr pMap, dbptr pKFDB, commptr pComm, eSystemState SysState, size_t UniqueId)
    : mFrameId(defpair),mVisId(-1),
      mbFirstConnection(true),mbNotErase(false),
      mbToBeErased(false),
      mpORBvocabulary(pVoc),mpMap(pMap), mpKeyFrameDB(pKFDB),
      mbIsEmpty(false),mbPoseLock(false),mbPoseChanged(false),mbSentOnce(true),mbInOutBuffer(false),
      mLoopQuery(defpair),mMatchQuery(defpair),
      mBALocalForKF(defpair),mBAFixedForKF(defpair),mBAGlobalForKF(defpair),
      mSysState(SysState),mbOmitSending(false),
      mbLoopCorrected(false),
      mbBad(false),
      mbAck(true),mbFromServer(false),mbUpdatedByServer(false),mCorrected_MM(defpair),mbSendFull(false)
{
    mId = defpair;
    mUniqueId = UniqueId;
    mspComm.insert(pComm);
}

KeyFrame::KeyFrame(Frame &F, mapptr pMap, dbptr pKFDB, commptr pComm, eSystemState SysState, size_t UniqueId)
    : mFrameId(F.mId),mUniqueId(UniqueId),mTimeStamp(F.mTimeStamp),mVisId(-1),
      mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
      mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
      mTrackReferenceForFrame(defpair), mFuseTargetForKF(defpair),
      mnLoopWords(0), mRelocQuery(defpair), mnRelocWords(0),
      fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
      N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn), mDescriptors(F.mDescriptors.clone()),
      mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
      mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
      mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
      mnMaxY(F.mnMaxY), mK(F.mK), mvpMapPoints(F.mvpMapPoints),
      mbFirstConnection(true), mpParent(nullptr), mbNotErase(false),
      mbToBeErased(false), mbBad(false), mHalfBaseline(0),
      mpORBvocabulary(F.mpORBvocabulary),mpMap(pMap),mpKeyFrameDB(pKFDB),
      mbIsEmpty(false),mbPoseLock(false),mbPoseChanged(false),mbInOutBuffer(false), mbSentOnce(false),
      mLoopQuery(defpair),mMatchQuery(defpair),
      mBALocalForKF(defpair),mBAFixedForKF(defpair),mBAGlobalForKF(defpair),
      mSysState(SysState),mbOmitSending(false),
      mbLoopCorrected(false),
      mbAck(false),mbFromServer(false),mbUpdatedByServer(false),mCorrected_MM(defpair),mbSendFull(true)
{
    mId=make_pair(nNextId++,F.mId.second);

    mspComm.insert(pComm);

    mGrid.resize(mnGridCols);
    for(int i=0; i<mnGridCols;i++)
    {
        mGrid[i].resize(mnGridRows);
        for(int j=0; j<mnGridRows; j++)
            mGrid[i][j] = F.mGrid[i][j];
    }

    SetPose(F.mTcw,false);

    mvbMapPointsLock.resize(N,false);

    if(mUniqueId == 0 && !mId.first == 0) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m KeyFrame::KeyFrame(..): mUniqueId not set" << endl;

    mdInsertStamp = ros::Time::now().toNSec();
}

KeyFrame::KeyFrame(ccmslam_msgs::KF* pMsg, vocptr pVoc, mapptr pMap, dbptr pKFDB, commptr pComm, eSystemState SysState, size_t UniqueId, g2o::Sim3 mg2oS_wcurmap_wclientmap)
    : mFrameId(defpair),mVisId(-1),
      mbFirstConnection(true),mbNotErase(false),
      mbToBeErased(false),
      mpORBvocabulary(pVoc),mpMap(pMap), mpKeyFrameDB(pKFDB),
      mbIsEmpty(false),mbPoseLock(false),mbPoseChanged(false),mbSentOnce(true),mbInOutBuffer(false),
      mLoopQuery(defpair),mMatchQuery(defpair),
      mBALocalForKF(defpair),mBAFixedForKF(defpair),mBAGlobalForKF(defpair),
      mSysState(SysState),mbOmitSending(false),
      mbLoopCorrected(false),
      mbBad(false),
      mbAck(true),mbFromServer(false),mbUpdatedByServer(false),mCorrected_MM(defpair),mbSendFull(false)
{
    if(pMsg->mbBad)
    {
         cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << " Incoming KF message: mbBad == true" << endl;
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

    this->WriteMembersFromMessage(pMsg,mg2oS_wcurmap_wclientmap);

    AssignFeaturesToGrid();

    mdInsertStamp = ros::Time::now().toNSec();

    mbOmitSending = false;

    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // Call EstablishInitialConnectionsX()
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
}

void KeyFrame::EstablishInitialConnectionsServer()
{
    //this is necessary, because we cannot use shared_from_this() in constructor

    #ifdef LOGGING
    ccptr pCC;
    pCC = mpMap->GetCCPtr(this->mId.second);
    pCC->mpLogger->SetKF(__LINE__,this->mId.second);
    #endif

    {
        unique_lock<mutex> lock(mMutexFeatures);

        for(int idx=0;idx<mvpMapPoints.size();++idx)
        {
            mpptr pMPi = mvpMapPoints[idx];

            if(pMPi)
            {
                pMPi->AddObservation(shared_from_this(),idx);
                pMPi->ComputeDistinctiveDescriptors();
                pMPi->UpdateNormalAndDepth();
            }
            else
            {
                //nullptr -- do not use
            }
        }
    }

    #ifdef LOGGING
    pCC->mpLogger->SetKF(__LINE__,this->mId.second);
    #endif
}

void KeyFrame::EstablishInitialConnectionsClient()
{
    //this is necessary, because we cannot use shared_from_this() in constructor

    {
        unique_lock<mutex> lock(mMutexFeatures);

        for(int idx=0;idx<mvpMapPoints.size();++idx)
        {
            mpptr pMPi = mvpMapPoints[idx];

            if(pMPi)
            {
                pMPi->AddObservation(shared_from_this(),idx);
                pMPi->ComputeDistinctiveDescriptors();
                pMPi->UpdateNormalAndDepth();
            }
            else
            {
                //nullptr -- do not use
            }
        }
    }
}

void KeyFrame::AssignFeaturesToGrid()
{
    int nReserve = 0.5f*N/(mnGridCols*mnGridRows);
    mGrid.resize(mnGridCols);
    for(unsigned int i=0; i<mnGridCols;i++)
    {
        mGrid[i].resize(mnGridRows);
        for (unsigned int j=0; j<mnGridRows;j++)
            mGrid[i][j].reserve(nReserve);
    }

    for(int i=0;i<N;i++)
    {
        const cv::KeyPoint &kp = mvKeysUn[i];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))
            mGrid[nGridPosX][nGridPosY].push_back(i);
    }
}

void KeyFrame::ProcessAfterLoad(map<idpair,idpair> saved_kf_ids_to_sys_ids) {

    mvKeysUn.reserve(mKeysUnAsCvMat.cols);
    for(int i=0;i<mKeysUnAsCvMat.cols;++i) {
        cv::KeyPoint kp;
        kp.pt.x     = mKeysUnAsCvMat.at<float>(0,i);
        kp.pt.y     = mKeysUnAsCvMat.at<float>(1,i);
        kp.angle    = mKeysUnAsCvMat.at<float>(2,i);
        kp.octave   = mKeysUnAsCvMat.at<float>(3,i);
        kp.response = mKeysUnAsCvMat.at<float>(4,i);
        kp.size     = mKeysUnAsCvMat.at<float>(5,i);
        mvKeysUn.push_back(kp);
    }
//    std::cout << "KF " << mId.first << "|" << mId.second << " #KPs: " << mvKeysUn.size() << std::endl;

    mvpMapPoints.resize(N,nullptr);
    mvbMapPointsLock.resize(N,false);

    this->ComputeBoW();

    this->AssignFeaturesToGrid();

    for(auto kf_id : mvLoopEdges_minimal) {
        idpair correct_id;
        if(kf_id.second != 0) {
            if(!saved_kf_ids_to_sys_ids.count(kf_id)) {
                std::cout << COUTERROR << "ID ERROR" << std::endl;
                exit(-1);
            }
            correct_id = saved_kf_ids_to_sys_ids[kf_id];
        } else {
            correct_id = kf_id;
        }
        auto kf = mpMap->GetKfPtr(correct_id);
        if(kf) mspLoopEdges.insert(kf);
    }
}

bool KeyFrame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=mnGridCols || posY<0 || posY>=mnGridRows)
        return false;

    return true;
}

void KeyFrame::ComputeBoW()
{
    if(mBowVec.empty() || mFeatVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        // Feature vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

void KeyFrame::SetPose(const cv::Mat &Tcw_, bool bLock, bool bIgnorePoseMutex)
{
    if(mbPoseLock && mSysState == eSystemState::CLIENT)
        return;

    {
        if(!bIgnorePoseMutex)
            unique_lock<mutex> lock(mMutexPose);
            //do not acquire mMutexOut, since this might be called from Update-method.

        Tcw_.copyTo(Tcw);
        cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
        cv::Mat tcw = Tcw.rowRange(0,3).col(3);
        cv::Mat Rwc = Rcw.t();
        Ow = -Rwc*tcw;

        Twc = cv::Mat::eye(4,4,Tcw.type());
        Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
        Ow.copyTo(Twc.rowRange(0,3).col(3));
        cv::Mat center = (cv::Mat_<float>(4,1) << mHalfBaseline, 0 , 0, 1);
        Cw = Twc*center;

        if(mSysState == eSystemState::CLIENT)
            mdPoseTime = ros::Time::now().toNSec();

        if(bLock)
        {
            mbPoseLock = true;
        }
    }

    if(!mbOmitSending && this->IsSent())
    {
        mbPoseChanged = true;
        if(mSysState == CLIENT)
            SendMe();
    }
}

void KeyFrame::SetPoseFromTcp(const Mat &Tcp_, bool bLock, kfptr pParent)
{
    Tcp_.copyTo(mTcp);

    Tcw = mTcp * pParent->GetPose();

    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    cv::Mat Rwc = Rcw.t();
    Ow = -Rwc*tcw;

    Twc = cv::Mat::eye(4,4,Tcw.type());
    Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
    Ow.copyTo(Twc.rowRange(0,3).col(3));
    cv::Mat center = (cv::Mat_<float>(4,1) << mHalfBaseline, 0 , 0, 1);
    Cw = Twc*center;

    if(bLock)
    {
        mbPoseLock = true;
    }
}

cv::Mat KeyFrame::GetPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.clone();
}

cv::Mat KeyFrame::GetPoseInverse()
{
    unique_lock<mutex> lock(mMutexPose);
    return Twc.clone();
}

cv::Mat KeyFrame::GetCameraCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return Ow.clone();
}

cv::Mat KeyFrame::GetStereoCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return Cw.clone();
}

cv::Mat KeyFrame::GetRotation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).colRange(0,3).clone();
}

cv::Mat KeyFrame::GetTranslation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).col(3).clone();
}

cv::Mat KeyFrame::GetTcp()
{
    unique_lock<mutex> lock(mMutexPose);
    return mTcp.clone();
}

void KeyFrame::AddConnection(kfptr pKF, const int &weight)
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(!mConnectedKeyFrameWeights.count(pKF))
            mConnectedKeyFrameWeights[pKF]=weight;
        else if(mConnectedKeyFrameWeights[pKF]!=weight)
            mConnectedKeyFrameWeights[pKF]=weight;
        else
            return;
    }

    UpdateBestCovisibles();
}

void KeyFrame::UpdateBestCovisibles()
{
    unique_lock<mutex> lock(mMutexConnections);
    vector<pair<int,kfptr> > vPairs;
    vPairs.reserve(mConnectedKeyFrameWeights.size());
    for(map<kfptr,int>::iterator mit=mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
       vPairs.push_back(make_pair(mit->second,mit->first));

    sort(vPairs.begin(),vPairs.end());
    list<kfptr> lKFs;
    list<int> lWs;
    for(size_t i=0, iend=vPairs.size(); i<iend;i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    mvpOrderedConnectedKeyFrames = vector<kfptr>(lKFs.begin(),lKFs.end());
    mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
}

set<KeyFrame::kfptr> KeyFrame::GetConnectedKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    set<kfptr> s;
    for(map<kfptr,int>::iterator mit=mConnectedKeyFrameWeights.begin();mit!=mConnectedKeyFrameWeights.end();mit++)
        s.insert(mit->first);
    return s;
}

vector<KeyFrame::kfptr> KeyFrame::GetVectorCovisibleKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mvpOrderedConnectedKeyFrames;
}

vector<KeyFrame::kfptr> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
{
    unique_lock<mutex> lock(mMutexConnections);
    if((int)mvpOrderedConnectedKeyFrames.size()<N)
        return mvpOrderedConnectedKeyFrames;
    else
        return vector<kfptr>(mvpOrderedConnectedKeyFrames.begin(),mvpOrderedConnectedKeyFrames.begin()+N);

}

vector<KeyFrame::kfptr> KeyFrame::GetCovisiblesByWeight(const int &w)
{
    unique_lock<mutex> lock(mMutexConnections);

    if(mvpOrderedConnectedKeyFrames.empty())
        return vector<kfptr>();

    vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(),mvOrderedWeights.end(),w,KeyFrame::weightComp);
    if(it==mvOrderedWeights.end())
        return vector<kfptr>();
    else
    {
        int n = it-mvOrderedWeights.begin();
        return vector<kfptr>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin()+n);
    }
}

int KeyFrame::GetWeight(kfptr pKF)
{
    unique_lock<mutex> lock(mMutexConnections);
    if(mConnectedKeyFrameWeights.count(pKF))
        return mConnectedKeyFrameWeights[pKF];
    else
        return 0;
}

void KeyFrame::AddMapPoint(mpptr pMP, const size_t &idx, bool bLock)
{
    unique_lock<mutex> lock(mMutexFeatures);

    if(mvbMapPointsLock[idx] && mSysState == eSystemState::CLIENT)
    {
        return;
    }

    mpptr pMPold = mvpMapPoints[idx];
    if(pMPold)
        pMPold->EraseObservation(shared_from_this());

    mvpMapPoints[idx]=pMP;

    if(bLock)
    {
        mvbMapPointsLock[idx] = true;
    }
}

void KeyFrame::EraseMapPointMatch(const size_t &idx, bool bLock)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(this->IsEmpty()) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m KeyFrame::EraseMapPointMatch(): trying to erase MP from empty KF" << endl;

    if(mvbMapPointsLock[idx] && mSysState == eSystemState::CLIENT)
    {
        return;
    }

    mvpMapPoints[idx]=nullptr;

    if(bLock)
    {
        mvbMapPointsLock[idx] = true;
    }
}

void KeyFrame::EraseMapPointMatch(mpptr pMP, bool bLock)
{    
    int idx = pMP->GetIndexInKeyFrame(this->shared_from_this());
    if(idx>=0)
    {
        if(mvbMapPointsLock[idx] && mSysState == eSystemState::CLIENT)
        {
            return;
        }

        mvpMapPoints[idx]=nullptr;

        if(bLock)
        {
            mvbMapPointsLock[idx] = true;
        }
    }
    else
    {
        vector<mpptr>::iterator vit = std::find(mvpMapPoints.begin(),mvpMapPoints.end(),pMP);

        if(vit == mvpMapPoints.end())
        {
            //MP not in KF -- nothing to erase
        }
        else
        {
            //MP thinks it is not in KF, but KF has it associated to feat
            int id = vit - mvpMapPoints.begin();
            mvpMapPoints[id] = nullptr;
        }
    }
}

void KeyFrame::ReplaceMapPointMatch(const size_t &idx, mpptr pMP, bool bLock, bool bOverrideLock)
{
    unique_lock<mutex> lock(mMutexFeatures); //was not here in ORB implementation

    if(mvbMapPointsLock[idx] && mSysState == eSystemState::CLIENT)
        return;

    if(mvpMapPoints[idx] && mvpMapPoints[idx]->mbDoNotReplace) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m KeyFrame::ReplaceMapPointMatch(...): mbDoNotReplace true" << endl;

    mvpMapPoints[idx]=pMP;

    if(bLock)
    {
        mvbMapPointsLock[idx] = true;
    }
}

set<KeyFrame::mpptr> KeyFrame::GetMapPoints()
{
    unique_lock<mutex> lock(mMutexFeatures);
    set<mpptr> s;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        if(!mvpMapPoints[i])
            continue;
        mpptr pMP = mvpMapPoints[i];
        if(!pMP->isBad())
            s.insert(pMP);
    }
    return s;
}

int KeyFrame::TrackedMapPoints(const int &minObs)
{
    unique_lock<mutex> lock(mMutexFeatures);

    int nPoints=0;
    const bool bCheckObs = minObs>0;
    for(int i=0; i<N; i++)
    {
        mpptr pMP = mvpMapPoints[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(bCheckObs)
                {
                    if(mvpMapPoints[i]->Observations()>=minObs)
                        nPoints++;
                }
                else
                    nPoints++;
            }
        }
    }

    return nPoints;
}

vector<KeyFrame::mpptr> KeyFrame::GetMapPointMatches()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints;
}

KeyFrame::mpptr KeyFrame::GetMapPoint(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints[idx];
}

bool KeyFrame::IsMpLocked(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvbMapPointsLock[idx];
}

void KeyFrame::UpdateConnections(bool bIgnoreMutex)
{
    bool bSetBad = false;

    map<kfptr,int> KFcounter;

    vector<mpptr> vpMP;

    {
        unique_lock<mutex> lockMPs(mMutexFeatures);
        vpMP = mvpMapPoints;
    }

    //For all map points in keyframe check in which other keyframes are they seen
    //Increase counter for those keyframes
    for(vector<mpptr>::iterator vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)
    {
        mpptr pMP = *vit;

        if(!pMP)
            continue;

        if(pMP->isBad())
            continue;

        map<kfptr,size_t> observations = pMP->GetObservations();

        for(map<kfptr,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            if(mit->first->mId == this->mId)
                continue;
            KFcounter[mit->first]++;
        }
    }

    if(KFcounter.empty())
    {
        return;
    }

    //If the counter is greater than threshold add connection
    //In case no keyframe counter is over threshold add the one with maximum counter
    int nmax=0;
    kfptr pKFmax=nullptr;
    int th = 15;

    vector<pair<int,kfptr> > vPairs;
    vPairs.reserve(KFcounter.size());
    for(map<kfptr,int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend; mit++)
    {
        if(mit->second>nmax)
        {
            nmax=mit->second;
            pKFmax=mit->first;
        }
        if(mit->second>=th)
        {
            vPairs.push_back(make_pair(mit->second,mit->first));
            (mit->first)->AddConnection(this->shared_from_this(),mit->second);
        }
    }

    if(vPairs.empty())
    {
        vPairs.push_back(make_pair(nmax,pKFmax));
        pKFmax->AddConnection(this->shared_from_this(),nmax);
    }

    sort(vPairs.begin(),vPairs.end());
    list<kfptr> lKFs;
    list<int> lWs;
    for(size_t i=0; i<vPairs.size();i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    {
        unique_lock<mutex> lockCon(mMutexConnections);

        mConnectedKeyFrameWeights = KFcounter;
        mvpOrderedConnectedKeyFrames = vector<kfptr>(lKFs.begin(),lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

        if(mbFirstConnection && mId.first!=0)
        {
            if(mSysState == eSystemState::CLIENT)
            {                
                mpParent = mvpOrderedConnectedKeyFrames.front();
                if(mspChildrens.count(mpParent) || mpParent->mbFromServer);
                {
                    //THIS is alread parent of the desginated parent
                    //Furthermore, do not use KFs sent from Server as parents
                    vector<kfptr>::iterator vit = mvpOrderedConnectedKeyFrames.begin();
                    while(mspChildrens.count(mpParent) || mpParent->mbFromServer)
                    {
                        ++vit;
                        if(vit == mvpOrderedConnectedKeyFrames.end())
                        {
                            if(this->mId.second == mpMap->mMapId)
                            {
                                bSetBad = true;
                                break;
                            }
                            else
                            {
                                mpParent = nullptr;
                                break;
                            }
                        }
                        mpParent = *vit;
                    }
                }
            }
            else if(mSysState == eSystemState::SERVER)
            {
                //Enforce tree structure on server
                vector<kfptr>::iterator vit = mvpOrderedConnectedKeyFrames.begin();
                kfptr pPC = *vit;
                while(!(pPC->mId.first < this->mId.first))
                {
                    ++vit;
                    if(vit == mvpOrderedConnectedKeyFrames.end())
                    {
                        //strategy: use nearest predecessor
                        {
                            for(int itid=1;itid<10;itid++)
                            {
                                pPC = mpMap->GetKfPtr(mId.first-itid,mId.second);
                                if(pPC)
                                    break;
                            }

                            if(!pPC)
                            {
                                std::cout << "No predecessor" << std::endl;
                                throw estd::infrastructure_ex();
                            }

                            break;
                        }
                    }
                    pPC = *vit;
                }
                mpParent = pPC;
            }
            if(!bSetBad)
            {
                if(mpParent)
                {
                    mpParent->AddChild(this->shared_from_this());
                    mbFirstConnection = false;
                }
                else if(!(mSysState == eSystemState::CLIENT && this->mId.second != mpMap->mMapId))
                {
                    cout << "\033[1;31m!!!!! FATAL !!!!!\033[0m " << __func__ << ":" << __LINE__ << ": cannot find parent" << endl;
                    throw infrastructure_ex();
                }
                else
                {
                    //On the client and cannot process KF from other client -- ignore data
                    {
                        for(map<kfptr,int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
                            mit->first->EraseConnection(this->shared_from_this());
                    }

                    for(size_t i=0; i<mvpMapPoints.size(); i++)
                    {
                        if(mvpMapPoints[i])
                        {
                            mpptr pMPx = mvpMapPoints[i];
                            pMPx->EraseObservation(this->shared_from_this(),false,true);
                        }
                    }


                    {
                        unique_lock<mutex> lock1(mMutexFeatures);

                        mConnectedKeyFrameWeights.clear();
                        mvpOrderedConnectedKeyFrames.clear();

                        if(!mspChildrens.empty())
                            cout << "\033[1;33m!!! WARN !!!\033[0m " << __func__ << ":" << __LINE__ << " mspChildrens assumed to be empty at this point" << endl;
                        mbBad = true;
                    }

                    mpMap->EraseKeyFrame(this->shared_from_this());
                    mpKeyFrameDB->erase(this->shared_from_this());
                }
            }
        }
    }

    if(!bSetBad && mpParent && mpParent->mId == this->mId)
    {
        cout << "\033[1;31m!!!!! FATAL ERROR !!!!!\033[0m " << __func__ << ":"  << __LINE__ << " child->mId == this->mId" << endl;
        cout << "This: " << mId.first << "|" << mId.second << " -- Parent: " << mpParent->mId.first << "|" << mpParent->mId.second << endl;
        throw infrastructure_ex();
    }

    if(bSetBad)
    {
        this->SetBadFlag(false,true);
    }

    #ifdef DEBUGGING2
    {
        kfptr pKFp = this->GetParent();
        if(pKFp)
        {
            if(pKFp->mId == this->mId)
            {
                std::cout << COUTERROR << "KF " << this->mId.first << "|" << this->mId.second << " : is its own parent" << std::endl;
            }
        }
        else
        {
            if(!mId.first==0)
                std::cout << COUTERROR << "KF " << this->mId.first << "|" << this->mId.second << " : no parent" << std::endl;
        }
    }
    #endif
}

void KeyFrame::AddChild(kfptr pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);

    mspChildrens.insert(pKF);
}

void KeyFrame::EraseChild(kfptr pKF, bool bIgnoreMutex)
{
    if(!bIgnoreMutex)
        unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.erase(pKF);
}

void KeyFrame::ChangeParent(kfptr pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);

    if(mpParent)
    {
        mpParent->EraseChild(shared_from_this(),true);
    }

    mpParent = pKF;

    pKF->AddChild(this->shared_from_this());
}

set<KeyFrame::kfptr> KeyFrame::GetChilds()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens;
}

KeyFrame::kfptr KeyFrame::GetParent(bool bIgnorePoseMutex)
{
    if(!bIgnorePoseMutex)
        unique_lock<mutex> lockCon(mMutexConnections);
    return mpParent;
}

bool KeyFrame::hasChild(kfptr pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens.count(pKF);
}

void KeyFrame::AddLoopEdge(kfptr pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mbNotErase = true;
    mspLoopEdges.insert(pKF);
}

set<KeyFrame::kfptr> KeyFrame::GetLoopEdges()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspLoopEdges;
}

void KeyFrame::SetNotErase()
{
    unique_lock<mutex> lock(mMutexConnections);
    mbNotErase = true;
}

void KeyFrame::SetErase()
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mspLoopEdges.empty())
        {
            mbNotErase = false;
        }
    }

    if(mbToBeErased)
    {
        SetBadFlag();
    }
}

void KeyFrame::SetBadFlag(bool bSuppressMapAction, bool bNoParent)
{

    #ifdef LOGGING
    ccptr pCC;
    if(this->mSysState == SERVER)
    {
        pCC = mpMap->GetCCPtr(this->mId.second);
        pCC->mpLogger->SetKF(__LINE__,this->mId.second);
    }
    #endif

    {
        if(mbBad)
        {
            #ifdef LOGGING
            if(this->mSysState == SERVER)
                pCC->mpLogger->SetKF(__LINE__,this->mId.second);
            #endif
            return;
        }
    }

    {
        unique_lock<mutex> lock(mMutexConnections);

        if(mId.first==0)
        {
            #ifdef LOGGING
            if(this->mSysState == SERVER)
                pCC->mpLogger->SetKF(__LINE__,this->mId.second);
            #endif
            return;
        }
        else if(mbNotErase)
        {
            if(mSysState == eSystemState::CLIENT)
            {
                cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << " mbNotErase not supposed to be  TRUE at any time on client" << endl;
                throw infrastructure_ex();
            }

            mbToBeErased = true;
            #ifdef LOGGING
            if(this->mSysState == SERVER)
                pCC->mpLogger->SetKF(__LINE__,this->mId.second);
            #endif
            return;
        }

        for(map<kfptr,int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
            mit->first->EraseConnection(this->shared_from_this());
    }

    for(size_t i=0; i<mvpMapPoints.size(); i++)
    {
        if(mvpMapPoints[i])
        {
                mpptr pMPx = mvpMapPoints[i];
                pMPx->EraseObservation(this->shared_from_this(),false,true);
        }
    }

    {
        unique_lock<mutex> lock(mMutexConnections);
        unique_lock<mutex> lock1(mMutexFeatures);

        mConnectedKeyFrameWeights.clear();
        mvpOrderedConnectedKeyFrames.clear();

        // Update Spanning Tree
        set<kfptr> sParentCandidates;
        sParentCandidates.insert(mpParent);

        // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
        // Include that children as new parent candidate for the rest
        while(!mspChildrens.empty())
        {
            bool bContinue = false;

            int max = -1;
            kfptr pC;
            kfptr pP;

            for(set<kfptr>::iterator sit=mspChildrens.begin(), send=mspChildrens.end(); sit!=send; sit++)
            {
                kfptr pKF = *sit;
                if(pKF->isBad())
                    continue;

                // Check if a parent candidate is connected to the keyframe
                vector<kfptr> vpConnected = pKF->GetVectorCovisibleKeyFrames();
                for(size_t i=0, iend=vpConnected.size(); i<iend; i++)
                {
                    for(set<kfptr>::iterator spcit=sParentCandidates.begin(), spcend=sParentCandidates.end(); spcit!=spcend; spcit++)
                    {
                        if(pKF->hasChild(*spcit))
                            continue;

                        if(mSysState == eSystemState::SERVER)
                        {
                            //Enforce tree structure on server
                            if(!((*spcit)->mId < pKF->mId))
                                continue;
                        }

                        if((*spcit)->mbFromServer)
                            continue; // do not use KFs from Server as parents

                        if(vpConnected[i]->mId == (*spcit)->mId)
                        {
                            int w = pKF->GetWeight(vpConnected[i]);
                            if(w>max)
                            {
                                pC = pKF;
                                pP = vpConnected[i];
                                max = w;
                                bContinue = true;
                            }
                        }
                    }
                }

                #ifdef DEBUGGING2
                {
                    kfptr pKFp = pKF->GetParent();
                    if(pKFp)
                    {
                        if(pKFp->mId == pKF->mId)
                        {
                            std::cout << COUTERROR << "KF " << pKF->mId.first << "|" << pKF->mId.second << " : is its own parent" << std::endl;
                        }
                    }
                    else
                    {
                        if(!pKF->mId.first==0)
                            std::cout << COUTERROR << "KF " << pKF->mId.first << "|" << pKF->mId.second << " : no parent" << std::endl;
                    }
                }
                #endif
            }

            if(bContinue)
            {
                pC->ChangeParent(pP);
                sParentCandidates.insert(pC);
                mspChildrens.erase(pC);
            }
            else
                break;
        }

        // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
        if(!mspChildrens.empty())
        {
            while(!mspChildrens.empty()) //cannot use "for(set<kfptr>::iterator sit=mspChildrens.begin(); sit!=mspChildrens.end(); sit++)" here --> ChangeParent will erase a child, this results in a memory leak
            {

                kfptr pCi = *(mspChildrens.begin());

                pCi->ChangeParent(mpParent);

                #ifdef DEBUGGING2
                {
                    kfptr pKFp = pCi->GetParent();
                    if(pKFp)
                    {
                        if(pKFp->mId == pCi->mId)
                        {
                            std::cout << COUTERROR << "KF " << pCi->mId.first << "|" << pCi->mId.second << " : is its own parent" << std::endl;
                        }
                    }
                    else
                    {
                        if(!pCi->mId.first==0)
                            std::cout << COUTERROR << "KF " << pCi->mId.first << "|" << pCi->mId.second << " : no parent" << std::endl;
                    }
                }
                #endif
            }
        }

        if(!bNoParent)
        {
            //When KF comes from Server, and can not acquire MPs, it also has no parent, since there are no connected KFs
            if(mpParent)
            {
                mpParent->EraseChild(this->shared_from_this());
                mTcp = Tcw*mpParent->GetPoseInverse().clone();
            }
            else
            {
                Tcw.copyTo(mTcp);
                mpParent = mpMap->GetKfPtr(0,mId.second);
            }
        }
        mbBad = true;
    }

    if(!bSuppressMapAction)
        mpMap->EraseKeyFrame(this->shared_from_this());

    mpKeyFrameDB->erase(this->shared_from_this());

    #ifdef LOGGING
    if(this->mSysState == SERVER)
        pCC->mpLogger->SetKF(__LINE__,this->mId.second);
    #endif
}

void KeyFrame::EraseConnection(kfptr pKF)
{
    bool bUpdate = false;
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mConnectedKeyFrameWeights.count(pKF))
        {
            mConnectedKeyFrameWeights.erase(pKF);
            bUpdate=true;
        }
    }

    if(bUpdate)
        UpdateBestCovisibles();
}

vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=mnGridCols)
        return vIndices;

    const int nMaxCellX = min((int)mnGridCols-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=mnGridRows)
        return vIndices;

    const int nMaxCellY = min((int)mnGridRows-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool KeyFrame::IsInImage(const float &x, const float &y) const
{
    return (x>=mnMinX && x<mnMaxX && y>=mnMinY && y<mnMaxY);
}

float KeyFrame::ComputeSceneMedianDepth(const int q)
{
    vector<mpptr> vpMapPoints;
    cv::Mat Tcw_;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPose);
        vpMapPoints = mvpMapPoints;
        Tcw_ = Tcw.clone();
    }

    vector<float> vDepths;
    vDepths.reserve(N);
    cv::Mat Rcw2 = Tcw_.row(2).colRange(0,3);
    Rcw2 = Rcw2.t();
    float zcw = Tcw_.at<float>(2,3);
    for(int i=0; i<N; i++)
    {
        if(mvpMapPoints[i])
        {
            mpptr pMP = mvpMapPoints[i];
            if(pMP->IsEmpty()) continue;
            cv::Mat x3Dw = pMP->GetWorldPos();
            float z = Rcw2.dot(x3Dw)+zcw;
            vDepths.push_back(z);
        }
    }

    sort(vDepths.begin(),vDepths.end());

    return vDepths[(vDepths.size()-1)/q];
}

void KeyFrame::SaveData() const {
    // pre-process data
    mmMapPoints_minimal.clear();
    mvLoopEdges_minimal.clear();
    for (size_t indx = 0; indx < mvpMapPoints.size(); indx++) {
        if(mvpMapPoints[indx] != nullptr)
            mmMapPoints_minimal.insert(std::make_pair(indx, mvpMapPoints[indx]->mId));
    }
    for(auto kf : mspLoopEdges)
        mvLoopEdges_minimal.push_back(kf->mId);

    mKeysUnAsCvMat = cv::Mat(6,mvKeysUn.size(),5);

    for(int i=0;i<mvKeysUn.size();++i) {
        mKeysUnAsCvMat.at<float>(0,i) = mvKeysUn[i].pt.x;
        mKeysUnAsCvMat.at<float>(1,i) = mvKeysUn[i].pt.y;
        mKeysUnAsCvMat.at<float>(2,i) = mvKeysUn[i].angle;
        mKeysUnAsCvMat.at<float>(3,i) = mvKeysUn[i].octave;
        mKeysUnAsCvMat.at<float>(4,i) = mvKeysUn[i].response;
        mKeysUnAsCvMat.at<float>(5,i) = mvKeysUn[i].size;
    }
}

void KeyFrame::SendMe()
{
    if(mspComm.empty())
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m KeyFrame::SendMe(): no Comm ptrs" << endl;
        throw infrastructure_ex();
    }

    if(this->IsSent() && !this->IsInOutBuffer())
    {
        for(set<commptr>::const_iterator sit = mspComm.begin();sit!=mspComm.end();++sit)
        {
            commptr pComm = *sit;
            pComm->PassKftoComm(this->shared_from_this());
        }
    }
}

void KeyFrame::SetSendFull()
{
    {
        unique_lock<mutex> lock(mMutexOut);
        mbSendFull = true;
    }

    this->SendMe();
}

void KeyFrame::ReplaceMap(mapptr pNewMap)
{
    unique_lock<mutex> lock1(mMutexFeatures,defer_lock);
    unique_lock<mutex> lock2(mMutexConnections,defer_lock);
    unique_lock<mutex> lock3(mMutexPose,defer_lock);
    unique_lock<mutex> lockOut(mMutexOut,defer_lock);

    lock(lock1,lock2,lock3,lockOut);

    mpMap = pNewMap;
}

void KeyFrame::ReduceMessage(ccmslam_msgs::KF *pMsgFull, ccmslam_msgs::KFred *pMsgRed)
{
    pMsgRed->mnId = pMsgFull->mnId;
    pMsgRed->mClientId = pMsgFull->mClientId;
    pMsgRed->mUniqueId = pMsgFull->mUniqueId;
    pMsgRed->mTcpred = pMsgFull->mTcpred;
    pMsgRed->mTcpar = pMsgFull->mTcpar;
    pMsgRed->mpPred_KfId = pMsgFull->mpPred_KfId;
    pMsgRed->mpPred_KfClientId = pMsgFull->mpPred_KfClientId;
    pMsgRed->mpPar_KfId = pMsgFull->mpPar_KfId;
    pMsgRed->mpPar_KfClientId = pMsgFull->mpPar_KfClientId;
    pMsgRed->mbBad = pMsgFull->mbBad;
}

void KeyFrame::ConvertToMessage(ccmslam_msgs::Map &msgMap, g2o::Sim3 mg2oS_wcurmap_wclientmap, kfptr pRefKf, bool bForceUpdateMsg)
{
    unique_lock<mutex> lockOut(mMutexOut);

    if((mbSendFull || mSysState == eSystemState::SERVER) && !bForceUpdateMsg)
    {
        ccmslam_msgs::KF Msg;

        unique_lock<mutex> lock1(mMutexFeatures,defer_lock);
        unique_lock<mutex> lock2(mMutexConnections,defer_lock);
        unique_lock<mutex> lock3(mMutexPose,defer_lock);

        lock(lock1,lock2,lock3);

        if(mSysState == eSystemState::SERVER)
        {
            float s = mg2oS_wcurmap_wclientmap.inverse().scale();
            s = 1/s;

            cv::Mat Twp = cv::Mat(4,4,5);
            cv::Mat Tcp  = cv::Mat(4,4,5);

            if(this->mId != pRefKf->mId)
            {
                Twp = pRefKf->GetPoseInverse();
                Tcp = Tcw * Twp;
                Tcp.at<float>(0,3) *=(1./s);
                Tcp.at<float>(1,3) *=(1./s);
                Tcp.at<float>(2,3) *=(1./s);
            }
            else
            {
                Twp = cv::Mat::eye(4,4,5);

                cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
                cv::Mat tcw = Tcw.rowRange(0,3).col(3);
                g2o::Sim3 g2oS_c_wm(Converter::toMatrix3d(Rcw),Converter::toVector3d(tcw),1.0); //cam - world map

                g2o::Sim3 g2oS_c_wc = g2oS_c_wm*mg2oS_wcurmap_wclientmap;

                Eigen::Matrix3d eigR = g2oS_c_wc.rotation().toRotationMatrix();
                Eigen::Vector3d eigt = g2oS_c_wc.translation();
                float scale = mg2oS_wcurmap_wclientmap.inverse().scale();
                scale = 1/scale;
                eigt *=(1./scale); //[R t/s;0 1]
                cv::Mat T_c_wc = Converter::toCvSE3(eigR,eigt);

                Tcp = T_c_wc * Twp;
            }

            Converter::CvMatToMsgArrayFixedSize<ccmslam_msgs::KF::_mTcpred_type,float>(Tcp,Msg.mTcpred);

            Msg.mpPred_KfId = static_cast<uint16_t>(pRefKf->mId.first);
            Msg.mpPred_KfClientId = static_cast<uint8_t>(pRefKf->mId.second);

            Msg.mpPar_KfId = KFRANGE;
            Msg.mpPar_KfClientId = MAPRANGE;

            Msg.mbBad = mbBad;
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
            if(this->mId.first == 0)
            {
                Converter::CvMatToMsgArrayFixedSize<ccmslam_msgs::KF::_mTcpred_type,float>(Tcw,Msg.mTcpred);
            }
            else
            {
                //pose relative to predecessor
                kfptr pPred = mpMap->GetPredecessor(shared_from_this());
                mTcp = Tcw*pPred->GetPoseInverse();
                Msg.mpPred_KfId = static_cast<uint16_t>(pPred->mId.first);
                Msg.mpPred_KfClientId = static_cast<uint8_t>(pPred->mId.second);
                Converter::CvMatToMsgArrayFixedSize<ccmslam_msgs::KF::_mTcpred_type,float>(mTcp,Msg.mTcpred);

                //pose relative to parent's parent
                if(!mpParent)
                {
                    cout << COUTFATAL << ": no parent" << endl;
                    throw infrastructure_ex();
                }
                else
                {
                    kfptr pParPar = mpParent->GetParent();
                    if(!pParPar)
                        pParPar = mpParent; //if mpParents has no parent, use mpParent
                    else if(pParPar->mId == this->mId)
                    {
                        pParPar = mpParent; //if mpParents has no parent, use mpParent
                    }

                    mTcp = Tcw*pParPar->GetPoseInverse();
                    Msg.mpPar_KfId = static_cast<uint16_t>(pParPar->mId.first);
                    Msg.mpPar_KfClientId = static_cast<uint8_t>(pParPar->mId.second);
                    Converter::CvMatToMsgArrayFixedSize<ccmslam_msgs::KF::_mTcpar_type,float>(mTcp,Msg.mTcpar);

                    Msg.mbServerBA = false;
                }
            }

            //    Msg.mbBad = mbBad;
            Msg.mbBad = false; //necessary in case that KF is in out buffer when trimmed from Map
            Msg.mbAck = mbAck;

            Msg.bSentOnce = mbSentOnce;
            mbSentOnce = true;
            mbPoseChanged = false;
            mbSendFull = false;

            ccptr pCC = mpMap->GetCCPtr(this->mId.second);
            cv::Mat T_SC = Converter::toCvMat(pCC->mT_SC);
            Converter::CvMatToMsgArrayFixedSize<ccmslam_msgs::KF::_mT_SC_type,float>(T_SC,Msg.mT_SC);
        }

        //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        //   ++
        //   ++
        // ++++++   Const values
        //  ++++
        //   ++

        Msg.mnGridCols = mnGridCols;
        Msg.mnGridRows = mnGridRows;
        Msg.mfGridElementWidthInv = mfGridElementWidthInv;
        Msg.mfGridElementHeightInv = mfGridElementHeightInv;

        Msg.fx = fx;
        Msg.fy = fy;
        Msg.cx = cx;
        Msg.cy = cy;
        Msg.invfx = invfx;
        Msg.invfy = invfy;

        Msg.N = static_cast<int16_t>(N);

        for(int idx=0;idx<mvKeysUn.size();++idx) Msg.mvKeysUn.push_back(Converter::toCvKeyPointMsg(mvKeysUn[idx]));

        for(int idx=0;idx<mDescriptors.rows;++idx)
        {
            ccmslam_msgs::Descriptor MsgDesc;
            for(int idy=0;idy<mDescriptors.cols;++idy)
            {
                MsgDesc.mDescriptor[idy]=mDescriptors.at<uint8_t>(idx,idy);
            }
            Msg.mDescriptors.push_back(MsgDesc);
        }

        Msg.mnScaleLevels = static_cast<int8_t>(mnScaleLevels);
        Msg.mfScaleFactor = mfScaleFactor;
        Msg.mfLogScaleFactor = mfLogScaleFactor;
        for(int idx=0;idx<mvScaleFactors.size();++idx) Msg.mvScaleFactors[idx] = mvScaleFactors[idx];
        for(int idx=0;idx<mvLevelSigma2.size();++idx) Msg.mvLevelSigma2[idx] = mvLevelSigma2[idx];
        for(int idx=0;idx<mvInvLevelSigma2.size();++idx) Msg.mvInvLevelSigma2[idx] = mvInvLevelSigma2[idx];

        Msg.mnMinX = static_cast<int16_t>(mnMinX);
        Msg.mnMinY = static_cast<int16_t>(mnMinY);
        Msg.mnMaxX = static_cast<int16_t>(mnMaxX);
        Msg.mnMaxY = static_cast<int16_t>(mnMaxY);

        Converter::CvMatToMsgArrayFixedSize<ccmslam_msgs::KF::_mK_type,float>(mK,Msg.mK);

        //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        //   ++
        //   ++
        // ++++++   Map points
        //  ++++
        //   ++

        for(int idx=0;idx<mvpMapPoints.size();++idx)
        {
            if(mvpMapPoints[idx] && !mvpMapPoints[idx]->isBad())
            {
                Msg.mvpMapPoints_Ids.push_back(static_cast<uint32_t>(mvpMapPoints[idx]->mId.first));
                Msg.mvpMapPoints_ClientIds.push_back(static_cast<uint8_t>(mvpMapPoints[idx]->mId.second));
                Msg.mvpMapPoints_VectId.push_back(static_cast<uint16_t>(idx));
            }
        }

        //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        Msg.mnId = static_cast<uint16_t>(mId.first);
        Msg.mClientId = static_cast<uint8_t>(mId.second);
        Msg.mUniqueId = static_cast<uint32_t>(mUniqueId);
        Msg.dTimestamp = mTimeStamp;

        msgMap.Keyframes.push_back(Msg);
    }
    else
    {
        if(mSysState == eSystemState::SERVER)
        {
            cout << COUTFATAL << " must no be used by server" << endl;
            throw infrastructure_ex();
        }

        ccmslam_msgs::KFred Msg;

        unique_lock<mutex> lockPose(mMutexPose);

            if(mbPoseChanged)
            {
                if(this->mId.first == 0)
                {
                    Converter::CvMatToMsgArrayFixedSize<ccmslam_msgs::KFred::_mTcpred_type,float>(Tcw,Msg.mTcpred);
                }
                else
                {
                    //pose relative to predecessor
                    kfptr pPred;
                    if(this->mId.second == mpMap->mMapId) //native KF
                        pPred = mpMap->GetPredecessor(shared_from_this());
                    else
                    {
                        pPred = mpParent;
                    }
                    mTcp = Tcw*pPred->GetPoseInverse();
                    Msg.mpPred_KfId = static_cast<uint16_t>(pPred->mId.first);
                    Msg.mpPred_KfClientId = static_cast<uint8_t>(pPred->mId.second);
                    Converter::CvMatToMsgArrayFixedSize<ccmslam_msgs::KFred::_mTcpred_type,float>(mTcp,Msg.mTcpred);

                    //pose relative to parent
                    if(!mpParent)
                    {
                        cout << COUTFATAL << ": no parent" << endl;
                        throw infrastructure_ex();
                    }
                    else
                    {
                        kfptr pParPar = mpParent->GetParent();
                        if(!pParPar)
                            pParPar = mpParent; //if mpParents has no parent, use mpParent
                        else if(pParPar->mId == this->mId)
                        {
                            pParPar = mpParent; //if mpParents has no parent, use mpParent
                        }

                        mTcp = Tcw*pParPar->GetPoseInverse();
                        Msg.mpPar_KfId = static_cast<uint16_t>(pParPar->mId.first);
                        Msg.mpPar_KfClientId = static_cast<uint8_t>(pParPar->mId.second);
                        Converter::CvMatToMsgArrayFixedSize<ccmslam_msgs::KF::_mTcpar_type,float>(mTcp,Msg.mTcpar);
                    }
                }

                Msg.mbServerBA = false;

                mbPoseChanged = false;

                Msg.mbBad = false; //necessary in case that KF is in out buffer when trimmed from Map
                Msg.mnId = static_cast<uint16_t>(mId.first);
                Msg.mClientId = static_cast<uint8_t>(mId.second);
                Msg.mUniqueId = static_cast<uint32_t>(mUniqueId);

                msgMap.KFUpdates.push_back(Msg);
            }
            else
            {
                //pose has not changed - do nothing
            }
    }
}

void KeyFrame::UpdateFromMessage(ccmslam_msgs::KF *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap)
{
    //if KF already exists, we reduce it to an Update-Msg to use common interfaces

    ccmslam_msgs::KFred *pMsgRed = new ccmslam_msgs::KFred();

    this->ReduceMessage(pMsg,pMsgRed);

    this->UpdateFromMessage(pMsgRed,mg2oS_wcurmap_wclientmap);

    delete pMsgRed;
}

void KeyFrame::UpdateFromMessage(ccmslam_msgs::KFred *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap)
{
    mbOmitSending = true;

    unique_lock<mutex> lockOut(mMutexOut);
    unique_lock<mutex> lock(mMutexPose);

    if(mSysState == eSystemState::CLIENT)
    {
        bool bSetPose = this->SetPoseFromMessage(pMsg,mg2oS_wcurmap_wclientmap);

        if(!bSetPose)
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
            if(this->mId.first != 0)
            {
                bool bSetPose = this->SetPoseFromMessage(pMsg,mg2oS_wcurmap_wclientmap);

                if(!bSetPose)
                {
                    mbOmitSending = false;
                    return;
                }
            }
            else
            {
                //first KF has no parent

                cv::Mat tempTcw = cv::Mat(4,4,5);
                Converter::MsgArrayFixedSizeToCvMat<ccmslam_msgs::KF::_mTcpred_type,float>(tempTcw,pMsg->mTcpred);
                cv::Mat Rcw = tempTcw.rowRange(0,3).colRange(0,3);
                cv::Mat tcw = tempTcw.rowRange(0,3).col(3);
                g2o::Sim3 g2oS_c_wc(Converter::toMatrix3d(Rcw),Converter::toVector3d(tcw),1.0); //cam - world client

                g2o::Sim3 g2oS_c_wm = g2oS_c_wc*(mg2oS_wcurmap_wclientmap.inverse()); //cam client - world map

                Eigen::Matrix3d eigR = g2oS_c_wm.rotation().toRotationMatrix();
                Eigen::Vector3d eigt = g2oS_c_wm.translation();
                float s = static_cast<float>(mg2oS_wcurmap_wclientmap.scale());
                s = 1/s;
                eigt *=(1./s); //[R t/s;0 1]
                cv::Mat T_c_wm = Converter::toCvSE3(eigR,eigt);

                this->SetPose(T_c_wm,false,true);
            }
        }
    }

    mbOmitSending = false;
}

void KeyFrame::WriteMembersFromMessage(ccmslam_msgs::KF *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap)
{
    mbSentOnce=true;

    mId = make_pair(pMsg->mnId,pMsg->mClientId);
    mTimeStamp = pMsg->dTimestamp;
    mnGridCols = pMsg->mnGridCols;
    mnGridRows=pMsg->mnGridRows;
    mfGridElementWidthInv=pMsg->mfGridElementWidthInv;
    mfGridElementHeightInv=pMsg->mfGridElementHeightInv;
    fx=pMsg->fx;
    fy=pMsg->fy;
    cx=pMsg->cx;
    cy=pMsg->cy;
    invfx=pMsg->invfx;
    invfy=pMsg->invfy;

    N=pMsg->N;
    mnScaleLevels=pMsg->mnScaleLevels;
    mfScaleFactor=pMsg->mfScaleFactor;
    mfLogScaleFactor=pMsg->mfLogScaleFactor;
    mnMinX=pMsg->mnMinX;
    mnMinY=pMsg->mnMinY;
    mnMaxX=pMsg->mnMaxX;
    mnMaxY=pMsg->mnMaxY;

    mvKeysUn.resize(pMsg->mvKeysUn.size());
    for(int idx=0;idx<pMsg->mvKeysUn.size();++idx)
        mvKeysUn[idx] = Converter::fromCvKeyPointMsg(pMsg->mvKeysUn[idx]);

    ccmslam_msgs::Descriptor TempDesc = pMsg->mDescriptors[0];
    int iBoundY = static_cast<int>(TempDesc.mDescriptor.size());
    int iBoundX = static_cast<int>(pMsg->mDescriptors.size());
    mDescriptors = cv::Mat(iBoundX,iBoundY,0);

    for(int idx=0;idx<pMsg->mDescriptors.size();++idx)
    {
        ccmslam_msgs::Descriptor DescMsg = pMsg->mDescriptors[idx];
        for(int idy=0;idy<iBoundY;++idy)
        {
            mDescriptors.at<uint8_t>(idx,idy)=DescMsg.mDescriptor[idy];
        }
    }

    for(int idx=0;idx<mnScaleLevels;++idx)
        mvScaleFactors.push_back(pMsg->mvScaleFactors[idx]);

    for(int idx=0;idx<mnScaleLevels;++idx)
        mvLevelSigma2.push_back(pMsg->mvLevelSigma2[idx]);

    for(int idx=0;idx<mnScaleLevels;++idx)
        mvInvLevelSigma2.push_back(pMsg->mvInvLevelSigma2[idx]);

    mK = cv::Mat(3,3,5);
    Converter::MsgArrayFixedSizeToCvMat<ccmslam_msgs::KF::_mK_type,float>(mK,pMsg->mK);

    if(!mBowVec.empty() || !mFeatVec.empty())
    {
        cout << "\033[1;33m!!! WARN !!!\033[0m " << __func__ << ":" << __LINE__ << " !mBowVec.empty() || !mFeatVec.empty()" << endl;
    }

    vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
    // Feature vector associate features with nodes in the 4th level (from leaves up)
    // We assume the vocabulary tree has 6 levels, change the 4 otherwise
    mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);

    //find mappoints

    mvpMapPoints.resize(N,nullptr);
    mvbMapPointsLock.resize(N,false);

    for(int idx=0;idx<pMsg->mvpMapPoints_Ids.size();++idx)
    {
        size_t FeatId = pMsg->mvpMapPoints_VectId[idx];

        mpptr pMP = mpMap->GetMpPtr(pMsg->mvpMapPoints_Ids[idx],pMsg->mvpMapPoints_ClientIds[idx]);

        if(pMP)
        {
            mvpMapPoints[FeatId] = pMP;
        }
        else
        {
            //if MP not in Map, we ignore it. Might be deleted, or comes in later and is then (hopefully) added to this KF
        }
    }

    Tcw = cv::Mat(4,4,5);
    Twc = cv::Mat(4,4,5);
    Ow = cv::Mat(3,1,5);
    Cw = cv::Mat(4,1,5);
    mTcp = cv::Mat(4,4,5);

    if(mSysState == eSystemState::CLIENT)
    {
        // CLIENT

        bool bSetPose = this->SetPoseFromMessage(pMsg,mg2oS_wcurmap_wclientmap);

        if(!bSetPose)
        {
            mvpMapPoints.clear();
            mbBad = true;
            return;
        }
    }
    else if(mSysState == eSystemState::SERVER)
    {
        // SERVER

        cv::Mat T_SC = cv::Mat(4,4,5);
        Converter::MsgArrayFixedSizeToCvMat<ccmslam_msgs::KF::_mT_SC_type,float>(T_SC,pMsg->mT_SC);
        mT_SC = Converter::toMatrix4d(T_SC);

        if(this->mId.first != 0)
        {
            bool bSetPose = this->SetPoseFromMessage(pMsg,mg2oS_wcurmap_wclientmap);

            if(!bSetPose)
            {
                mvpMapPoints.clear();
                mbBad = true;
                return;
            }
        }
        else
        {
            //first KF has no parent

            cv::Mat tempTcw = cv::Mat(4,4,5);
            Converter::MsgArrayFixedSizeToCvMat<ccmslam_msgs::KF::_mTcpred_type,float>(tempTcw,pMsg->mTcpred);

            float s = static_cast<float>(mg2oS_wcurmap_wclientmap.scale());
            tempTcw.at<float>(0,3) /=(1./s);
            tempTcw.at<float>(1,3) /=(1./s);
            tempTcw.at<float>(2,3) /=(1./s);

            this->SetPose(tempTcw,false);
        }
    }
}

bool KeyFrame::SetPoseFromMessage(ccmslam_msgs::KF *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap)
{
    ccmslam_msgs::KFred *pMsgRed = new ccmslam_msgs::KFred();

    this->ReduceMessage(pMsg,pMsgRed);

    bool bReturn = this->SetPoseFromMessage(pMsgRed,mg2oS_wcurmap_wclientmap);

    delete pMsgRed;

    return bReturn;
}

bool KeyFrame::SetPoseFromMessage(ccmslam_msgs::KFred *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap)
{
    if(mSysState == eSystemState::CLIENT)
    {
        // CLIENT

        idpair MsgRefId = make_pair(pMsg->mpPred_KfId,pMsg->mpPred_KfClientId);

        cv::Mat tempTcw = cv::Mat(4,4,5);

        if(MsgRefId == this->mId)
        {
            Converter::MsgArrayFixedSizeToCvMat<ccmslam_msgs::KF::_mTcpred_type,float>(tempTcw,pMsg->mTcpred);
        }
        else
        {
            kfptr pPred = mpMap->GetKfPtr(MsgRefId);
            if(!pPred || pPred->isBad())
            {
                return false;
            }

            cv::Mat Tpw = pPred->GetPose();

            cv::Mat Tcp = cv::Mat(4,4,5);
            Converter::MsgArrayFixedSizeToCvMat<ccmslam_msgs::KF::_mTcpred_type,float>(Tcp,pMsg->mTcpred);

            tempTcw = Tcp * Tpw;
        }

        this->SetPose(tempTcw,false,true);
    }
    else if(mSysState == eSystemState::SERVER)
    {
        // SERVER

        if(this->mId.first != 0)
        {
            cv::Mat tempTcp = cv::Mat(4,4,5);

            kfptr pRef = mpMap->GetKfPtr(pMsg->mpPred_KfId,pMsg->mpPred_KfClientId);

            if(pRef)
            {
                Converter::MsgArrayFixedSizeToCvMat<ccmslam_msgs::KF::_mTcpred_type,float>(tempTcp,pMsg->mTcpred);
            }

            if(!pRef)
            {
                pRef = mpMap->GetKfPtr(pMsg->mpPar_KfId,pMsg->mpPar_KfClientId);

                if(pRef)
                {
                    Converter::MsgArrayFixedSizeToCvMat<ccmslam_msgs::KF::_mTcpar_type,float>(tempTcp,pMsg->mTcpar);
                }
            }

            if(!pRef)
            {
                //check -- maybe we can use erased pointer
                pRef = mpMap->GetErasedKfPtr(pMsg->mpPred_KfId,pMsg->mpPred_KfClientId);

                if(pRef)
                {
                    Converter::MsgArrayFixedSizeToCvMat<ccmslam_msgs::KF::_mTcpred_type,float>(tempTcp,pMsg->mTcpred);
                }
            }

            if(!pRef)
            {
                //check -- maybe we can use erased pointer
                pRef = mpMap->GetErasedKfPtr(pMsg->mpPar_KfId,pMsg->mpPar_KfClientId);

                if(pRef)
                {
                    Converter::MsgArrayFixedSizeToCvMat<ccmslam_msgs::KF::_mTcpar_type,float>(tempTcp,pMsg->mTcpar);
                }
            }

            if(!pRef)
            {
                //there is no reference pointer available -- ignore this message
                return false;
            }

            float s = static_cast<float>(mg2oS_wcurmap_wclientmap.scale());
            s = 1/s; //warn
            tempTcp.at<float>(0,3) *=(1./s);
            tempTcp.at<float>(1,3) *=(1./s);
            tempTcp.at<float>(2,3) *=(1./s);

            cv::Mat tempTcw = cv::Mat(4,4,5);

            if(!pRef->isBad())
            {
                cv::Mat Tpw = pRef->GetPose();
                tempTcw = tempTcp * Tpw;
            }
            else
            {
                kfptr pRefRef = pRef->GetParent();

                if(!pRefRef)
                {
                    cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << ":"  << __LINE__ << " parent is bad or not existing" << endl;
                    cout << "this: " << this->mId.first << "|" << this->mId.second << endl;
                    cout << "pRef: " << pRef->mId.first << "|" << pRef->mId.second << endl;
                    cout << "!pRefRef" << endl;
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
                tempTcw = tempTcp * T_p_pp * T_pp_w;
            }

            this->SetPose(tempTcw,false,true);
        }
        else
        {
            //first KF has no parent

            cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << ":" << __LINE__ << " must not be called for KF 0" << endl;
            throw estd::infrastructure_ex();
        }
    }

    return true;
}

bool KeyFrame::CanBeForgotten()
{
    if(mId.first == 0)
        return false;
    unique_lock<mutex> lock(mMutexOut);

    if(mbSentOnce && mbAck && !mbInOutBuffer)
        return true;
    else
        return false;
}

void KeyFrame::RemapMapPointMatch(mpptr pMP, const size_t &idx_now, const size_t &idx_new)
{
    unique_lock<mutex> lock(mMutexFeatures);

    mvpMapPoints[idx_now] = nullptr;
    mvpMapPoints[idx_new] = pMP;

    pMP->RemapObservationId(shared_from_this(),idx_new);

    if(mvbMapPointsLock[idx_now])
    {
        mvbMapPointsLock[idx_new] = true;
        mvbMapPointsLock[idx_now] = false;
    }
}

std::string KeyFrame::GetId()
{
    std::stringstream kfid;
    kfid << mId.first << "|" << mId.second;
    return kfid.str();
}

} //end ns;
