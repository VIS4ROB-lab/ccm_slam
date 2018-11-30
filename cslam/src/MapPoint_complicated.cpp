#include <cslam/MapPoint.h>

namespace cslam {

long unsigned int MapPoint::nNextId=0;
mutex MapPoint::mGlobalMutex;

MapPoint::MapPoint(const cv::Mat &Pos, kfptr pRefKF, mapptr pMap, size_t ClientId, commptr pComm, eSystemState SysState, size_t UniqueId)
    : mFirstKfId(pRefKF->mId), mFirstFrame(pRefKF->mFrameId), mUniqueId(UniqueId),
      nObs(0), mTrackReferenceForFrame(defpair),mLastFrameSeen(defpair),
      mpReplaced(nullptr), mfMinDistance(0), mfMaxDistance(0),
      mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
      mpMap(pMap),
//      mpComm(pComm),
      mbIsEmpty(false), mbPoseLock(false),mbPoseChanged(false), mbSentOnce(false),mbInOutBuffer(false),mbDescriptorChanged(false),mbNormalAndDepthChanged(false),
      mLoopPointForKF_LC(defpair), mCorrectedByKF_LC(defpair),mCorrectedReference_LC(-1),
      mLoopPointForKF_MM(defpair), mCorrectedByKF_MM(defpair),mCorrectedReference_MM(-1),
      mBAGlobalForKF(defpair),mBALocalForKF(defpair),
      mFuseCandidateForKF(defpair),
      mSysState(SysState),mbDoNotReplace(false),mbOmitSending(false),
      mbChangedByServer(false),mbLoopCorrected(false),mbMultiUse(false),
      mPubCount(0)
//      mbDescriptorSet(false),mbNormalAndDepthSet(false)
{
    mspComm.insert(pComm);

    Pos.copyTo(mWorldPos);
    mRefPos = mpRefKF->GetPose() * mWorldPos;
    mNormalVector = cv::Mat::zeros(3,1,CV_32F);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
//    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    while(!mpMap->LockPointCreation()){usleep(mpMap->GetLockSleep());}
    mId=make_pair(nNextId++,ClientId);
    mpMap->UnLockPointCreation();

    if(mUniqueId == 0 && !mId.first == 0) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m MapPoint::MapPoint(..): mUniqueId not set" << endl;
}

//MapPoint::MapPoint(const cv::Mat &Pos, mapptr pMap, frameptr pFrame, const int &idxF, commptr pComm, eSystemState SysState, size_t UniqueId)
//    : mFirstKfId(defpair),mFirstFrame(pFrame->mId), mUniqueId(UniqueId),
//      nObs(0), mTrackReferenceForFrame(defpair), mLastFrameSeen(defpair),
//      mpReplaced(nullptr), mfMinDistance(0), mfMaxDistance(0),
//      mpRefKF(nullptr), mnVisible(1), mnFound(1), mbBad(false),
//      mpMap(pMap),
////      mpComm(pComm),
//      mbIsEmpty(false), mbPoseLock(false),mbPoseChanged(false), mbSentOnce(false),mbInOutBuffer(false),mbDescriptorChanged(false),mbNormalAndDepthChanged(false),
//      mLoopPointForKF_LC(defpair), mCorrectedByKF_LC(defpair),mCorrectedReference_LC(-1),
//      mLoopPointForKF_MM(defpair), mCorrectedByKF_MM(defpair),mCorrectedReference_MM(-1),
//      mBAGlobalForKF(defpair),mBALocalForKF(defpair),
//      mFuseCandidateForKF(defpair),
//      mSysState(SysState),mbDoNotReplace(false),mbOmitSending(false),
//      mbChangedByServer(false),mbLoopCorrected(false),mbMultiUse(false)
////      mbDescriptorSet(false),mbNormalAndDepthSet(false)
//{
//    mspComm.insert(pComm);

//    Pos.copyTo(mWorldPos);
//    cv::Mat Ow = pFrame->GetCameraCenter();
//    mNormalVector = mWorldPos - Ow;
//    mNormalVector = mNormalVector/cv::norm(mNormalVector);

//    cv::Mat PC = Pos - Ow;
//    const float dist = cv::norm(PC);
//    //cout << "MapPoint() -> dist: " << dist << endl;
//    const int level = pFrame->mvKeysUn[idxF].octave;
//    //cout << "MapPoint() -> level: " << level << endl;
//    const float levelScaleFactor =  pFrame->mvScaleFactors[level];
//    const int nLevels = pFrame->mnScaleLevels;

//    mfMaxDistance = dist*levelScaleFactor;
//    mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];

//    pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

//    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
////    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
//    while(!mpMap->LockPointCreation()){usleep(mpMap->GetLockSleep());}
//    mId=make_pair(nNextId++,pFrame->mId.second);
//    mpMap->UnLockPointCreation();

//    if(mUniqueId == 0 && !mId.first == 0) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m MapPoint::MapPoint(..): mUniqueId not set" << endl;
//}

MapPoint::MapPoint(bool NonUsedValue, mapptr pMap, eSystemState SysState) //the "NonUsedValue" is for safety, to not accidentally construct emtpy objects
    : mpMap(pMap),
//      mpComm(nullptr),
      mId(defpair), mFirstKfId(defpair), mUniqueId(defid),
      mbIsEmpty(true),mbBad(true), mbPoseLock(false), nObs(0),
      mnFound(0),mnVisible(0),
      mSysState(SysState),mbDoNotReplace(false),mbOmitSending(false),
      mbChangedByServer(false),mbLoopCorrected(false),mbMultiUse(false),
      mPubCount(0)
{
    //pass map to MP to be able to erase itself from MP buffer
}

MapPoint& MapPoint::operator=(MapPoint&& rhs)
{
      mpMap = rhs.mpMap;
//      mpComm = rhs.mpComm;
      mspComm = rhs.mspComm;
      mbSentOnce = move(rhs.mbSentOnce); mId = move(rhs.mId); mFirstKfId = move(rhs.mFirstKfId);mUniqueId = move(rhs.mUniqueId),
      mWorldPos = move(rhs.mWorldPos);mObservations = rhs.mObservations;mNormalVector = move(rhs.mNormalVector);mDescriptor = move(rhs.mDescriptor);
      mpRefKF = rhs.mpRefKF;mpReplaced = rhs.mpReplaced;mnVisible = move(rhs.mnVisible);mnFound = move(rhs.mnFound);mbBad = move(rhs.mbBad);mfMinDistance = move(rhs.mfMinDistance);mfMaxDistance = move(rhs.mfMaxDistance);
      mbIsEmpty = move(rhs.mbIsEmpty);
      mbPoseLock = move(rhs.mbPoseLock);
      mObservationsLock = rhs.mObservationsLock;
      nObs = move(rhs.nObs);

      mLoopPointForKF_LC=move(rhs.mLoopPointForKF_LC);
      mCorrectedByKF_LC=move(rhs.mCorrectedByKF_LC);
      mCorrectedReference_LC=move(rhs.mCorrectedReference_LC);
//      mBAGlobalForKF_LC=move(rhs.mBAGlobalForKF_LC);

      mLoopPointForKF_MM=move(rhs.mLoopPointForKF_MM);
      mCorrectedByKF_MM=move(rhs.mCorrectedByKF_MM);
      mCorrectedReference_MM=move(rhs.mCorrectedReference_MM);

//      mBAGlobalForKF_MM=move(rhs.mBAGlobalForKF_MM);
      mBAGlobalForKF=move(rhs.mBAGlobalForKF);
//      mnLoopPointForKFClientId = move(rhs.mnLoopPointForKFClientId);
//      mnCorrectedByKFClientId = move(rhs.mnCorrectedByKFClientId);
//      mnCorrectedReferenceClientId = move(rhs.mnCorrectedReferenceClientId);
      mSysState = rhs.mSysState;
      mbDoNotReplace = move(rhs.mbDoNotReplace);
      mbOmitSending = move(rhs.mbOmitSending);

//      mbDescriptorSet = move(rhs.mbDescriptorSet);
//      mbNormalAndDepthSet = move(rhs.mbNormalAndDepthSet);

      mbInOutBuffer = move(rhs.mbInOutBuffer);
      mbChangedByServer = move(rhs.mbChangedByServer);
      mbLoopCorrected = move(rhs.mbLoopCorrected);
      mbMultiUse = move(rhs.mbMultiUse);
      mPubCount = move(rhs.mPubCount);
}

MapPoint::MapPoint(cslam_msgs::MapPoint* pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap, g2o::Sim3 mg2oS_loop, mapptr pMap, commptr pComm, eSystemState SysState, size_t UniqueId)
    : mId(make_pair(pMsg->mnId,pMsg->mClientId)), mFirstKfId(make_pair(pMsg->mnFirstKFid,pMsg->mnFirstKfClientId)), mUniqueId(UniqueId),
      nObs(0),mpReplaced(nullptr),
      mpMap(pMap),
//      mpComm(pComm),
      mbIsEmpty(false), mbPoseLock(false),mbPoseChanged(false), mbSentOnce(false),mbInOutBuffer(false),mbDescriptorChanged(false),mbNormalAndDepthChanged(false),
      mLoopPointForKF_LC(defpair), mCorrectedByKF_LC(defpair),mCorrectedReference_LC(defid),
      mLoopPointForKF_MM(defpair), mCorrectedByKF_MM(defpair),mCorrectedReference_MM(defid),
      mBAGlobalForKF(defpair),mBALocalForKF(defpair),
      mFuseCandidateForKF(defpair),
      mSysState(SysState),mbDoNotReplace(false),mbOmitSending(false),
      mbChangedByServer(false),mbLoopCorrected(false),mbMultiUse(false),
      mbBad(false),mPubCount(0)
//      mbDescriptorSet(true),mbNormalAndDepthSet(true)
{
    //constructor for server input

    mspComm.insert(pComm);

    mbOmitSending = true;

    if(pMsg->mbBad)
    {
         cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << "Incoming KF message: mbBad == true" << endl;
         throw infrastructure_ex();
    }

    //set KF connections

    for(int idx=0;idx<pMsg->mObservations_KFIDs.size();++idx)
    {
        kfptr pKFi = mpMap->GetKfPtr(pMsg->mObservations_KFIDs[idx],pMsg->mObservations_KFClientIDs[idx]);

        if(pKFi && !pKFi->isBad()) //theoretically, it can not be bad, otherwise it would not be in the map...
        {
            mObservations[pKFi]=pMsg->mObservations_n[idx];
            ++nObs;
        }
    }

    if(nObs == 0)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << ": no Observations" << endl;
        //Check: Better solution for this
        mbBad = true;
        return;
    }

    //set pos

    mWorldPos = cv::Mat(3,1,5);
    mRefPos = cv::Mat(3,1,5);

    cv::Mat P3D_ref = cv::Mat(3,1,5); //world client

    Converter::MsgArrayFixedSizeToCvMat<cslam_msgs::MapPoint::_mRefPos_type,float>(P3D_ref,pMsg->mRefPos);

    float s = static_cast<double>(mg2oS_wcurmap_wclientmap.inverse().scale());

    P3D_ref *=(1./s);

    //get parent -- might be valid, but could also be NULLPTR, e.g. if deleted

    mpRefKF = mpMap->GetMpPtr(pMsg->mpRefKFId,pMsg->mpRefKFClientId);

    //if parent does not exist (e.g. deleted), get another parent

    if(!mpRefKF)
    {
        cv::Mat T_cref_cmsg;
        mpRefKF = mpMap->HandleMissingParent(pMsg->mpRefKFId,pMsg->mpRefKFClientId,T_cref_cmsg,mpRefKF);

        cv::Mat R_cr_co = T_cref_cmsg.rowRange(0,3).colRange(0,3);
        cv::Mat t_cr_co = T_cref_cmsg.rowRange(0,3).col(3);

        P3D_ref = R_cr_co * P3D_ref + t_cr_co;
    }

    if(!mpRefKF)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << "no mpRefKF" << endl;
        throw estd::infrastructure_ex();
    }

    this->SetWorldPosFromRef(P3D_ref,false);;

    if(!mObservations.count(mpRefKF))
    {
        //mpRefKf should be observing the MP

        mpRefKF = mObservations.begin()->first; //should not be bad, we checked taht when we added the KF

//        mpRefKF = nullptr;
//        std::map<kfptr,size_t>::const_iterator mitRef = mObservations.begin();
//        while(!mpRefKF)
//        {
//            if(mitRef == mObservations.end()) break;
//            if(!(mitRef->first->isBad()))
//                mpRefKF=mitRef->first;
//            else
//                ++mitRef;
//        }
    }

    if(mpRefKF->isBad())
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << ": mpRefKF is BAD" << endl;
    }

    mNormalVector = cv::Mat(3,1,5);
    Converter::MsgArrayFixedSizeToCvMat<macslam_msgs::macMapPoint::_mNormalVector_type,float>(mNormalVector,pMsg->mNormalVector);
    mDescriptor = cv::Mat(1,32,0);
    Converter::MsgArrayFixedSizeToCvMat<macslam_msgs::macMapPoint::_mDescriptor_type,uint8_t>(mDescriptor,pMsg->mDescriptor);

    mnFound = nObs;
    mnVisible = nObs;

    mbMultiUse = pMsg->mbMultiUse;

    mbSentOnce = true; //set at the end of constructor to prevent SendMe() from sending while using constructor

    mbOmitSending = false;

    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // Call EstablishInitialConnectionsServer()
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
}

void MapPoint::EstablishInitialConnectionsServer()
{
    //this is necessary, because we cannot use shared_from_this() in constructor

    mbOmitSending = true;

    for(map<kfptr,size_t>::iterator mit = mObservations.begin();mit != mObservations.end(); ++mit)
    {
        kfptr pKFi = mit->first;
        size_t idx = mit->second;

        if(pKFi)
        {
            if(pKFi->IsMpLocked(idx))
            {
                //ToDO: can maybe merge MPs in this case

                if(shared_from_this()->mId == pKFi->GetMapPoint(idx)->mId)
                {
                    //already there

                    cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << "MP arrived first & already associated to KF" << endl;
                    throw infrastructure_ex();
                }
                else
                {
                    //delete KF association
                    mObservations.erase(mit);
                    nObs--;

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
                }
            }
            else
            {
                if(shared_from_this()->mId == pKFi->GetMapPoint(idx)->mId)
                {
                    //already there

                    cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << "MP arrived first & already associated to KF" << endl;
                    throw infrastructure_ex();
                }

                pKFi->mbOmitSending = true;
                pKFi->AddMapPoint(shared_from_this(),pMsg->mObservations_n[idx],false);
                pKFi->mbOmitSending = false;
            }
        }
        else
        {
            cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << " pKFi is NULLPTR" << endl;
        }
    }

//    this->ComputeDistinctiveDescriptors(); //descriptor comes with first message
    this->UpdateNormalAndDepth();

    if(nObs == 0)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << ": no Observations" << endl;
        //Check: Better solution for this
        mbBad = true;
        return;
    }

    if(!mpRefKF)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << "no mpRefKF" << endl;
        throw estd::infrastructure_ex();
    }

    //ToDo: Check for more KFs seeing this MP

    mbOmitSending = false;
}


MapPoint::MapPoint(cslam_msgs::MapPoint* pMsg, mapptr pMap, commptr pComm, eSystemState SysState, size_t UniqueId)
    : mId(make_pair(pMsg->mnId,pMsg->mClientId)), mFirstKfId(make_pair(pMsg->mnFirstKFid,pMsg->mnFirstKfClientId)), mUniqueId(UniqueId),
      nObs(0),mpReplaced(nullptr),
      mpMap(pMap),
//      mpComm(pComm),
      mbIsEmpty(false), mbPoseLock(false),mbPoseChanged(false), mbSentOnce(false),mbInOutBuffer(false),mbDescriptorChanged(false),mbNormalAndDepthChanged(false),
      mLoopPointForKF_LC(defpair), mCorrectedByKF_LC(defpair),mCorrectedReference_LC(defid),
      mLoopPointForKF_MM(defpair), mCorrectedByKF_MM(defpair),mCorrectedReference_MM(defid),
      mBAGlobalForKF(defpair),mBALocalForKF(defpair),
      mFuseCandidateForKF(defpair),
      mSysState(SysState),mbDoNotReplace(false),mbOmitSending(false),
      mbChangedByServer(false),mbLoopCorrected(false),mbMultiUse(false)
//      mbDescriptorSet(true),mbNormalAndDepthSet(true)
{
    //constructor for client

    mspComm.insert(pComm);

    mbOmitSending = true;

    mbBad = pMsg->mbBad;

    mWorldPos = cv::Mat(3,1,5);
    mRefPos = cv::Mat(3,1,5);

    cv::Mat P3D_wc = cv::Mat(3,1,5); //world client
    cv::Mat P3D_ref = cv::Mat(3,1,5); //world client

    if(mSysState == 0)
    {
        Converter::MsgArrayFixedSizeToCvMat<cslam_msgs::MapPoint::_mWorldPos_type,float>(P3D_wc,pMsg->mWorldPos);
    }
    else if(mSysState == 1)
    {
        Converter::MsgArrayFixedSizeToCvMat<cslam_msgs::MapPoint::_mRefPos_type,float>(P3D_wc,pMsg->mRefPos);

        mpRefKF = mpMap->GetMpPtr(pMsg->mpRefKFId,pMsg->mpRefKFClientId);

        if(!mpRefKF)
        {
            mpRefKF = mpMap->GetParentReplacement(pMsg->mpRefKFId,pMsg->mpRefKFClientId,T_pold_pnew);

            cv::Mat T_pold_pnew;
            mpMap->GetTfToParent(pMsg->mpRefKFId,pMsg->mpRefKFClientId,T_pold_pnew,mpRefKF);

            cv::Mat Rno = T_pold_pnew.inv().rowRange(0,3).colRange(0,3);
            cv::Mat tno = T_pold_pnew.inv().rowRange(0,3).col(3);

            P3D_ref = Rno * P3D_ref + tno;
        }

        if(!mpRefKF)
        {
            cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << "no mpRefKF" << endl;
            throw estd::infrastructure_ex();
        }
    }

    float s = static_cast<double>(mg2oS_wcurmap_wclientmap.inverse().scale());

    P3D_wc *=(1./s);
    P3D_ref *=(1./s);

    if(mSysState == 0)
    {
        this->SetWorldPos(P3D_wc,false);;
    }
    else if(mSysState == 1)
    {
        this->SetWorldPosFromRef(P3D_ref,false);;
    }

    mNormalVector = cv::Mat(3,1,5);
    Converter::MsgArrayFixedSizeToCvMat<macslam_msgs::macMapPoint::_mNormalVector_type,float>(mNormalVector,pMsg->mNormalVector);
    mDescriptor = cv::Mat(1,32,0);
    Converter::MsgArrayFixedSizeToCvMat<macslam_msgs::macMapPoint::_mDescriptor_type,uint8_t>(mDescriptor,pMsg->mDescriptor);

    //ToDo: create connections

    if(mSysState == 0)
    {
        //ToDo: assign mpRefKf

        if(!mpRefKF)
        {
            cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << "no mpRefKF" << endl;
            throw estd::infrastructure_ex();
        }
    }

    if(mSysState == eSystemState::CLIENT) mbDoNotReplace = true;
    //ToDo: maybe remove this

//    mnVisible = pMsg->mnVisible;
//    mnFound = pMsg->mnFound;

    mfMinDistance = pMsg->mfMinDistance;
    mfMaxDistance = pMsg->mfMaxDistance;

    mbMultiUse = pMsg->mbMultiUse;

    mbSentOnce = true; //set at the end of constructor to prevent SendMe() from sending while using constructor

    mbOmitSending = false;
}

void MapPoint::SetWorldPos(const cv::Mat &Pos, bool bLock)
{
    if(mbPoseLock && mSysState == eSystemState::CLIENT)
        return;

    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    Pos.copyTo(mWorldPos);

    if(mpRefKF)
    {
        T_cref_w =  mpRefKF->GetPose();

        cv::Mat Rcw = T_cref_w.rowRange(0,3).colRange(0,3);
        cv::Mat tcw = T_cref_w.rowRange(0,3).col(3);

        mRefPos = Rcw * mWorldPos + tcw;
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

    if(!mbOmitSending && mbSentOnce)
    {
        mbPoseChanged = true;
        SendMe();
    }
}

void MapPoint::SetWorldPosFromRef(const Mat &Pos, bool bLock)
{
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);

    Pos.copyTo(mRefPos);

    if(mpRefKF)
    {
        T_w_cref =  mpRefKF->GetPoseInverse();

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

void MapPoint::AddObservation(kfptr pKF, size_t idx, bool bLock)
{
    if(this->IsEmpty())
    {
        cout << "\033[1;33m!!! WARN !!!\033[0m Client " << mId.second << ": In \"MapPointBase::AddObservation(...)\": trying to add observation to EMPTY MP" << endl;
    }
    else
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF))
            return;

        if(mObservationsLock.count(pKF) && mSysState == eSystemState::CLIENT)
            return;

        mObservations[pKF]=idx;

        nObs++;

        if(bLock)
        {
            mObservationsLock[pKF] = true;
        }

//        if(mbSentOnce && !mbOmitSending)
//        {
//            if(!pKF)
//            {
//                cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m MapPoint::AddObservation(...): adding nullptr" << endl;
//            }
//            else
//            {
//                mvpNewObs.push_back(pKF);
//                mvpNewObsIds.push_back(idx);
//                mvbNewObsErase.push_back(false);
//            }
//        }

//        SendMe();
    }
}

void MapPoint::EraseObservation(kfptr pKF, bool bLock)
{
    if(this->IsEmpty())
    {
//        cout << "\033[1;33m!!! WARN !!!\033[0m Client " << mClientId << ": In \"MapPointBase::EraseObservation(...)\": trying to erase observation to EMPTY MP" << endl;
        //ignore -- MP will be overwritten when non-empty constructor ist called
    }
    else
    {
//        cout << "MapPoint::EraseObservation --> mpRefKF->mnId: " << mpRefKF->mnId << endl;
//        cout << "MapPoint::EraseObservation --> pKF->mnId: " << pKF->mnId << endl;
        if(mObservationsLock.count(pKF) && mSysState == eSystemState::CLIENT)
            return;

        bool bBad=false;
        {
            unique_lock<mutex> lock(mMutexFeatures);
            if(mObservations.count(pKF))
            {
                int idx = mObservations[pKF];

                    nObs--;

                mObservations.erase(pKF);

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
//                        mpRefKF=mObservations.begin()->first;
                    else
                        mpRefKF=nullptr;
                }

//                if(mpRefKF->isBad() && !mpRefKF->IsEmpty()) cout << "\033[1;33m!!! WARN !!!\033[0m Client " << mClientId << ": In \"MapPointBase::EraseObservation(...)\": mpRefKf is bad" << endl;

                // If only 2 observations or less, discard point
                if(nObs<=2)
                    bBad=true;
            }
        }

        if(bBad)
            SetBadFlag();

//        if(mbSentOnce && !mbOmitSending)
//        {
//            mvpNewObs.push_back(pKF);
//            mvpNewObsIds.push_back(defid);
//            mvbNewObsErase.push_back(true);
//            SendMe();
//        }



        if(mpRefKF && mpRefKF->isBad() && !mbBad)
        {
            cout << "\033[1;33m!!! WARN !!!\033[0m Client " << mId.second << ": In \"MapPointBase::EraseObservation(...)\": mpRefKf is bad (may be empty)" << endl;
            cout << "MP: " << this->mId.first << endl;
            cout << "mpRefKF: " << mpRefKF->mId.first << "|" << mpRefKF->mId.second;
        }

        if(!mpRefKF)
        {
            cout << "\033[1;33m!!! WARN !!!\033[0m Client " << mId.second << ": In \"MapPointBase::EraseObservation(...)\": no mpRefKf available - workaround: set MP bad" << endl;
            cout << "mbBad true? " << (mbBad == true) << endl;
            if(!mbBad) SetBadFlag();
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

void MapPoint::SetBadFlag()
{
//    cout << " Client " << mClientId << ": \"MapPointBase::SetBadFlag(...)\" MP #" << mnId << endl;

    unique_lock<mutex> lockMap(mMapMutex);

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
        if(pKF->IsEmpty()) continue;
        pKF->EraseMapPointMatch(mit->second);
    }

//    if(this->IsEmpty())
//    {
//        mpMap->DeleteFromMpBuffer(this->shared_from_this());  //tempout
//        mpMap->EraseMapPoint(this->shared_from_this());
//    }
//    else
        mpMap->EraseMapPoint(this->shared_from_this());

//    if(!mbOmitSending && mbSentOnce)
//    {
//        SendMe();
//    }
}

MapPoint::mpptr MapPoint::GetReplaced()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mpReplaced;
}

void MapPoint::Replace(mpptr pMP)
{
//    if(pMP->mnId==this->mnId && pMP->mClientId==this->mClientId) //ID-Tag
    if(pMP->mId==this->mId) //ID-Tag
        return;

    unique_lock<mutex> lockMap(mMapMutex);

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
            pKF->ReplaceMapPointMatch(mit->second, pMP);
            pMP->AddObservation(pKF,mit->second);
        }
        else
        {
            pKF->EraseMapPointMatch(mit->second);
        }
    }
    pMP->IncreaseFound(nfound);
    pMP->IncreaseVisible(nvisible);
    pMP->ComputeDistinctiveDescriptors();

//    if(this->IsEmpty())
//    {
//        mpMap->DeleteFromMpBuffer(this->shared_from_this()); //tempout
//        mpMap->EraseMapPoint(this->shared_from_this());
//    }
//    else
        mpMap->EraseMapPoint(this->shared_from_this());

//    if(!mbOmitSending && mbSentOnce)
//    {
//        SendMe();
//    }
}

void MapPoint::ReplaceAndLock(mpptr pMP)
{
//    if(pMP->mnId==this->mnId && pMP->mClientId==this->mClientId) //ID-Tag
    if(pMP->mId==this->mId) //ID-Tag
        return;

    unique_lock<mutex> lockMap(mMapMutex);

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

//    if(this->IsEmpty())
//    {
//        mpMap->DeleteFromMpBuffer(this->shared_from_this());  //tempout
//        mpMap->EraseMapPoint(this->shared_from_this());
//    }
//    else
        mpMap->EraseMapPoint(this->shared_from_this());

//    SendMe();
}

void MapPoint::IncreaseVisible(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnVisible+=n;

    if(mbSentOnce)
    {
        mnVisibleAdded += n;
    }

//    if(!mbOmitSending && mbSentOnce)
//    {
//        SendMe();
//    }
}

void MapPoint::IncreaseFound(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnFound+=n;

//    if(mbSentOnce)
//    {
//        mnFoundAdded += n;
//    }

//    if(!mbOmitSending && mbSentOnce)
//    {
//        SendMe();
//    }
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

int MapPoint::GetIndexInKeyFrame(kfptr pKF)
{
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

    if(!mpRefKF) cout <<  "Client " << mId.second << " MP " << mId.first << ": \033[1;35m!!!!! HAZARD !!!!!\033[0m MapPointBase::UpdateNormalAndDepth(...): mpRefKf is nullptr" << endl;

    if(pRefKF->IsEmpty()) return;

    if(observations.empty())
        return;

    cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
    int n=0;
    for(map<kfptr,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        kfptr pKF = mit->first;

        if(pKF->isBad()) continue; //safety first, could also be empty...

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

//    mbNormalAndDepthSet = true;
    mbNormalAndDepthChanged = true;

//    if(!mbOmitSending && mbSentOnce)
//    {
//        SendMe();
//    }
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

int MapPoint::PredictScale(const float &currentDist, const float &logScaleFactor)
{
    float ratio;
    {
        unique_lock<mutex> lock3(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    return ceil(log(ratio)/logScaleFactor);
}

void MapPoint::SendMe()
{
////    if(!mbDescriptorSet || !mbNormalAndDepthSet) return;
////    if(mbSentOnce && !this->IsInOutBuffer()) mpComm->PassMptoComm(this->shared_from_this());

//    if(mspComm.empty())
//    {
//        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m MapPoint::SendMe(): no Comm ptrs" << endl;
////        throw infrastructure_ex();
//        cout << "bad?: " << (mbBad == true) << endl;
//        cout << "empty?: " << (mbIsEmpty == true) << endl;
//        return;
//    }

////    cout << "MapPoint::SendMe(): #commptrs: " << mspComm.size() << endl;

//    if(mbSentOnce && !this->IsInOutBuffer())
//    {
//        for(set<commptr>::const_iterator sit = mspComm.begin();sit!=mspComm.end();++sit)
//        {
//            commptr pComm = *sit;
//            pComm->PassMptoComm(this->shared_from_this());
//        }
//    }
}

void MapPoint::SendMeMultiUse()
{
//    cout << __func__ << __LINE__ << endl;

//    if(mspComm.empty())
//    {
//        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m MapPoint::SendMe(): no Comm ptrs" << endl;
////        throw infrastructure_ex();
//        cout << "bad?: " << (mbBad == true) << endl;
//        cout << "empty?: " << (mbIsEmpty == true) << endl;
//        return;
//    }

////    cout << "MapPoint::SendMe(): #commptrs: " << mspComm.size() << endl;

//    cout << "mbSentOnce: " << mbSentOnce << endl;
//    cout << "this->IsInOutBuffer(): " << this->IsInOutBuffer() << endl;

//    if(mbSentOnce && !this->IsInOutBuffer())
//    {
//        for(set<commptr>::const_iterator sit = mspComm.begin();sit!=mspComm.end();++sit)
//        {
//            commptr pComm = *sit;
//            cout << "pass to comm" << endl;
//            pComm->PassMptoComm(this->shared_from_this());
//        }
//    }
}

void MapPoint::ShowMyValues()
{
    cout << "******* MP VALUES *******" << endl;

    cout << "mnId: " << mId.first << endl;
    cout << "mClientId: " << mId.second << endl;
    cout << "mnFirstKFid: " << mFirstKfId.first << endl;
    cout << "mnFirstKfClientId: " << mFirstKfId.second << endl;
    cout << "mbBad: " << mbBad << endl;
    cout << "mbSentOnce: " << mbSentOnce << endl;

    cout << "mfMinDistance: " << mfMinDistance << endl;
    cout << "mfMaxDistance: " << mfMaxDistance << endl;
    cout << "mnVisible: " << mnVisible << endl;
    cout << "mnFound: " << mnFound << endl;

    cout << "mWorldPos: rows|cols|type: " << mWorldPos.rows << "|" << mWorldPos.cols << "|" << mWorldPos.type() << endl;
    cout << "mWorldPos: " << mWorldPos << endl;

    cout << "mObservations.size(): " << mObservations.size() << endl;

    cout << "mNormalVector: rows|cols|type: " << mNormalVector.rows << "|" << mNormalVector.cols << "|" << mNormalVector.type() << endl;
    cout << "mNormalVector: " << mNormalVector << endl;

    cout << "mDescriptor: rows|cols|type: " << mDescriptor.rows << "|" << mDescriptor.cols << "|" << mDescriptor.type() << endl;
    if(mDescriptor.rows > 0) cout << "mDescriptor[0]: " << static_cast<int>(mDescriptor.at<uint8_t>(0)) << endl;
    if(mDescriptor.rows > 11) cout << "mDescriptor[11]: " <<static_cast<int>(mDescriptor.at<uint8_t>(11)) << endl;

    if(mpRefKF)
    {
        cout << "mpRefKF->mnId: " << mpRefKF->mId.first << endl;
        cout << "mpRefKF->mClientId: " << mpRefKF->mId.second << endl;
    }
    else
    {
        cout << "mpRefKF: " << -1 << endl;
    }

    if(mpReplaced)
    {
        cout << "mpReplaced->mnId: " << mpReplaced->mId.first << endl;
        cout << "mpReplaced->mClientId: " << mpReplaced->mId.second << endl;
    }
    else
    {
        cout << "mpReplaced: " << -1 << endl;
    }
}

void MapPoint::ShowMyObservations()
{
    std::stringstream* ss;
    ss = new stringstream;

    for(std::map<kfptr,size_t>::iterator mit = mObservations.begin();mit!=mObservations.end();++mit)
    {
        *ss << "(" << (mit->first)->mId.first << ";" << (mit->first)->mId.second << ")" << "|" << mit->second << " $ ";
    }

    cout << "MapPoint " << mId.first << "," << mId.second << ": " << ss->str() << endl;

    delete ss;
}

void MapPoint::ReplaceMap(mapptr pNewMap)
{
    unique_lock<mutex> lockMap(mMapMutex);
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
//        mbDescriptorSet = true;
//        mbDescriptorChanged = true;
    }

//    if(!mbOmitSending && mbSentOnce)
//    {
//        SendMe();
//    }
}

void MapPoint::ConvertToMessageClient(cslam_msgs::MapPoint &Msg)
{
    if(mSysState != eSystemState::CLIENT)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m System Type Mismatch" << endl;
        throw infrastructure_ex();
    }

    unique_lock<mutex> lock1(mMutexFeatures,defer_lock);
    unique_lock<mutex> lock2(mMutexPos,defer_lock);
    unique_lock<mutex> lock3(mMutexOut,defer_lock);

    lock(lock1,lock2,lock3);

    //check format
    {
        bool bCheckSize = (mWorldPos.rows != 3) || (mWorldPos.cols != 1) || (mNormalVector.rows != 3) || (mNormalVector.cols != 1) || (mDescriptor.rows != 1) || (mDescriptor.cols != 32);

        if(bCheckSize)
        {
            cout << "In \"MapPoint::ConvertToMessage(...)\": member matrix SIZE not as expected in ROS message" << endl;
            cout << "Id: " << mId.first << "|" << mId.second << endl;
            cout << "mWorldPos.rows: " << mWorldPos.rows << " -- expected: 3" << endl;
            cout << "mWorldPos.cols: " << mWorldPos.cols << " -- expected: 1" << endl;
            cout << "mNormalVector.rows: " << mNormalVector.rows << " -- expected: 3" << endl;
            cout << "mNormalVector.cols: " << mNormalVector.cols << " -- expected: 1" << endl;
            cout << "mDescriptor.rows: " << mDescriptor.rows << " -- expected: 1" << endl;
            cout << "mDescriptor.rows: " << mDescriptor.cols << " -- expected: 32" << endl;
            throw estd::infrastructure_ex();
        }

        bool bCheckTypes = (mDescriptor.type() != 0);

        if(bCheckTypes)
        {
            cout << "In \"KeyFrame::ConvertToMessage(...)\": member matrix TYPES not as expected in ROS message" << endl;
            throw estd::infrastructure_ex();
        }
    }

    Msg.mbBad = mbBad;
    Msg.mnId = mId.first;
    Msg.mClientId = mId.second;
    Msg.mUniqueId = mUniqueId;
    Msg.bSentOnce = mbSentOnce;

    Msg.mnFirstKFid = mFirstKfId.first;
    Msg.mnFirstKfClientId = mFirstKfId.second;

    Msg.mbPoseOnly = false;

    Msg.mbMultiUse = mbMultiUse;

    if(!mpRefKF)
    {
        cout << "\033[1;31m!!!!! FATAL ERROR !!!!!\033[0m " << __func__ << __LINE__ << ": MP has no parent" << endl;
        throw infrastructure_ex();
    }

    if(mbSentOnce)
    {
        if(mbPoseChanged)
        {
            Converter::CvMatToMsgArrayFixedSize<cslam_msgs::MapPoint::_mWorldPos_type,float>(mWorldPos,Msg.mWorldPos);

            cv::Mat Rcw = mpRefKF->GetPose().rowRange(0,3).colRange(0,3);
            cv::Mat tcw = mpRefKF->GetPose().rowRange(0,3).col(3);

            cv::Mat RefPos = Rcw * mWorldPos + tcw;

            Converter::CvMatToMsgArrayFixedSize<cslam_msgs::MapPoint::_mRefPos_type,float>(RefPos,Msg.mRefPos);

            Msg.mbPoseChanged = true;
            mbPoseChanged = false;
        }

//        for(std::map<kfptr,size_t>::const_iterator mit=mObservations.begin();mit!=mObservations.end();++mit)
//        {
//            if(!mit->first->isBad())
//            {
//                Msg.mObservations_KFIDs.push_back(mit->first->mId.first);
//                Msg.mObservations_KFClientIDs.push_back(mit->first->mId.second);
//                Msg.mObservations_n.push_back(mit->second);
//            }
//        }

        Msg.mpRefKFId = static_cast<int64>(mpRefKF->mId.first);
        Msg.mpRefKFClientId = static_cast<int64>(mpRefKF->mId.second);

//        Msg.mnVisible = mnVisibleAdded;
//        Msg.mnFound = mnFoundAdded;

//        mvpNewObs.clear();
//        mvpNewObsIds.clear();
//        mvbNewObsErase.clear();
//        mnVisibleAdded = 0;
//        mnFoundAdded = 0;

//        if(mbDescriptorChanged)
//        {
//            Converter::CvMatToMsgArrayFixedSize<macslam_msgs::macMapPoint::_mDescriptor_type,uint8_t>(mDescriptor,Msg.mDescriptor);
//            Msg.mbDescriptorChanged = true;
//            mbDescriptorChanged = false;
//        }

//        if(mbNormalAndDepthChanged)
//        {
//            Converter::CvMatToMsgArrayFixedSize<macslam_msgs::macMapPoint::_mNormalVector_type,float>(mNormalVector,Msg.mNormalVector);
//            Msg.mfMinDistance = mfMinDistance;
//            Msg.mfMaxDistance = mfMaxDistance;
//            Msg.mbNormalAndDepthChanged = true;
//            mbNormalAndDepthChanged = false;
//        }
    }
    else
    {
        //Pos
        Converter::CvMatToMsgArrayFixedSize<cslam_msgs::MapPoint::_mWorldPos_type,float>(mWorldPos,Msg.mWorldPos);

        cv::Mat Rcw = mpRefKF->GetPose().rowRange(0,3).colRange(0,3);
        cv::Mat tcw = mpRefKF->GetPose().rowRange(0,3).col(3);

        cv::Mat RefPos = Rcw * mWorldPos + tcw;

        Converter::CvMatToMsgArrayFixedSize<cslam_msgs::MapPoint::_mRefPos_type,float>(RefPos,Msg.mRefPos);

        Msg.mbPoseChanged = true;
        mbPoseChanged = false;

//        if(!mvpNewObs.empty() && mSysState==eSystemState::CLIENT) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m MapPoint::ConvertToMessage(...): mvpNewMapPoints not empty" << endl;

        Converter::CvMatToMsgArrayFixedSize<macslam_msgs::macMapPoint::_mNormalVector_type,float>(mNormalVector,Msg.mNormalVector);

        //Descriptor

        Converter::CvMatToMsgArrayFixedSize<macslam_msgs::macMapPoint::_mDescriptor_type,uint8_t>(mDescriptor,Msg.mDescriptor);

        //observation

        Msg.mpRefKFId = static_cast<int64>(mpRefKF->mId.first);
        Msg.mpRefKFClientId = static_cast<int64>(mpRefKF->mId.second);

        for(std::map<kfptr,size_t>::const_iterator mit=mObservations.begin();mit!=mObservations.end();++mit)
        {
            if(!mit->first->isBad())
            {
                Msg.mObservations_KFIDs.push_back(mit->first->mId.first);
                Msg.mObservations_KFClientIDs.push_back(mit->first->mId.second);
                Msg.mObservations_n.push_back(mit->second);
            }
        }

//        Msg.mnVisible = static_cast<int16_t>(mnVisible);
//        Msg.mnFound = static_cast<int16_t>(mnFound);


//        if(mpReplaced)
//        {
//            Msg.mpReplaced_Id = static_cast<int64>(mpReplaced->mId.first);
//            Msg.mpReplaced_ClientId = static_cast<int64>(mpReplaced->mId.second);
//        }
//        else
//        {
//            Msg.mpReplaced_Id = static_cast<int64>(-1);
//            Msg.mpReplaced_ClientId = static_cast<int64>(-1);
//        }

        Msg.mfMinDistance = mfMinDistance;
        Msg.mfMaxDistance = mfMaxDistance;

        mbSentOnce = true;

//        mvpNewObs.clear();
//        mvpNewObsIds.clear();
//        mvbNewObsErase.clear();
//        mnVisibleAdded = 0;
//        mnFoundAdded = 0;
//        mbPoseChanged = false;
    }

    ++mPubCount;
    mbInOutBuffer = false;
}

//void MapPoint::ConvertForeignPointToMessageClient(macslam_msgs::macMapPoint &Msg)
//{
//    if(mSysState != eSystemState::CLIENT)
//    {
//        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m System Type Mismatch" << endl;
//        throw infrastructure_ex();
//    }

//    unique_lock<mutex> lock1(mMutexFeatures,defer_lock);
//    unique_lock<mutex> lock2(mMutexPos,defer_lock);
//    unique_lock<mutex> lock3(mMutexOut,defer_lock);

//    lock(lock1,lock2,lock3);

//    Msg.mnId = mId.first;
//    Msg.mClientId = mId.second;

//    Msg.mbMultiUse = mbMultiUse;
//}

//void MapPoint::ConvertToMessageServer(macslam_msgs::macMapPoint &Msg, g2o::Sim3 mg2oS_wcurmap_wclientmap)
//{
//    if(mSysState != eSystemState::SERVER)
//    {
//        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m System Type Mismatch" << endl;
//        throw infrastructure_ex();
//    }

//    unique_lock<mutex> lock1(mMutexFeatures,defer_lock);
//    unique_lock<mutex> lock2(mMutexPos,defer_lock);
//    unique_lock<mutex> lock3(mMutexOut,defer_lock);

//    lock(lock1,lock2,lock3);

//    {
//        bool bCheckSize = (mWorldPos.rows != 3) || (mWorldPos.cols != 1) || (mNormalVector.rows != 3) || (mNormalVector.cols != 1) || (mDescriptor.rows != 1) || (mDescriptor.cols != 32);

//        if(bCheckSize)
//        {
//            cout << "In \"MapPoint::ConvertToMessage(...)\": member matrix SIZE not as expected in ROS message" << endl;
//            cout << "Id: " << mId.first << "|" << mId.second << endl;
//            cout << "mWorldPos.rows: " << mWorldPos.rows << " -- expected: 3" << endl;
//            cout << "mWorldPos.cols: " << mWorldPos.cols << " -- expected: 1" << endl;
//            cout << "mNormalVector.rows: " << mNormalVector.rows << " -- expected: 3" << endl;
//            cout << "mNormalVector.cols: " << mNormalVector.cols << " -- expected: 1" << endl;
//            cout << "mDescriptor.rows: " << mDescriptor.rows << " -- expected: 1" << endl;
//            cout << "mDescriptor.rows: " << mDescriptor.cols << " -- expected: 32" << endl;
//            throw estd::infrastructure_ex();
//        }

//        bool bCheckTypes = (mDescriptor.type() != 0);

//        if(bCheckTypes)
//        {
//            cout << "In \"KeyFrame::ConvertToMessage(...)\": member matrix TYPES not as expected in ROS message" << endl;
//            throw estd::infrastructure_ex();
//        }
//    }

//    Msg.bSentOnce = mbSentOnce;
//    Msg.mbBad = mbBad;

//    Msg.mnId = mId.first;
//    Msg.mClientId = mId.second;
//    Msg.mUniqueId = mUniqueId;
//    Msg.mnFirstKFid = mFirstKfId.first;
//    Msg.mnFirstKfClientId = mFirstKfId.second;

//    Msg.mbPoseOnly = false;

//    Msg.mbMultiUse = mbMultiUse;

//    Eigen::Matrix<double,3,1> eigP3D_wm = Converter::toVector3d(mWorldPos);
//    Eigen::Matrix<double,3,1> eigP3D_wc = (mg2oS_wcurmap_wclientmap.inverse()).map(eigP3D_wm); //client map

//    cv::Mat ClientPos = Converter::toCvMat(eigP3D_wc);

//    Converter::CvMatToMsgArrayFixedSize<macslam_msgs::macMapPoint::_mWorldPos_type,float>(ClientPos,Msg.mWorldPos);

//    Msg.mbPoseChanged = mbPoseChanged;


//    for(std::map<kfptr,size_t>::const_iterator mit=mObservations.begin();mit!=mObservations.end();++mit)
//    {
//        Msg.mObservations_KFIDs.push_back(mit->first->mId.first);
//        Msg.mObservations_KFClientIDs.push_back(mit->first->mId.second);
//        Msg.mObservations_n.push_back(mit->second);
//    }

//    Converter::CvMatToMsgArrayFixedSize<macslam_msgs::macMapPoint::_mNormalVector_type,float>(mNormalVector,Msg.mNormalVector);

//    Converter::CvMatToMsgArrayFixedSize<macslam_msgs::macMapPoint::_mDescriptor_type,uint8_t>(mDescriptor,Msg.mDescriptor);

//    if(mpRefKF)
//    {
//        Msg.mpRefKFId = static_cast<int64>(mpRefKF->mId.first);
//        Msg.mpRefKFClientId = static_cast<int64>(mpRefKF->mId.second);
//    }
//    else
//    {
//        cout <<  "Client " << mId.second << " MP " << mId.first << ": \033[1;35m!!!!! HAZARD !!!!!\033[0m MapPoint::ConvertToMessage(...): mpRefKf is nullptr" << endl;
//        Msg.mpRefKFId = static_cast<int64>(-1);
//        Msg.mpRefKFClientId = static_cast<int64>(-1);
//    }

//    Msg.mnVisible = static_cast<int16_t>(mnVisible);
//    Msg.mnFound = static_cast<int16_t>(mnFound);

//    Msg.mbBad = mbBad;

//    if(mpReplaced)
//    {
//        Msg.mpReplaced_Id = static_cast<int64>(mpReplaced->mId.first);
//        Msg.mpReplaced_ClientId = static_cast<int64>(mpReplaced->mId.second);
//    }
//    else
//    {
//        Msg.mpReplaced_Id = static_cast<int64>(-1);
//        Msg.mpReplaced_ClientId = static_cast<int64>(-1);
//    }

//    Msg.mfMinDistance = mfMinDistance;
//    Msg.mfMaxDistance = mfMaxDistance;

//    mbSentOnce = true;

//    mbPoseChanged = false;
//    mbInOutBuffer = false;
//}

//bool MapPoint::ConvertToPosMessageServer(macslam_msgs::macMapPoint &Msg, g2o::Sim3 mg2oS_wcurmap_wclientmap)
//{
//    if(mSysState != eSystemState::SERVER)
//    {
//        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m System Type Mismatch" << endl;
//        throw infrastructure_ex();
//    }

//    unique_lock<mutex> lock2(mMutexPos,defer_lock);
//    unique_lock<mutex> lock3(mMutexOut,defer_lock);

//    if(!mbPoseChanged) return false;

//    Msg.mnId = mId.first;
//    Msg.mClientId = mId.second;
//    Msg.mUniqueId = mUniqueId;
//    Msg.mbPoseOnly = true;

//    Msg.mbMultiUse = mbMultiUse;

//    Eigen::Matrix<double,3,1> eigP3D_wm = Converter::toVector3d(mWorldPos);
//    Eigen::Matrix<double,3,1> eigP3D_wc = (mg2oS_wcurmap_wclientmap.inverse()).map(eigP3D_wm); //client map

//    cv::Mat ClientPos = Converter::toCvMat(eigP3D_wc);

//    Converter::CvMatToMsgArrayFixedSize<macslam_msgs::macMapPoint::_mWorldPos_type,float>(ClientPos,Msg.mWorldPos);

//    mbPoseChanged = false;

//    mbInOutBuffer = false;

//    return true;
//}

void MapPoint::UpdateFromMessageServer(cslam_msgs::MapPoint *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap)
{
    if(mSysState != eSystemState::SERVER)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m System Type Mismatch" << endl;
        throw infrastructure_ex();
    }

    if(!mpRefKF)
    {
        cout << "\033[1;31m!!!!! FATAL ERROR !!!!!\033[0m " << __func__ << __LINE__ << ": MP has no parent" << endl;
        throw infrastructure_ex();
    }

    if(mpRefKF->isBad())
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << ": mpRefKF is BAD" << endl;
    }

    mbOmitSending = true;

    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lockMap(mMapMutex);

    //Pos

    if(pMsg->mbPoseChanged)
    {
        unique_lock<mutex> lock(mMutexPos);

        if(!mbPoseLock)
        {
            unique_lock<mutex> lock2(mGlobalMutex);


            idpair MsgParentId = make_pair(pMsg->mpRefKFId,pMsg->mpRefKFClientId);
            cout << __func__ << __LINE__ << ": Msg parent id: " << pMsg->mpRefKFId << "|" << pMsg->mpRefKFClientId << " -- derived pair: " << MsgParentId.first << "|" << MsgParentId.second << endl;

            cv::Mat P3D_ref = cv::Mat(3,1,5); //world client

            Converter::MsgArrayFixedSizeToCvMat<cslam_msgs::MapPoint::_mRefPos_type,float>(P3D_ref,pMsg->mRefPos);

            float s = static_cast<double>(mg2oS_wcurmap_wclientmap.inverse().scale());

            P3D_ref *=(1./s);

            if(mpRefKF->mId != MsgParentId)
            {
                cv::Mat T_pold_pnew;
                mpMap->GetTfToParent(pMsg->mpRefKFId,pMsg->mpRefKFClientId,T_pold_pnew,mpRefKF);

                cv::Mat Rno = T_pold_pnew.inv().rowRange(0,3).colRange(0,3);
                cv::Mat tno = T_pold_pnew.inv().rowRange(0,3).col(3);

                P3D_ref = Rno * P3D_ref + tno;
            }
            this->SetWorldPosFromRef(P3D_ref,false);;
        }
    }

    //Observations

    for(int idx=0;idx<pMsg->mObservations_KFIDs.size();++idx)
    {
        kfptr pKFi = mpMap->GetKfPtr(pMsg->mObservations_KFIDs[idx],pMsg->mObservations_KFClientIDs[idx]);
        size_t FeatId = pMsg->mObservations_n[idx];

        if(!pKFi)
            continue; //KF is nullptr - cannot be used anyway

        bool bCurLocked = false;
        map<kfptr,bool>::iterator mitLock = mObservationsLock.find(pKFi);

        if(mitLock == mObservationsLock.end())
        {
            bLocked = false;
        }
        else
        {
            bLocked = mitLock->second;
        }

        if(bCurLocked)
        {
            //KF association locked -- will not be changed
            // This means this MP has already seen pKFi, and either has an association to it (to the same feature as proposed by this message or another) or this Observation was delted by the server
            continue;
        }

        //Here, we know: association to pKFi is not locked for this MP -- eitherbecause it does not exist so far, or it exists but is not locked
        //check the other way round -- is the the MP asscociation for Feature FeatId locked in pKFi

        if(pKFi->IsMpLocked(FeatId))
        {
            //This means pKFi already has a MP associated to Feature FeatId, and it is locked (or it was deleted by the server who afterwards locked it)
            //It could be pMPi, it could also be another MP -- but it is locked, we cannot change it
            //ToDo: if it is a different MP, we could maybe merge
            continue;
        }

        // get MP associated to FeatId for pKFi (if no association, this is nulltpr
        mpptr pMPCur = pKFi->GetMapPoint(FeatId);

        //Also pKFi would accept a change for FeatId -- first check if association already exists

        size_t CurId;

        map<kfptr,size_t>::iterator mitObs = mObservations.find(pKFi);

        if(mitObs == mObservations.end())
        {
            //no association
            CurId = -1;
        }
        else
        {
            CurId = mitObs->second;
        }

        //no check different options

        if(CurId == -1)
        {
            //THIS has so far no association to pKFi
            if(!pMPCur)
            {
                //pKFi has also no associated MP for feature FeatId -- so add

                mObservations[pKF]= FeatId;

                pKFi->mbOmitSending = true;
                pKFi->AddMapPoint(shared_from_this(),FeatId,false);
                pKFi->mbOmitSending = false;

                ++nObs;
                ++mnVisible;
                ++mnFound;
            }
            else
            {
                if(pMPCur->mId == this->mId)
                {
                    //pKFi->mvpMapPoints[FeatId] = THIS, but THIS->mObservations[pKFi] does not exist
                    cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << "consistency error #1" << endl;
                }
                else
                {
                    //replace pKFi->mvpMapPoints[FeatId]
                    mObservations[pKF]= FeatId;

                    pKFi->mbOmitSending = true;
                    pKFi->ReplaceMapPointMatch(FeatId,shared_from_this(),false);
                    pKFi->mbOmitSending = false;

                    pMPCur->mbOmitSending = true;
                    pMPCur->EraseObservation(pKFi,false);
                    pMPCur->mbOmitSending = false;

                    ++nObs;
                    ++mnVisible;
                    ++mnFound;
                }
            }
        }
        else
        {
            //THIS->mObservations[pKFi] exists

            if(CurId == FeatId)
            {
                if(!pMPCur)
                {
                    //This means: THIS->mObservations[pKFi] = (FeatId == CurId), but pKFi->mvpMapPoints[FeatId == CurId] = nullptr
                    cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << "consistency error #1" << endl;
                }
                else
                {
                    if(pMPCur->mId == this->mId)
                    {
                        //everything the same -- nothing to do
                    }
                    else
                    {
                        //This means: THIS->mObservations[pKFi] = (FeatId == CurId), but pKFi->mvpMapPoints[FeatId == CurId] != THIS
                        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << "consistency error #2" << endl;
                    }
                }
            }
            else
            {
                //THIS wants to change its associated Feature in pKFi
                cout << "\033[1;33m!!! WARN !!!\033[0m " << __func__ << __LINE__ << "MP wants to be mapped to another feature in pKFi" << endl;

                if(!pMPCur)
                {
                    //This means: THIS->mObservations[pKFi] = CurId, and should be changed to THIS->mObservations[pKFi] = FeatId, with pKFi->mvpMapPoints[FeatId] = nullptr so far
                    //weird, but still does not destroy consistency

                    mObservations[pKF]= FeatId;

                    pKFi->mbOmitSending = true;
                    pKFi->EraseMapPointMatch(CurId,false);
                    pKFi->AddMapPoint(shared_from_this(),FeatId,false);
                    pKFi->mbOmitSending = false;
                }
                else
                {
                    if(pMPCur->mId == this->mId)
                    {
                        //This means: THIS->mObservations[pKFi] = CurId, but pKFi->mvpMapPoints[FeatId] = THIS
                        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << "consistency error #3" << endl;
                    }
                    else
                    {
                        //This means: THIS->mObservations[pKFi] = CurId, and should be changed to THIS->mObservations[pKFi] = FeatId, with pKFi->mvpMapPoints[FeatId] = another MP that will be replace
                        //same here: weird, but still does not destroy consistency

                        mObservations[pKF]= FeatId;

                        pKFi->mbOmitSending = true;
                        pKFi->EraseMapPointMatch(CurId,false);
                        pKFi->AddMapPoint(shared_from_this(),FeatId,false);
                        pKFi->mbOmitSending = false;

                        pMPCur->mbOmitSending = true;
                        pMPCur->EraseObservation(pKFi,false);
                        pMPCur->mbOmitSending = false;
                    }
                }
            }
        }
    }

    this->UpdateNormalAndDepth();
    this->ComputeDistinctiveDescriptors();

    if(!mbMultiUse)
        mbMultiUse = pMsg->mbMultiUse;

    mbOmitSending = false;

//    if(bBad)
//        SetBadFlag();

//    if(mpRefKF && mpRefKF->isBad() && !mbBad)
//    {
//        cout << "\033[1;33m!!! WARN !!!\033[0m Client " << mId.second << ": In \"MapPoint::UpdateFromMessageV2(...)\": mpRefKf is bad (may be empty)" << endl;
//        cout << "MP: " << this->mId.first << endl;
//        cout << "mpRefKF: " << mpRefKF->mId.first << "|" << mpRefKF->mId.second << endl;
//                    cout << "mpRefKf empty?: " << mpRefKF->IsEmpty() << endl;
//    }

//    if(!mpRefKF)
//    {
//        cout << "\033[1;33m!!! WARN !!!\033[0m Client " << mId.second << ": In \"MapPoint::UpdateFromMessageV2(...)\": no mpRefKf available - workaround: set MP bad" << endl;
//        cout << "mbBad true? " << (mbBad == true) << endl;
//        if(!mbBad) SetBadFlag();
//    }
}

void MapPoint::UpdatePoseFromMessageServer(cslam_msgs::MapPoint *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap)
{
    if(mSysState != eSystemState::SERVER)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m System Type Mismatch" << endl;
        throw infrastructure_ex();
    }

    if(!mpRefKF)
    {
        cout << "\033[1;31m!!!!! FATAL ERROR !!!!!\033[0m " << __func__ << __LINE__ << ": MP has no parent" << endl;
        throw infrastructure_ex();
    }

    if(mpRefKF->isBad())
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << ": mpRefKF is BAD" << endl;
    }

    mbOmitSending = true;

    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lockMap(mMapMutex);

    //Pos

    if(pMsg->mbPoseChanged)
    {
        unique_lock<mutex> lock(mMutexPos);

        if(!mbPoseLock)
        {
            unique_lock<mutex> lock2(mGlobalMutex);


            idpair MsgParentId = make_pair(pMsg->mpRefKFId,pMsg->mpRefKFClientId);
            cout << __func__ << __LINE__ << ": Msg parent id: " << pMsg->mpRefKFId << "|" << pMsg->mpRefKFClientId << " -- derived pair: " << MsgParentId.first << "|" << MsgParentId.second << endl;

            cv::Mat P3D_ref = cv::Mat(3,1,5); //world client

            Converter::MsgArrayFixedSizeToCvMat<cslam_msgs::MapPoint::_mRefPos_type,float>(P3D_ref,pMsg->mRefPos);

            float s = static_cast<double>(mg2oS_wcurmap_wclientmap.inverse().scale());

            P3D_ref *=(1./s);

            if(mpRefKF->mId != MsgParentId)
            {
                cv::Mat T_cref_cmsg;
                mpRefKF = mpMap->HandleMissingParent(pMsg->mpRefKFId,pMsg->mpRefKFClientId,T_cref_cmsg,mpRefKF);

                cv::Mat R_cr_co = T_cref_cmsg.rowRange(0,3).colRange(0,3);
                cv::Mat t_cr_co = T_cref_cmsg.rowRange(0,3).col(3);

                P3D_ref = R_cr_co * P3D_ref + t_cr_co;
            }
            this->SetWorldPosFromRef(P3D_ref,false);;
        }
    }

    this->UpdateNormalAndDepth();

    if(!mbMultiUse)
        mbMultiUse = pMsg->mbMultiUse;

    mbOmitSending = false;
}

//void MapPoint::UpdateFromMessageClient(macslam_msgs::macMapPoint *pMsg)
//{
//    if(mSysState != eSystemState::CLIENT)
//    {
//        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m System Type Mismatch" << endl;
//        throw infrastructure_ex();
//    }

//    cv::Mat P3D_wc = cv::Mat(3,1,5); //world client
//    Converter::MsgArrayFixedSizeToCvMat<macslam_msgs::macMapPoint::_mWorldPos_type,float>(P3D_wc,pMsg->mWorldPos);
//    P3D_wc.copyTo(P3D_wc);

//    mbPoseChanged == false;

//    if(!mbMultiUse)
//        mbMultiUse = pMsg->mbMultiUse;
//}

//void MapPoint::EraseInOutBuffer()
//{
//    if(mspComm.empty())
//    {
//        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m MapPoint::EraseInOutBuffer(...): no Comm ptrs" << endl;
//        throw infrastructure_ex();
//    }

//    if(mbSentOnce && !this->IsInOutBuffer())
//    {
//        for(set<commptr>::const_iterator sit = mspComm.begin();sit!=mspComm.end();++sit)
//        {
//            commptr pComm = *sit;
//            pComm->DeleteMpFromBuffer(shared_from_this());
//            mbInOutBuffer = false;
//        }
//    }
//}

} //end ns
