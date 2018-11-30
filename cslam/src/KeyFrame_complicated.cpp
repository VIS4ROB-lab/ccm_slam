#include <cslam/KeyFrame.h>

namespace cslam {

size_t KeyFrame::nNextId=0;

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
//      mpComm(pComm),
      mpORBvocabulary(F.mpORBvocabulary),mpMap(pMap),mpKeyFrameDB(pKFDB),
      mbIsEmpty(false),mbPoseLock(false),mbPoseChanged(false),mbInOutBuffer(false), mbSentOnce(false),
      mLoopQuery(defpair),mMatchQuery(defpair),
      mBALocalForKF(defpair),mBAFixedForKF(defpair),mBAGlobalForKF(defpair),
      mSysState(SysState),mbOmitSending(false),
      mbChangedByServer(false),mbLoopCorrected(false),
      mPubCount(0)
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
}

KeyFrame::KeyFrame(bool NonUsedValue, mapptr pMap, eSystemState SysState) //the "NonUsedValue" is for safety, to not accidentally construct emtpy objects
    : mpORBvocabulary(nullptr), mpMap(pMap), mpKeyFrameDB(nullptr),
//      mpComm(nullptr),
      mId(defpair), mUniqueId(defid), mTimeStamp(-1.0),mVisId(-1),
      mnGridCols(-1), mnGridRows(-1), mfGridElementWidthInv(-1.0), mfGridElementHeightInv(-1.0),
      fx(-1.0f), fy(-1.0f), cx(-1.0f), cy(-1.0f), invfx(-1.0f), invfy(-1.0f),
      N(-1.0f),
      mvKeys(vector<cv::KeyPoint>()), mvKeysUn(vector<cv::KeyPoint>()), mDescriptors(cv::Mat()), mK(cv::Mat()),
      mnScaleLevels(-1.0f), mfScaleFactor(-1.0f), mfLogScaleFactor(-1.0f),
      mvScaleFactors(vector<float>()), mvLevelSigma2(vector<float>()), mvInvLevelSigma2(vector<float>()),
      mnMinX(-defid), mnMinY(-defid), mnMaxX(defid), mnMaxY(defid),
      mbIsEmpty(true),mbToBeErased(false),mbBad(true),mbPoseLock(false),
      mbFirstConnection(true),
      mFrameId(defpair), mbPoseChanged(false),
      mSysState(SysState),mbOmitSending(false),
      mbChangedByServer(false),mbLoopCorrected(false),
      mPubCount(0)
{
    //pass map to KF to be able to erase itself from KF buffer
}

KeyFrame& KeyFrame::operator=(KeyFrame&& rhs)
{
      mpORBvocabulary = rhs.mpORBvocabulary;mpMap = rhs.mpMap;mpKeyFrameDB = rhs.mpKeyFrameDB;
//      mpComm = rhs.mpComm;
      mspComm = rhs.mspComm;
      mId = move(rhs.mId);mTimeStamp = move(rhs.mTimeStamp);mUniqueId = move(rhs.mUniqueId);mVisId=move(rhs.mVisId);
      mnGridCols = move(rhs.mnGridCols);mnGridRows = move(rhs.mnGridRows);mfGridElementWidthInv = move(rhs.mfGridElementWidthInv);mfGridElementHeightInv = move(rhs.mfGridElementHeightInv);
      fx = move(rhs.fx);fy = move(rhs.fy);cx = move(rhs.cx);cy = move(rhs.cy);invfx = move(rhs.invfx);invfy = move(rhs.invfy);
      N = move(rhs.N);
      mvKeys = move(rhs.mvKeys);mvKeysUn = move(rhs.mvKeysUn);mDescriptors = move(rhs.mDescriptors);mK = move(rhs.mK);
      mnScaleLevels = move(rhs.mnScaleLevels);mfScaleFactor = move(rhs.mfScaleFactor);mfLogScaleFactor = move(rhs.mfLogScaleFactor);
      mvScaleFactors = move(rhs.mvScaleFactors);mvLevelSigma2 = move(rhs.mvLevelSigma2);mvInvLevelSigma2 = move(rhs.mvInvLevelSigma2);
      mnMinX = move(rhs.mnMinX);mnMinY = move(rhs.mnMinY);mnMaxX = move(rhs.mnMaxX);mnMaxY = move(rhs.mnMaxY);
      mbSentOnce = move(rhs.mbSentOnce);mbIsEmpty = move(rhs.mbIsEmpty);
      mBowVec = move(rhs.mBowVec);mFeatVec = move(rhs.mFeatVec);
      mTcp = move(rhs.mTcp);Tcw = move(rhs.Tcw);Twc = move(rhs.Twc);Ow = move(rhs.Ow);Cw = move(rhs.Cw);
      mvpMapPoints = rhs.mvpMapPoints;mConnectedKeyFrameWeights = rhs.mConnectedKeyFrameWeights;mvpOrderedConnectedKeyFrames = rhs.mvpOrderedConnectedKeyFrames;mvOrderedWeights = move(rhs.mvOrderedWeights);
      mbFirstConnection = rhs.mbFirstConnection;mpParent = rhs.mpParent;
      mspChildrens = rhs.mspChildrens;mspLoopEdges = rhs.mspLoopEdges;
      mbBad = move(rhs.mbBad);
      mGrid = move(rhs.mGrid);
      mbToBeErased = move(rhs.mbToBeErased);
      mbPoseLock = move(rhs.mbPoseLock);
      mvbMapPointsLock = rhs.mvbMapPointsLock;
//      mnLoopQueryClientId = move(rhs.mnLoopQueryClientId);
      mLoopQuery = move(rhs.mLoopQuery);
      mMatchQuery = move(rhs.mMatchQuery);
//      mBAFixedForKF = move(rhs.mBAFixedForKF);
      mSysState = move(rhs.mSysState);
      mbOmitSending = move(rhs.mbOmitSending);
      mbInOutBuffer = move(rhs.mbInOutBuffer);
      mbChangedByServer = move(rhs.mbChangedByServer);
      mbLoopCorrected = move(rhs.mbLoopCorrected);
      mPubCount = move(rhs.mPubCount);
}

KeyFrame::KeyFrame(cslam_msgs::KeyFrame* pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap, vocptr pVoc, mapptr pMap, dbptr pKFDB, commptr pComm,
                   std::vector<cv::KeyPoint> vKeys, std::vector<cv::KeyPoint>  vKeysUn, cv::Mat Descriptors, cv::Mat K,
                   std::vector<float> vScaleFactors, std::vector<float> vLevelSigma2, std::vector<float> vInvLevelSigma2, eSystemState SysState, size_t UniqueId)
    : mFrameId(defpair),mId(make_pair(pMsg->mnId,pMsg->mClientId)),mUniqueId(UniqueId),mTimeStamp(pMsg->dTimestamp),mVisId(-1),
      mnGridCols(pMsg->mnGridCols), mnGridRows(pMsg->mnGridRows), mfGridElementWidthInv(pMsg->mfGridElementWidthInv), mfGridElementHeightInv(pMsg->mfGridElementHeightInv),
      fx(pMsg->fx), fy(pMsg->fy), cx(pMsg->cx), cy(pMsg->cy), invfx(pMsg->invfx), invfy(pMsg->invfy),
      N(pMsg->N),mvKeys(vKeys), mvKeysUn(vKeysUn), mDescriptors(Descriptors), mK(K),
      mnScaleLevels(pMsg->mnScaleLevels), mfScaleFactor(pMsg->mfScaleFactor), mfLogScaleFactor(pMsg->mfLogScaleFactor),
      mvScaleFactors(vScaleFactors), mvLevelSigma2(vLevelSigma2), mvInvLevelSigma2(vInvLevelSigma2),
      mnMinX(pMsg->mnMinX), mnMinY(pMsg->mnMinY), mnMaxX(pMsg->mnMaxX), mnMaxY(pMsg->mnMaxY),
      mbFirstConnection(true),mbNotErase(false),
      mbToBeErased(false),
      mpORBvocabulary(pVoc),mpMap(pMap), mpKeyFrameDB(pKFDB),
//      mpComm(pComm),
      mbIsEmpty(false),mbPoseLock(false),mbPoseChanged(false),mbInOutBuffer(false),
      mLoopQuery(defpair),mMatchQuery(defpair),
      mBALocalForKF(defpair),mBAFixedForKF(defpair),mBAGlobalForKF(defpair),
      mSysState(SysState),mbOmitSending(false),
      mbChangedByServer(false),mbLoopCorrected(false),
      mbBad(false),mPubCount(0)
{
    //constructor for server input

    //Performance: use "reserve" for vectors
    //mBowVect --> Map, no pre-allocation possible
    //mFeatVec --> Map, no pre-allocation possible

    if(pMsg->mbBad)
    {
         cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << "Incoming KF message: mbBad == true" << endl;
         throw infrastructure_ex();
    }

    mspComm.insert(pComm);

    mbSentOnce=true;

    for(int idx=0;idx<pMsg->mBowVec_WordIds.size();++idx)
    {
        mBowVec.addWeight(pMsg->mBowVec_WordIds[idx],pMsg->mBowVec_WordValues[idx]);
    }

    for(int idx=0;idx<pMsg->mFeatVec_NodeIds.size();++idx)
    {
        mFeatVec[idx].reserve(mFeatVec_WordIds[idx].uintvec.size());

        for(int idy=0;idy<pMsg->mFeatVec_WordIds[idx].uintvec.size();++idy)
        {
            mFeatVec[idx].push_back(pMsg->mFeatVec_WordIds[idx].uintvec[idy]);
        }
    }

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
        //if MP not in Map, we ignore it. Might be deleted, or comes in later and is then (hopefully) added to this KF
    }

    Tcw = cv::Mat(4,4,5);
    Twc = cv::Mat(4,4,5);
    Ow = cv::Mat(3,1,5);
    Cw = cv::Mat(4,1,5);

    mTcp = cv::Mat(4,4,5);

    cv::Mat tempTcp = cv::Mat(4,4,5);

    Converter::MsgArrayFixedSizeToCvMat<macslam_msgs::macKeyFrame::_Tcp_type,float>(tempTcp,pMsg->Tcp);

//    cout << "KeyFrame::KeyFrame(from msg): tempTcw: " << tempTcw << endl;

    float s = static_cast<float>(mg2oS_wcurmap_wclientmap.inverse().scale());

    tempTcp.at<float>(0,3) *=(1./s);
    tempTcp.at<float>(1,3) *=(1./s);
    tempTcp.at<float>(2,3) *=(1./s);

//    cout << "KeyFrame::KeyFrame(from msg): T_c_wm: " << T_c_wm << endl;

    //get parent -- might be valid, but could also be NULLPTR, e.g. if deleted

    mpParent = mpMap->GetKfPtr(pMsg->mpParent_KfId,pMsg->mpParent_KfClientId);

    //if parent does not exist (e.g. deleted), get another parent

    if(!mpParent)
    {
        cv::Mat T_cref_cmsg;
        mpParent = mpMap->HandleMissingParent(pMsg->mpRefKFId,pMsg->mpRefKFClientId,T_cref_cmsg,mpParent);
        tempTcp = tempTcp * T_cref_cmsg.inv();
    }

    if(!mpParent)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << "no mpParent" << endl;
        throw estd::infrastructure_ex();
    }

    this->SetPoseFromTcp(tempTcp,false);

    AssignFeaturesToGrid();

    mbOmitSending = false;

    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // Call EstablishInitialConnectionsServer()
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
}

void KeyFrame::EstablishInitialConnectionsServer()
{
    //this is necessary, because we cannot use shared_from_this() in constructor

    mbOmitSending = true;

    for(int idx=0;idx<mvpMapPoints.size();++idx)
    {
        mpptr pMPi = mvpMapPoints[idx];

        if(pMP)
        {
            if(pMPi->IsObsLocked(shared_from_this()))
            {
                if(pMPi->IsInKeyFrame(shared_from_this()))
                {
                    //already there

                    cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << "KF arrived first & already associated to MP" << endl;
                    throw infrastructure_ex();
                }
                else
                {
                    //delete MP association
                    mvpMapPoints[idx] = nullptr;

                    //ToDo: in this case, MPs could maybe be merged
                }
            }
            else
            {
                if(pMPi->IsInKeyFrame(shared_from_this()))
                {
                    cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << "KF arrived first & already associated to MP" << endl;
                    throw infrastructure_ex();
                }

                pMPi->mbOmitSending = true;
                pMPi->AddObservation(shared_from_this(),idx);
                pMPi->mbOmitSending = false;
            }
        }
        else
        {
//            cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << " pMPi is NULLPTR" << endl;
        }
    }

    this->UpdateConnections();

    mbOmitSending = false;

    //ToDo: Match MPs of connected KFs to find more matches
}

KeyFrame::KeyFrame(cslam_msgs::KeyFrame* pMsg, vocptr pVoc, mapptr pMap, dbptr pKFDB, commptr pComm,
                   std::vector<cv::KeyPoint> vKeys, std::vector<cv::KeyPoint>  vKeysUn, cv::Mat Descriptors, cv::Mat K,
                   std::vector<float> vScaleFactors, std::vector<float> vLevelSigma2, std::vector<float> vInvLevelSigma2, eSystemState SysState, size_t UniqueId)
    : mFrameId(defpair),mId(make_pair(pMsg->mnId,pMsg->mClientId)),mUniqueId(UniqueId),mTimeStamp(pMsg->dTimestamp),mVisId(-1),
      mnGridCols(pMsg->mnGridCols), mnGridRows(pMsg->mnGridRows), mfGridElementWidthInv(pMsg->mfGridElementWidthInv), mfGridElementHeightInv(pMsg->mfGridElementHeightInv),
      fx(pMsg->fx), fy(pMsg->fy), cx(pMsg->cx), cy(pMsg->cy), invfx(pMsg->invfx), invfy(pMsg->invfy),
      N(pMsg->N),mvKeys(vKeys), mvKeysUn(vKeysUn), mDescriptors(Descriptors), mK(K),
      mnScaleLevels(pMsg->mnScaleLevels), mfScaleFactor(pMsg->mfScaleFactor), mfLogScaleFactor(pMsg->mfLogScaleFactor),
      mvScaleFactors(vScaleFactors), mvLevelSigma2(vLevelSigma2), mvInvLevelSigma2(vInvLevelSigma2),
      mnMinX(pMsg->mnMinX), mnMinY(pMsg->mnMinY), mnMaxX(pMsg->mnMaxX), mnMaxY(pMsg->mnMaxY),
      mbFirstConnection(true),mbNotErase(false),
      mbToBeErased(false),
      mpORBvocabulary(pVoc),mpMap(pMap), mpKeyFrameDB(pKFDB),
//      mpComm(pComm),
      mbIsEmpty(false),mbPoseLock(false),mbPoseChanged(false),mbInOutBuffer(false),
      mLoopQuery(defpair),mMatchQuery(defpair),
      mBALocalForKF(defpair),mBAFixedForKF(defpair),mBAGlobalForKF(defpair),
      mSysState(SysState),mbOmitSending(false),
      mbChangedByServer(false),mbLoopCorrected(false)
{
    //constructor for client input

    //Performance: use "reserve" for vectors

    mspComm.insert(pComm);

    mbOmitSending = true;

    mbSentOnce=true;

    for(int idx=0;idx<pMsg->mBowVec_WordIds.size();++idx)
    {
        mBowVec.addWeight(pMsg->mBowVec_WordIds[idx],pMsg->mBowVec_WordValues[idx]);
    }

    for(int idx=0;idx<pMsg->mFeatVec_NodeIds.size();++idx)
    {
        for(int idy=0;idy<pMsg->mFeatVec_WordIds[idx].uintvec.size();++idy)
        {
            mFeatVec[idx].push_back(pMsg->mFeatVec_WordIds[idx].uintvec[idy]);
        }
    }

    Tcw = cv::Mat(4,4,5);
    Twc = cv::Mat(4,4,5);
    Ow = cv::Mat(3,1,5);
    Cw = cv::Mat(4,1,5);

    cv::Mat tempT = cv::Mat(4,4,5);

    if(mSysState == 0)
    {
        Converter::MsgArrayFixedSizeToCvMat<macslam_msgs::macKeyFrame::_Tcw_type,float>(tempT,pMsg->Tcw);
    }
    else if(mSysState == 1)
    {
        Converter::MsgArrayFixedSizeToCvMat<macslam_msgs::macKeyFrame::_Tcp_type,float>(tempT,pMsg->Tcp);
    }

//    cout << "KeyFrame::KeyFrame(from msg): tempTcw: " << tempTcw << endl;

    float s = static_cast<float>(mg2oS_wcurmap_wclientmap.inverse().scale());
    tempT.at<float>(0,3) *=(1./s);
    tempT.at<float>(1,3) *=(1./s);
    tempT.at<float>(2,3) *=(1./s);

//    cout << "KeyFrame::KeyFrame(from msg): T_c_wm: " << T_c_wm << endl;

    if(mSysState == 0)
    {
        this->SetPose(tempT,false);
    }
    else if(mSysState == 1)
    {
        mpParent = mpMap->GetKfPtr(pMsg->mpParent_KfId,pMsg->mpParent_KfClientId);
        this->SetPoseFromTcp(tempT,false);
    }

    mvpMapPoints.resize(N,nullptr);
    mvbMapPointsLock.resize(N,false);

    if(pMsg->mbBad)
         cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << "Incoming KF message: mbBad == true" << endl;

    AssignFeaturesToGrid();

    mbOmitSending = false;
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

void KeyFrame::SetPose(const cv::Mat &Tcw_, bool bLock)
{
    if(mbPoseLock && mSysState == eSystemState::CLIENT)
        return;

    unique_lock<mutex> lock(mMutexPose);
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

    if(mpParent)
    {
        mTcp = Tcw*mpParent->GetPoseInverse();
    }

    if(bLock)
    {
        //bLock == false --> Pose comes from client
        mbPoseLock = true;
    }

    if(!mbOmitSending && mbSentOnce)
    {
        mbPoseChanged = true;
        SendMe();
    }
}

void KeyFrame::SetPoseFromTcp(const Mat &Tcp_, bool bLock)
{
//    if(mbPoseLock && mSysState == eSystemState::CLIENT)
//        return;

    unique_lock<mutex> lock(mMutexPose);
    unique_lock<mutex> lockCon(mMutexConnections);

    Tcp_.copyTo(mTcp);

    Tcw = mTcp * mpParent->GetPose();

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
        //bLock == false --> Pose comes from client
        mbPoseLock = true;
    }

//    if(!mbOmitSending && mbSentOnce)
//    {
//        mbPoseChanged = true;
//        SendMe();
//    }
}

void KeyFrame::SetPoseDebug(const cv::Mat &Tcw_, bool bLock)
{
    cout << "--- KeyFrame::SetPoseDebug(...) ---" << endl;

    if(mbPoseLock && mSysState == eSystemState::CLIENT)
        return;

    unique_lock<mutex> lock(mMutexPose);
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

    if(bLock)
    {
        //bLock == false --> Pose comes from client
        mbPoseLock = true;
    }

    cout << "mbOmitSending: " <<mbOmitSending << endl;
    cout << "mbSentOnce: " <<mbSentOnce << endl;
    cout << "mbPoseChanged: " <<mbPoseChanged << endl;

    if(!mbOmitSending && mbSentOnce)
    {
        mbPoseChanged = true;
        SendMe();
    }

    cout << "mbPoseChanged: " <<mbPoseChanged << endl;

    cout << "--- KeyFrame::SetPoseDebug(...) END---" << endl;
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
        return;

//    if(pMP->IsEmpty()) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m KeyFrame::AddMapPoint(): Adding empty MP" << endl;

    mvpMapPoints[idx]=pMP;

    if(bLock)
    {
        mvbMapPointsLock[idx] = true;
    }

//    if(!pMP)
//    {

//    }
//    else
//    {
//        if(!mbOmitSending && mbSentOnce)
//        {
////            mvpNewMapPoints.push_back(pMP);
////            mvpNewMapPointsVectIds.push_back(idx);
////            mvbNewMapPointsDelete.push_back(false);
//            SendMe();
//        }
//    }
}

void KeyFrame::EraseMapPointMatch(const size_t &idx, bool bLock)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(this->IsEmpty()) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m KeyFrame::EraseMapPointMatch(): trying to erase MP from empty KF" << endl;

    if(mvbMapPointsLock[idx] && mSysState == eSystemState::CLIENT)
        return;

    mpptr pMP =  mvpMapPoints[idx];
    mvpMapPoints[idx]=nullptr;

    if(bLock)
    {
        mvbMapPointsLock[idx] = true;
    }

//    if(pMP)
//    {
//        if(!mbOmitSending && mbSentOnce)
//        {
//            mvpNewMapPoints.push_back(pMP);
//            mvpNewMapPointsVectIds.push_back(idx);
//            mvbNewMapPointsDelete.push_back(true);
//            SendMe();
//        }
//    }
}

void KeyFrame::EraseMapPointMatch(mpptr pMP, bool bLock)
{    
    int idx = pMP->GetIndexInKeyFrame(this->shared_from_this());
    if(idx>=0)
    {
        if(mvbMapPointsLock[idx] && mSysState == eSystemState::CLIENT)
            return;

        mpptr pMP =  mvpMapPoints[idx];
        mvpMapPoints[idx]=nullptr;

        if(bLock)
        {
            mvbMapPointsLock[idx] = true;
        }

//        if(pMP)
//        {
//            if(!mbOmitSending && mbSentOnce)
//            {
//                mvpNewMapPoints.push_back(pMP);
//                mvpNewMapPointsVectIds.push_back(idx);
//                mvbNewMapPointsDelete.push_back(true);
//                SendMe();
//            }
//        }
    }
}

void KeyFrame::ReplaceMapPointMatch(const size_t &idx, mpptr pMP, bool bLock)
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

//    if(!mbOmitSending && mbSentOnce)
//    {
//        if(!pMP) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m KeyFrame::ReplaceMapPointMatch(...): adding nullptr" << endl;
////        mvpNewMapPoints.push_back(pMP);
////        mvpNewMapPointsVectIds.push_back(idx);
////        mvbNewMapPointsDelete.push_back(false);
//        SendMe();
//    }
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

void KeyFrame::UpdateConnections()
{
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
//            if(mit->first->mnId==this->mnId && mit->first->mClientId==this->mClientId) //ID-Tag
            if(mit->first->mId == this->mId) //ID-Tag
                continue;
            KFcounter[mit->first]++;
        }
    }

    // This should not happen
    if(KFcounter.empty())
        return;

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

        // mspConnectedKeyFrames = spConnectedKeyFrames;
        mConnectedKeyFrameWeights = KFcounter;
        mvpOrderedConnectedKeyFrames = vector<kfptr>(lKFs.begin(),lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

        if(mbFirstConnection && mId.first!=0)
        {
            mpParent = mvpOrderedConnectedKeyFrames.front();
            mpParent->AddChild(this->shared_from_this());
            mbFirstConnection = false;

            mTcp = Tcw*mpParent->GetPoseInverse();
        }
    }
}

//void KeyFrame::FindMoreMPsInNeighbors()
//{

//}

void KeyFrame::AddChild(kfptr pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.insert(pKF);
}

void KeyFrame::EraseChild(kfptr pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.erase(pKF);
}

void KeyFrame::ChangeParent(kfptr pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mpParent = pKF;
    mTcp = Tcw*mpParent->GetPoseInverse();
    pKF->AddChild(this->shared_from_this());
}

set<KeyFrame::kfptr> KeyFrame::GetChilds()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens;
}

KeyFrame::kfptr KeyFrame::GetParent()
{
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

//    cout << "KF " << this->mnId << ": KeyFrame::SetErase() -> mbNotErase:" << mbNotErase << endl;

    if(mbToBeErased)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m KeyFrame::SetErase(): mbToBeErased true" << endl;
        SetBadFlag();
    }
}

void KeyFrame::SetBadFlag()
{
//    cout << "KF " << this->mnId << ": KeyFrame::SetBadFlag()" << endl;
//    cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m KeyFrame::SetBadFlag() called" << endl;

    unique_lock<mutex> lockMap(mMapMutex);

    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mId.first==0)
            return;
        else if(mbNotErase)
        {
            mbToBeErased = true;
//            cout << "KF " << this->mId.first << ": KeyFrame::SetBadFlag() -> mbNotErase:" << mbNotErase << endl;
            return;
        }
    }

    for(map<kfptr,int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
        mit->first->EraseConnection(this->shared_from_this());

    for(size_t i=0; i<mvpMapPoints.size(); i++)
        if(mvpMapPoints[i])
            mvpMapPoints[i]->EraseObservation(this->shared_from_this());
    {
        unique_lock<mutex> lock(mMutexConnections);
        unique_lock<mutex> lock1(mMutexFeatures);

        mConnectedKeyFrameWeights.clear();
        mvpOrderedConnectedKeyFrames.clear();

        // Update Spanning Tree
        set<kfptr> sParentCandidates;
        sParentCandidates.insert(mpParent);

//        cout << "mpParent == null?: " << (mpParent == nullptr) << endl;
//        cout << "mpParent empty?: " << mpParent->IsEmpty() << endl;
//        cout << "mpParent bad?: " << mpParent->isBad() << endl;
//        cout << "#children: " << mspChildrens.size() << endl;

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
//                    int countit = 0;
                    for(set<kfptr>::iterator spcit=sParentCandidates.begin(), spcend=sParentCandidates.end(); spcit!=spcend; spcit++)
                    {
//                        cout << "iteration #: " << countit << endl;
//                        ++countit;
//                        cout << "connected KF == null?: " << (vpConnected[i] == nullptr) << endl;
//                        cout << "parent candidate KF == null?: " << (*spcit == nullptr) << endl;
//                        cout << "connected KF empty?: " << vpConnected[i]->IsEmpty() << endl;
//                        cout << "parent candidate KF empty?: " << (*spcit)->IsEmpty() << endl;
//                        cout << "connected KF bad?: " << vpConnected[i]->isBad() << endl;
//                        cout << "parent candidate KF bad?: " << (*spcit)->isBad() << endl;
//                        if(vpConnected[i]->mnId == (*spcit)->mnId && vpConnected[i]->mClientId == (*spcit)->mClientId) //ID-Tag
                        if(vpConnected[i]->mId == (*spcit)->mId) //ID-Tag
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
            for(set<kfptr>::iterator sit=mspChildrens.begin(); sit!=mspChildrens.end(); sit++)
            {
                (*sit)->ChangeParent(mpParent);
            }

        mpParent->EraseChild(this->shared_from_this());
        mTcp = Tcw*mpParent->GetPoseInverse();
        mbBad = true;
    }

//    if(this->IsEmpty())
//        mpMap->DeleteFromKfBuffer(this->shared_from_this()); // tempout
//    else
    {
        mpMap->EraseKeyFrame(this->shared_from_this());
        mpKeyFrameDB->erase(this->shared_from_this());
    }

//    if(!mbOmitSending && mbSentOnce)
//    {
//        SendMe();
//    }
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

void KeyFrame::SendMe()
{
    if(mspComm.empty())
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m KeyFrame::SendMe(): no Comm ptrs" << endl;
        throw infrastructure_ex();
    }

//    cout << "KeyFrame::SendMe(): #commptrs: " << mspComm.size() << endl;

    if(mbSentOnce && !this->IsInOutBuffer())
    {
        for(set<commptr>::const_iterator sit = mspComm.begin();sit!=mspComm.end();++sit)
        {
            commptr pComm = *sit;
            pComm->PassKftoComm(this->shared_from_this());
        }
    }
}

void KeyFrame::SendMeDebug()
{
//    cout << "--- KeyFrame::SendMeDebug() ---" << endl;

//    static int countsent = 0;

//    cout << "mbSentOnce: " <<mbSentOnce << endl;
//    cout << "countsent: " <<countsent << endl;
//    cout << "this->IsInOutBuffer(): " <<this->IsInOutBuffer() << endl;

//    if(mbSentOnce && !this->IsInOutBuffer())
//    {
//        for(set<commptr>::const_iterator sit = mspComm.begin();sit!=mspComm.end();++sit)
//        {
//            commptr pComm = *sit;
//            pComm->PassKftoComm(this->shared_from_this());
//            ++countsent;
//        }
//    }

//    cout << "countsent: " <<countsent << endl;
//    cout << "this->IsInOutBuffer(): " <<this->IsInOutBuffer() << endl;
//    cout << "--- KeyFrame::SendMeDebug() END---" << endl;
}

void KeyFrame::ShowMyValues()
{
    cout << "******* KF VALUES *******" << endl;

    cout << "mnId: " << mId.first << endl;
    cout << "mClientId: " << mId.second << endl;
    cout << "mTimeStamp: " << mTimeStamp << endl;
    cout << "mbBad: " << mbBad << endl;
    cout << "mbSentOnce: " << mbSentOnce << endl;

    cout << "+++ FIXED VALUES +++" << endl;

    cout << "mnGridCols: " << mnGridCols << endl;
    cout << "mnGridRows: " << mnGridRows << endl;
    cout << "mfGridElementWidthInv: " << mfGridElementWidthInv << endl;
    cout << "mfGridElementHeightInv: " << mfGridElementHeightInv << endl;

    cout << "fx: " << fx << endl;
    cout << "fy: " << fy << endl;
    cout << "cx: " << cx << endl;
    cout << "cy: " << cy << endl;
    cout << "invfx: " << invfx << endl;
    cout << "invfy: " << invfy << endl;
    cout << "N: " << N << endl;

    cout << "mnMinX: " << mnMinX << endl;
    cout << "mnMinY: " << mnMinY << endl;
    cout << "mnMaxX: " << mnMaxX << endl;
    cout << "mnMaxY: " << mnMaxY << endl;

    cout << "mK: rows|cols|type: " << mK.rows << "|" << mK.cols << "|" << mK.type() << endl;
    cout << "mK: " << mK << endl;

    cout << "mnScaleLevels: " << mnScaleLevels << endl;
    cout << "mfScaleFactor: " << mfScaleFactor << endl;
    cout << "mfLogScaleFactor: " << mfLogScaleFactor << endl;

    for(int idx=0;idx<mvScaleFactors.size();++idx) cout << "mvScaleFactors[" << idx << "]: " << mvScaleFactors[idx] << endl;
    for(int idx=0;idx<mvLevelSigma2.size();++idx) cout << "mvLevelSigma2[" << idx << "]: " << mvLevelSigma2[idx] << endl;
    for(int idx=0;idx<mvInvLevelSigma2.size();++idx) cout << "mvInvLevelSigma2[" << idx << "]: " << mvInvLevelSigma2[idx] << endl;

    cout << "+++ FEATURES +++" << endl;

    cout << "mvKeys.size(): " << mvKeys.size() << endl;
    if(mvKeys.size() > 3)
    {
        cv::KeyPoint kp = mvKeys[3];
        cout << "mvKeys[3]: x|y|angle|octave|size|response: " << kp.pt.x << "|" << kp.pt.y << "|" << kp.angle << "|" << kp.octave << "|" << kp.size << "|" << kp.response << endl;
    }
    if(mvKeys.size() > 101)
    {
        cv::KeyPoint kp = mvKeys[101];
        cout << "mvKeys[101]: x|y|angle|octave|size|response: " << kp.pt.x << "|" << kp.pt.y << "|" << kp.angle << "|" << kp.octave << "|" << kp.size << "|" << kp.response << endl;
    }
    if(mvKeys.size() > 513)
    {
        cv::KeyPoint kp = mvKeys[513];
        cout << "mvKeys[513]: x|y|angle|octave|size|response: " << kp.pt.x << "|" << kp.pt.y << "|" << kp.angle << "|" << kp.octave << "|" << kp.size << "|" << kp.response << endl;
    }

    cout << "mvKeysUn.size(): " << mvKeysUn.size() << endl;
    if(mvKeysUn.size() > 3)
    {
        cv::KeyPoint kp = mvKeysUn[3];
        cout << "mvKeysUn[3]: x|y|angle|octave|size|response: " << kp.pt.x << "|" << kp.pt.y << "|" << kp.angle << "|" << kp.octave << "|" << kp.size << "|" << kp.response << endl;
    }
    if(mvKeysUn.size() > 101)
    {
        cv::KeyPoint kp = mvKeysUn[101];
        cout << "mvKeysUn[101]: x|y|angle|octave|size|response: " << kp.pt.x << "|" << kp.pt.y << "|" << kp.angle << "|" << kp.octave << "|" << kp.size << "|" << kp.response << endl;
    }
    if(mvKeysUn.size() > 513)
    {
        cv::KeyPoint kp = mvKeysUn[513];
        cout << "mvKeysUn[513]: x|y|angle|octave|size|response: " << kp.pt.x << "|" << kp.pt.y << "|" << kp.angle << "|" << kp.octave << "|" << kp.size << "|" << kp.response << endl;
    }

    cout << "mDescriptors: rows|cols|type: " << mDescriptors.rows << "|" << mDescriptors.cols << "|" << mDescriptors.type() << endl;
    if(mDescriptors.rows > 5 && mDescriptors.cols > 5) cout << "mDescriptors[5,5]: " << static_cast<int>(mDescriptors.at<uint8_t>(5,5)) << endl;
    if(mDescriptors.rows > 30 && mDescriptors.cols > 137) cout << "mDescriptors[30,137]: " << static_cast<int>(mDescriptors.at<uint8_t>(30,137)) << endl;
    if(mDescriptors.rows > 17 && mDescriptors.cols > 367) cout << "mDescriptors[17,367]: " << static_cast<int>(mDescriptors.at<uint8_t>(17,367)) << endl;

    cout << "mBowVec.size(): " << mBowVec.size() << endl;
    int count = 0;
    for(DBoW2::BowVector::const_iterator mit=mBowVec.begin();mit!=mBowVec.end();++mit)
    {
        if(count == 2 || count == 17)
        {
            cout << "BowVec Key: " << mit->first << endl;
            cout << "BowVec Value: " << mit->second << endl;
        }
        ++count;
    }

    cout << "mFeatVec.size(): " << mFeatVec.size() << endl;
    count = 0;
    for(DBoW2::FeatureVector::const_iterator mit=mFeatVec.begin();mit!=mFeatVec.end();++mit)
    {
        if(count == 2 || count == 17)
        {
            cout << "FeatVec Key: " << mit->first << endl;
            std::vector<unsigned int> vInt = mit->second;
            cout << "FeatVec Value Size: " << vInt.size() << endl;
            if(vInt.size() > 0) cout << "FeatVec Value[0]: " << vInt[0] << endl;
            if(vInt.size() > 1) cout << "FeatVec Value[1]: " << vInt[1] << endl;
        }
        ++count;
    }

    cout << "+++ POSES VALUES +++" << endl;

    cout << "mTcp: " << mTcp << endl;
    cout << "Tcw: " << Tcw << endl;
    cout << "Twc: " << Twc << endl;
    cout << "Ow: " << Ow << endl;
    cout << "Cw: " << Cw << endl;

    cout << "+++ Connections +++" << endl;

    cout << "mvpMapPoints.size(): " << mvpMapPoints.size() << endl;
    cout << "mConnectedKeyFrameWeights.size(): " << mConnectedKeyFrameWeights.size() << endl;
    cout << "mvpOrderedConnectedKeyFrames.size(): " << mvpOrderedConnectedKeyFrames.size() << endl;
    cout << "mvOrderedWeights.size(): " << mvOrderedWeights.size() << endl;
    cout << "mspChildrens.size(): " << mspChildrens.size() << endl;
    cout << "mspLoopEdges.size(): " << mspLoopEdges.size() << endl;
}

void KeyFrame::ConvertToMessageClient(cslam_msgs::KeyFrame &Msg)
{
    if(mSysState != eSystemState::CLIENT)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m System Type Mismatch" << endl;
        throw infrastructure_ex();
    }

    unique_lock<mutex> lock1(mMutexFeatures,defer_lock);
    unique_lock<mutex> lock2(mMutexConnections,defer_lock);
    unique_lock<mutex> lock3(mMutexPose,defer_lock);
    unique_lock<mutex> lock4(mMutexOut,defer_lock);

    lock(lock1,lock2,lock3,lock4);

    //check format
    {
        bool bCheckSize = (mDescriptors.cols != 32) || (mK.rows != 3) || (mK.cols != 3)
                || (Tcw.rows != 4) || (Tcw.cols != 4) || (Twc.rows != 4) || (Twc.cols != 4)
                || (Ow.rows != 3) || (Ow.cols != 1) || (Cw.rows != 4) || (Cw.cols != 1)
                || (mvScaleFactors.size() != 8) || (mvLevelSigma2.size() != 8) || (mvInvLevelSigma2.size() != 8);

        if(bCheckSize)
        {
            cout << "In \"KeyFrame::ConvertToMessage(...)\": member matrix SIZE not as expected in ROS message" << endl;
            throw estd::infrastructure_ex();
        }

        bool bCheckTypes = (mDescriptors.type() != 0);

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
    Msg.dTimestamp = mTimeStamp;
    Msg.bSentOnce = mbSentOnce;

    if(mpParent)
    {
        Msg.mpParent_KfId = mpParent->mId.first;
        Msg.mpParent_KfClientId = mpParent->mId.second;
    }
    else
    {
        cout << "\033[1;31m!!!!! FATAL ERROR !!!!!\033[0m " << __func__ << __LINE__ << ": KF has no parent" << endl;
        throw infrastructure_ex();
    }

    Msg.mbPoseOnly = false;

    if(mbSentOnce)
    {
        if(mbPoseChanged)
        {
//            Converter::CvMatToMsgArrayFixedSize<macslam_msgs::macKeyFrame::_Tcw_type,float>(Tcw,Msg.Tcw);

            Converter::CvMatToMsgArrayFixedSize<macslam_msgs::macKeyFrame::_mTcp_type,float>(mTcp,Msg.mTcp);

            Msg.mbPoseChanged = true;
            mbPoseChanged = false;
        }

//        for(int idx=0;idx<mvpMapPoints.size();++idx)
//        {
//            if(mvpMapPoints[idx] && !mvpMapPoints[idx]->isBad())
//            {
//                Msg.mvpMapPoints_Ids.push_back(mvpMapPoints[idx]->mId.first);
//                Msg.mvpMapPoints_ClientIds.push_back(mvpMapPoints[idx]->mId.second);
//                Msg.mvpMapPoints_VectId.push_back(idx);
//            }
//        }

//        for(int idx=0;idx<mvpNewMapPoints.size();++idx)
//        {
//            if(!(mvpNewMapPoints[idx]))
//            {
//                Msg.mvpMapPoints_Ids.push_back(defid);
//                Msg.mvpMapPoints_ClientIds.push_back(defid);
//                Msg.mvpMapPoints_VectId.push_back(mvpNewMapPointsVectIds[idx]);
//                Msg.mvpMapPoints_delete.push_back(mvbNewMapPointsDelete[idx]);
//            }
//            else
//            {
//                Msg.mvpMapPoints_Ids.push_back(mvpNewMapPoints[idx]->mId.first);
//                Msg.mvpMapPoints_ClientIds.push_back(mvpNewMapPoints[idx]->mId.second);
//                Msg.mvpMapPoints_VectId.push_back(mvpNewMapPointsVectIds[idx]);
//                Msg.mvpMapPoints_delete.push_back(mvbNewMapPointsDelete[idx]);
//            }
//        }

//        mvpNewMapPoints.clear();
//        mvpNewMapPointsVectIds.clear();
//        mvbNewMapPointsDelete.clear();
    }
    else //for server to client, always send all info. Cannot be sure that client has KF in map
    {
        //first time, send all stuff

        //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        //   ++
        //   ++
        // ++++++   Const values
        //  ++++
        //   ++

        //Performance: maybe its faster to use differen msgs -> less big messages when KF was sent once

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

        for(int idx=0;idx<mvKeys.size();++idx) Msg.mvKeys.push_back(Converter::toCvKeyPointMsg(mvKeys[idx]));
        for(int idx=0;idx<mvKeysUn.size();++idx) Msg.mvKeysUn.push_back(Converter::toCvKeyPointMsg(mvKeysUn[idx]));

        for(int idx=0;idx<mDescriptors.rows;++idx)
        {
            cslam_msgs::Descriptor MsgDesc;
            for(int idy=0;idy<mDescriptors.cols;++idy)
            {
                MsgDesc.mDescriptor[idy]=mDescriptors.at<uint8_t>(idx,idy);
            }
            Msg.mDescriptors.push_back(MsgDesc);
        }

        //Performance: We can compute BOW and Feat Vector, or send. Check what is faster... -> sending apparently faster

        for(DBoW2::BowVector::const_iterator mit=mBowVec.begin();mit!=mBowVec.end();++mit)
        {
            Msg.mBowVec_WordIds.push_back(mit->first);
            Msg.mBowVec_WordValues.push_back(mit->second);
        }

        for(DBoW2::FeatureVector::const_iterator mit=mFeatVec.begin();mit!=mFeatVec.end();++mit)
        {
            Msg.mFeatVec_NodeIds.push_back(mit->first);
            macslam_msgs::machUIntVec MsgVect;
            for(int idx=0;idx<mit->second.size();++idx)
            {
                MsgVect.uintvec.push_back(mit->second[idx]);
            }
            Msg.mFeatVec_WordIds.push_back(MsgVect);
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

        Converter::CvMatToMsgArrayFixedSize<macslam_msgs::macKeyFrame::_mK_type,float>(mK,Msg.mK);

        //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        //   ++
        //   ++
        // ++++++   Poses
        //  ++++
        //   ++

        Converter::CvMatToMsgArrayFixedSize<macslam_msgs::macKeyFrame::_mTcp_type,float>(mTcp,Msg.mTcp);

        Msg.mbPoseChanged = mbPoseChanged;

        Converter::CvMatToMsgArrayFixedSize<macslam_msgs::macKeyFrame::_Tcw_type,float>(Tcw,Msg.Tcw);

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
                Msg.mvpMapPoints_Ids.push_back(mvpMapPoints[idx]->mId.first);
                Msg.mvpMapPoints_ClientIds.push_back(mvpMapPoints[idx]->mId.second);
                Msg.mvpMapPoints_VectId.push_back(idx);
            }
        }

//        if(!mvpNewMapPoints.empty()) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m KeyFrame::ConvertToMessage(...): mvpNewMapPoints not empty" << endl;

        mbSentOnce = true;

//        mvpNewMapPoints.clear();
//        mvpNewMapPointsVectIds.clear();
//        mvbNewMapPointsDelete.clear();
        mbPoseChanged = false;
    }

    ++mPubCount;
    mbInOutBuffer = false;
}

//void KeyFrame::ConvertToMessageServer(macslam_msgs::macKeyFrame &Msg, g2o::Sim3 mg2oS_wcurmap_wclientmap)
//{
//    if(mSysState != eSystemState::SERVER)
//    {
//        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m System Type Mismatch" << endl;
//        throw infrastructure_ex();
//    }

//    unique_lock<mutex> lock1(mMutexFeatures,defer_lock);
//    unique_lock<mutex> lock2(mMutexConnections,defer_lock);
//    unique_lock<mutex> lock3(mMutexPose,defer_lock);
//    unique_lock<mutex> lock4(mMutexOut,defer_lock);

//    lock(lock1,lock2,lock3,lock4);

//    //check format
//    {
//        bool bCheckSize = (mDescriptors.cols != 32) || (mK.rows != 3) || (mK.cols != 3)
//                || (Tcw.rows != 4) || (Tcw.cols != 4) || (Twc.rows != 4) || (Twc.cols != 4)
//                || (Ow.rows != 3) || (Ow.cols != 1) || (Cw.rows != 4) || (Cw.cols != 1)
//                || (mvScaleFactors.size() != 8) || (mvLevelSigma2.size() != 8) || (mvInvLevelSigma2.size() != 8);

//        if(bCheckSize)
//        {
//            cout << "In \"KeyFrame::ConvertToMessage(...)\": member matrix SIZE not as expected in ROS message" << endl;
//            throw estd::infrastructure_ex();
//        }

//        bool bCheckTypes = (mDescriptors.type() != 0);

//        if(bCheckTypes)
//        {
//            cout << "In \"KeyFrame::ConvertToMessage(...)\": member matrix TYPES not as expected in ROS message" << endl;
//            throw estd::infrastructure_ex();
//        }
//    }

//    Msg.mbBad = mbBad;
//    Msg.mnId = mId.first;
//    Msg.mClientId = mId.second;
//    Msg.mUniqueId = mUniqueId;
//    Msg.dTimestamp = mTimeStamp;
//    Msg.bSentOnce = mbSentOnce;

//    Msg.mbPoseOnly = false;

//    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//    //   ++
//    //   ++
//    // ++++++   Const values
//    //  ++++
//    //   ++

//    //Performance: maybe its faster to use differen msgs -> less big messages when KF was sent once

//    Msg.mnGridCols = mnGridCols;
//    Msg.mnGridRows = mnGridRows;
//    Msg.mfGridElementWidthInv = mfGridElementWidthInv;
//    Msg.mfGridElementHeightInv = mfGridElementHeightInv;

//    Msg.fx = fx;
//    Msg.fy = fy;
//    Msg.cx = cx;
//    Msg.cy = cy;
//    Msg.invfx = invfx;
//    Msg.invfy = invfy;

//    Msg.N = static_cast<int16_t>(N);

//    for(int idx=0;idx<mvKeys.size();++idx) Msg.mvKeys.push_back(Converter::toCvKeyPointMsg(mvKeys[idx]));
//    for(int idx=0;idx<mvKeysUn.size();++idx) Msg.mvKeysUn.push_back(Converter::toCvKeyPointMsg(mvKeysUn[idx]));

//    for(int idx=0;idx<mDescriptors.rows;++idx)
//    {
//        macslam_msgs::machDescriptor MsgDesc;
//        for(int idy=0;idy<mDescriptors.cols;++idy)
//        {
//            MsgDesc.mDescriptor[idy]=mDescriptors.at<uint8_t>(idx,idy);
//        }
//        Msg.mDescriptors.push_back(MsgDesc);
//    }

//    //Performance: We can compute BOW and Feat Vector, or send. Check what is faster... -> sending apparently faster

//    for(DBoW2::BowVector::const_iterator mit=mBowVec.begin();mit!=mBowVec.end();++mit)
//    {
//        Msg.mBowVec_WordIds.push_back(mit->first);
//        Msg.mBowVec_WordValues.push_back(mit->second);
//    }

//    for(DBoW2::FeatureVector::const_iterator mit=mFeatVec.begin();mit!=mFeatVec.end();++mit)
//    {
//        Msg.mFeatVec_NodeIds.push_back(mit->first);
//        macslam_msgs::machUIntVec MsgVect;
//        for(int idx=0;idx<mit->second.size();++idx)
//        {
//            MsgVect.uintvec.push_back(mit->second[idx]);
//        }
//        Msg.mFeatVec_WordIds.push_back(MsgVect);
//    }

//    Msg.mnScaleLevels = static_cast<int8_t>(mnScaleLevels);
//    Msg.mfScaleFactor = mfScaleFactor;
//    Msg.mfLogScaleFactor = mfLogScaleFactor;
//    for(int idx=0;idx<mvScaleFactors.size();++idx) Msg.mvScaleFactors[idx] = mvScaleFactors[idx];
//    for(int idx=0;idx<mvLevelSigma2.size();++idx) Msg.mvLevelSigma2[idx] = mvLevelSigma2[idx];
//    for(int idx=0;idx<mvInvLevelSigma2.size();++idx) Msg.mvInvLevelSigma2[idx] = mvInvLevelSigma2[idx];

//    Msg.mnMinX = static_cast<int16_t>(mnMinX);
//    Msg.mnMinY = static_cast<int16_t>(mnMinY);
//    Msg.mnMaxX = static_cast<int16_t>(mnMaxX);
//    Msg.mnMaxY = static_cast<int16_t>(mnMaxY);

//    Converter::CvMatToMsgArrayFixedSize<macslam_msgs::macKeyFrame::_mK_type,float>(mK,Msg.mK);

//    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//    //   ++
//    //   ++
//    // ++++++   Poses
//    //  ++++
//    //   ++

//    Converter::CvMatToMsgArrayFixedSize<macslam_msgs::macKeyFrame::_mTcp_type,float>(mTcp,Msg.mTcp);

//    Msg.mbPoseChanged = mbPoseChanged;

//    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
//    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
//    g2o::Sim3 g2oS_c_wm(Converter::toMatrix3d(Rcw),Converter::toVector3d(tcw),1.0); //cam - world map

//    g2o::Sim3 g2oS_c_wc = g2oS_c_wm*mg2oS_wcurmap_wclientmap;

//    Eigen::Matrix3d eigR = g2oS_c_wc.rotation().toRotationMatrix();
//    Eigen::Vector3d eigt = g2oS_c_wc.translation();
//    double s = g2oS_c_wc.scale();
//    eigt *=(1./s); //[R t/s;0 1]
//    cv::Mat T_c_wc = Converter::toCvSE3(eigR,eigt);

//    Converter::CvMatToMsgArrayFixedSize<macslam_msgs::macKeyFrame::_Tcw_type,float>(T_c_wc,Msg.Tcw); //cam - world client

//    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//    //   ++
//    //   ++
//    // ++++++   Map points
//    //  ++++
//    //   ++

//    for(int idx=0;idx<mvpMapPoints.size();++idx)
//    {
//        if(!(mvpMapPoints[idx]))
//        {
//            Msg.mvpMapPoints_Ids.push_back(defid);
//            //continue; //cout << "!!!!!!!!!!!!!!!!!!!!!!! mvpMapPoints contains empty ptrs !!!!!!!!!!!!!!!!!!" << endl;
//            Msg.mvpMapPoints_ClientIds.push_back(defid);
//        }
//        else
//        {
//            Msg.mvpMapPoints_Ids.push_back(mvpMapPoints[idx]->mId.first);
//            Msg.mvpMapPoints_ClientIds.push_back(mvpMapPoints[idx]->mId.second);
//        }
//    }

//    mbSentOnce = true;

//    mbPoseChanged = false;

//    mbInOutBuffer = false;
//}

//bool KeyFrame::ConvertToPoseMessageServer(macslam_msgs::macKeyFrame &Msg, g2o::Sim3 mg2oS_wcurmap_wclientmap)
//{
//    if(mSysState != eSystemState::SERVER)
//    {
//        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m System Type Mismatch" << endl;
//        throw infrastructure_ex();
//    }

//    unique_lock<mutex> lock3(mMutexPose,defer_lock);
//    unique_lock<mutex> lock4(mMutexOut,defer_lock);

//    if(!mbPoseChanged) return false;

//    Msg.mnId = mId.first;
//    Msg.mClientId = mId.second;
//    Msg.mUniqueId = mUniqueId;
//    Msg.mbPoseOnly = true;

////    Msg.mbLoopCorrected = mbLoopCorrected;

//    Msg.mbBad = mbBad;

//    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
//    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
//    g2o::Sim3 g2oS_c_wm(Converter::toMatrix3d(Rcw),Converter::toVector3d(tcw),1.0); //cam - world map

//    g2o::Sim3 g2oS_c_wc = g2oS_c_wm*mg2oS_wcurmap_wclientmap;

//    Eigen::Matrix3d eigR = g2oS_c_wc.rotation().toRotationMatrix();
//    Eigen::Vector3d eigt = g2oS_c_wc.translation();
//    double s = g2oS_c_wc.scale();
//    eigt *=(1./s); //[R t/s;0 1]
//    cv::Mat T_c_wc = Converter::toCvSE3(eigR,eigt);

//    Converter::CvMatToMsgArrayFixedSize<macslam_msgs::macKeyFrame::_Tcw_type,float>(T_c_wc,Msg.Tcw); //cam - world client

//    mbPoseChanged = false;

//    mbInOutBuffer = false;

//    return true;
//}

void KeyFrame::UpdateFromMessageServer(cslam_msgs::KeyFrame *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap)
{
    if(mSysState != eSystemState::SERVER)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m System Type Mismatch" << endl;
        throw infrastructure_ex();
    }

    if(!mpParent)
    {
        cout << "\033[1;31m!!!!! FATAL ERROR !!!!!\033[0m " << __func__ << __LINE__ << ": KF has no parent" << endl;
        throw infrastructure_ex();
    }

    mbOmitSending = true;

    unique_lock<mutex> lockFeats(mMutexFeatures);
    unique_lock<mutex> lockMap(mMapMutex);
    unique_lock<mutex> lockOut(mMutexOut);
//    unique_lock<mutex> lockPose(mMutexPose);


    if(!mbPoseLock)
    {
        if(pMsg->mbPoseChanged)
        {
            //manage mpParent -- might be different, since local map might not contain parent from server
            idpair MsgParentId = make_pair(pMsg->mpParent_KfId,pMsg->mpParent_KfClientId);
            cout << __func__ << __LINE__ << ": Msg parent id: " << pMsg->mpParent_KfId << "|" << pMsg->mpParent_KfClientId << " -- derived pair: " << MsgParentId.first << "|" << MsgParentId.second << endl;


            cv::Mat tempTcp = cv::Mat(4,4,5);

            Converter::MsgArrayFixedSizeToCvMat<cslam_msgs::KeyFrame::_Tcp_type,float>(tempTcp,pMsg->Tcp);

            float s = static_cast<float>(mg2oS_wcurmap_wclientmap.inverse().scale());
            tempTcp.at<float>(0,3) *=(1./s);
            tempTcp.at<float>(1,3) *=(1./s);
            tempTcp.at<float>(2,3) *=(1./s);

            if(mpParent->mId != MsgParentId)
            {
                cv::Mat T_pold_pnew;
                mpMap->GetTfToParent(pMsg->mpParent_KfId,pMsg->mpParent_KfClientId,T_pold_pnew,mpParent);
                tempTcp = tempTcp * T_pold_pnew;
            }

            this->SetPoseFromTcp(tempTcp,false);
        }
    }

    for(int idx=0;idx<pMsg->mvpMapPoints_Ids.size();++idx)
    {
        size_t FeatId = pMsg->mvpMapPoints_VectId[idx];

        if(pMsg->mvpMapPoints_Ids[idx] == defid)
        {
            cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m KeyFrame::UpdateFromMessage(..): no nullptrs should be received" << endl;
            continue;
        }

        mpptr CurMP = mvpMapPoints[FeatId];
        mpptr pMPi = mpMap->GetMpPtr(pMsg->mvpMapPoints_Ids[idx],pMsg->mvpMapPoints_ClientIds[idx]);

        if(!pMPi)
            continue; //MP is nullptr - cannot be used anyway

        size_t testId = pMPi->GetIndexInKeyFrame(shared_from_this());

        if(testId != -1)
        {
            //is already in this KF
            if(testId != FeatId)
            {
                //This means: pKF->mvpMapPoints[testId] = pMPi, and now THIS wants to also pKF->mvpMapPoints[FeatId] = pMPi
                cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << "consistency error" << endl;
            }
        }

        if(mvbMapPointsLock[FeatId])
        {
            //locked -- will not be changed
            //This means this KF already has a MP associated to Feature at FeatId, and it is locked. Maybe its pMPi, maybe another MP, or maybe the MP was deleted by the server and therefore cannot be changed by a update from client info
            continue;
        }

        if(pMPi->IsObsLocked(shared_from_this()))
        {
            //locked -- cannot change

            //This means pMPi is already associated to this KF, maybe with the Feature from FeatId, maybe with another feature, or maybe the connection was deleted by the server
            continue;
        }

        //ToDo: if mvbMapPointsLock[FeatId] != pMPi, and a lock is present, one could check if a fusion is possible

        if(CurMP)
        {
            //CurMP exists --> already a MP associated for Feature FeatId
            if(pMPi->mId == CurMP->mId)
            {
                //MPs are the same
                continue;
            }
            else
            {
                //replace
                mvpMapPoints[FeatId] = pMPi;
                pMPi->AddObservation(shared_from_this(),FeatId);
            }
        }
        else
        {
            //add
            mvpMapPoints[FeatId] = pMPi;
            pMPi->AddObservation(shared_from_this(),FeatId);
        }

//        //Add/Replace MP
//        mpptr pMP = mpMap->GetMpPtr(pMsg->mvpMapPoints_Ids[idx],pMsg->mvpMapPoints_ClientIds[idx]);

//        if(pMP)
//        {
//            mvpMapPoints[FeatId] = pMP;
//            pMP->AddObservation(shared_from_this(),FeatId);
//        }
    }

    mbOmitSending = false;
}

void KeyFrame::UpdatePoseFromMessageServer(cslam_msgs::KeyFrame *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap)
{
    if(mSysState != eSystemState::SERVER)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m System Type Mismatch" << endl;
        throw infrastructure_ex();
    }

    if(!mpParent)
    {
        cout << "\033[1;31m!!!!! FATAL ERROR !!!!!\033[0m " << __func__ << __LINE__ << ": KF has no parent" << endl;
        throw infrastructure_ex();
    }

    mbOmitSending = true;

    unique_lock<mutex> lockFeats(mMutexFeatures);
    unique_lock<mutex> lockMap(mMapMutex);
    unique_lock<mutex> lockOut(mMutexOut);
//    unique_lock<mutex> lockPose(mMutexPose);


    if(!mbPoseLock)
    {
        if(pMsg->mbPoseChanged)
        {
            //manage mpParent -- might be different, since local map might not contain parent from server
            idpair MsgParentId = make_pair(pMsg->mpParent_KfId,pMsg->mpParent_KfClientId);
            cout << __func__ << __LINE__ << ": Msg parent id: " << pMsg->mpParent_KfId << "|" << pMsg->mpParent_KfClientId << " -- derived pair: " << MsgParentId.first << "|" << MsgParentId.second << endl;


            cv::Mat tempTcp = cv::Mat(4,4,5);

            Converter::MsgArrayFixedSizeToCvMat<cslam_msgs::KeyFrame::_Tcp_type,float>(tempTcp,pMsg->Tcp);

            float s = static_cast<float>(mg2oS_wcurmap_wclientmap.inverse().scale());
            tempTcp.at<float>(0,3) *=(1./s);
            tempTcp.at<float>(1,3) *=(1./s);
            tempTcp.at<float>(2,3) *=(1./s);

            if(mpParent->mId != MsgParentId)
            {
                cv::Mat T_cref_cmsg;
                mpParent = mpMap->HandleMissingParent(pMsg->mpRefKFId,pMsg->mpRefKFClientId,T_cref_cmsg,mpParent);
                tempTcp = tempTcp * T_cref_cmsg.inv();
            }

            this->SetPoseFromTcp(tempTcp,false);
        }
    }

    mbOmitSending = false;
}

//void KeyFrame::UpdateFromMessageClient(macslam_msgs::macKeyFrame *pMsg)
//{
//    if(mSysState != eSystemState::CLIENT)
//    {
//        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m System Type Mismatch" << endl;
//        throw infrastructure_ex();
//    }

//    unique_lock<mutex> lock(mMutexPose);

//    cv::Mat tempTcw = cv::Mat(4,4,5);
//    Converter::MsgArrayFixedSizeToCvMat<macslam_msgs::macKeyFrame::_Tcw_type,float>(tempTcw,pMsg->Tcw);

//    tempTcw.copyTo(Tcw);
//    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
//    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
//    cv::Mat Rwc = Rcw.t();
//    Ow = -Rwc*tcw;

//    Twc = cv::Mat::eye(4,4,Tcw.type());
//    Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
//    Ow.copyTo(Twc.rowRange(0,3).col(3));
//    cv::Mat center = (cv::Mat_<float>(4,1) << mHalfBaseline, 0 , 0, 1);
//    Cw = Twc*center;

//    mbPoseChanged == false;
//}

} //end ns;
