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


#include <cslam/Tracking.h>

namespace cslam {

Tracking::Tracking(ccptr pCC, vocptr pVoc, viewptr pFrameViewer, mapptr pMap, dbptr pKFDB, const string &strCamPath, size_t ClientId)
    : mState(NO_IMAGES_YET),mpCC(pCC),mpORBVocabulary(pVoc), mpKeyFrameDB(pKFDB), mpInitializer(nullptr),
      mpViewer(pFrameViewer), mpMap(pMap), mLastRelocFrameId(make_pair(0,0)), mClientId(ClientId)
{
    cv::FileStorage fSettings(strCamPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    const int nFeatures = params::extractor::miNumFeatures;
    const float fScaleFactor = params::extractor::mfScaleFactor;
    const int nLevels = params::extractor::miNumLevels;
    const int iIniThFAST = params::extractor::miIniThFAST;
    const int iMinThFAST = params::extractor::miNumThFAST;

    mpORBextractor.reset(new ORBextractor(nFeatures,fScaleFactor,nLevels,iIniThFAST,iMinThFAST
                                          ));
    mpIniORBextractor.reset(new ORBextractor(2*nFeatures,fScaleFactor,nLevels,iIniThFAST,iMinThFAST
                                             ));
    if(!mpMap || !mpORBVocabulary || !mpKeyFrameDB || !mpCC)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << ": nullptr given"<< endl;
        if(!mpMap) cout << "mpMap == nullptr" << endl;
        if(!mpORBVocabulary) cout << "mpORBVocabulary == nullptr" << endl;
        if(!mpKeyFrameDB) cout << "mpKeyFrameDB == nullptr" << endl;
        if(!mpCC) cout << "mpCC == nullptr" << endl;
        throw estd::infrastructure_ex();
    }

    if(!mpViewer)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << ": nullptr given"<< endl;
        if(!mpViewer) cout << "mpViewer == nullptr" << endl;
        throw estd::infrastructure_ex();
    }
}

cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
{
    mImGray = im;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        mCurrentFrame.reset(new Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mK,mDistCoef,mClientId));
    else
    {
        mCurrentFrame.reset(new Frame(mImGray,timestamp,mpORBextractor,mpORBVocabulary,mK,mDistCoef,mClientId));
    }

    Track();

    return mCurrentFrame->mTcw.clone();
}

void Tracking::Track()
{
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;

    // Get Map Mutex -> Map cannot be changed
    while(!mpMap->LockMapUpdate()){
        usleep(params::timings::miLockSleep);
    }

    //Comm Mutex cannot be acquired here. In case of wrong initialization, there is mutual dependency in the call of reset()

    if(mState==NOT_INITIALIZED)
    {
        MonocularInitialization();

        if(params::vis::mbActive)
            mpViewer->UpdateAndDrawFrame();
        else
        {
            cout << "\033[1;35m!!! +++ Tracking: Init +++ !!!\033[0m" << endl;
        }

        if(mState!=OK)
        {
            mpMap->UnLockMapUpdate();
            return;
        }
    }
    else
    {
        // Get Communicator Mutex -> Comm cannot publish. Assure no publishing whilst changing data
        if(params::sys::mbStrictLock) while(!mpCC->LockTracking()){
            usleep(params::timings::miLockSleep);
        }

        // System is initialized. Track Frame.
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if(mState==OK)
        {
            // Local Mapping might have changed some MapPoints tracked in last frame
            CheckReplacedInLastFrame();

            if(mVelocity.empty() || mCurrentFrame->mId.first<mLastRelocFrameId.first+2)
            {
                bOK = TrackReferenceKeyFrame();
            }
            else
            {
                bOK = TrackWithMotionModel();
                if(!bOK){
                    bOK = TrackReferenceKeyFrame();
                }
            }
        }
        else
        {
            cout << "\033[1;35m!!! +++ Tracking: Lost +++ !!!\033[0m" << endl;
            bOK = false;
        }

        mCurrentFrame->mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(bOK) bOK = TrackLocalMap();

        if(bOK)
            mState = OK;
        else
            mState=LOST;

        // Update drawer
        if(params::vis::mbActive) mpViewer->UpdateAndDrawFrame();

        // If tracking were good, check if we insert a keyframe
        if(bOK)
        {
            // Update motion model
            if(!mLastFrame->mTcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame->GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame->GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame->mTcw*LastTwc;
            }
            else
                mVelocity = cv::Mat();

            // Clean VO matches
            for(int i=0; i<mCurrentFrame->N; i++)
            {
                mpptr pMP = mCurrentFrame->mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame->mvbOutlier[i] = false;
                        mCurrentFrame->mvpMapPoints[i]=nullptr;
                    }
            }

            // Check if we need to insert a new keyframe
            if(NeedNewKeyFrame())
                CreateNewKeyFrame();

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            for(int i=0; i<mCurrentFrame->N;i++)
            {
                if(mCurrentFrame->mvpMapPoints[i] && mCurrentFrame->mvbOutlier[i])
                    mCurrentFrame->mvpMapPoints[i]=nullptr;
            }
        }

        // Reset if the camera get lost soon after initialization
        if(mState==LOST)
        {
            if(mpMap->KeyFramesInMap()<=params::tracking::miInitKFs)
            {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                if(params::sys::mbStrictLock) mpCC->UnLockTracking();
                mpMap->UnLockMapUpdate();
                mpCC->mpCH->Reset();

                return;
            }
        }

        if(!mCurrentFrame->mpReferenceKF)
            mCurrentFrame->mpReferenceKF = mpReferenceKF;

        mLastFrame.reset(new Frame(*mCurrentFrame));

        if(params::sys::mbStrictLock) mpCC->UnLockTracking();
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    if(!mCurrentFrame->mTcw.empty())
    {
        cv::Mat Tcr = mCurrentFrame->mTcw*mCurrentFrame->mpReferenceKF->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame->mTimeStamp);
        mlbLost.push_back(mState==LOST);
    }
    else
    {
        // This can happen if tracking is lost
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState==LOST);
    }

    mpMap->UnLockMapUpdate();
}

void Tracking::MonocularInitialization()
{
    if(!mpInitializer)
    {
        // Set Reference Frame
        if(mCurrentFrame->mvKeys.size()>100)
        {
            mInitialFrame.reset(new Frame(*mCurrentFrame));
            mLastFrame.reset(new Frame(*mCurrentFrame));
            mvbPrevMatched.resize(mCurrentFrame->mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame->mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame->mvKeysUn[i].pt;

            if(mpInitializer) mpInitializer = nullptr;

            mpInitializer.reset(new Initializer(*mCurrentFrame,1.0,200));

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            return;
        }
    }
    else
    {
        // Try to initialize
        if((int)mCurrentFrame->mvKeys.size()<=100)
        {
            mpInitializer = nullptr;
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(*mInitialFrame,*mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

        // Check if there are enough correspondences
        if(nmatches<100)
        {
            //delete mpInitializer;
            mpInitializer = nullptr;
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if(mpInitializer->Initialize(*mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame->SetPose(cv::Mat::eye(4,4,CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame->SetPose(Tcw);

            CreateInitialMapMonocular();
        }
    }
}

void Tracking::CreateInitialMapMonocular()
{
    // Get Communicator Mutex -> Comm cannot publish. Assure no publishing whilst changing data
    while(!mpCC->LockTracking()){
        usleep(params::timings::miLockSleep);
    }

    // Create KeyFrames
    kfptr pKFini{new KeyFrame(*mInitialFrame,mpMap,mpKeyFrameDB,mpComm,eSystemState::CLIENT,-1)};
    kfptr pKFcur{new KeyFrame(*mCurrentFrame,mpMap,mpKeyFrameDB,mpComm,eSystemState::CLIENT,-1)};

    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);

        mpptr pMP{new MapPoint(worldPos,pKFcur,mpMap,mClientId,mpComm,eSystemState::CLIENT,-1)};

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame->mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame->mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpMap->AddMapPoint(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

    Optimizer::GlobalBundleAdjustemntClient(mpMap,mClientId,20);

    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;

    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100)
    {
        cout << "Wrong initialization, reseting..." << endl;
        mpCC->UnLockTracking();
        Reset();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w,false);

    // Scale points
    vector<mpptr> vpAllMapPoints = pKFini->GetMapPointMatches();

    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            mpptr pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth,false);
        }
    }

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame->SetPose(pKFcur->GetPose());
    mLastKeyFrameId=mCurrentFrame->mId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame->mpReferenceKF = pKFcur;

    mLastFrame.reset(new Frame(*mCurrentFrame));

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mState=OK;

    mpCC->UnLockTracking();
}

void Tracking::CheckReplacedInLastFrame()
{
    for(int i =0; i<mLastFrame->N; i++)
    {
        mpptr pMP = mLastFrame->mvpMapPoints[i];

        if(pMP)
        {
            mpptr pRep = pMP->GetReplaced();
            if(pRep)
            {
                vector<mpptr>::iterator vit = std::find(mLastFrame->mvpMapPoints.begin(),mLastFrame->mvpMapPoints.end(),pRep);

                if(vit != mLastFrame->mvpMapPoints.end())
                {
                    int curId = vit - mLastFrame->mvpMapPoints.begin();

                    const cv::Mat dMP = pRep->GetDescriptor();

                    const cv::Mat &dF_curId = mLastFrame->mDescriptors.row(curId);
                    const cv::Mat &dF_i = mLastFrame->mDescriptors.row(i);

                    double dist_curId = ORBmatcher::DescriptorDistance(dMP,dF_curId);
                    double dist_i = ORBmatcher::DescriptorDistance(dMP,dF_i);

                    if(dist_i <= dist_curId)
                    {
                        mLastFrame->mvpMapPoints[curId] = nullptr;
                        mLastFrame->mvpMapPoints[i] = pRep;
                    }
                    else
                    {
                        //keep old id -- do nothing
                    }
                }
                else
                {
                    mLastFrame->mvpMapPoints[i] = pRep;
                }
            }
        }
    }
}


bool Tracking::TrackReferenceKeyFrame()
{
    // Compute Bag of Words vector
    mCurrentFrame->ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<mpptr> vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpReferenceKF,*mCurrentFrame,vpMapPointMatches);

    if(nmatches<params::tracking::miTrackWithRefKfInlierThresSearch)
        return false;

    mCurrentFrame->mvpMapPoints = vpMapPointMatches;
    mCurrentFrame->SetPose(mLastFrame->mTcw);

    Optimizer::PoseOptimizationClient(*mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame->N; i++)
    {
        if(mCurrentFrame->mvpMapPoints[i])
        {
            if(mCurrentFrame->mvbOutlier[i])
            {
                mpptr pMP = mCurrentFrame->mvpMapPoints[i];

                mCurrentFrame->mvpMapPoints[i]=nullptr;
                mCurrentFrame->mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mLastFrameSeen = mCurrentFrame->mId;
                nmatches--;
            }
            else if(mCurrentFrame->mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    return nmatchesMap>=params::tracking::miTrackWithRefKfInlierThresOpt; //10
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    kfptr pRef = mLastFrame->mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();

    mLastFrame->SetPose(Tlr*pRef->GetPose());
}

bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.9,true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();

    mCurrentFrame->SetPose(mVelocity*mLastFrame->mTcw);

    fill(mCurrentFrame->mvpMapPoints.begin(),mCurrentFrame->mvpMapPoints.end(),nullptr);

    // Project points seen in previous frame
    int th;
    th=7;
    int nmatches = matcher.SearchByProjection(*mCurrentFrame,*mLastFrame,th);

    // If few matches, uses a wider window search
    if(nmatches<20)
    {
        fill(mCurrentFrame->mvpMapPoints.begin(),mCurrentFrame->mvpMapPoints.end(),nullptr);
        nmatches = matcher.SearchByProjection(*mCurrentFrame,*mLastFrame,2*th);
    }

    if(nmatches<params::tracking::miTrackWithMotionModelInlierThresSearch) //20
        return false;

    // Optimize frame pose with all matches
    Optimizer::PoseOptimizationClient(*mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame->N; i++)
    {
        if(mCurrentFrame->mvpMapPoints[i])
        {
            if(mCurrentFrame->mvbOutlier[i])
            {
                mpptr pMP = mCurrentFrame->mvpMapPoints[i];

                mCurrentFrame->mvpMapPoints[i]=nullptr;
                mCurrentFrame->mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mLastFrameSeen = mCurrentFrame->mId;
                nmatches--;
            }
            else if(mCurrentFrame->mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    return nmatchesMap>=params::tracking::miTrackWithMotionModelInlierThresOpt; //10
}

bool Tracking::TrackLocalMap()
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    UpdateLocalMap();

    SearchLocalPoints();

    // Optimize Pose
    Optimizer::PoseOptimizationClient(*mCurrentFrame);
    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for(int i=0; i<mCurrentFrame->N; i++)
    {
        if(mCurrentFrame->mvpMapPoints[i])
        {
            if(!mCurrentFrame->mvbOutlier[i])
            {
                mCurrentFrame->mvpMapPoints[i]->IncreaseFound();
                mnMatchesInliers++;
            }

        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if(mCurrentFrame->mId.first<mLastRelocFrameId.first+params::tracking::miMaxFrames && mnMatchesInliers<50)
    {
        return false;
    }

    if(mnMatchesInliers<params::tracking::miTrackLocalMapInlierThres) //30
        return false;
    else
        return true;
}


bool Tracking::NeedNewKeyFrame()
{
    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    const int nKFs = mpMap->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(mCurrentFrame->mId.first<mLastRelocFrameId.first+params::tracking::miMaxFrames && nKFs>params::tracking::miMaxFrames)
    {
        return false;
    }

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame->mId.first>=mLastKeyFrameId.first+params::tracking::miMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame->mId.first>=mLastKeyFrameId.first+params::tracking::miMinFrames && bLocalMappingIdle);
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ((mnMatchesInliers<nRefMatches*params::tracking::mfThRefRatio /*|| ratioMap<thMapRatio*/) && mnMatchesInliers>params::tracking::miMatchesInliersThres);

    if((c1a||c1b)&&c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            return false;
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame()
{
    if(!mpLocalMapper->SetNotStop(true))
        return;

    kfptr pKF{new KeyFrame(*mCurrentFrame,mpMap,mpKeyFrameDB,mpComm,eSystemState::CLIENT,-1)};

    std::vector<mpptr> vpM = pKF->GetMapPointMatches();
    for(vector<mpptr>::const_iterator vit = vpM.begin();vit!=vpM.end();++vit)
    {
        mpptr pMPi = *vit;

        if(!pMPi)
            continue;

        if(pMPi->isBad())
            continue;

        if(pMPi->mId.second != mClientId)
        {
            pMPi->SetMultiUse();
        }
    }

    mpReferenceKF = pKF;
    mCurrentFrame->mpReferenceKF = pKF;

    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);

    mLastKeyFrameId = mCurrentFrame->mId;
    mpLastKeyFrame = pKF;
}

void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    for(vector<mpptr>::iterator vit=mCurrentFrame->mvpMapPoints.begin(), vend=mCurrentFrame->mvpMapPoints.end(); vit!=vend; vit++)
    {
        mpptr pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = nullptr;
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mLastFrameSeen = mCurrentFrame->mId;
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch=0;

    int seen = 0;
    int bad = 0;
    int notinfrustrum = 0;

    // Project points in frame and check its visibility
    for(vector<mpptr>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        mpptr pMP = *vit;
        if(pMP->mLastFrameSeen == mCurrentFrame->mId)
        {
            ++seen;
            continue;
        }
        if(pMP->isBad())
        {
            ++bad;
            continue;
        }
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame->isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
        else ++notinfrustrum;
    }

    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame->mId.first<mLastRelocFrameId.first+2)
        {
            th=5;
        }

        matcher.SearchByProjection(*mCurrentFrame,mvpLocalMapPoints,th);
    }
}

void Tracking::UpdateLocalMap()
{
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
//    UpdateLocalKeyFrames();
//    UpdateLocalPoints();

    mvpLocalMapPoints = mpMap->GetAllMapPoints();
}

void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    for(vector<kfptr>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        kfptr pKF = *itKF;
        const vector<mpptr> vpMPs = pKF->GetMapPointMatches();

        int empty = 0;

        for(vector<mpptr>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            mpptr pMP = *itMP;
            if(!pMP)
            {
                ++empty;
                continue;
            }
            if(pMP->mTrackReferenceForFrame==mCurrentFrame->mId)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mTrackReferenceForFrame=mCurrentFrame->mId;
            }
        }
    }
}


void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<kfptr,int> keyframeCounter;
    for(int i=0; i<mCurrentFrame->N; i++)
    {
        if(mCurrentFrame->mvpMapPoints[i])
        {
            mpptr pMP = mCurrentFrame->mvpMapPoints[i];
            if(!pMP->isBad())
            {
                const map<kfptr,size_t> observations = pMP->GetObservations();
                for(map<kfptr,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrame->mvpMapPoints[i]=nullptr;
            }
        }
    }

    if(keyframeCounter.empty())
        return;

    int max=0;
    kfptr pKFmax= nullptr;

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<kfptr,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        kfptr pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        pKF->mTrackReferenceForFrame = mCurrentFrame->mId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<kfptr>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80)
            break;

        kfptr pKF = *itKF;

        const vector<kfptr> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(vector<kfptr>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            kfptr pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mTrackReferenceForFrame!=mCurrentFrame->mId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mTrackReferenceForFrame=mCurrentFrame->mId;
                    break;
                }
            }
        }

        const set<kfptr> spChilds = pKF->GetChilds();
        for(set<kfptr>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            kfptr pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mTrackReferenceForFrame!=mCurrentFrame->mId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mTrackReferenceForFrame=mCurrentFrame->mId;
                    break;
                }
            }
        }

        kfptr pParent = pKF->GetParent();
        if(pParent)
        {
            if(pParent->mTrackReferenceForFrame!=mCurrentFrame->mId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mTrackReferenceForFrame=mCurrentFrame->mId;
                break;
            }
        }
    }

    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame->mpReferenceKF = mpReferenceKF;
    }
}

void Tracking::Reset()
{
    cout << "System Reseting" << endl;
    mpViewer->RequestReset();
    mpLocalMapper->RequestReset();
    mpKeyFrameDB->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;
    MapPoint::nNextId = 0;

    if(mpInitializer)
        mpInitializer = nullptr;

    mpComm->RequestReset();

    mpMap->clear();

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    cout << "Reseting Done..." << endl;
}

Tracking::kfptr Tracking::GetReferenceKF()
{
    //Works w/o mutex, since tracking is locked when this method is called by comm

    if(!mpCC->IsTrackingLocked())
        cout << "\033[1;33m!!! WARN !!!\033[0m " << __func__ << ":" << __LINE__ << " Tracking assumed to be locked " << endl;

    return mpReferenceKF;
}

} //end ns
