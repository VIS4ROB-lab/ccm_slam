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

#include <cslam/LoopFinder.h>

namespace cslam {

LoopFinder::LoopFinder(ccptr pCC, dbptr pDB, vocptr pVoc, mapptr pMap)
    : mpCC(pCC), mpKFDB(pDB), mpVoc(pVoc), mpMap(pMap),
      mbResetRequested(false),
      mbFixScale(false),mKFcount(0),
      mKFNewLoopThres(params::placerec::miNewLoopThres),
      mnCovisibilityConsistencyTh(params::placerec::miCovisibilityConsistencyTh)
{
    stringstream* ss;
    ss = new stringstream;
    *ss << "LoopMarker" << mpCC->mClientId;
    string PubTopicName = ss->str();
    delete ss;


    mPubMarker = mNh.advertise<visualization_msgs::Marker>(PubTopicName,10);

    if(!mpCC || !mpKFDB || !mpVoc || !mpMap)
    {
        cout << ": In \"LoopFinder::LoopFinder(...)\": nullptr ex" << endl;
        throw estd::infrastructure_ex();
    }

    if(params::vis::mbActive)
    {
        mMarkerMsg.type = visualization_msgs::Marker::LINE_LIST;
        mMarkerMsg.color = Colors::msgRed();
        mMarkerMsg.action = visualization_msgs::Marker::ADD;
        mMarkerMsg.scale.x = params::vis::mfLoopMarkerSize;
        mMarkerMsg.id = 1;
    }
}

void LoopFinder::Run()
{
    while(1)
    {
        #ifdef LOGGING
        mpCC->mpLogger->SetLoop(__LINE__,0);
        #endif

        #ifdef DONOTINTERRUPTMERGE
        if(CheckKfQueue() && !mpMap->isMergeStepGBA()) //should either way not close loops while map merging, but who knows, maybe buffer is full or sth like this...
        #else
        if(CheckKfQueue())
        #endif
        {
            while(!mpCC->LockPlaceRec()){usleep(params::timings::miLockSleep);}

            // Detect loop candidates and check covisibility consistency
            bool bDetect = DetectLoop();

            if(bDetect)
            {
                // Compute similarity transformation [sR|t]
                bool bSim3 = ComputeSim3();
                if(bSim3)
                {
                    // Perform loop fusion and pose graph optimization
                    CorrectLoop();
                }
            }
            mpCC->UnLockPlaceRec();
        }

        ResetIfRequested();

        #ifdef LOGGING
        mpCC->mpLogger->SetLoop(__LINE__,this->mpCC->mClientId);
        #endif

        usleep(params::timings::server::miPlaceRecRateRate);
    }
}

bool LoopFinder::DetectLoop()
{
    {
        unique_lock<mutex> lock(mMutexKfInQueue);
        mpCurrentKF = mlKfInQueue.front();
        mlKfInQueue.pop_front();
        // Avoid that a keyframe can be erased while it is being process by this thread
        mpCurrentKF->SetNotErase();
    }

    ++mKFcount;

    //If the map contains less than 10 KF or less than 10 KF have passed from last loop detection
    if(mpMap->GetAllKeyFrames().size() < 10 || mKFcount < mKFNewLoopThres)
    {
        mpCurrentKF->SetErase();
        return false;
    }

    // Compute reference BoW similarity score
    // This is the lowest score to a connected keyframe in the covisibility graph
    // We will impose loop candidates to have a higher similarity than this
    const vector<kfptr> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();
    const DBoW2::BowVector &CurrentBowVec = mpCurrentKF->mBowVec;
    float minScore = 1;
    for(size_t i=0; i<vpConnectedKeyFrames.size(); i++)
    {
        kfptr pKF = vpConnectedKeyFrames[i];
        if(pKF->isBad())
            continue;
        const DBoW2::BowVector &BowVec = pKF->mBowVec;

        float score = mpVoc->score(CurrentBowVec, BowVec);

        if(score<minScore)
            minScore = score;
    }

    // Query the database imposing the minimum score
    vector<kfptr> vpCandidateKFs = mpKFDB->DetectLoopCandidates(mpCurrentKF, minScore*0.8);

    // If there are no loop candidates, just add new keyframe and return false
    if(vpCandidateKFs.empty())
    {
        mvConsistentGroups.clear();
        mpCurrentKF->SetErase();
        return false;
    }

    // For each loop candidate check consistency with previous loop candidates
    // Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
    // A group is consistent with a previous group if they share at least a keyframe
    // We must detect a consistent loop in several consecutive keyframes to accept it
    mvpEnoughConsistentCandidates.clear();

    vector<ConsistentGroup> vCurrentConsistentGroups; //pair <set<KF*>,int> --> int counts consistent groups found for this group
    vector<bool> vbConsistentGroup(mvConsistentGroups.size(),false);
    //mvConsistentGroups stores the last found consistent groups.

    for(size_t i=0, iend=vpCandidateKFs.size(); i<iend; i++)
    {
        kfptr pCandidateKF = vpCandidateKFs[i];

        set<kfptr> spCandidateGroup = pCandidateKF->GetConnectedKeyFrames();
        spCandidateGroup.insert(pCandidateKF);
        //group with candidate and connected KFs

        bool bEnoughConsistent = false;
        bool bConsistentForSomeGroup = false;
        for(size_t iG=0, iendG=mvConsistentGroups.size(); iG<iendG; iG++)
        {
            set<kfptr> sPreviousGroup = mvConsistentGroups[iG].first;

            bool bConsistent = false;
            for(set<kfptr>::iterator sit=spCandidateGroup.begin(), send=spCandidateGroup.end(); sit!=send;sit++)
            {
                if(sPreviousGroup.count(*sit))
                {
                    //KF found that is contained in candidate's group and comparison group
                    bConsistent=true;
                    bConsistentForSomeGroup=true;
                    break;
                }
            }

            if(bConsistent)
            {
                int nPreviousConsistency = mvConsistentGroups[iG].second;
                int nCurrentConsistency = nPreviousConsistency + 1;
                if(!vbConsistentGroup[iG])
                {
                    ConsistentGroup cg = make_pair(spCandidateGroup,nCurrentConsistency);
                    vCurrentConsistentGroups.push_back(cg);
                    vbConsistentGroup[iG]=true; //this avoids to include the same group more than once
                }
                if(nCurrentConsistency>=mnCovisibilityConsistencyTh && !bEnoughConsistent)
                {
                    mvpEnoughConsistentCandidates.push_back(pCandidateKF);
                    bEnoughConsistent=true; //this avoids to insert the same candidate more than once
                }
            }
        }

        // If the group is not consistent with any previous group insert with consistency counter set to zero
        if(!bConsistentForSomeGroup)
        {
            ConsistentGroup cg = make_pair(spCandidateGroup,0); //For "ConsistentGroup" the "int" is initialized with 0
            vCurrentConsistentGroups.push_back(cg);
        }
    }

    // Update Covisibility Consistent Groups
    mvConsistentGroups = vCurrentConsistentGroups;

    if(mvpEnoughConsistentCandidates.empty())
    {
        mpCurrentKF->SetErase();
        return false;
    }
    else
    {
        return true;
    }

    mpCurrentKF->SetErase();
    return false;
}

bool LoopFinder::ComputeSim3()
{
    const int nInitialCandidates = mvpEnoughConsistentCandidates.size();
    //vector with KF candidates that could match -- no ordering

    // We compute first ORB matches for each candidate
    // If enough matches are found, we setup a Sim3Solver
    ORBmatcher matcher(0.75,true);

    vector<Sim3Solver*> vpSim3Solvers;
    vpSim3Solvers.resize(nInitialCandidates);

    vector<vector<mpptr> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nInitialCandidates);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nInitialCandidates);

    int nCandidates=0; //candidates with enough matches

    for(int i=0; i<nInitialCandidates; i++)
    {
        kfptr pKF = mvpEnoughConsistentCandidates[i];

        // avoid that local mapping erase it while it is being processed in this thread
        pKF->SetNotErase();

        if(pKF->isBad())
        {
            vbDiscarded[i] = true;
            cout << "MapMatcher::ComputeSim3(): bad KF: " << endl;
            continue;
        }

        int nmatches = matcher.SearchByBoW(mpCurrentKF,pKF,vvpMapPointMatches[i]);
        //vvpMapPointMatches[KF candidate index][CurKF MP-IDc] = best matching MP of candidate KF for MP-IDc of CurKF

        if(nmatches<params::opt::mMatchesThres)
        {
            vbDiscarded[i] = true;
            continue;
        }
        else
        {
            Sim3Solver* pSolver = new Sim3Solver(mpCurrentKF,pKF,vvpMapPointMatches[i],mbFixScale);

            pSolver->SetRansacParameters(params::opt::mProbability,params::opt::mMinInliers,params::opt::mMaxIterations);
            vpSim3Solvers[i] = pSolver;
        }

        nCandidates++;
    }

    bool bMatch = false;

    // Perform alternatively RANSAC iterations for each candidate
    // until one is succesful or all fail
    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nInitialCandidates; i++)
        {
            if(vbDiscarded[i])
                continue;

            kfptr pKF = mvpEnoughConsistentCandidates[i];

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            Sim3Solver* pSolver = vpSim3Solvers[i];
            cv::Mat Scm  = pSolver->iterate(params::opt::mSolverIterations,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
                //cout << "MapMatcher::ComputeSim3(): discard KF: " << endl;
            }

            // If RANSAC returns a Sim3, perform a guided matching and optimize with all correspondences
            if(!Scm.empty())
            {
                vector<mpptr> vpMapPointMatches(vvpMapPointMatches[i].size(), static_cast<mpptr>(NULL));
                for(size_t j=0, jend=vbInliers.size(); j<jend; j++)
                {
                    if(vbInliers[j])
                       vpMapPointMatches[j]=vvpMapPointMatches[i][j];
                }

                cv::Mat R = pSolver->GetEstimatedRotation();
                cv::Mat t = pSolver->GetEstimatedTranslation();
                const float s = pSolver->GetEstimatedScale();
                matcher.SearchBySim3(mpCurrentKF,pKF,vpMapPointMatches,s,R,t,7.5);

                g2o::Sim3 gScm(Converter::toMatrix3d(R),Converter::toVector3d(t),s);

                const int nInliers = Optimizer::OptimizeSim3(mpCurrentKF, pKF, vpMapPointMatches, gScm, 10, mbFixScale);

                // If optimization is succesful stop ransacs and continue
                if(nInliers>=params::opt::mInliersThres)
                {
                    bMatch = true;
                    mpMatchedKF = pKF;
                    g2o::Sim3 gSmw(Converter::toMatrix3d(pKF->GetRotation()),Converter::toVector3d(pKF->GetTranslation()),1.0);
                    mg2oScw = gScm*gSmw;
                    mScw = Converter::toCvMat(mg2oScw);

                    mvpCurrentMatchedPoints = vpMapPointMatches;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        for(int i=0; i<nInitialCandidates; i++)
             mvpEnoughConsistentCandidates[i]->SetErase();
        mpCurrentKF->SetErase();
        return false;
    }

    // Retrieve MapPoints seen in Loop Keyframe and neighbors
    vector<kfptr> vpLoopConnectedKFs = mpMatchedKF->GetVectorCovisibleKeyFrames();
    vpLoopConnectedKFs.push_back(mpMatchedKF);
    mvpLoopMapPoints.clear();
    for(vector<kfptr>::iterator vit=vpLoopConnectedKFs.begin(); vit!=vpLoopConnectedKFs.end(); vit++)
    {
        kfptr pKF = *vit;
        vector<mpptr> vpMapPoints = pKF->GetMapPointMatches();
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            mpptr pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad() && pMP->mLoopPointForKF_LC!=mpCurrentKF->mId) //ID Tag
                {
                    mvpLoopMapPoints.push_back(pMP);
                    pMP->mLoopPointForKF_LC = mpCurrentKF->mId;
                }
            }
        }
    }

    // Find more matches projecting with the computed Sim3
    // so far, mvpCurrentMatchedPoints[CurKF MP-IDc] = best matching MP of MatchKF for MP-IDc of CurKF
    matcher.SearchByProjection(mpCurrentKF, mScw, mvpLoopMapPoints, mvpCurrentMatchedPoints,10);
    // finds more MPs. It is possible that an MP is already observed by CurKF (also in LC case, e.g. because of a previous loop that established the connection), but this method finds another descriptor in CurKF that fits better. Than later when adding/replacing MPs, this MP is observed twice by CurKF, since the ids of new and exisitng observation don't match -- need to catch this

    // If enough matches accept Loop
    int nTotalMatches = 0;
    for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
    {
        if(mvpCurrentMatchedPoints[i])
            nTotalMatches++;
    }

    if(nTotalMatches>=params::opt::mTotalMatchesThres)
    {
        for(int i=0; i<nInitialCandidates; i++)
            if(mvpEnoughConsistentCandidates[i]!=mpMatchedKF)
                mvpEnoughConsistentCandidates[i]->SetErase();
        return true;
    }
    else
    {
        for(int i=0; i<nInitialCandidates; i++)
            mvpEnoughConsistentCandidates[i]->SetErase();
        mpCurrentKF->SetErase();
        return false;
    }
}

void LoopFinder::CorrectLoop()
{
    cout << "\033[1;32m!!! LOOP FOUND !!!\033[0m" << endl;

    this->PublishLoopEdges();

    // Make sure no other module can start GBA

    bool bProblem = mpMap->isNoStartGBA();

    if(bProblem)
    {
        std::cout << __func__ << ":" << __LINE__ << "LC Module " << mpCC->mClientId << ": Rejecting Loop Closure because GBA is active" << std::endl;
        return;
        //below can cause deadlock - merging has locked GBA and wants to lock PlaceRec, PlaceRec waits for GBA to get available
//        std::cout << __func__ << ":" << __LINE__ << "LC Module " << mpCC->mClientId << ": Waiting for GBA to be able to Start" << std::endl;
//        while(mpMap->isNoStartGBA())
//        {
//            usleep(params::timings::miLockSleep);
//        }
//        std::cout << __func__ << ":" << __LINE__  << "Continue" << std::endl;
    }



    mpMap->setNoStartGBA();

    // If a Global Bundle Adjustment is running, abort it
    if(mpMap->isRunningGBA())
    {
        mpMap->StopGBA();
    }

    set<idpair> sChangedKFs;

    bool b0 = false;
    bool b1 = false;
    bool b2 = false;
    bool b3 = false;

    unique_lock<mutex> lockComm0, lockComm1, lockComm2, lockComm3;
    unique_lock<mutex> lockMapping0, lockMapping1, lockMapping2, lockMapping3;

    set<ccptr> spCC = mpMap->GetCCPtrs();
    if(spCC.size() != mpMap->msuAssClients.size()) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": spCC.size() != mpMap->msuAssClients.size()" << endl;

    cout << "--- Lock threads" << endl;

    for(set<ccptr>::iterator sit = spCC.begin();sit!=spCC.end();++sit)
    {
        ccptr pCC = *sit;

        if(pCC->mClientId > 3) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId out of bounds (" << pCC->mClientId << ")" << endl;
        if(!(mpMap->msuAssClients.count(pCC->mClientId))) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId in pCC but not in msuAssClients" << endl;
        switch(pCC->mClientId)
        {
            case(static_cast<size_t>(0)):
                if(b0) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId found twice" << endl;
                b0 = true;
                while(!pCC->LockComm()){usleep(params::timings::miLockSleep);}
                #ifdef LOGGING
//                pCC->mpLogger->SetMappingLock(__LINE__,pCC->mClientId);
                #endif
                while(!pCC->LockMapping()){usleep(params::timings::miLockSleep);}
                break;
            case(static_cast<size_t>(1)):
                if(b1) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId found twice" << endl;
                b1 = true;
                while(!pCC->LockComm()){usleep(params::timings::miLockSleep);}
                #ifdef LOGGING
//                pCC->mpLogger->SetMappingLock(__LINE__,pCC->mClientId);
                #endif
                while(!pCC->LockMapping()){usleep(params::timings::miLockSleep);}
                break;
            case(static_cast<size_t>(2)):
                if(b2) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId found twice" << endl;
                b2 = true;
                while(!pCC->LockComm()){usleep(params::timings::miLockSleep);}
                #ifdef LOGGING
//                pCC->mpLogger->SetMappingLock(__LINE__,pCC->mClientId);
                #endif
                while(!pCC->LockMapping()){usleep(params::timings::miLockSleep);}
                break;
            case(static_cast<size_t>(3)):
                if(b3) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId found twice" << endl;
                b3 = true;
                while(!pCC->LockComm()){usleep(params::timings::miLockSleep);}
                #ifdef LOGGING
//                pCC->mpLogger->SetMappingLock(__LINE__,pCC->mClientId);
                #endif
                while(!pCC->LockMapping()){usleep(params::timings::miLockSleep);}
                break;
            default: cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId out of bounds" << endl;
        }
    }

    #ifdef LOGGING
    mpCC->mpLogger->SetLoop(__LINE__,0);
    #endif

    for(set<ccptr>::iterator sit = spCC.begin();sit!=spCC.end();++sit)
    {
        ccptr pCC = *sit;
        pCC->mbOptActive = true;
    }

    // Ensure current keyframe is updated
    mpCurrentKF->UpdateConnections();

    // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
    mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
    mvpCurrentConnectedKFs.push_back(mpCurrentKF);

    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
    CorrectedSim3[mpCurrentKF]=mg2oScw;
    cv::Mat Twc = mpCurrentKF->GetPoseInverse();

    cv::Mat Rcur = Twc.rowRange(0,3).colRange(0,3);
    cv::Mat tcur = Twc.rowRange(0,3).col(3);
//    g2o::Sim3 g2oScur(Converter::toMatrix3d(Rcur),Converter::toVector3d(tcur),1.0);

    mKFcount = 0;

    #ifdef LOGGING
    mpCC->mpLogger->SetLoop(__LINE__,0);
    #endif

    // Get Map Mutex
    while(!mpMap->LockMapUpdate()){usleep(params::timings::miLockSleep);}

    #ifdef LOGGING
    mpCC->mpLogger->SetLoop(__LINE__,0);
    #endif

    cout << "--- transform KFs/MPs" << endl;

    for(vector<kfptr>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
    {
        kfptr pKFi = *vit;

        cv::Mat Tiw = pKFi->GetPose();

        if(pKFi!=mpCurrentKF)
        {
            cv::Mat Tic = Tiw*Twc;
            cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
            cv::Mat tic = Tic.rowRange(0,3).col(3);
            g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);
            g2o::Sim3 g2oCorrectedSiw = g2oSic*mg2oScw;
            //Pose corrected with the Sim3 of the loop closure
            CorrectedSim3[pKFi]=g2oCorrectedSiw;
        }

        cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
        cv::Mat tiw = Tiw.rowRange(0,3).col(3);
        g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
        //Pose without correction
        NonCorrectedSim3[pKFi]=g2oSiw;
    }

    // Correct all MapPoints obsrved by current keyframe and neighbors, so that they align with the other side of the loop
    for(KeyFrameAndPose::iterator mit=CorrectedSim3.begin(), mend=CorrectedSim3.end(); mit!=mend; mit++)
    {
        kfptr pKFi = mit->first;
        g2o::Sim3 g2oCorrectedSiw = mit->second;
        g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

        g2o::Sim3 g2oSiw =NonCorrectedSim3[pKFi];

        vector<mpptr> vpMPsi = pKFi->GetMapPointMatches();
        for(size_t iMP=0, endMPi = vpMPsi.size(); iMP<endMPi; iMP++)
        {
            mpptr pMPi = vpMPsi[iMP];
            if(!pMPi)
                continue;
            if(pMPi->isBad())
                continue;
            if(pMPi->mCorrectedByKF_LC==mpCurrentKF->mId)
                continue;

            // Project with non-corrected pose and project back with corrected pose
            cv::Mat P3Dw = pMPi->GetWorldPos();
            Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
            Eigen::Matrix<double,3,1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));

            cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
            pMPi->SetWorldPos(cvCorrectedP3Dw,true);
            pMPi->mCorrectedByKF_LC = mpCurrentKF->mId; //ID Tag
            pMPi->mCorrectedReference_LC = mpCurrentKF->mUniqueId;
            pMPi->UpdateNormalAndDepth();
        }

        // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
        Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
        Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
        double s = g2oCorrectedSiw.scale();

        eigt *=(1./s); //[R t/s;0 1]

        cv::Mat correctedTiw = Converter::toCvSE3(eigR,eigt);

        pKFi->SetPose(correctedTiw,true);
        sChangedKFs.insert(pKFi->mId);

        // Make sure connections are updated
        pKFi->UpdateConnections();
    }

    cout << "--- update matched MPs" << endl;

    // Start Loop Fusion
    // Update matched map points and replace if duplicated
    for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
    {
        if(mvpCurrentMatchedPoints[i])
        {
            if(mvpCurrentMatchedPoints[i]->GetIndexInKeyFrame(mpCurrentKF) != -1)
            {
                continue;
            }

            mpptr pLoopMP = mvpCurrentMatchedPoints[i];
            mpptr pCurMP = mpCurrentKF->GetMapPoint(i);
            if(pCurMP)
                pCurMP->Replace(pLoopMP,true);
            else
            {
                mpCurrentKF->AddMapPoint(pLoopMP,i,true);
                pLoopMP->AddObservation(mpCurrentKF,i,true);
                pLoopMP->ComputeDistinctiveDescriptors();
            }
        }
    }

    // Project MapPoints observed in the neighborhood of the loop keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    SearchAndFuse(CorrectedSim3,mvpLoopMapPoints);

    // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
    map<kfptr, set<kfptr> > LoopConnections;

    for(vector<kfptr>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
    {
        kfptr pKFi = *vit;
        vector<kfptr> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();

        // Update connections. Detect new links.
        pKFi->UpdateConnections();
        LoopConnections[pKFi]=pKFi->GetConnectedKeyFrames();
        for(vector<kfptr>::iterator vit_prev=vpPreviousNeighbors.begin(), vend_prev=vpPreviousNeighbors.end(); vit_prev!=vend_prev; vit_prev++)
        {
            LoopConnections[pKFi].erase(*vit_prev);
        }
        for(vector<kfptr>::iterator vit2=mvpCurrentConnectedKFs.begin(), vend2=mvpCurrentConnectedKFs.end(); vit2!=vend2; vit2++)
        {
            LoopConnections[pKFi].erase(*vit2);
        }
    }

    cout << "--- optimize Ess grapgh" << endl;

    // Optimize graph
    Optimizer::OptimizeEssentialGraphLoopClosure(mpMap, mpMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections, mbFixScale);

    // Add loop edge
    mpMatchedKF->AddLoopEdge(mpCurrentKF);
    mpCurrentKF->AddLoopEdge(mpMatchedKF);

    idpair nLoopKF = mpCurrentKF->mId;

    mpMap->setRunningGBA();
    mpMap->setFinishedGBA();
    mpMap->mbStopGBA = false;

    #ifdef DEBUGGING2
    mpMap->CheckStructure();
    #endif

    // Launch a new thread to perform Global Bundle Adjustment
    cout << "--- Launch GBA thread" << endl;
    mpMap->mpThreadGBA = new thread(&LoopFinder::RunGBA,this,nLoopKF,sChangedKFs);

    std::cout << "LoopFinder: Wait for GBA to finish" << std::endl;
    while(mpMap->isRunningGBA()) {
        usleep(10000);
    }
    std::cout << "LoopFinder: GBA finished - continue" << std::endl;

    for(set<ccptr>::iterator sit = spCC.begin();sit!=spCC.end();++sit)
    {
        ccptr pCC = *sit;
        pCC->UnLockComm();
    }
    mpMap->UnLockMapUpdate();
    mpMap->unsetNoStartGBA();

    cout << "\033[1;32m!!! LOOP CLOSED !!!\033[0m" << endl;
    usleep(1000000);
    this->ClearLoopEdges();
}

void LoopFinder::SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap, std::vector<mpptr> vpLoopMapPoints)
{
    ORBmatcher matcher(0.8);

    for(KeyFrameAndPose::const_iterator mit=CorrectedPosesMap.begin(), mend=CorrectedPosesMap.end(); mit!=mend;mit++)
    {
        kfptr pKF = mit->first;

        g2o::Sim3 g2oScw = mit->second;
        cv::Mat cvScw = Converter::toCvMat(g2oScw);

        vector<mpptr> vpReplacePoints(vpLoopMapPoints.size(),nullptr);
        matcher.Fuse(pKF,cvScw,vpLoopMapPoints,4,vpReplacePoints);

        // Get Map Mutex
        const int nLP = vpLoopMapPoints.size();
        for(int i=0; i<nLP;i++)
        {
            mpptr pRep = vpReplacePoints[i];
            if(pRep)
            {
                pRep->Replace(vpLoopMapPoints[i],true);
            }
        }
    }
}

void LoopFinder::InsertKF(kfptr pKF)
{
    unique_lock<mutex> lock(mMutexKfInQueue);
    mlKfInQueue.push_back(pKF);
}

int LoopFinder::GetNumKFsinQueue()
{
    unique_lock<mutex> lock(mMutexKfInQueue,defer_lock);
    if(lock.try_lock())
    {
        return mlKfInQueue.size();
    }
    else
        return -1;
}

bool LoopFinder::CheckKfQueue()
{
    unique_lock<mutex> lock(mMutexKfInQueue);
    return (!mlKfInQueue.empty());
}

void LoopFinder::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        mlKfInQueue.clear();
        mKFcount=0;

        vector<kfptr> vpKFs = mpMap->GetAllKeyFrames();
        for(vector<kfptr>::iterator vit=vpKFs.begin();vit!=vpKFs.end();++vit)
        {
            kfptr pKF = *vit;
            mpKFDB->erase(pKF);
        }

        mbResetRequested=false;
    }
}

void LoopFinder::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequested)
            {
                break;
            }
        }
        usleep(params::timings::server::miPlaceRecRateRate);
    }
}

void LoopFinder::PublishLoopEdges()
{
    stringstream* ss;
    ss = new stringstream;
    *ss << "LoopEdgesClient" << mpCC->mClientId << "_red";
    string ns = ss->str();
    delete ss;

    mMarkerMsg.header.frame_id = mpMap->mOdomFrame;
    mMarkerMsg.header.stamp = ros::Time::now();
    mMarkerMsg.ns = ns;
    mMarkerMsg.points.clear();

    cv::Mat T1 = mpCurrentKF->GetPoseInverse();
    cv::Mat T2 = mpMatchedKF->GetPoseInverse();

    geometry_msgs::Point p1;
    geometry_msgs::Point p2;

    p1.x = params::vis::mfScaleFactor*((double)(T1.at<float>(0,3)));
    p1.y = params::vis::mfScaleFactor*((double)(T1.at<float>(1,3)));
    p1.z = params::vis::mfScaleFactor*((double)(T1.at<float>(2,3)));

    p2.x = params::vis::mfScaleFactor*((double)(T2.at<float>(0,3)));
    p2.y = params::vis::mfScaleFactor*((double)(T2.at<float>(1,3)));
    p2.z = params::vis::mfScaleFactor*((double)(T2.at<float>(2,3)));

    mMarkerMsg.points.push_back(p1);
    mMarkerMsg.points.push_back(p2);

    mPubMarker.publish(mMarkerMsg);
}

void LoopFinder::ClearLoopEdges()
{
    stringstream* ss;
    ss = new stringstream;
    *ss << "LoopEdgesClient" << mpCC->mClientId << "_red";
    string ns = ss->str();
    delete ss;

    visualization_msgs::Marker MarkerMsg;

    MarkerMsg.header.frame_id = mpMap->mOdomFrame;
    MarkerMsg.header.stamp = ros::Time::now();
    MarkerMsg.ns = ns;
    MarkerMsg.action = 3;
    MarkerMsg.id = 1;

    mPubMarker.publish(MarkerMsg);
}

//#ifdef BADEV
void LoopFinder::ChangeMap(mapptr pMap)
{
    if(mpMap->isRunningGBA())
    {
        mpMap->StopGBA();
    }

    mpMap = pMap;
}

void LoopFinder::RunGBA(idpair nLoopKF, set<idpair> sChangedKFs)
{
    cout << "-> Starting Global Bundle Adjustment" << endl;

//    struct timeval tStart,tNow;
//    double dEl;
//    gettimeofday(&tStart,NULL);

    Optimizer::MapFusionGBA(mpMap,mpCC->mClientId,params::opt::mGBAIterations,&(mpMap->mbStopGBA),nLoopKF,true);

//    gettimeofday(&tNow,NULL);
//    dEl = (tNow.tv_sec - tStart.tv_sec) + (tNow.tv_usec - tStart.tv_usec) / 1000000.0;
//    cout << "Optimization Time: Agents|KFs|MPs|Time: " << mpMap->GetCCPtrs().size() << "|" << mpMap->GetAllKeyFrames().size() << "|" << mpMap->GetAllMapPoints().size() << "|" << dEl << endl;

    set<ccptr> spCC = mpMap->GetCCPtrs();

//    cout << "\033[1;33m!!! WARN !!!\033[0m Client " << mpCC->mClientId << ": In \"LoopFinder::CorrectLoop()\": fix GBA loop KF num when fixing id tags" << endl;

    #ifdef FINALBA
    if(!mpMap->mbStopGBA)
    #endif
    {
        unique_lock<mutex> lock(mpMap->mMutexGBA);

////        mpMap->LockMapUpdate()
//        while(!mpMap->LockMapUpdate()){usleep(params::timings::miLockSleep);}

        cout << "-> Global Bundle Adjustment finished" << endl;
        cout << "-> Updating map ..." << endl;

        // Correct keyframes starting at map first keyframe
        list<kfptr> lpKFtoCheck(mpMap->mvpKeyFrameOrigins.begin(),mpMap->mvpKeyFrameOrigins.end());

        cout << "--> Updating KFs ..." << endl;

        while(!lpKFtoCheck.empty())
        {
            kfptr pKF = lpKFtoCheck.front();
            const set<kfptr> sChilds = pKF->GetChilds();
            cv::Mat Twc = pKF->GetPoseInverse();
            for(set<kfptr>::const_iterator sit=sChilds.begin();sit!=sChilds.end();sit++)
            {
                kfptr pChild = *sit;
                if(pChild->mBAGlobalForKF!=nLoopKF)
                {
                    cv::Mat Tchildc = pChild->GetPose()*Twc;
                    pChild->mTcwGBA = Tchildc*pKF->mTcwGBA;//*Tcorc*pKF->mTcwGBA;
                    #ifdef DEBUGGING2
                    if(!(pChild->mTcwGBA.dims >= 2))
                        std::cout << COUTERROR << " KF" << pChild->mId.first << "|" << pChild->mId.second << ": !(pChild->mTcwGBA.dims >= 2)" << std::endl;
                    #endif
                    pChild->mBAGlobalForKF=nLoopKF;

                }
                lpKFtoCheck.push_back(pChild);
            }

            #ifdef DEBUGGING2
            if(!(pKF->mTcwGBA.dims >= 2))
                std::cout << COUTERROR << " KF" << pKF->mId.first << "|" << pKF->mId.second << ": !(pKF->mTcwGBA.dims >= 2)" << std::endl;
            #endif

            pKF->mTcwBefGBA = pKF->GetPose();
            #ifdef DEBUGGING2
            if(!(pKF->mTcwBefGBA.dims >= 2))
                std::cout << COUTERROR << " KF" << pKF->mId.first << "|" << pKF->mId.second << ": !(pKF->mTcwBefGBA.dims >= 2)" << std::endl;
            #endif
            pKF->SetPose(pKF->mTcwGBA,true);
            pKF->mbLoopCorrected = true;
            sChangedKFs.insert(pKF->mId);
            lpKFtoCheck.pop_front();
        }

        cout << "--> Updating MPs ..." << endl;

        // Correct MapPoints
        const vector<mpptr> vpMPs = mpMap->GetAllMapPoints();

        for(size_t i=0; i<vpMPs.size(); i++)
        {
            mpptr pMP = vpMPs[i];

            if(pMP->isBad())
                continue;

    //        if(pMP->mnBAGlobalForKF==nLoopKF)
            if(pMP->mBAGlobalForKF==nLoopKF)
            {
                // If optimized by Global BA, just update
                #ifdef DEBUGGING2
                if(!(pMP->mPosGBA.dims >= 2))
                    std::cout << COUTERROR << " MP" << pMP->mId.first << "|" << pMP->mId.second << ": !(pMP->mPosGBA.dims >= 2)" << std::endl;
                #endif
                pMP->SetWorldPos(pMP->mPosGBA,true);
                pMP->mbLoopCorrected = true;
            }
            else
            {
                // Update according to the correction of its reference keyframe
                kfptr pRefKF = pMP->GetReferenceKeyFrame();

                if(!pRefKF)
                {
                    cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": pRefKf is nullptr" << endl;
                    continue;
                }

                if(pRefKF->mBAGlobalForKF!=nLoopKF)
                    continue;

                #ifdef DEBUGGING2
                if(!(pRefKF->mTcwBefGBA.dims >= 2))
                {
                    std::cout << COUTERROR << " KF" << pRefKF->mId.first << "|" << pRefKF->mId.second << ": !(pRefKF->mTcwBefGBA.dims >= 2)" << std::endl;
                    std::cout << "bad? " << (int)pRefKF->isBad() << std::endl;
                    std::cout << "mBAGlobalForKF: " << pRefKF->mBAGlobalForKF.first << "|" << pRefKF->mBAGlobalForKF.second << std::endl;
                    std::cout << "nLoopKF: " << nLoopKF.first << "|" << nLoopKF.second << std::endl;
//                    kfptr pKFp = pRefKF->GetParent();
//                    std::cout << "Parent " << pKFp->mId.first << "|" << pKFp->mId.second << " -- bad: " << (int)pKFp->isBad() << std::endl;

                    std::cout << "KF Lineage: " << std::endl;
                    kfptr pKFp = pRefKF->GetParent();
                    while(pKFp)
                    {
                        std::cout << "--> " << pKFp->mId.first << "|" << pKFp->mId.second << " -- bad: " << (int)pKFp->isBad() << std::endl;
                        if(pKFp == pKFp->GetParent())
                        {
                            std::cout << "--> " << pKFp->mId.first << "|" << pKFp->mId.second << " -- bad: " << (int)pKFp->isBad() << std::endl;
                            break;
                        }
                        pKFp = pKFp->GetParent();
                    }

                    pRefKF->mTcwBefGBA = pRefKF->GetPose();
                }
                #endif

                // Map to non-corrected camera
                cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0,3).colRange(0,3);
                cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0,3).col(3);
                cv::Mat Xc = Rcw*pMP->GetWorldPos()+tcw;

                // Backproject using corrected camera
                cv::Mat Twc = pRefKF->GetPoseInverse();
                cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
                cv::Mat twc = Twc.rowRange(0,3).col(3);

                pMP->SetWorldPos(Rwc*Xc+twc,true);
                pMP->mbLoopCorrected = true;
            }
        }

        cout << "-> Map updated!" << endl;

        #ifdef FINALBA
        mpMap->unsetGBAinterrupted();
        #endif

//        mpMap->UnLockMapUpdate();
//        if(!mpMap->UnLockMapUpdate())
//        {
//            cout << COUTFATAL << "Attempt to UnLock MapUpdate -- was not locked" << endl;
//            throw estd::infrastructure_ex();
//        }

//        cout << "Changed KF poses: " << sChangedKFs.size() << endl;
    }
    #ifdef FINALBA
    else
    {
        cout << COUTNOTICE << "GBA interrupted" << endl;
        mpMap->setGBAinterrupted();
    }
    #endif

    if(params::stats::mbWriteKFsToFile)
    {
        for(int it=0;it<4;++it)
        {
            std::stringstream ss;
            ss << params::stats::msOutputDir << "KF_GBA_" << it << ".csv";
            mpMap->WriteStateToCsv(ss.str(),it);
        }
    }

//    cout << "Update Map State" << endl;

//    std::cout << __func__ << ":" << __LINE__ << ": call setFinishedGBA()" << std::endl;
    mpMap->setFinishedGBA();
//    LLL
    mpMap->unsetRunningGBA();

//    cout << "Unlock Mapping" << endl;

    for(set<ccptr>::iterator sit = spCC.begin();sit!=spCC.end();++sit)
    {
        ccptr pCC = *sit;
        pCC->mbOptimized= true;

//        std::cout << __func__ << ":" << __LINE__ << ": Unlock Mapping " << pCC->mClientId << std::endl;

        pCC->UnLockMapping();
//        if(!pCC->UnLockMapping())
//        {
//            cout << COUTFATAL << "Attempt to UnLock Comm -- was not locked" << endl;
//            throw estd::infrastructure_ex();
//        }
        pCC->mbOptActive = false;
    }

    cout << "-> Leave Thread" << endl;
}
//#endif

}
