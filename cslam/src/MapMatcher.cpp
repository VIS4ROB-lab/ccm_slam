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

#include <cslam/MapMatcher.h>

namespace cslam {

MapMatcher::MapMatcher(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, dbptr pDB, vocptr pVoc, mapptr pMap0, mapptr pMap1, mapptr pMap2, mapptr pMap3)
    : mNh(Nh), mNhPrivate(NhPrivate),
      mpKFDB(pDB), mpVoc(pVoc), mpMap0(pMap0), mpMap1(pMap1), mpMap2(pMap2), mpMap3(pMap3),
      mLastLoopKFid(0),
      mbFixScale(false),
      mnCovisibilityConsistencyTh(params::placerec::miCovisibilityConsistencyTh)
{

    if(pMap0) mmpMaps[*(pMap0->msuAssClients.begin())]=pMap0;
    if(pMap1) mmpMaps[*(pMap1->msuAssClients.begin())]=pMap1;
    if(pMap2) mmpMaps[*(pMap2->msuAssClients.begin())]=pMap2;
    if(pMap3) mmpMaps[*(pMap3->msuAssClients.begin())]=pMap3;

    if(pMap0) mspMaps.insert(pMap0);
    if(pMap1) mspMaps.insert(pMap1);
    if(pMap2) mspMaps.insert(pMap2);
    if(pMap3) mspMaps.insert(pMap3);

    mPubMarker = mNh.advertise<visualization_msgs::Marker>("MapMatcherMarkers",10);

    mMatchMatrix =  cv::Mat::zeros(4,4,2);

    mMapMatchEdgeMsg.header.frame_id = "world";
    mMapMatchEdgeMsg.header.stamp = ros::Time::now();
    mMapMatchEdgeMsg.ns = "MapMatchEdges_red";
    mMapMatchEdgeMsg.type = visualization_msgs::Marker::LINE_LIST;
    mMapMatchEdgeMsg.color = Colors::msgRed();
    mMapMatchEdgeMsg.action = visualization_msgs::Marker::ADD;
    mMapMatchEdgeMsg.scale.x = params::vis::mfLoopMarkerSize;
    mMapMatchEdgeMsg.id = 1;
}

void MapMatcher::Run()
{
    double CovGraphMarkerSize;
    mNhPrivate.param("MarkerSizeServer",CovGraphMarkerSize,0.001);
    mpMapMerger.reset(new MapMerger(shared_from_this()));

    #ifdef LOGGING
    KeyFrame::ccptr pCC = mpMap0->GetCCPtr(0);
    if(!pCC)
    {
        std::cout << COUTERROR << "pCC not valid" << std::endl;
        KILLSYS
    }
    #endif

    while(1)
    {
        #ifdef LOGGING
        pCC->mpLogger->SetMatch(__LINE__,0);
        #endif

        if(CheckKfQueue())
        {
            bool bDetect = DetectLoop();
            if(bDetect)
            {
                bool bSim3 = ComputeSim3();
                if(bSim3)
                {
                    // Perform loop fusion and pose graph optimization
                    CorrectLoop();
                }
            }
        }

        #ifdef LOGGING
        pCC->mpLogger->SetMatch(__LINE__,0);
        #endif

        usleep(params::timings::server::miPlaceRecRateRate);
    }
}

bool MapMatcher::DetectLoop()
{
    {
        unique_lock<mutex> lock(mMutexKfInQueue);
        mpCurrentKF = mlKfInQueue.front();

        mlKfInQueue.pop_front();
        // Avoid that a keyframe can be erased while it is being process by this thread
        mpCurrentKF->SetNotErase();
    }

    //If the map contains less than 10 KF or less than 10 KF have passed from last loop detection
    if(mpCurrentKF->mId.first<params::placerec::miStartMapMatchingAfterKf)
    {
        mpCurrentKF->SetErase();
        return false;
    }

    mpCurrMap = mpCurrentKF->GetMapptr(); //get map of KF

    if(!mpCurrMap)
    {
        cout << ": In \"MapMatcher::DetectLoop()\": mpCurrMap is nullptr -> KF not contained in any map" << endl;
        throw estd::infrastructure_ex();
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
    vector<kfptr> vpCandidateKFs = mpKFDB->DetectMapMatchCandidates(mpCurrentKF, minScore, mpCurrMap);

    // If there are no loop candidates, just add new keyframe and return false
    if(vpCandidateKFs.empty())
    {
        mmvConsistentGroups[mpCurrMap].clear();
        mpCurrentKF->SetErase();
        return false;
    }

    // For each loop candidate check consistency with previous loop candidates
    // Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
    // A group is consistent with a previous group if they share at least a keyframe
    // We must detect a consistent loop in several consecutive keyframes to accept it
    mvpEnoughConsistentCandidates.clear();

    vector<ConsistentGroup> vCurrentConsistentGroups; //pair <set<KF*>,int> --> int counts consistent groups found for this group
    vector<bool> vbConsistentGroup(mmvConsistentGroups[mpCurrMap].size(),false);
    //mvConsistentGroups stores the last found consistent groups.

    for(size_t i=0, iend=vpCandidateKFs.size(); i<iend; i++)
    {
        kfptr pCandidateKF = vpCandidateKFs[i];

        set<kfptr> spCandidateGroup = pCandidateKF->GetConnectedKeyFrames();
        spCandidateGroup.insert(pCandidateKF);
        //group with candidate and connected KFs

        bool bEnoughConsistent = false;
        bool bConsistentForSomeGroup = false;
        for(size_t iG=0, iendG=mmvConsistentGroups[mpCurrMap].size(); iG<iendG; iG++)
        {
            set<kfptr> sPreviousGroup = mmvConsistentGroups[mpCurrMap][iG].first;

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
                int nPreviousConsistency = mmvConsistentGroups[mpCurrMap][iG].second;
                int nCurrentConsistency = nPreviousConsistency + 1;
                if(!vbConsistentGroup[iG])
                {
                    ConsistentGroup cg = make_pair(spCandidateGroup,nCurrentConsistency);
                    vCurrentConsistentGroups.push_back(cg);
                    vbConsistentGroup[iG]=true; //this avoid to include the same group more than once
                }
                if(nCurrentConsistency>=mnCovisibilityConsistencyTh && !bEnoughConsistent)
                {
                    mvpEnoughConsistentCandidates.push_back(pCandidateKF);
                    bEnoughConsistent=true; //this avoid to insert the same candidate more than once
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
    mmvConsistentGroups[mpCurrMap] = vCurrentConsistentGroups;

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

bool MapMatcher::ComputeSim3()
{
    const int nInitialCandidates = mvpEnoughConsistentCandidates.size();

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
            continue;
        }

        int nmatches = matcher.SearchByBoW(mpCurrentKF,pKF,vvpMapPointMatches[i]);

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
                if(!pMP->isBad() && pMP->mLoopPointForKF_MM!=mpCurrentKF->mId) //ID Tag
                {
                    mvpLoopMapPoints.push_back(pMP);
                    pMP->mLoopPointForKF_MM = mpCurrentKF->mId;
                }
            }
        }
    }

    // Find more matches projecting with the computed Sim3
    matcher.SearchByProjection(mpCurrentKF, mScw, mvpLoopMapPoints, mvpCurrentMatchedPoints,10);

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

void MapMatcher::CorrectLoop()
{
    cout << "\033[1;32m!!! MAP MATCH FOUND !!!\033[0m" << endl;

    set<size_t> suAssCLientsCurr = mpCurrentKF->GetMapptr()->msuAssClients;
    set<size_t> suAssCLientsMatch = mpMatchedKF->GetMapptr()->msuAssClients;

    for(set<size_t>::iterator sit = suAssCLientsCurr.begin();sit!=suAssCLientsCurr.end();++sit)
    {
        size_t idc = *sit;
        for(set<size_t>::iterator sit2 = suAssCLientsMatch.begin();sit2!=suAssCLientsMatch.end();++sit2)
        {
            size_t idm = *sit2;

            if(idc == idm) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMatcher::CorrectLoop()\": Associated Clients of matched and current map intersect" << endl;

            mMatchMatrix.at<uint16_t>(idc,idm) = mMatchMatrix.at<uint16_t>(idc,idm) + 1;
            mMatchMatrix.at<uint16_t>(idm,idc) = mMatchMatrix.at<uint16_t>(idm,idc)  + 1;
        }
    }

    if(mpCurrentKF->mId.second == mpMatchedKF->mId.second) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMatcher::CorrectLoop()\": Matched KFs belong to same client" << endl;
    if(!mpCurrMap->msuAssClients.count(mpCurrentKF->mId.second)) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMatcher::CorrectLoop()\": Current KFs does not belong to current map" << endl;
    if(mpCurrMap->msuAssClients.count(mpMatchedKF->mId.second)) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMatcher::CorrectLoop()\": Matched KFs belongs to current map" << endl;

    if(params::vis::mbActive)
        PublishLoopEdges();

    mapptr pMatchedMap = mpMatchedKF->GetMapptr();

    MapMatchHit MMH(mpCurrentKF,mpMatchedKF,mg2oScw,mvpLoopMapPoints,mvpCurrentMatchedPoints);
    mFoundMatches[mpCurrMap][pMatchedMap].push_back(MMH);
    mFoundMatches[pMatchedMap][mpCurrMap].push_back(MMH);

    if(mFoundMatches[mpCurrMap][pMatchedMap].size() >= 1)
    {
        vector<MapMatchHit> vMatches = mFoundMatches[mpCurrMap][pMatchedMap];

        mapptr pMergedMap = mpMapMerger->MergeMaps(mpCurrMap,pMatchedMap,vMatches);
    }

    this->ClearLoopEdges();
}

void MapMatcher::PublishLoopEdges()
{
    mMapMatchEdgeMsg.points.clear();

    tf::StampedTransform Tf_W_Curr,Tf_W_Matched;
    string FrameIdCurr,FrameIdMatched;

    FrameIdCurr = mpCurrentKF->GetMapptr()->mOdomFrame;
    FrameIdMatched = mpMatchedKF->GetMapptr()->mOdomFrame;

    try
    {
        mTfListen.lookupTransform("world",FrameIdCurr, ros::Time(0), Tf_W_Curr);
        mTfListen.lookupTransform("world",FrameIdMatched, ros::Time(0), Tf_W_Matched);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        return;
    }

    cv::Mat TCurr = mpCurrentKF->GetPoseInverse();
    cv::Mat TMatch = mpMatchedKF->GetPoseInverse();

    tf::Point PTfCurr{params::vis::mfScaleFactor*((double)(TCurr.at<float>(0,3))),params::vis::mfScaleFactor*((double)(TCurr.at<float>(1,3))),params::vis::mfScaleFactor*((double)(TCurr.at<float>(2,3)))};
    tf::Point PTfMatch{params::vis::mfScaleFactor*((double)(TMatch.at<float>(0,3))),params::vis::mfScaleFactor*((double)(TMatch.at<float>(1,3))),params::vis::mfScaleFactor*((double)(TMatch.at<float>(2,3)))};

    PTfCurr = Tf_W_Curr*PTfCurr;
    PTfMatch = Tf_W_Matched*PTfMatch;

    geometry_msgs::Point PCurr;
    geometry_msgs::Point PMatch;

    tf::pointTFToMsg(PTfCurr,PCurr);
    tf::pointTFToMsg(PTfMatch,PMatch);

    mMapMatchEdgeMsg.points.push_back(PCurr);
    mMapMatchEdgeMsg.points.push_back(PMatch);

    mPubMarker.publish(mMapMatchEdgeMsg);
}

void MapMatcher::ClearLoopEdges()
{
    mMapMatchEdgeMsg.action = 3;
    mPubMarker.publish(mMapMatchEdgeMsg);
    mMapMatchEdgeMsg.action = visualization_msgs::Marker::ADD;
}

void MapMatcher::InsertKF(kfptr pKF)
{
    unique_lock<mutex> lock(mMutexKfInQueue);

    mlKfInQueue.push_back(pKF);
}

void MapMatcher::EraseKFs(vector<kfptr> vpKFs)
{
    unique_lock<mutex> lock(mMutexKfInQueue);

    for(vector<kfptr>::iterator vit=vpKFs.begin();vit!=vpKFs.end();++vit)
    {
        kfptr pKF = *vit;
        std::list<kfptr>::iterator lit = find(mlKfInQueue.begin(),mlKfInQueue.end(),pKF);
        if(lit!=mlKfInQueue.end()) mlKfInQueue.erase(lit);
    }
}

int MapMatcher::GetNumKFsinQueue()
{
    unique_lock<mutex> lock(mMutexKfInQueue,defer_lock);
    if(lock.try_lock())
    {
        return mlKfInQueue.size();
    }
    else
        return -1;
}

bool MapMatcher::CheckKfQueue()
{
    unique_lock<mutex> lock(mMutexKfInQueue);

    return (!mlKfInQueue.empty());
}

}
