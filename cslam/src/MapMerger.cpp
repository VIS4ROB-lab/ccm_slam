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

#include <cslam/MapMerger.h>

namespace cslam {

MapMerger::MapMerger(matchptr pMatcher)
    : bIsBusy(false), mpMatcher(pMatcher)
{
    if(!mpMatcher)
    {
        ROS_ERROR_STREAM("In \" MapMerger::MapMerger()\": nullptr passed");
        throw estd::infrastructure_ex();
    }
}

MapMerger::mapptr MapMerger::MergeMaps(mapptr pMapCurr, mapptr pMapMatch, vector<MapMatchHit> vMatchHits)
{
    // Make sure no other module can start GBA
    bool bProblem = false;

    if(pMapCurr->isNoStartGBA())
    {
        bProblem = true;
    }

    if(pMapMatch->isNoStartGBA())
    {
        bProblem = true;
    }

    if(bProblem)
    {
        std::cout << __func__ << ":" << __LINE__ << " Waiting for GBA to be able to Start" << std::endl;
        while(pMapCurr->isNoStartGBA() || pMapMatch->isNoStartGBA())
        {
            usleep(params::timings::miLockSleep);
        }
        std::cout << __func__ << ":" << __LINE__  << "Continue" << std::endl;
    }

    pMapCurr->setNoStartGBA();
    pMapMatch->setNoStartGBA();

    // If a Global Bundle Adjustment is running, abort it
    if(pMapCurr->isRunningGBA())
        pMapCurr->StopGBA();

    if(pMapMatch->isRunningGBA())
        pMapMatch->StopGBA();

    this->SetBusy();

    bool b0 = false;
    bool b1 = false;
    bool b2 = false;
    bool b3 = false;

    set<ccptr> spCCC = pMapCurr->GetCCPtrs();
    set<ccptr> spCCM = pMapMatch->GetCCPtrs();

    #ifdef LOGGING
    ccptr pCClog = *(spCCC.begin());
    pCClog->mpLogger->SetMerge(__LINE__,0);
    #endif

    if(spCCC.size() != pMapCurr->msuAssClients.size())
    {
        cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps()\": spCCC.size() != pMapCurr->msuAssClients.size()" << endl;
        cout << "Map id: " << pMapCurr->mMapId << endl;
        cout << "Associated client IDs:" << endl;
        for(set<size_t>::const_iterator sit = pMapCurr->msuAssClients.begin();sit!=pMapCurr->msuAssClients.end();++sit)
            cout << *sit << endl;
        cout << "Associated pCCs:" << endl;
        for(set<ccptr>::const_iterator sit = spCCC.begin();sit!=spCCC.end();++sit)
            cout << (*sit)->mClientId << endl;
        throw estd::infrastructure_ex();
    }

    if(spCCM.size() != pMapMatch->msuAssClients.size())
    {
        cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps()\": spCCM.size() != pMapMatch->msuAssClients.size()" << endl;
        cout << "Map id: " << pMapMatch->mMapId << endl;
        cout << "Associated client IDs:" << endl;
        for(set<size_t>::const_iterator sit = pMapMatch->msuAssClients.begin();sit!=pMapMatch->msuAssClients.end();++sit)
            cout << *sit << endl;
        cout << "Associated pCCs:" << endl;
        for(set<ccptr>::const_iterator sit = spCCM.begin();sit!=spCCM.end();++sit)
            cout << (*sit)->mClientId << endl;
        throw estd::infrastructure_ex();
    }

    for(set<ccptr>::iterator sit = spCCC.begin();sit!=spCCC.end();++sit)
    {
        ccptr pCC = *sit;

        cout << "spCCC: pCC->mClientId: " << pCC->mClientId << endl;

        if(pCC->mClientId > 3) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps()\": associated ClientId out of bounds (" << pCC->mClientId << ")" << endl;
        if(!(pMapCurr->msuAssClients.count(pCC->mClientId))) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps()\": associated ClientId in pCC but not in msuAssClients" << endl;
        switch(pCC->mClientId)
        {
            case(static_cast<size_t>(0)):
                if(b0) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps()\": associated ClientId found twice" << endl;
                b0 = true;
                while(!pCC->LockComm()){usleep(params::timings::miLockSleep);}
                #ifdef LOGGING
//                pCC->mpLogger->SetMappingLock(__LINE__,pCC->mClientId);
                #endif
                while(!pCC->LockMapping()){usleep(params::timings::miLockSleep);}
                while(!pCC->LockPlaceRec()){usleep(params::timings::miLockSleep);}
                break;
            case(static_cast<size_t>(1)):
                if(b1) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps()\": associated ClientId found twice" << endl;
                b1 = true;
                while(!pCC->LockComm()){usleep(params::timings::miLockSleep);}
                #ifdef LOGGING
//                pCC->mpLogger->SetMappingLock(__LINE__,pCC->mClientId);
                #endif
                while(!pCC->LockMapping()){usleep(params::timings::miLockSleep);}
                while(!pCC->LockPlaceRec()){usleep(params::timings::miLockSleep);}
                break;
            case(static_cast<size_t>(2)):
                if(b2) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps()\": associated ClientId found twice" << endl;
                b2 = true;
                while(!pCC->LockComm()){usleep(params::timings::miLockSleep);}
                #ifdef LOGGING
//                pCC->mpLogger->SetMappingLock(__LINE__,pCC->mClientId);
                #endif
                while(!pCC->LockMapping()){usleep(params::timings::miLockSleep);}
                while(!pCC->LockPlaceRec()){usleep(params::timings::miLockSleep);}
                break;
            case(static_cast<size_t>(3)):
                if(b3) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps()\": associated ClientId found twice" << endl;
                b3 = true;
                while(!pCC->LockComm()){usleep(params::timings::miLockSleep);}
                #ifdef LOGGING
//                pCC->mpLogger->SetMappingLock(__LINE__,pCC->mClientId);
                #endif
                while(!pCC->LockMapping()){usleep(params::timings::miLockSleep);}
                while(!pCC->LockPlaceRec()){usleep(params::timings::miLockSleep);}
                break;
            default: cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps()\": associated ClientId out of bounds" << endl;
        }
    }

    #ifdef LOGGING
    pCClog->mpLogger->SetMerge(__LINE__,0);
    #endif

    for(set<ccptr>::iterator sit = spCCM.begin();sit!=spCCM.end();++sit)
    {
        ccptr pCC = *sit;

        cout << "spCCM: pCC->mClientId: " << pCC->mClientId << endl;

        if(pCC->mClientId > 3) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps()\": associated ClientId out of bounds (" << pCC->mClientId << ")" << endl;
        if(!(pMapMatch->msuAssClients.count(pCC->mClientId))) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps()\": associated ClientId in pCC but not in msuAssClients" << endl;
        switch(pCC->mClientId)
        {
            case(static_cast<size_t>(0)):
                if(b0) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps()\": associated ClientId found twice" << endl;
                b0 = true;
                while(!pCC->LockComm()){usleep(params::timings::miLockSleep);}
                while(!pCC->LockMapping()){usleep(params::timings::miLockSleep);}
                while(!pCC->LockPlaceRec()){usleep(params::timings::miLockSleep);}
                break;
            case(static_cast<size_t>(1)):
                if(b1) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps()\": associated ClientId found twice" << endl;
                b1 = true;
                while(!pCC->LockComm()){usleep(params::timings::miLockSleep);}
                while(!pCC->LockMapping()){usleep(params::timings::miLockSleep);}
                while(!pCC->LockPlaceRec()){usleep(params::timings::miLockSleep);}
                break;
            case(static_cast<size_t>(2)):
                if(b2) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps()\": associated ClientId found twice" << endl;
                b2 = true;
                while(!pCC->LockComm()){usleep(params::timings::miLockSleep);}
                while(!pCC->LockMapping()){usleep(params::timings::miLockSleep);}
                while(!pCC->LockPlaceRec()){usleep(params::timings::miLockSleep);}
                break;
            case(static_cast<size_t>(3)):
                if(b3) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps()\": associated ClientId found twice" << endl;
                b3 = true;
                while(!pCC->LockComm()){usleep(params::timings::miLockSleep);}
                while(!pCC->LockMapping()){usleep(params::timings::miLockSleep);}
                while(!pCC->LockPlaceRec()){usleep(params::timings::miLockSleep);}
                break;
            default: cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps()\": associated ClientId out of bounds" << endl;
        }
    }

    #ifdef LOGGING
    pCClog->mpLogger->SetMerge(__LINE__,0);
    #endif

    // Get Map Mutex
    // Lock all mutexes
    while(!pMapCurr->LockMapUpdate()){usleep(params::timings::miLockSleep);}
    while(!pMapMatch->LockMapUpdate()){usleep(params::timings::miLockSleep);}

    for(set<ccptr>::iterator sit = spCCC.begin();sit!=spCCC.end();++sit)
    {
        (*sit)->mbOptActive = true;
    }

    for(set<ccptr>::iterator sit = spCCM.begin();sit!=spCCM.end();++sit)
    {
        (*sit)->mbOptActive = true;
    }

    if(pMapCurr == nullptr || pMapMatch == nullptr)
    {
        cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps\": at least one map is nullptr" << endl;
        this->SetIdle();
        return nullptr;
    }

    //create new map
    mapptr pFusedMap{new Map(pMapMatch,pMapCurr)};
    while(!pFusedMap->LockMapUpdate()){usleep(params::timings::miLockSleep);}
    pFusedMap->UpdateAssociatedData();

    size_t IdC = vMatchHits[0].mpKFCurr->mId.second;
    size_t IdM = vMatchHits[0].mpKFMatch->mId.second;

    g2o::Sim3 g2oS_wm_wc; //world match - world curr

    //optimize
    idpair nLoopKf;

    int idx = 0;
    kfptr pKFCur = vMatchHits[idx].mpKFCurr;
    kfptr pKFMatch = vMatchHits[idx].mpKFMatch;
    g2o::Sim3 g2oScw = vMatchHits[idx].mg2oScw;
    std::vector<mpptr> vpCurrentMatchedPoints = vMatchHits[idx].mvpCurrentMatchedPoints;
    std::vector<mpptr> vpLoopMapPoints = vMatchHits[idx].mvpLoopMapPoints;

    vector<kfptr> vpKeyFramesCurr = pMapCurr->GetAllKeyFrames();

    if(IdC != pKFCur->mId.second || IdM != pKFMatch->mId.second)
        cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps\": client ID mismatch" << endl;

    // Ensure current keyframe is updated
    pKFCur->UpdateConnections();

    // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
    mvpCurrentConnectedKFs = pKFCur->GetVectorCovisibleKeyFrames();
    mvpCurrentConnectedKFs.push_back(pKFCur);

    nLoopKf = pKFCur->mId;

    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
    CorrectedSim3[pKFCur]=g2oScw;
    cv::Mat Twc = pKFCur->GetPoseInverse();

    {
        cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
        cv::Mat twc = Twc.rowRange(0,3).col(3);
        g2o::Sim3 g2oSwc(Converter::toMatrix3d(Rwc),Converter::toVector3d(twc),1.0);
        g2oS_wm_wc = (g2oScw.inverse())*(g2oSwc.inverse());
    }

    KeyFrameAndPose CorrectedSim3All, NonCorrectedSim3All;
    CorrectedSim3All[pKFCur]=g2oScw;

    for(vector<kfptr>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
    {
        kfptr pKFi = *vit;

        cv::Mat Tiw = pKFi->GetPose();

        if(pKFi!=pKFCur)
        {
            cv::Mat Tic = Tiw*Twc;
            cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
            cv::Mat tic = Tic.rowRange(0,3).col(3);
            g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);
            g2o::Sim3 g2oCorrectedSiw = g2oSic*g2oScw;
            //Pose corrected with the Sim3 of the loop closure
            CorrectedSim3[pKFi]=g2oCorrectedSiw;
        }

        cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
        cv::Mat tiw = Tiw.rowRange(0,3).col(3);
        g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
        //Pose without correction
        NonCorrectedSim3[pKFi]=g2oSiw;
    }

    for(vector<kfptr>::iterator vit = vpKeyFramesCurr.begin();vit!=vpKeyFramesCurr.end();++vit)
    {
        kfptr pKFi = *vit;

        KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKFi);

        if(it!=CorrectedSim3.end())
        {
            CorrectedSim3All[pKFi] = it->second;

            KeyFrameAndPose::const_iterator it2 = NonCorrectedSim3.find(pKFi);
            if(it2==NonCorrectedSim3.end()) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps\": Siw for KF in CorrectedSim3 but not in NonCorrectedSim3" << endl;

            NonCorrectedSim3All[pKFi] = NonCorrectedSim3[pKFi];
        }
        else
        {
            cv::Mat Tiw = pKFi->GetPose();
            //CorrectedSim3All
            cv::Mat Tic = Tiw*Twc;
            cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
            cv::Mat tic = Tic.rowRange(0,3).col(3);
            g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);
            g2o::Sim3 g2oCorrectedSiw = g2oSic*g2oScw;
            //Pose corrected with the Sim3 of the loop closure
            CorrectedSim3All[pKFi]=g2oCorrectedSiw;
            //NonCorrectedSim3All
            cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
            cv::Mat tiw = Tiw.rowRange(0,3).col(3);
            g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
            //Pose without correction
            NonCorrectedSim3All[pKFi]=g2oSiw;
        }
    }

    // Correct MapPoints and KeyFrames of current map
    for(KeyFrameAndPose::iterator mit=CorrectedSim3All.begin(), mend=CorrectedSim3All.end(); mit!=mend; mit++)
    {
        kfptr pKFi = mit->first;
        g2o::Sim3 g2oCorrectedSiw = mit->second;
        g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

        g2o::Sim3 g2oSiw =NonCorrectedSim3All[pKFi];

        vector<mpptr> vpMPsi = pKFi->GetMapPointMatches();
        for(size_t iMP=0, endMPi = vpMPsi.size(); iMP<endMPi; iMP++)
        {
            mpptr pMPi = vpMPsi[iMP];
            if(!pMPi)
                continue;
            if(pMPi->isBad())
                continue;
            if(pMPi->mCorrectedByKF_MM==pKFCur->mId)
                continue;

            // Project with non-corrected pose and project back with corrected pose
            cv::Mat P3Dw = pMPi->GetWorldPos();
            Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
            Eigen::Matrix<double,3,1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));

            cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
            pMPi->SetWorldPos(cvCorrectedP3Dw,true);
            pMPi->mCorrectedByKF_MM = pKFCur->mId;
            pMPi->mCorrectedReference_MM = pKFCur->mUniqueId;
            pMPi->UpdateNormalAndDepth();
        }

        // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
        Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
        Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
        double s = g2oCorrectedSiw.scale();

        eigt *=(1./s); //[R t/s;0 1]

        cv::Mat correctedTiw = Converter::toCvSE3(eigR,eigt);

        pKFi->SetPose(correctedTiw,true);

        // Make sure connections are updated
        pKFi->UpdateConnections();

        pKFi->mCorrected_MM = pKFCur->mId;
    }

    map<idpair,kfptr>  mpErasedKFs = pMapCurr->GetMmpErasedKeyFrames();
    map<idpair,mpptr>  mpErasedMPs = pMapCurr->GetMmpErasedMapPoints();

    for(map<idpair,kfptr>::iterator mitEr = mpErasedKFs.begin();mitEr != mpErasedKFs.end();++mitEr)
    {
        kfptr pKFi = mitEr->second;

        if(!pKFi)
        {
            cout << "\033[1;33m!!! WARN !!!\033[0m " << __func__ << ":" << __LINE__ << ": KF is nullptr" << endl;
            continue;
        }

        if(!pKFi->isBad())
            cout << "\033[1;33m!!! WARN !!!\033[0m " << __func__ << ":" << __LINE__ << ": KF is erased, but !bad" << endl;

        if(pKFi->mCorrected_MM == pKFCur->mId)
        {
            cout << "\033[1;33m!!! WARN !!!\033[0m " << __func__ << ":" << __LINE__ << ": KF already corrected" << endl;
            continue;
        }

        kfptr pP = pKFi->GetParent();
        cv::Mat Tcp = pKFi->mTcp;
        while(pP->isBad())
        {
            Tcp = pP->mTcp;
            pP = pP->GetParent();
        }

        pKFi->mCorrected_MM = pKFCur->mId;
    }

    for(map<idpair,mpptr>::iterator mitEr = mpErasedMPs.begin();mitEr != mpErasedMPs.end();++mitEr)
    {
        mpptr pMPi = mitEr->second;

        if(!pMPi)
        {
            cout << "\033[1;33m!!! WARN !!!\033[0m " << __func__ << ":" << __LINE__ << ": MP is nullptr" << endl;
            continue;
        }

        if(!pMPi->isBad())
            cout << "\033[1;33m!!! WARN !!!\033[0m " << __func__ << ":" << __LINE__ << ": MP is erased, but !bad" << endl;

        if(pMPi->mCorrectedByKF_MM == pKFCur->mId)
        {
            cout << "\033[1;33m!!! WARN !!!\033[0m " << __func__ << ":" << __LINE__ << ": MP already corrected" << endl;
            continue;
        }

        //do not correct erase MPs
    }

    // Start Loop Fusion
    // Update matched map points and replace if duplicated
    for(size_t i=0; i<vpCurrentMatchedPoints.size(); i++)
    {
        if(vpCurrentMatchedPoints[i])
        {
            mpptr pLoopMP = vpCurrentMatchedPoints[i];
            mpptr pCurMP = pKFCur->GetMapPoint(i);
            if(pCurMP)
            {
                pCurMP->ReplaceAndLock(pLoopMP);
            }
            else
            {
                pKFCur->AddMapPoint(pLoopMP,i,true); //lock this MapPoint
                pLoopMP->AddObservation(pKFCur,i,true);
                pLoopMP->ComputeDistinctiveDescriptors();
            }
        }
    }

    // Project MapPoints observed in the neighborhood of the loop keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    SearchAndFuse(CorrectedSim3,vpLoopMapPoints);

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

    // Optimize graph
    Optimizer::OptimizeEssentialGraphMapFusion(pFusedMap, pKFMatch, pKFCur, LoopConnections, false);

    // Add loop edge
    pKFMatch->AddLoopEdge(pKFCur);
    pKFCur->AddLoopEdge(pKFMatch);

    cout << "Essential graph optimized" << endl;

    cout << ">>>>> MapMerger::MergeMaps --> Global Bundle Adjustment" << endl;

    pFusedMap->setRunningGBA();
    pFusedMap->setFinishedGBA();
    pFusedMap->mbStopGBA = false;

    #ifdef DONOTINTERRUPTMERGE
    pFusedMap->setMergeStepGBA();
    #endif

    #ifdef DEBUGGING2
    pFusedMap->CheckStructure();
    #endif

    // Launch a new thread to perform Global Bundle Adjustment
    cout << "--- Launch GBA thread" << endl;
    pFusedMap->mpThreadGBA = new thread(&MapMerger::RunGBA,this,nLoopKf,pFusedMap);

//    std::cout << "MapMerger: Wait for GBA to finish" << std::endl;
//    while(pFusedMap->isRunningGBA()) {
//        usleep(10000);
//    }
//    std::cout << "MapMerger: GBA finished - continue" << std::endl;

    cout << "\033[1;32;41m!!! MAPS MERGED !!!\033[0m" << endl;
    this->SetIdle();

    //delete old maps and set new ones in threads

    pMapCurr->SetOutdated();
    pMapMatch->SetOutdated();

    set<ccptr> spCCF = pFusedMap->GetCCPtrs();
    for(set<ccptr>::iterator sit = spCCF.begin();sit!=spCCF.end();++sit)
    {
        ccptr pCC = *sit;
        chptr pCH = pCC->mpCH;
        if(spCCC.count(pCC))
        {
            pCH->ChangeMap(pFusedMap,g2oS_wm_wc);
            pCC->mbGotMerged = true;
        }
        else
        {
            pCH->ChangeMap(pFusedMap,g2o::Sim3());
            pCH->ClearCovGraph(pMapCurr->mMapId);
        }
        pCC->UnLockComm();
        pCC->UnLockPlaceRec();
    }

    pMapCurr->UnLockMapUpdate();
    pMapMatch->UnLockMapUpdate();
    pFusedMap->UnLockMapUpdate();

    pMapCurr->unsetNoStartGBA();
    pMapMatch->unsetNoStartGBA();
    pFusedMap->unsetNoStartGBA();

    #ifdef LOGGING
    pCClog->mpLogger->SetMerge(__LINE__,0);
    #endif

    return pFusedMap;
}

void MapMerger::SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap, std::vector<mpptr> vpLoopMapPoints)
{
    ORBmatcher matcher(0.8);

    for(KeyFrameAndPose::const_iterator mit=CorrectedPosesMap.begin(), mend=CorrectedPosesMap.end(); mit!=mend;mit++)
    {
        kfptr pKF = mit->first;

        g2o::Sim3 g2oScw = mit->second;
        cv::Mat cvScw = Converter::toCvMat(g2oScw);

        vector<mpptr> vpReplacePoints(vpLoopMapPoints.size(),nullptr);
        matcher.Fuse(pKF,cvScw,vpLoopMapPoints,4,vpReplacePoints);

        const int nLP = vpLoopMapPoints.size();
        for(int i=0; i<nLP;i++)
        {
            mpptr pRep = vpReplacePoints[i];
            if(pRep)
            {
                pRep->ReplaceAndLock(vpLoopMapPoints[i]);
            }
        }
    }
}

void MapMerger::SetBusy()
{
    unique_lock<mutex> lock(mMutexBusy);
    bIsBusy = true;
}

void MapMerger::SetIdle()
{
    unique_lock<mutex> lock(mMutexBusy);
    bIsBusy = false;
}

bool MapMerger::isBusy()
{
    unique_lock<mutex> lock(mMutexBusy);
    return bIsBusy;
}

void MapMerger::RunGBA(idpair nLoopKf, mapptr pFusedMap)
{
    cout << "-> Starting Global Bundle Adjustment" << endl;

    Optimizer::MapFusionGBA(pFusedMap,pFusedMap->mMapId,params::opt::mGBAIterations,&(pFusedMap->mbStopGBA),nLoopKf,true);

    set<ccptr> spCC = pFusedMap->GetCCPtrs();

    #ifdef FINALBA
    if(!pFusedMap->mbStopGBA)
    #endif
    {
        unique_lock<mutex> lock(pFusedMap->mMutexGBA);

//        while(!pFusedMap->LockMapUpdate()){usleep(params::timings::miLockSleep);}

        cout << "-> Global Bundle Adjustment finished" << endl;
        cout << "-> Updating map ..." << endl;

        // Correct keyframes starting at map first keyframe
        list<kfptr> lpKFtoCheck(pFusedMap->mvpKeyFrameOrigins.begin(),pFusedMap->mvpKeyFrameOrigins.end());

        cout << "--> Updating KFs ..." << endl;

        while(!lpKFtoCheck.empty())
        {
            kfptr pKF = lpKFtoCheck.front();
            const set<kfptr> sChilds = pKF->GetChilds();
            cv::Mat Twc = pKF->GetPoseInverse();
            for(set<kfptr>::const_iterator sit=sChilds.begin();sit!=sChilds.end();sit++)
            {
                kfptr pChild = *sit;
                if(pChild->mBAGlobalForKF!=nLoopKf)
                {
                    cv::Mat Tchildc = pChild->GetPose()*Twc;
                    pChild->mTcwGBA = Tchildc*pKF->mTcwGBA;
                    #ifdef DEBUGGING2
                    if(!(pChild->mTcwGBA.dims >= 2))
                        std::cout << COUTERROR << " KF" << pChild->mId.first << "|" << pChild->mId.second << ": !(pChild->mTcwGBA.dims >= 2)" << std::endl;
                    #endif
                    pChild->mBAGlobalForKF=nLoopKf;

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
            lpKFtoCheck.pop_front();
        }

        cout << "--> Updating MPs ..." << endl;

        // Correct MapPoints
        const vector<mpptr> vpMPs = pFusedMap->GetAllMapPoints();

        for(size_t i=0; i<vpMPs.size(); i++)
        {
            mpptr pMP = vpMPs[i];

            if(pMP->isBad())
                continue;

            if(pMP->mBAGlobalForKF==nLoopKf)
            {
                // If optimized by Global BA, just update
                #ifdef DEBUGGING2
                if(!(pMP->mPosGBA.dims >= 2))
                    std::cout << COUTERROR << " MP" << pMP->mId.first << "|" << pMP->mId.second << ": !(pMP->mPosGBA.dims >= 2)" << std::endl;
                #endif
                pMP->SetWorldPos(pMP->mPosGBA,true);
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

                if(pRefKF->mBAGlobalForKF!=nLoopKf)
                    continue;

                #ifdef DEBUGGING2
                if(!(pRefKF->mTcwBefGBA.dims >= 2))
                {
                    std::cout << COUTERROR << " KF" << pRefKF->mId.first << "|" << pRefKF->mId.second << ": !(pRefKF->mTcwBefGBA.dims >= 2)" << std::endl;
                    std::cout << "bad? " << (int)pRefKF->isBad() << std::endl;
                    std::cout << "mBAGlobalForKF: " << pRefKF->mBAGlobalForKF.first << "|" << pRefKF->mBAGlobalForKF.second << std::endl;
                    std::cout << "nLoopKF: " << nLoopKf.first << "|" << nLoopKf.second << std::endl;
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
            }
        }

        cout << "-> Map updated!" << endl;

        #ifdef FINALBA
        pFusedMap->unsetGBAinterrupted();
        #endif

        #ifdef DONOTINTERRUPTMERGE
        pFusedMap->unsetMergeStepGBA();
        #endif

//        pFusedMap->UnLockMapUpdate();
    }
    #ifdef FINALBA
    else
    {
        cout << COUTNOTICE << "GBA interrupted" << endl;

        #ifdef DONOTINTERRUPTMERGE
        if(pFusedMap->isMergeStepGBA())
        {
            cout << COUTFATAL << endl;
            KILLSYS
        }
        #endif

        pFusedMap->setGBAinterrupted();
    }
    #endif

    if(params::stats::mbWriteKFsToFile)
    {
        for(int it=0;it<4;++it)
        {
            std::stringstream ss;
            ss << params::stats::msOutputDir << "KF_GBA_" << it << ".csv";
            pFusedMap->WriteStateToCsv(ss.str(),it);
        }
    }

    pFusedMap->setFinishedGBA();
    pFusedMap->unsetRunningGBA();

    for(set<ccptr>::iterator sit = spCC.begin();sit!=spCC.end();++sit)
    {
        ccptr pCC = *sit;
        pCC->mbOptimized= true;

        pCC->UnLockMapping();

        pCC->mbOptActive = false;
    }

    cout << "-> Leave Thread" << endl;
}

}
