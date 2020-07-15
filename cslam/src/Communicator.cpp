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

#include <cslam/Communicator.h>

namespace cslam {

Communicator::Communicator(ccptr pCC, vocptr pVoc, mapptr pMap, dbptr pKFDB, bool bLoadedMap)
    : mpCC(pCC),
      mNh(pCC->mNh), mNhPrivate(pCC->mNhPrivate),
      mpVoc(pVoc), mpMap(pMap), mpDatabase(pKFDB),
      mClientId(pCC->mClientId), mbResetRequested(false),
      mNearestKfId(defpair),mpNearestKF(nullptr),
      mdPeriodicTime((pCC->mSysState == eSystemState::CLIENT) ? params::comm::client::mfPubPeriodicTime : params::comm::server::mfPubPeriodicTime),
      mdLastTimePub(0.0),mnMaxKfIdSent(0),
      mKfItBound((pCC->mSysState == eSystemState::CLIENT) ? params::comm::client::miKfItBound : params::comm::server::miKfItBound),
      mMpItBound((pCC->mSysState == eSystemState::CLIENT) ? params::comm::client::miMpItBound : params::comm::server::miMpItBound),
      mKfItBoundPub(params::comm::client::miKfPubMax),
      mMpItBoundPub(params::comm::client::miMpPubMax),
      mnEmptyMsgs(0),
      mbLoadedMap(bLoadedMap)
{
    mMsgCountLastMapMsg = 0;
    mOutMapCount = 0;
    mServerMapCount = 0;

    mnWeakAckKF = KFRANGE;
    mnWeakAckMP = MPRANGE;

    if(mbLoadedMap) return; //do not register communication infrastructure (publisher/subscriber) when map is loaded

    //Topics
    std::stringstream* ss;
    string PubMapTopicName, MapInTopicName, SysType;

    if(mpCC->mSysState == eSystemState::CLIENT)
    {
        SysType = "Client";

        //Subscriber
        mNhPrivate.param("MapInTopicName",MapInTopicName,std::string("nospec"));
        mSubMap = mNh.subscribe<ccmslam_msgs::Map>(MapInTopicName,params::comm::client::miSubMapBufferSize,boost::bind(&Communicator::MapCbClient,this,_1));

        //Publisher
        ss = new stringstream;
        *ss << "MapOut" << SysType << mClientId;
        PubMapTopicName = ss->str();
        mPubMap = mNh.advertise<ccmslam_msgs::Map>(PubMapTopicName,params::comm::client::miPubMapBufferSize);
    }
    else if(mpCC->mSysState == eSystemState::SERVER)
    {
        SysType = "Server";

        //Subscriber
        ss = new stringstream;
        *ss << "MapInTopicName" << mClientId;
        mNhPrivate.param(ss->str(),MapInTopicName,std::string("nospec"));
        mSubMap = mNh.subscribe<ccmslam_msgs::Map>(MapInTopicName,params::comm::server::miSubMapBufferSize,boost::bind(&Communicator::MapCbServer,this,_1));

        //Publisher
        ss = new stringstream;
        *ss << "MapOut" << SysType << mClientId;
        PubMapTopicName = ss->str();
        mPubMap = mNh.advertise<ccmslam_msgs::Map>(PubMapTopicName,params::comm::server::miPubMapBufferSize);
    }
    else
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Communicator: invalid systems state: " << mpCC->mSysState << endl;
        throw infrastructure_ex();
    }

    delete ss;

    if(MapInTopicName=="nospec" )
    {
        cout << "Client " << mClientId << " \033[1;31m!!!!! ERROR !!!!!\033[0m Communicator::Communicator(...): bad IN topic name" << endl;
        throw estd::infrastructure_ex();
    }
}

void Communicator::RunClient()
{
    while(true)
    {
        if(params::sys::mbStrictLock)
        {
            while(!mpCC->LockMapping()){
                usleep(params::timings::miLockSleep);
            }
            while(!mpCC->LockTracking()){
                usleep(params::timings::miLockSleep);
            }
        }
        else
        {
            while(!mpMap->LockMapUpdate()){usleep(params::timings::miLockSleep);}
        }

        this->PublishMapClient();

        {
            unique_lock<mutex> lock(mMutexBuffersIn);

            if(!mlBufKFin.empty())
            {
                this->ProcessKfInClient();
            }

            if(!mlBufMPin.empty())
            {
                this->ProcessMpInClient();
            }
        }

        for(list<kfptr>::iterator lit = mlpAddedKfs.begin();lit != mlpAddedKfs.end();++lit)
        {
            kfptr pKFi = *lit;

            if(pKFi->GetMapPoints().size() == 0)
            {
                pKFi->SetBadFlag(false,true);
            }
            else
            {
                pKFi->UpdateConnections();
            }
        }
        mlpAddedKfs.clear();

        if(params::sys::mbStrictLock)
        {
            mpCC->UnLockMapping();
            mpCC->UnLockTracking();
        }
        else
        {
            mpMap->UnLockMapUpdate();
        }

        ResetIfRequested();

        usleep(params::timings::client::miCommRate);
    }
}

void Communicator::RunServer()
{
    while(true)
    {
        #ifdef LOGGING
        mpCC->mpLogger->SetComm(__LINE__,mClientId);
        #endif

        while(!mpCC->LockComm()){usleep(params::timings::miLockSleep);}

        #ifdef LOGGING
        mpCC->mpLogger->SetComm(__LINE__,mClientId);
        #endif

        while(!mpMap->LockMapUpdate()){usleep(params::timings::miLockSleep);}

        #ifdef LOGGING
        mpCC->mpLogger->SetComm(__LINE__,mClientId);
        #endif

        this->PublishMapServer();

        {
            unique_lock<mutex> lock(mMutexBuffersIn);

            if(!mlBufKFin.empty())
            {
                this->ProcessKfInServer();
            }

            if(!mlBufMPin.empty())
            {
                #ifdef TRACELOCK
                mpCC->mpLogger->SetComm(__LINE__,mClientId);
                #endif

                this->ProcessMpInServer();

                #ifdef TRACELOCK
                mpCC->mpLogger->SetComm(__LINE__,mClientId);
                #endif
            }
        }

        #ifdef TRACELOCK
        mpCC->mpLogger->SetComm(__LINE__,mClientId);
        #endif

        mpCC->UnLockComm();
        mpMap->UnLockMapUpdate();

        #ifdef LOGGING
        mpCC->mpLogger->SetComm(__LINE__,mClientId);
        #endif

        ResetIfRequested();

        usleep(params::timings::server::miCommRate);
    }
}

void Communicator::MapCbClient(ccmslam_msgs::MapConstPtr pMsg)
{
    for(int it = 0; it < pMsg->vAckKFs.size() ; ++it)
    {
        if(mlKfOpenAcks.empty())
        {
            break;
        }

        int IDi = pMsg->vAckKFs[it];
        list<AckPairKF>::iterator lit = mlKfOpenAcks.begin();
        while(lit != mlKfOpenAcks.end())
        {
            AckPairKF APi = *lit;

            if(APi.first == IDi)
            {
                kfptr pKFi = APi.second;
                pKFi->Ack();
                lit = mlKfOpenAcks.erase(lit);
                break;
            }
            else if(APi.first < IDi)
            {
                kfptr pKFi = APi.second;
                pKFi->SetSendFull();
                lit = mlKfOpenAcks.erase(lit);
                continue;
            }

            ++lit;
        }
    }

    //Weak Acks
    size_t nWeakAckKF = pMsg->WeakAckKF;

    if(nWeakAckKF != KFRANGE)
    {
        list<AckPairKF>::iterator lit = mlKfOpenAcks.begin();
        while(lit != mlKfOpenAcks.end())
        {
            AckPairKF APi = *lit;

            if(APi.first <= nWeakAckKF)
            {
                kfptr pKFi = APi.second;
                pKFi->SetSendFull();
                lit = mlKfOpenAcks.erase(lit);
                continue;
            }

            ++lit;
        }
    }

    for(int it = 0; it < pMsg->vAckMPs.size() ; ++it)
    {
        if(mlMpOpenAcks.empty())
        {
            break;
        }

        int IDi = pMsg->vAckMPs[it];
        list<AckPairMP>::iterator lit = mlMpOpenAcks.begin();
        while(lit != mlMpOpenAcks.end())
        {
            AckPairMP APi = *lit;

            if(APi.first == IDi)
            {
                mpptr pMPi = APi.second;
                pMPi->Ack();
                lit = mlMpOpenAcks.erase(lit);
                break;
            }
            if(APi.first < IDi)
            {
                mpptr pMPi = APi.second;
                pMPi->SetSendFull();
                lit = mlMpOpenAcks.erase(lit);
                continue;
            }

            ++lit;
        }
    }

    size_t nWeakAckMP = pMsg->WeakAckMP;

    if(nWeakAckMP != MPRANGE)
    {
        list<AckPairMP>::iterator lit = mlMpOpenAcks.begin();
        while(lit != mlMpOpenAcks.end())
        {
            AckPairMP APi = *lit;

            if(APi.first <= nWeakAckMP)
            {
                mpptr pMPi = APi.second;
                pMPi->SetSendFull();
                lit = mlMpOpenAcks.erase(lit);
                continue;
            }

            ++lit;
        }
    }

    //Pack'em in the input buffers

    if(!pMsg->Keyframes.empty())
    {
        if(pMsg->MapPoints.empty())
        {
            return;
        }

        unique_lock<mutex> lock(mMutexBuffersIn);

        ccmslam_msgs::KF msg = pMsg->Keyframes[0];
        kfptr pRefKf = mpMap->GetKfPtr(msg.mpPred_KfId,msg.mpPred_KfClientId);
        if(!pRefKf)
        {
            return;
        }

        mlBufKFin.clear(); //only use most recent information
        mlBufMPin.clear();

        //Keyframes
        for(int idx=0;idx<pMsg->Keyframes.size();++idx)
        {
            ccmslam_msgs::KF msgFull = pMsg->Keyframes[idx];
            ccmslam_msgs::KFred msgRed;
            msgRed.mClientId = MAPRANGE;
            mlBufKFin.push_back(make_pair(msgFull,msgRed));
        }

        //MapPoints
        for(int idx=0;idx<pMsg->MapPoints.size();++idx)
        {
            ccmslam_msgs::MP msgFull = pMsg->MapPoints[idx];
            ccmslam_msgs::MPred msgRed;
            msgRed.mClientId = MAPRANGE;
            mlBufMPin.push_back(make_pair(msgFull,msgRed));
        }
    }
    else
    {
//        cout << "+++++ NO KFs +++++" << endl;
    }
}

void Communicator::MapCbServer(ccmslam_msgs::MapConstPtr pMsg)
{
    {
        unique_lock<mutex> lock(mMutexBuffersIn);

        if(pMsg->KFUpdates.size() > 0)
        {
            for(int idx=0;idx<pMsg->KFUpdates.size();++idx)
            {
                ccmslam_msgs::KF msgFull;
                msgFull.mClientId = MAPRANGE;
                ccmslam_msgs::KFred msgRed = pMsg->KFUpdates[idx];
                mlBufKFin.push_back(make_pair(msgFull,msgRed));
            }
        }

        if(pMsg->Keyframes.size() > 0)
        {
            for(int idx=0;idx<pMsg->Keyframes.size();++idx)
            {
                ccmslam_msgs::KF msgFull = pMsg->Keyframes[idx];
                ccmslam_msgs::KFred msgRed;
                msgRed.mClientId = MAPRANGE;
                mlBufKFin.push_back(make_pair(msgFull,msgRed));
            }
        }

        if(pMsg->MPUpdates.size() > 0)
        {
            for(int idx=0;idx<pMsg->MPUpdates.size();++idx)
            {
                ccmslam_msgs::MP msgFull;
                msgFull.mClientId = MAPRANGE;
                ccmslam_msgs::MPred msgRed = pMsg->MPUpdates[idx];
                mlBufMPin.push_back(make_pair(msgFull,msgRed));
            }
        }

        if(pMsg->MapPoints.size() > 0)
        {
            for(int idx=0;idx<pMsg->MapPoints.size();++idx)
            {
                ccmslam_msgs::MP msgFull = pMsg->MapPoints[idx];
                ccmslam_msgs::MPred msgRed;
                msgRed.mClientId = MAPRANGE;
                mlBufMPin.push_back(make_pair(msgFull,msgRed));
            }
        }
    }

    {
        unique_lock<mutex> lock(mMutexNearestKf);

        mNearestKfId = make_pair(pMsg->ClosestKf_Id,pMsg->ClosestKf_ClientId);
        if(mNearestKfId.first != KFRANGE)
        {
            kfptr pKF = mpMap->GetKfPtr(pMsg->ClosestKf_Id,pMsg->ClosestKf_ClientId);
            if(pKF)
                mpNearestKF = pKF;
        }
    }

    #ifdef INTERRUPTBA
    if(pMsg->KFUpdates.size() > 0 || pMsg->Keyframes.size() > 0 || pMsg->MPUpdates.size() || pMsg->MapPoints.size() > 0)
    {
        if(mpMap->isRunningGBA())
        {
            mpMap->StopGBA();
        }

        mnEmptyMsgs = 0;
    }
    else
    {
        if(mpMap->GetMaxKFid() > 10)
        {
            //make sure map was initialized

            ++mnEmptyMsgs;

            const size_t nThresFinished = (size_t)(30.0 * params::comm::client::mfPubFreq);
            //idea: if for n sec only empty msgs arrive, this client should be finished. With working connection, in n sec arrive n*Pub_freq_client msgs
            if(mnEmptyMsgs >= nThresFinished)
            {
                #ifdef LOGGING
                mpCC->mpLogger->SetFinished(mpCC->mClientId);
                #endif

                #ifdef FINALBA
                if(mpMap->isGBAinterrupted() && !mpMap->isRunningGBA())
                {
                    cout << "Comm " << mpCC->mClientId << ": Trigger GBA" << endl;
                    mpMap->RequestBA(mpCC->mClientId);
                    mnEmptyMsgs = 0; //reset this value, otherwise a finished module will request BA in every interation -- not good if other module is still running
                }
                #endif
            }
        }
    }
    #endif
}

void Communicator::PublishMapClient()
{
    double dTimeDiff = ros::Time::now().toSec() - mdLastTimePub;
    if(dTimeDiff < mdPeriodicTime)
        return;

    mdLastTimePub = ros::Time::now().toSec();

    if(mpMap->KeyFramesInMap()<=params::tracking::miInitKFs) //starting sending not before tracking is initialized stably
        return;

    unique_lock<mutex> lockOut(mMutexBuffersOut);

    ccmslam_msgs::Map msgMap;
    kfptr pKFFront;

    int ItCount = 0;

    //Keyframes
    {
        kfptr pCurKf;
        set<kfptr>::iterator sit = mspBufferKfOut.begin();

        while(!mspBufferKfOut.empty() && (ItCount < mKfItBoundPub)) //Do not call checkBuffer()-method -- it also need the BufferOutMutex
        {
            if(sit == mspBufferKfOut.end())
                break;

            pCurKf = *sit;

            if(pCurKf->isBad())
            {
                sit = mspBufferKfOut.erase(sit);
                continue;
            }

            int nFullKFs = msgMap.Keyframes.size(); //we need this to determine whether a full KF was added or not

            pCurKf->ConvertToMessage(msgMap);

            pCurKf->UnMarkInOutBuffer();

            if(msgMap.Keyframes.size() > nFullKFs)
            {
                AckPairKF pAck = make_pair(pCurKf->mId.first,pCurKf);
                mlKfOpenAcks.push_back(pAck);

                if(pCurKf->mId.first > mnMaxKfIdSent && pCurKf->mId.second == mClientId)
                    mnMaxKfIdSent = pCurKf->mId.first;
            }

            if(!pKFFront)
                pKFFront = pCurKf;

            sit = mspBufferKfOut.erase(sit);
            ++ItCount;

            #ifndef HIDEBUFFERLIMITS
            if(ItCount == mKfItBoundPub)
            {
                cout << "\033[1;34m!!! NOTICE !!!\033[0m " << __func__ << ":" << __LINE__ << " reached OUTPUT KF iteration limit [buffsize: " << mspBufferKfOut.size() << "]" << endl;
            }
            #endif
        }
    }

    if(!pKFFront)
    {
        pKFFront = mpKFLastFront;
    }

    if(!pKFFront && mpCC->mpCH->GetTrackPtr()->mState==Tracking::eTrackingState::OK)
    {
        return;
    }
    else
        mpKFLastFront = pKFFront;

    ItCount = 0;

    //MapPoints
    {
        mpptr pCurMp;
        set<mpptr>::iterator sit = mspBufferMpOut.begin();

        while(!mspBufferMpOut.empty() && (ItCount < mMpItBoundPub)) //Do not call checkBuffer()-method -- it also need the BufferOutMutex
        {
            if(sit == mspBufferMpOut.end())
                break;

            pCurMp = *sit;

            if(pCurMp->isBad())
            {
                sit = mspBufferMpOut.erase(sit);
                continue;
            }

            if(!pCurMp->IsSent() && mnMaxKfIdSent < pCurMp->GetMaxObsKFId())
            {
                ++sit;
                continue;
            }

            int nFullMPs = msgMap.MapPoints.size(); //we need this to determine whether a full MP was added or not

            pCurMp->ConvertToMessage(msgMap,pKFFront);

            pCurMp->UnMarkInOutBuffer();

            if(msgMap.MapPoints.size() > nFullMPs)
            {
                AckPairMP pAck = make_pair(pCurMp->mId.first,pCurMp);
                mlMpOpenAcks.push_back(pAck);
            }

            sit = mspBufferMpOut.erase(sit);
            ++ItCount;

            #ifndef HIDEBUFFERLIMITS
            if(ItCount == mMpItBoundPub)
            {
                cout << "\033[1;34m!!! NOTICE !!!\033[0m " << __func__ << ":" << __LINE__ << " reached OUTPUT MP iteration limit -- size: " << mspBufferMpOut.size() << endl;
            }
            #endif
        }
    }

    kfptr pClosestKF = mpCC->mpCH->GetCurrentRefKFfromTracking();

    if(pClosestKF)
    {
        msgMap.ClosestKf_Id = static_cast<uint16_t>(pClosestKF->mId.first);
        msgMap.ClosestKf_ClientId = static_cast<uint8_t>(pClosestKF->mId.second);
    }
    else
    {
        msgMap.ClosestKf_Id = KFRANGE;
        msgMap.ClosestKf_ClientId = MAPRANGE;
    }

    bool bSend = !msgMap.Keyframes.empty() || !msgMap.KFUpdates.empty() || !msgMap.MapPoints.empty() || msgMap.MPUpdates.empty() || msgMap.ClosestKf_Id != KFRANGE;

    if(bSend)
    {
        ++mOutMapCount;
        msgMap.mMsgId = mOutMapCount;
        msgMap.header.stamp = ros::Time::now();
        mPubMap.publish(msgMap);
    }
}

void Communicator::PublishMapServer()
{
    double dTimeDiff = ros::Time::now().toSec() - mdLastTimePub;
    if(dTimeDiff < mdPeriodicTime)
        return;

    ccmslam_msgs::Map msgMap;

    mdLastTimePub = ros::Time::now().toSec();

    //KF Acks
    for(set<size_t>::iterator sit = msAcksKF.begin();sit != msAcksKF.end();)
    {
        size_t id = *sit;
        msgMap.vAckKFs.push_back((uint16_t)id);
        sit = msAcksKF.erase(sit);
    }

    //MP Acks
    for(set<size_t>::iterator sit = msAcksMP.begin();sit != msAcksMP.end();)
    {
        size_t id = *sit;
        msgMap.vAckMPs.push_back((uint32_t)id);
        sit = msAcksMP.erase(sit);
    }

    //Weak Acks
    msgMap.WeakAckKF = (uint16_t)mnWeakAckKF;
    mnWeakAckKF = KFRANGE;

    msgMap.WeakAckMP = (uint32_t)mnWeakAckMP;
    mnWeakAckMP = MPRANGE;

    //fill with vicitity information
    if(mpMap->KeyFramesInMap() > 10) //start after 10 KFs
    {
        unique_lock<mutex> lock(mMutexNearestKf);

        if(!mpNearestKF)
        {
            //start a new attempt to get the pointer
            kfptr pKF = mpMap->GetKfPtr(mNearestKfId);
            if(pKF)
                mpNearestKF = pKF;
        }

        if(mpNearestKF && mNearestKfId.first != KFRANGE)
        {
            if(mpNearestKF->mId != mNearestKfId)
            {
                //try again to get ptr
                kfptr pKF = mpMap->GetKfPtr(mNearestKfId);
                if(pKF)
                    mpNearestKF = pKF;
                else
                {
                    //try the closest ones -- maybe this can be found
                    for(size_t itclose = 1; itclose < 1+SERVERCURKFSEARCHITS; ++itclose)
                    {
                        pKF = mpMap->GetKfPtr(mNearestKfId.first - itclose,mNearestKfId.second);
                        if(pKF)
                        {
                            mpNearestKF = pKF;
                            break;
                        }
                    }
                }
            }

            //if we cannot find the current nearest KF, we use the last valid one
            mpMap->PackVicinityToMsg(mpNearestKF,msgMap,mpCC);
        }
    }

    //publish (if not empty)

    if(!msgMap.vAckKFs.empty() || !msgMap.vAckMPs.empty() || !msgMap.Keyframes.empty() || !msgMap.MapPoints.empty() || !(msgMap.WeakAckKF == KFRANGE) || !(msgMap.WeakAckMP == MPRANGE))
    {
        ++mServerMapCount;
        msgMap.mMsgId = mServerMapCount;
        msgMap.header.stamp = ros::Time::now();

        if(params::comm::server::miKfLimitToClient == 0)
        {
            msgMap.Keyframes.clear();
            msgMap.MapPoints.clear();
            msgMap.KFUpdates.clear();
            msgMap.MPUpdates.clear();
        }

        mPubMap.publish(msgMap);
    }
}

void Communicator::ProcessKfInClient()
{
    int ItCount = 0;

    while (!mlBufKFin.empty() && (ItCount < mKfItBound))
    {
        msgKFPair msgpair = mlBufKFin.front();
        mlBufKFin.pop_front();

        if(msgpair.first.mClientId != MAPRANGE)
        {
            ccmslam_msgs::KF* pMsg = new ccmslam_msgs::KF();

            *pMsg = msgpair.first;

            {
                idpair RefID = make_pair(pMsg->mpPred_KfId,pMsg->mpPred_KfClientId);
                kfptr pRefKf = mpMap->GetKfPtr(RefID);
                if(!pRefKf)
                {
                    delete pMsg;
                    continue;
                }
            }
            if(pMsg->mbBad)
            {
                delete pMsg;
                continue;
            }

            kfptr pKF = mpMap->GetKfPtr(pMsg->mnId,pMsg->mClientId);

            if(pKF)
            {
                if(pMsg->mbPoseChanged)
                {

                    pKF->UpdateFromMessage(pMsg);

                    pKF->UpdateConnections();
                }

            }
            else
            {
                pKF.reset(new KeyFrame(pMsg,mpVoc,mpMap,mpDatabase,shared_from_this(),mpCC->mSysState));

                pKF->mdServerTimestamp = ros::Time::now().toNSec();

                if(pKF->isBad())
                {
                    delete pMsg;
                    continue;
                }

                pKF->EstablishInitialConnectionsClient();
                if(pKF->isBad())
                {
                    delete pMsg;
                    continue;
                }

                mpMap->AddKeyFrame(pKF);
                mlpAddedKfs.push_back(pKF);
            }

            delete pMsg;
        }
        else if(msgpair.second.mClientId != MAPRANGE)
        {
            cout << "\033[1;31m!!!!! FATAL ERROR !!!!!\033[0m " << __func__ << ":"  << __LINE__ << " Reception of reduced KF not implemented" << endl;
            throw estd::infrastructure_ex();   
        }

        ++ItCount;

        #ifndef HIDEBUFFERLIMITS
        if(ItCount == mKfItBound)
        {
            cout << "\033[1;34m!!! NOTICE !!!\033[0m " << __func__ << ":" << __LINE__ << " -- Agent " << mpCC->mClientId << ": reached INPUT KF iteration limit" << "[" << mlBufKFin.size() << "]" << endl;
        }
        #endif
    }
}

void Communicator::ProcessKfInServer()
{
    int ItCount = 0;

    while (!mlBufKFin.empty() && (ItCount < mKfItBound))
    {
        msgKFPair msgpair = mlBufKFin.front();
        mlBufKFin.pop_front();

        if(msgpair.first.mClientId != MAPRANGE)
        {
            ccmslam_msgs::KF* pMsg = new ccmslam_msgs::KF();

            *pMsg = msgpair.first;
            {
                kfptr pKF = mpMap->GetKfPtr(pMsg->mnId,pMsg->mClientId);
                if(pKF)
                {
                    //Note: this can happen, if the Ack from server to client gets lost
                    msAcksKF.insert(pMsg->mnId);
                    delete pMsg;
                    continue;
                }
            }
            if(mpMap->IsKfDeleted(pMsg->mnId,pMsg->mClientId))
            {
                //Note: this can happen, if the Ack from server to client gets lost
                msAcksKF.insert(pMsg->mnId);
                delete pMsg;
                continue;
            }

            kfptr pKF{new KeyFrame(pMsg,mpVoc,mpMap,mpDatabase,shared_from_this(),mpCC->mSysState,mpCC->mpUID->GetId(),mpCC->mg2oS_wcurmap_wclientmap)};

            pKF->mdServerTimestamp = ros::Time::now().toNSec();

            if(pKF->isBad())
            {
                //could not be processed, but send weak ack
                this->SetWeakAckKF(pMsg->mnId);
                delete pMsg;
                continue;
            }

            pKF->EstablishInitialConnectionsServer();
            if(pKF->isBad())
            {
                this->SetWeakAckKF(pMsg->mnId);
                delete pMsg;
                continue;
            }

            pKF->UpdateConnections();

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
                    if(!(pKF->mId.first==0))
                        std::cout << COUTERROR << "KF " << pKF->mId.first << "|" << pKF->mId.second << " : no parent" << std::endl;
                }
            }
            #endif

            mpMap->AddKeyFrame(pKF);
            mpMapping->InsertKeyFrame(pKF);
            msAcksKF.insert(pMsg->mnId);

            if(pKF->mId.first == 0)
            {
                mpMap->mvpKeyFrameOrigins.push_back(pKF);
            }

            delete pMsg;
        }
        else if(msgpair.second.mClientId != MAPRANGE)
        {
            ccmslam_msgs::KFred* pMsg = new ccmslam_msgs::KFred();

            *pMsg = msgpair.second;

            kfptr pKF = mpMap->GetKfPtr(pMsg->mnId,pMsg->mClientId);

            if(pKF)
            {
                //everything ok
            }
            else
            {
                if(!mpMap->IsKfDeleted(pMsg->mnId,pMsg->mClientId))
                {
                    this->SetWeakAckKF(pMsg->mnId);
                }
            }

            if(!pKF)
            {
                delete pMsg;
                continue; //maybe it is deleted, maybe it got lost -- but it is not there
            }

            if(pKF->isBad())
            {
                delete pMsg;
                continue; //no need to process bad KFs
            }

            pKF->UpdateFromMessage(pMsg,mpCC->mg2oS_wcurmap_wclientmap);
            pKF->UpdateConnections();

            delete pMsg;
        }

        ++ItCount;

        #ifndef HIDEBUFFERLIMITS
        if(ItCount == mKfItBound)
        {
            cout << "\033[1;34m!!! NOTICE !!!\033[0m " << __func__ << ":" << __LINE__ << " -- Agent " << mpCC->mClientId << ": reached INPUT KF iteration limit" << "[" << mlBufKFin.size() << "]" << endl;
        }
        #endif
    }
}

void Communicator::ProcessMpInClient()
{
    int ItCount = 0;

    while (!mlBufMPin.empty() && (ItCount < mMpItBound))
    {
        msgMPPair msgpair = mlBufMPin.front();
        mlBufMPin.pop_front();

        if(msgpair.first.mClientId != MAPRANGE)
        {
            ccmslam_msgs::MP* pMsg = new ccmslam_msgs::MP();

            *pMsg = msgpair.first;

            {
                idpair RefID = make_pair(pMsg->mpPredKFId,pMsg->mpPredKFClientId);
                kfptr pRefKf = mpMap->GetKfPtr(RefID);
                if(!pRefKf)
                {
                    delete pMsg;
                    continue;
                }
            }

            if(pMsg->mbBad)
            {
                delete pMsg;
                continue;
            }

            mpptr pMP = mpMap->GetMpPtr(pMsg->mnId,pMsg->mClientId);

            if(pMP)
            {
                if(pMsg->mbPoseChanged)
                {
                    pMP->UpdateFromMessage(pMsg);

                    pMP->UpdateNormalAndDepth();
                }
            }
            else
            {
                pMP.reset(new MapPoint(pMsg,mpMap,shared_from_this(),mpCC->mSysState));

                if(pMP->isBad())
                {
                    delete pMsg;
                    continue;
                }

                pMP->EstablishInitialConnectionsClient();

                if(pMP->isBad())
                {
                    delete pMsg;
                    continue;
                }

                mpMap->AddMapPoint(pMP);
            }

            delete pMsg;
        }
        else if(msgpair.second.mClientId != MAPRANGE)
        {
            cout << "\033[1;31m!!!!! FATAL ERROR !!!!!\033[0m " << __func__ << ":"  << __LINE__ << " Reception of reduced KF not implemented" << endl;
            throw infrastructure_ex();
        }

        ++ItCount;
        #ifndef HIDEBUFFERLIMITS
        if(ItCount == mMpItBound)
        {
            cout << "\033[1;34m!!! NOTICE !!!\033[0m " << __func__ << ":" << __LINE__ << " -- Agent " << mpCC->mClientId << ": reached INPUT MP iteration limit" << "[" << mlBufMPin.size() << "]" << endl;
        }
        #endif
    }
}

void Communicator::ProcessMpInServer()
{
    int ItCount = 0;

    while (!mlBufMPin.empty() && (ItCount < mMpItBound))
    {
        msgMPPair msgpair = mlBufMPin.front();
        mlBufMPin.pop_front();

        if(msgpair.first.mClientId != MAPRANGE)
        {
            ccmslam_msgs::MP* pMsg = new ccmslam_msgs::MP();

            *pMsg = msgpair.first;

            {
                mpptr pMP = mpMap->GetMpPtr(pMsg->mnId,pMsg->mClientId);
                if(pMP)
                {
                    //Note: this can happen, if the Ack from server to client gets lost
                    msAcksMP.insert(pMsg->mnId);
                    delete pMsg;
                    continue;
                }
            }
            if(mpMap->IsMpDeleted(pMsg->mnId,pMsg->mClientId))
            {
                //Note: this can happen, if the Ack from server to client gets lost
                msAcksMP.insert(pMsg->mnId);
                delete pMsg;
                continue;
            }

            mpptr pMP{new MapPoint(pMsg,mpMap,shared_from_this(),mpCC->mSysState,mpCC->mpUID->GetId(),mpCC->mg2oS_wcurmap_wclientmap)};

            if(pMP->isBad())
            {
                //could not be processed, but send weak ack
                this->SetWeakAckMP(pMsg->mnId);
                delete pMsg;
                continue;
            }

            pMP->EstablishInitialConnectionsServer();

            if(pMP->isBad())
            {
                this->SetWeakAckMP(pMsg->mnId);
                delete pMsg;
                continue;
            }

            mpMap->AddMapPoint(pMP);
            msAcksMP.insert(pMsg->mnId);

            delete pMsg;
        }
        else if(msgpair.second.mClientId != MAPRANGE)
        {
            ccmslam_msgs::MPred* pMsg = new ccmslam_msgs::MPred();

            *pMsg = msgpair.second;

            mpptr pMP = mpMap->GetMpPtr(pMsg->mnId,pMsg->mClientId);

            if(pMP)
            {
                //everything ok
            }
            else
            {
                if(!mpMap->IsMpDeleted(pMsg->mnId,pMsg->mClientId))
                {
                    this->SetWeakAckMP(pMsg->mnId);
                }
            }

            if(!pMP)
            {    
                delete pMsg;
                continue; //maybe it is deleted, maybe it got lost -- but it is not there
            }

            if(pMP->isBad())
            {
                delete pMsg;
                continue; //no need to process bad MPs.
            }

            if(pMsg->mbBad)
            {
                delete pMsg;
                continue;
            }

            pMP->UpdateFromMessage(pMsg,mpCC->mg2oS_wcurmap_wclientmap);

            pMP->UpdateNormalAndDepth();

            delete pMsg;
        }

        ++ItCount;

        #ifndef HIDEBUFFERLIMITS
        if(ItCount == mMpItBound)
        {
            cout << "\033[1;34m!!! NOTICE !!!\033[0m " << __func__ << ":" << __LINE__ << " -- Agent " << mpCC->mClientId << ": reached INPUT MP iteration limit" << "[" << mlBufMPin.size() << "]" << endl;
        }
        #endif
    }
}

void Communicator::PassKftoComm(kfptr pKf)
{
    unique_lock<mutex> lockOut(mMutexBuffersOut);

    mspBufferKfOut.insert(pKf);
    pKf->MarkInOutBuffer();
}

void Communicator::PassMptoComm(mpptr pMp)
{
    unique_lock<mutex> lockOut(mMutexBuffersOut);

    mspBufferMpOut.insert(pMp);
    pMp->MarkInOutBuffer();

}

void Communicator::DeleteMpFromBuffer(mpptr pMP)
{
    unique_lock<mutex> lockOut(mMutexBuffersOut);

    set<mpptr>::iterator sit = find(mspBufferMpOut.begin(),mspBufferMpOut.end(),pMP);
    if(sit != mspBufferMpOut.end())
    {
        mspBufferMpOut.erase(sit);
    }
}

bool Communicator::CheckBufferKfIn()
{
    unique_lock<mutex> lock(mMutexBuffersIn);
    return(!mlBufKFin.empty());
}

bool Communicator::CheckBufferKfOut()
{
    unique_lock<mutex> lockOut(mMutexBuffersOut);
    return(!mspBufferKfOut.empty());
}

bool Communicator::CheckBufferMpIn()
{
    unique_lock<mutex> lock(mMutexBuffersIn);
    return(!mlBufMPin.empty());
}

bool Communicator::CheckBufferMpOut()
{
    unique_lock<mutex> lockOut(mMutexBuffersOut);
    return(!mspBufferMpOut.empty());
}

void Communicator::RequestReset()
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
        if(mpCC->mSysState == eSystemState::CLIENT)
            usleep(params::timings::client::miCommRate);
        else if(mpCC->mSysState == eSystemState::SERVER)
            usleep(params::timings::server::miCommRate);
        else KILLSYS
    }
}

void Communicator::ResetIfRequested()
{
    unique_lock<mutex> lockReset(mMutexReset);

    if(mbResetRequested)
    {
        if(mpCC->mSysState == eSystemState::CLIENT)
        {
            this->ResetCommunicator();
        }
        else if(mpCC->mSysState == eSystemState::SERVER)
        {
            this->ResetCommunicator();
            this->ResetMapping();
            this->ResetDatabase();
            this->ResetMap();
        }
        else
        {
            cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Communicator::ResetIfRequested(): invalid systems state: " << mpCC->mSysState << endl;
            throw infrastructure_ex();
        }
         mbResetRequested = false;
    }
}

void Communicator::ResetCommunicator()
{
    unique_lock<mutex> lockIn(mMutexBuffersIn);
    unique_lock<mutex> lockOut(mMutexBuffersOut);
    unique_lock<mutex> lock5(mMutexNearestKf);

    usleep(10000); // wait to give msg buffers time to empty

    mlBufKFin.clear();
    mlBufMPin.clear();
    mspBufferKfOut.clear();
    mspBufferMpOut.clear();

    msAcksKF.clear();
    msAcksMP.clear();

    mpNearestKF = nullptr;
    mNearestKfId = defpair;
}

void Communicator::ResetDatabase()
{
    vector<kfptr> vpKFs = mpMap->GetAllKeyFrames();

    for(vector<kfptr>::iterator vit=vpKFs.begin();vit!=vpKFs.end();++vit)
    {
        kfptr pKF = *vit;
        mpDatabase->erase(pKF);
    }

    mpDatabase->ResetMPs();
}

void Communicator::ResetMapping()
{
    mpMapping->RequestReset();
}

void Communicator::ResetMap()
{
    mpMap->clear();
}

void Communicator::SetWeakAckKF(size_t id)
{
    if(mnWeakAckKF == KFRANGE)
        mnWeakAckKF = id;
    else if(id > mnWeakAckKF)
        mnWeakAckKF = id;
}

void Communicator::SetWeakAckMP(size_t id)
{
    if(mnWeakAckMP == MPRANGE)
        mnWeakAckMP = id;
    else if(id > mnWeakAckMP)
        mnWeakAckMP = id;
}

bool Communicator::kfcmp::operator ()(const kfptr pA, const kfptr pB) const
{
    return pA->mId.first < pB->mId.first;
}

bool Communicator::mpcmp::operator ()(const mpptr pA, const mpptr pB) const
{
    return pA->mId.first < pB->mId.first;
}

} //end ns
