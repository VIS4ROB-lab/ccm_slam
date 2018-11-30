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

#include <cslam/ClientHandler.h>

namespace cslam {

ClientHandler::ClientHandler(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, vocptr pVoc, dbptr pDB, mapptr pMap, size_t ClientId, uidptr pUID, eSystemState SysState, const string &strCamFile, viewptr pViewer)
    : mpVoc(pVoc),mpKFDB(pDB),mpMap(pMap),
      mNh(Nh),mNhPrivate(NhPrivate),
      mClientId(ClientId), mpUID(pUID), mSysState(SysState),
      mstrCamFile(strCamFile),
      mpViewer(pViewer),mbReset(false)
{
    if(mpVoc == nullptr || mpKFDB == nullptr || mpMap == nullptr || (mpUID == nullptr && mSysState == eSystemState::SERVER))
    {
        cout << ("In \" ClientHandler::ClientHandler(...)\": nullptr exception") << endl;
        throw estd::infrastructure_ex();
    }

    mpMap->msuAssClients.insert(mClientId);

    mg2oS_wcurmap_wclientmap = g2o::Sim3(); //identity transformation

    if(mSysState == eSystemState::CLIENT)
    {
        std::string TopicNameCamSub;

        mNhPrivate.param("TopicNameCamSub",TopicNameCamSub,string("nospec"));
        mSubCam = mNh.subscribe<sensor_msgs::Image>(TopicNameCamSub,10,boost::bind(&ClientHandler::CamImgCb,this,_1));

        cout << "Camera Input topic: " << TopicNameCamSub << endl;
    }
}
#ifdef LOGGING
void ClientHandler::InitializeThreads(boost::shared_ptr<estd::mylog> pLogger)
#else
void ClientHandler::InitializeThreads()
#endif
{
    #ifdef LOGGING
    this->InitializeCC(pLogger);
    #else
    this->InitializeCC();
    #endif

    if(mSysState == eSystemState::CLIENT)
    {
        this->InitializeClient();
    }
    else if(mSysState == eSystemState::SERVER)
    {
        this->InitializeServer();
    }
    else
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m ClientHandler::InitializeThreads(): invalid systems state: " << mpCC->mSysState << endl;
        throw infrastructure_ex();
    }
}

#ifdef LOGGING
void ClientHandler::InitializeCC(boost::shared_ptr<mylog> pLogger)
#else
void ClientHandler::InitializeCC()
#endif
{
    std::stringstream* ss;

    mpCC.reset(new CentralControl(mNh,mNhPrivate,mClientId,mSysState,shared_from_this(),mpUID));

    if(mSysState == eSystemState::CLIENT)
    {
        ss = new stringstream;
        *ss << "FrameId";
        mNhPrivate.param(ss->str(),mpCC->mNativeOdomFrame,std::string("nospec"));
    }
    else if(mSysState == eSystemState::SERVER)
    {
        ss = new stringstream;
        *ss << "FrameId" << mClientId;
        mNhPrivate.param(ss->str(),mpCC->mNativeOdomFrame,std::string("nospec"));
    }
    else
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m ClientHandler::InitializeThreads(): invalid systems state: " << mpCC->mSysState << endl;
        throw infrastructure_ex();
    }

    if(mpCC->mNativeOdomFrame=="nospec")
    {
        ROS_ERROR_STREAM("In \" ServerCommunicator::ServerCommunicator(...)\": bad parameters");
        throw estd::infrastructure_ex();
    }

    {
        if(mSysState==CLIENT)
        {
            cv::FileStorage fSettings(mstrCamFile, cv::FileStorage::READ);

            float c0t00 = fSettings["Cam0.T00"];
            float c0t01 = fSettings["Cam0.T01"];
            float c0t02 = fSettings["Cam0.T02"];
            float c0t03 = fSettings["Cam0.T03"];
            float c0t10 = fSettings["Cam0.T10"];
            float c0t11 = fSettings["Cam0.T11"];
            float c0t12 = fSettings["Cam0.T12"];
            float c0t13 = fSettings["Cam0.T13"];
            float c0t20 = fSettings["Cam0.T20"];
            float c0t21 = fSettings["Cam0.T21"];
            float c0t22 = fSettings["Cam0.T22"];
            float c0t23 = fSettings["Cam0.T23"];
            float c0t30 = fSettings["Cam0.T30"];
            float c0t31 = fSettings["Cam0.T31"];
            float c0t32 = fSettings["Cam0.T32"];
            float c0t33 = fSettings["Cam0.T33"];
            mpCC->mT_SC << c0t00,c0t01,c0t02,c0t03,c0t10,c0t11,c0t12,c0t13,c0t20,c0t21,c0t22,c0t23,c0t30,c0t31,c0t32,c0t33;
        }
        else
        {
            //no mstrCamFile on Server...
        }
    }

    mpMap->mOdomFrame = mpCC->mNativeOdomFrame;
    mpMap->AddCCPtr(mpCC);

    #ifdef LOGGING
    mpCC->mpLogger = pLogger;
    #endif

    delete ss;
}

void ClientHandler::InitializeClient()
{
    cout << "Client " << mClientId << " --> Initialize Threads" << endl;

    //+++++ Create Drawers. These are used by the Viewer +++++
    mpViewer.reset(new Viewer(mpMap,mpCC));
    usleep(10000);
    //+++++ Initialize the Local Mapping thread +++++
    mpMapping.reset(new LocalMapping(mpCC,mpMap,mpKFDB,mpViewer));
    usleep(10000);
//    +++++ Initialize the communication thread +++++
    mpComm.reset(new Communicator(mpCC,mpVoc,mpMap,mpKFDB));
    mpComm->SetMapping(mpMapping);
    usleep(10000);
    mpMap->SetCommunicator(mpComm);
    mpMapping->SetCommunicator(mpComm);
    usleep(10000);
    //+++++ Initialize the tracking thread +++++
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracking.reset(new Tracking(mpCC, mpVoc, mpViewer, mpMap, mpKFDB, mstrCamFile, mClientId));
    usleep(10000);
    mpTracking->SetCommunicator(mpComm);
    mpTracking->SetLocalMapper(mpMapping);
    mpViewer->SetTracker(mpTracking);
    usleep(10000);
    //Launch Threads
    //Should no do that before, a fast system might already use a pointe before it was set -> segfault
    mptMapping.reset(new thread(&LocalMapping::RunClient,mpMapping));
    mptComm.reset(new thread(&Communicator::RunClient,mpComm));
    mptViewer.reset(new thread(&Viewer::RunClient,mpViewer));
    usleep(10000);
}

void ClientHandler::InitializeServer()
{
    cout << "Client " << mClientId << " --> Initialize Threads" << endl;

    //+++++ Initialize the Loop Finder thread and launch +++++
    mpLoopFinder.reset(new LoopFinder(mpCC,mpKFDB,mpVoc,mpMap));
    mptLoopClosure.reset(new thread(&LoopFinder::Run,mpLoopFinder));
    usleep(10000);
    //+++++ Initialize the Local Mapping thread +++++
    mpMapping.reset(new LocalMapping(mpCC,mpMap,mpKFDB,mpViewer));
    mpMapping->SetLoopFinder(mpLoopFinder); //tempout
    usleep(10000);
    //+++++ Initialize the communication thread +++++
    mpComm.reset(new Communicator(mpCC,mpVoc,mpMap,mpKFDB));
    mpComm->SetMapping(mpMapping);
    usleep(10000);
    mpMapping->SetCommunicator(mpComm);
    mpMap->SetCommunicator(mpComm);
    usleep(10000);
    //Launch Threads
    //Should not do that before, a fast system might already use a pointer before it was set -> segfault
    mptMapping.reset(new thread(&LocalMapping::RunServer,mpMapping));
    mptComm.reset(new thread(&Communicator::RunServer,mpComm));
    usleep(10000);
    if(mpCC->mpCH == nullptr)
    {
        ROS_ERROR_STREAM("ClientHandler::InitializeThreads()\": mpCC->mpCH is nullptr");
        throw estd::infrastructure_ex();
    }
}

void ClientHandler::ChangeMap(mapptr pMap, g2o::Sim3 g2oS_wnewmap_wcurmap)
{
    mpMap = pMap;

    mg2oS_wcurmap_wclientmap = g2oS_wnewmap_wcurmap*mg2oS_wcurmap_wclientmap;
    mpCC->mg2oS_wcurmap_wclientmap = mg2oS_wcurmap_wclientmap;

    bool bLockedComm = mpCC->LockComm(); //should be locked and therefore return false
//    #ifdef LOGGING
//    mpCC->mpLogger->SetMappingLock(__LINE__,mpCC->mClientId);
//    #endif
    bool bLockedMapping = mpCC->LockMapping(); //should be locked and therefore return false
//    #ifdef LOGGING
//    mpCC->mpLogger->SetMappingLock(__LINE__,mpCC->mClientId);
//    #endif

    if(bLockedComm || bLockedMapping)
    {
        if(bLockedComm) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m ClientHandler::ChangeMap(): Comm not locked: " << endl;
        if(bLockedMapping) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m ClientHandler::ChangeMap(): Mapping not locked: " << endl;
        throw infrastructure_ex();
    }

//    #ifdef LOGGING
//    mpCC->mpLogger->SetMappingLock(__LINE__,mpCC->mClientId);
//    #endif

    mpComm->ChangeMap(mpMap);
    mpMapping->ChangeMap(mpMap); //tempout
    mpLoopFinder->ChangeMap(mpMap); //tempout

//    #ifdef LOGGING
//    mpCC->mpLogger->SetMappingLock(__LINE__,mpCC->mClientId);
//    #endif
}


void ClientHandler::SetMapMatcher(matchptr pMatch)
{
    mpMapMatcher = pMatch;
    mpComm->SetMapMatcher(mpMapMatcher);
    mpMapping->SetMapMatcher(mpMapMatcher);
}

void ClientHandler::CamImgCb(sensor_msgs::ImageConstPtr pMsg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(pMsg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracking->Reset();
            mbReset = false;
        }
    }

    mpTracking->GrabImageMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
}

void ClientHandler::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

ClientHandler::kfptr ClientHandler::GetCurrentRefKFfromTracking()
{
    if(mpTracking->mState < 2)
        return nullptr;
    else
        return mpTracking->GetReferenceKF();
}

int ClientHandler::GetNumKFsinLoopFinder()
{
    if(mpLoopFinder)
        return mpLoopFinder->GetNumKFsinQueue();
    else
        return -1;
}

int ClientHandler::GetNumKFsinMapMatcher()
{
    if(mpMapMatcher)
        return mpMapMatcher->GetNumKFsinQueue();
    else
        return -1;
}

void ClientHandler::ClearCovGraph(size_t MapId)
{
    mpMapping->ClearCovGraph(MapId);
}

//#ifdef LOGGING
//void ClientHandler::SetLogger(boost::shared_ptr<mylog> pLogger)
//{
//    mpCC->mpLogger = pLogger;
//}
//#endif

} //end ns
