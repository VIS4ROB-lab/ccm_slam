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

#include <cslam/Viewer.h>

namespace cslam {

Viewer::Viewer(mapptr pMap, ccptr pCC)
    : mpMap(pMap),mpCC(pCC),mbDrawFrame(false)
{
    mState=Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));

    std::stringstream* ss;

    if(mpCC->mSysState == eSystemState::CLIENT)
    {
        ss = new stringstream;
        *ss << "ImageClient" << mpCC->mClientId;

        mpIT.reset(new image_transport::ImageTransport(mNh));
        mPubIm = mpIT->advertise(ss->str(),1);

        mSysType = "c";

        if(mpMap->mMapId == 0)
        {
            ss = new stringstream;
            *ss << "ClientMarkerMap" << mpMap->mMapId;
            mPubMarker0 = mNh.advertise<visualization_msgs::Marker>(ss->str(),10);

            ss = new stringstream;
            *ss << "ClientMapPointsMap" << mpMap->mMapId;
            mPubPcl0 = mNh.advertise<sensor_msgs::PointCloud2>(ss->str(),10);

            ss = new stringstream;
            *ss << "LocalMPs" << mpMap->mMapId;
            mPubLocalMPs = mNh.advertise<sensor_msgs::PointCloud2>(ss->str(),10);
        }
        else if(mpMap->mMapId == 1)
        {
            ss = new stringstream;
            *ss << "ClientMarkerMap" << mpMap->mMapId;
            mPubMarker1 = mNh.advertise<visualization_msgs::Marker>(ss->str(),10);

            ss = new stringstream;
            *ss << "ClientMapPointsMap" << mpMap->mMapId;
            mPubPcl1 = mNh.advertise<sensor_msgs::PointCloud2>(ss->str(),10);

            ss = new stringstream;
            *ss << "LocalMPs" << mpMap->mMapId;
            mPubLocalMPs = mNh.advertise<sensor_msgs::PointCloud2>(ss->str(),10);
        }
        else if(mpMap->mMapId == 2)
        {
            ss = new stringstream;
            *ss << "ClientMarkerMap" << mpMap->mMapId;
            mPubMarker2 = mNh.advertise<visualization_msgs::Marker>(ss->str(),10);

            ss = new stringstream;
            *ss << "ClientMapPointsMap" << mpMap->mMapId;
            mPubPcl2 = mNh.advertise<sensor_msgs::PointCloud2>(ss->str(),10);

            ss = new stringstream;
            *ss << "LocalMPs" << mpMap->mMapId;
            mPubLocalMPs = mNh.advertise<sensor_msgs::PointCloud2>(ss->str(),10);
        }
        else if(mpMap->mMapId == 3)
        {
            ss = new stringstream;
            *ss << "ClientMarkerMap" << mpMap->mMapId;
            mPubMarker3 = mNh.advertise<visualization_msgs::Marker>(ss->str(),10);

            ss = new stringstream;
            *ss << "ClientMapPointsMap" << mpMap->mMapId;
            mPubPcl3 = mNh.advertise<sensor_msgs::PointCloud2>(ss->str(),10);

            ss = new stringstream;
            *ss << "LocalMPs" << mpMap->mMapId;
            mPubLocalMPs = mNh.advertise<sensor_msgs::PointCloud2>(ss->str(),10);
        }
        else
            cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << ": mpMap->mMapId not in [0,3]" << endl;
    }
    else
    {
        mSysType = "s";

        //++++++++++ Map 0 +++++++++

        ss = new stringstream;
        *ss << "ServerMarkerMap0";
        mPubMarker0 = mNh.advertise<visualization_msgs::Marker>(ss->str(),10);

        ss = new stringstream;
        *ss << "ServerMapPointsMap0";
        mPubPcl0 = mNh.advertise<sensor_msgs::PointCloud2>(ss->str(),10);

        //++++++++++ Map 1 +++++++++

        ss = new stringstream;
        *ss << "ServerMarkerMap1";
        mPubMarker1 = mNh.advertise<visualization_msgs::Marker>(ss->str(),10);

        ss = new stringstream;
        *ss << "ServerMapPointsMap1";
        mPubPcl1 = mNh.advertise<sensor_msgs::PointCloud2>(ss->str(),10);

        //++++++++++ Map 2 +++++++++

        ss = new stringstream;
        *ss << "ServerMarkerMap2";
        mPubMarker2 = mNh.advertise<visualization_msgs::Marker>(ss->str(),10);

        ss = new stringstream;
        *ss << "ServerMapPointsMap2";
        mPubPcl2 = mNh.advertise<sensor_msgs::PointCloud2>(ss->str(),10);

        //++++++++++ Map 3 +++++++++

        ss = new stringstream;
        *ss << "ServerMarkerMap3";
        mPubMarker3 = mNh.advertise<visualization_msgs::Marker>(ss->str(),10);

        ss = new stringstream;
        *ss << "ServerMapPointsMap3";
        mPubPcl3 = mNh.advertise<sensor_msgs::PointCloud2>(ss->str(),10);
    }

    delete ss;

    mvNumPoints.resize(4,0);
}

void Viewer::RunClient()
{
    while(1)
    {
        if(this->CheckVisData())
        {
            unique_lock<mutex> lockVis(mMutexDrawMap);

            for(map<string,VisBundle>::iterator mit = mmVisData.begin();mit!=mmVisData.end();++mit)
            {
                msCurFrame = mit->first;
                mCurVisBundle = mit->second;

                if(params::vis::mbShowCovGraph)
                    this->PubCovGraph();

                if(params::vis::mbShowKFs)
                {
                    this->ClearMarkers(mCurVisBundle.mNativeId);

                    this->PubKeyFramesAsFrusta();
                }

                if(params::vis::mbShowTraj)
                    this->PubTrajectories();

                if(params::vis::mbShowMPs)
                    this->PubMapPointsAsCloud();
            }
            mmVisData.clear();
        }

        this->ResetIfRequested();

        usleep(params::timings::client::miViewerRate);
    }
}

void Viewer::RunServer()
{
    while(1)
    {
        if(this->CheckVisData())
        {
            unique_lock<mutex> lockVis(mMutexDrawMap);

            for(map<string,VisBundle>::iterator mit = mmVisData.begin();mit!=mmVisData.end();++mit)
            {
                msCurFrame = mit->first;
                mCurVisBundle = mit->second;

                if(msBlockedMaps.count(mCurVisBundle.mNativeId))
                    continue;

                if(params::vis::mbShowCovGraph)
                    this->PubCovGraph();

                if(params::vis::mbShowKFs)
                {
                    this->ClearMarkers(mCurVisBundle.mNativeId);

                    this->PubKeyFramesAsFrusta();
                }

                if(params::vis::mbShowTraj)
                    this->PubTrajectories();

                if(params::vis::mbShowMPs)
                {
                    this->PubMapPointsAsCloud();
                }
            }
            mmVisData.clear();
        }

        this->ResetIfRequested();

        usleep(params::timings::server::miViewerRate);
    }
}

void Viewer::UpdateAndDrawFrame()
{
    {
        unique_lock<mutex> lock(mMutexFrameDraw);

        mpTracker->mImGray.copyTo(mIm);
    }

    this->UpdateAndDraw();

    this->PubFramePoseAsFrustum();
}

void Viewer::UpdateAndDraw()
{
    this->UpdateFrame();
    cv::Mat im = this->DrawFrame();

    std_msgs::Header h;
    h.stamp = ros::Time::now();
    cv_bridge::CvImagePtr pImg{new cv_bridge::CvImage(h,sensor_msgs::image_encodings::BGR8,im)};
    mPubIm.publish(pImg->toImageMsg());

    this->PubLocalMPs();
}

void Viewer::PubLocalMPs()
{
    if(!(mpTracker->mState == Tracking::eTrackingState::OK))
        return;

    unique_lock<mutex> lock(mMutexFrameDraw);

    mvpFrameMPs = mpTracker->mCurrentFrame->mvpMapPoints;

    if(mvpFrameMPs.empty())
            return;

    pcl::PointCloud<pcl::PointXYZRGB> Cloud;

    for(vector<mpptr>::const_iterator vit=mvpFrameMPs.begin();vit!=mvpFrameMPs.end();++vit)
    {
        mpptr pMPi = *vit;

        if(!pMPi)
            continue;

        if((pMPi->IsEmpty()) || (pMPi->isBad()))
            continue;

        size_t ClientId = pMPi->mId.second;

        if(ClientId < 0 || ClientId > 3)
        {
            cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Map::PubMapPointsAsPclMsg(): pMPi->mId.second not in [0,3] -- mClientId: " << pMPi->mId.second << endl;
            throw infrastructure_ex();
        }

        pcl::PointXYZRGB p;
        cv::Mat Tworld = pMPi->GetWorldPos();
        p.x = (params::vis::mfScaleFactor)*((double)(Tworld.at<float>(0,0)));
        p.y = (params::vis::mfScaleFactor)*((double)(Tworld.at<float>(0,1)));
        p.z = (params::vis::mfScaleFactor)*((double)(Tworld.at<float>(0,2)));

        p.r = Colors::rgbGold().r();
        p.g = Colors::rgbGold().g();
        p.b = Colors::rgbGold().b();

        Cloud.points.push_back(p);
    }

    if(!Cloud.points.empty())
    {
        sensor_msgs::PointCloud2 pclMsg;
        pcl::toROSMsg(Cloud,pclMsg);
        pclMsg.header.frame_id = msCurFrame;
        pclMsg.header.stamp = ros::Time::now();
        mPubLocalMPs.publish(pclMsg);
    }
}

void Viewer::UpdateFrame()
{
    unique_lock<mutex> lock(mMutexFrameDraw);
    mvCurrentKeys=mpTracker->mCurrentFrame->mvKeys;
    N = mvCurrentKeys.size();
    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);
    mbOnlyTracking = false;

    if(mpTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        mvIniKeys=mpTracker->mInitialFrame->mvKeys;
        mvIniMatches=mpTracker->mvIniMatches;
    }
    else if(mpTracker->mLastProcessedState==Tracking::OK)
    {
        for(int i=0;i<N;i++)
        {
            mpptr pMP = mpTracker->mCurrentFrame->mvpMapPoints[i];
            if(pMP)
            {
                if(!mpTracker->mCurrentFrame->mvbOutlier[i])
                {
                    if(pMP->Observations()>0)
                        mvbMap[i]=true;
                    else
                        mvbVO[i]=true;
                }
            }
        }
    }
    mState=static_cast<int>(mpTracker->mLastProcessedState);
}

cv::Mat Viewer::DrawFrame()
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    int state; // Tracking state

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutexFrameDraw);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
        }
    } // destroy scoped mutex -> release mutex

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,CV_GRAY2BGR);

    //Draw
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                        cv::Scalar(FEATCOL));
            }
        }
    }
    else if(state==Tracking::OK) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        for(int i=0;i<N;i++)
        {
            if(vbVO[i] || vbMap[i])
            {
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;

                // This is a match to a MapPoint in the map
                if(vbMap[i])
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(FEATCOL));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(FEATCOL),-1);

                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    cv::circle(im,vCurrentKeys[i].pt,7,cv::Scalar(0,0,255),1);
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,0,255),-1);
                    mnTrackedVO++;
                }
            }
        }
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state,imWithInfo);

    return imWithInfo;
}

void Viewer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        if(!mbOnlyTracking)
            s << "TRACKING OK :  ";
        else
            s << "LOCALIZATION | ";
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
        s << "KFs: " << nKFs << "  MPs: " << nMPs << "  Matches: " << mnTracked;
        if(mnTrackedVO>0)
            s << ", + VO matches: " << mnTrackedVO;
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACKING LOST";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING VOCABULARY";
    }

    int baseline=0;
    {
        cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

        //Tracking Information

        imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
        im.copyTo(imText.rowRange(imText.rows-im.rows,imText.rows).colRange(0,im.cols));
        imText.rowRange(0,imText.rows-im.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
        cv::putText(imText,s.str(),cv::Point(5,15),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);
    }
}

void Viewer::DrawMap(mapptr pMap)
{
    if(!pMap) return;

    unique_lock<mutex> lock(mMutexDrawMap);

    VisBundle VB;
    VB.mmpKFs = pMap->GetMmpKeyFrames();
    VB.mmpMPs = pMap->GetMmpMapPoints();
    VB.mNativeId = pMap->mMapId;
    VB.mMaxKfId = pMap->GetMaxKFid();
    VB.mMaxKfIdUnique = pMap->GetMaxKFidUnique();

    if(mpCC->mSysState == eSystemState::SERVER)
    {
        vector<idpair> vMaxids;
        for(int it=0;it<4;++it)
            vMaxids.push_back(make_pair(KFRANGE,it));

        for(map<idpair,kfptr>::iterator mit = VB.mmpKFs.begin();mit!=VB.mmpKFs.end();++mit)
        {
            kfptr pKFi = mit->second;

            if(!pKFi || pKFi->isBad())
                continue;

            idpair maxpair = vMaxids[pKFi->mId.second];
            if(maxpair.first == KFRANGE)
                vMaxids[pKFi->mId.second] = pKFi->mId;
            else
            {
                if(pKFi->mId.first > maxpair.first)
                    vMaxids[pKFi->mId.second] = pKFi->mId;
            }
        }

        for(int it=0;it<4;++it)
        {
            idpair maxpair = vMaxids[it];
            if(maxpair.first != KFRANGE)
                VB.msCurKFIds.insert(maxpair);
        }
    }

    mmVisData[pMap->mOdomFrame] = VB;
}

void Viewer::PubKeyFramesAsFrusta()
{
    vector<visualization_msgs::Marker> vMsgs;
    std::stringstream* ss;

    for(int i=0;i<4;++i)
    {
        visualization_msgs::Marker KeyFrames;

        KeyFrames.header.frame_id = msCurFrame;
        KeyFrames.header.stamp = ros::Time::now();

        ss = new stringstream;
        *ss << mSysType << "KFs" << i << "Map" << mCurVisBundle.mNativeId;
        KeyFrames.ns = ss->str();

        KeyFrames.id=0;
        KeyFrames.type = visualization_msgs::Marker::LINE_LIST;
        KeyFrames.scale.x=params::vis::mfCamLineSize;
        KeyFrames.pose.orientation.w=1.0;
        KeyFrames.action=visualization_msgs::Marker::ADD;

        if(i == 0)
        {
            KeyFrames.color = params::colors::mc0.mColMsg;
        }
        else if(i == 1)
        {
            KeyFrames.color = params::colors::mc1.mColMsg;
        }
        else if(i == 2)
        {
            KeyFrames.color = params::colors::mc2.mColMsg;
        }
        else if(i == 3)
        {
            KeyFrames.color = params::colors::mc3.mColMsg;
        }
        KeyFrames.color.a = 1.0;

        vMsgs.push_back(KeyFrames);
    }
    
    if(mpCC->mSysState == eSystemState::CLIENT)
    {
        {
            visualization_msgs::Marker KeyFrames;

            KeyFrames.header.frame_id = msCurFrame;
            KeyFrames.header.stamp = ros::Time::now();

            ss = new stringstream;
            *ss << mSysType << "CurKfMap" << mCurVisBundle.mNativeId;
            KeyFrames.ns = ss->str();

            KeyFrames.id=0;
            KeyFrames.type = visualization_msgs::Marker::LINE_LIST;
            KeyFrames.scale.x=MUL*params::vis::mfCamLineSize;
            KeyFrames.pose.orientation.w=1.0;
            KeyFrames.action=visualization_msgs::Marker::ADD;

            KeyFrames.color = Colors::msgGold();

            vMsgs.push_back(KeyFrames);
        }
        {
            visualization_msgs::Marker KeyFrames;

            KeyFrames.header.frame_id = msCurFrame;
            KeyFrames.header.stamp = ros::Time::now();

            ss = new stringstream;
            *ss << mSysType << "KFsFromServMap" << mCurVisBundle.mNativeId;
            KeyFrames.ns = ss->str();

            KeyFrames.id=0;
            KeyFrames.type = visualization_msgs::Marker::LINE_LIST;
            KeyFrames.scale.x=params::vis::mfCamLineSize;
            KeyFrames.pose.orientation.w=1.0;
            KeyFrames.action=visualization_msgs::Marker::ADD;

            KeyFrames.color = Colors::msgRed();

            vMsgs.push_back(KeyFrames);
        }
    }

    delete ss;

    const float fScale = static_cast<float>(params::vis::mfScaleFactor);
    const float d = static_cast<float>(params::vis::mfCamSize);

    for(map<idpair,kfptr>::iterator mit = mCurVisBundle.mmpKFs.begin();mit!=mCurVisBundle.mmpKFs.end();++mit)
    {
        kfptr pKFi = mit->second;

        if((pKFi->IsEmpty()) || (pKFi->isBad())) continue;

        size_t ClientId = pKFi->mId.second;

        //Camera is a pyramid. Define in camera coordinate system
        cv::Mat o = (cv::Mat_<float>(4,1) << 0, 0, 0, 1);
        cv::Mat p1 = (cv::Mat_<float>(4,1) << d, d*0.8, d*0.5, 1);
        cv::Mat p2 = (cv::Mat_<float>(4,1) << d, -d*0.8, d*0.5, 1);
        cv::Mat p3 = (cv::Mat_<float>(4,1) << -d, -d*0.8, d*0.5, 1);
        cv::Mat p4 = (cv::Mat_<float>(4,1) << -d, d*0.8, d*0.5, 1);

        if(mpCC->mSysState == eSystemState::SERVER && mCurVisBundle.msCurKFIds.count(pKFi->mId))
        {
            //double size for current KFs
            //for clients, this is done by PubFramePoseAsFrustum
            o = (cv::Mat_<float>(4,1) << 0, 0, 0, 1);
            p1 = (cv::Mat_<float>(4,1) << MUL*d, MUL*d*0.8, MUL*d*0.5, 1);
            p2 = (cv::Mat_<float>(4,1) << MUL*d, -MUL*d*0.8, MUL*d*0.5, 1);
            p3 = (cv::Mat_<float>(4,1) << -MUL*d, -MUL*d*0.8, MUL*d*0.5, 1);
            p4 = (cv::Mat_<float>(4,1) << -MUL*d, MUL*d*0.8, MUL*d*0.5, 1);
        }

        cv::Mat Twc = pKFi->GetPoseInverse();
        cv::Mat ow = pKFi->GetCameraCenter();
        cv::Mat p1w = Twc*p1;
        cv::Mat p2w = Twc*p2;
        cv::Mat p3w = Twc*p3;
        cv::Mat p4w = Twc*p4;

        geometry_msgs::Point msgs_o,msgs_p1, msgs_p2, msgs_p3, msgs_p4;
        msgs_o.x=(fScale)*ow.at<float>(0);
        msgs_o.y=(fScale)*ow.at<float>(1);
        msgs_o.z=(fScale)*ow.at<float>(2);
        msgs_p1.x=(fScale)*p1w.at<float>(0);
        msgs_p1.y=(fScale)*p1w.at<float>(1);
        msgs_p1.z=(fScale)*p1w.at<float>(2);
        msgs_p2.x=(fScale)*p2w.at<float>(0);
        msgs_p2.y=(fScale)*p2w.at<float>(1);
        msgs_p2.z=(fScale)*p2w.at<float>(2);
        msgs_p3.x=(fScale)*p3w.at<float>(0);
        msgs_p3.y=(fScale)*p3w.at<float>(1);
        msgs_p3.z=(fScale)*p3w.at<float>(2);
        msgs_p4.x=(fScale)*p4w.at<float>(0);
        msgs_p4.y=(fScale)*p4w.at<float>(1);
        msgs_p4.z=(fScale)*p4w.at<float>(2);

        if(mpCC->mSysState == eSystemState::SERVER && mCurVisBundle.msCurKFIds.count(pKFi->mId))
        {
            //hits on server
            visualization_msgs::Marker KeyFrames;

            KeyFrames.header.frame_id = msCurFrame;
            KeyFrames.header.stamp = ros::Time::now();

            ss = new stringstream;
            *ss << mSysType << "CurKf" << pKFi->mId.second << "Map" << mCurVisBundle.mNativeId;
            KeyFrames.ns = ss->str();

            KeyFrames.id=0;
            KeyFrames.type = visualization_msgs::Marker::LINE_LIST;
            KeyFrames.scale.x=MUL*params::vis::mfCamLineSize;
            KeyFrames.pose.orientation.w=1.0;
            KeyFrames.action=visualization_msgs::Marker::ADD;

            if(pKFi->mId.second == 0)
            {
                KeyFrames.color = params::colors::mc0.mColMsg;
            }
            else if(pKFi->mId.second == 1)
            {
                KeyFrames.color = params::colors::mc1.mColMsg;
            }
            else if(pKFi->mId.second == 2)
            {
                KeyFrames.color = params::colors::mc2.mColMsg;
            }
            else if(pKFi->mId.second == 3)
            {
                KeyFrames.color = params::colors::mc3.mColMsg;
            }
            KeyFrames.color.a = 1.0;

            KeyFrames.points.push_back(msgs_o);
            KeyFrames.points.push_back(msgs_p1);
            KeyFrames.points.push_back(msgs_o);
            KeyFrames.points.push_back(msgs_p2);
            KeyFrames.points.push_back(msgs_o);
            KeyFrames.points.push_back(msgs_p3);
            KeyFrames.points.push_back(msgs_o);
            KeyFrames.points.push_back(msgs_p4);
            KeyFrames.points.push_back(msgs_p1);
            KeyFrames.points.push_back(msgs_p2);
            KeyFrames.points.push_back(msgs_p2);
            KeyFrames.points.push_back(msgs_p3);
            KeyFrames.points.push_back(msgs_p3);
            KeyFrames.points.push_back(msgs_p4);
            KeyFrames.points.push_back(msgs_p4);
            KeyFrames.points.push_back(msgs_p1);

            vMsgs.push_back(KeyFrames);
        }
        else if(mpCC->mSysState == eSystemState::CLIENT && (pKFi->mbFromServer || pKFi->mbUpdatedByServer))
        {
            vMsgs[5].points.push_back(msgs_o);
            vMsgs[5].points.push_back(msgs_p1);
            vMsgs[5].points.push_back(msgs_o);
            vMsgs[5].points.push_back(msgs_p2);
            vMsgs[5].points.push_back(msgs_o);
            vMsgs[5].points.push_back(msgs_p3);
            vMsgs[5].points.push_back(msgs_o);
            vMsgs[5].points.push_back(msgs_p4);
            vMsgs[5].points.push_back(msgs_p1);
            vMsgs[5].points.push_back(msgs_p2);
            vMsgs[5].points.push_back(msgs_p2);
            vMsgs[5].points.push_back(msgs_p3);
            vMsgs[5].points.push_back(msgs_p3);
            vMsgs[5].points.push_back(msgs_p4);
            vMsgs[5].points.push_back(msgs_p4);
            vMsgs[5].points.push_back(msgs_p1);
        }
        else
        {
            vMsgs[ClientId].points.push_back(msgs_o);
            vMsgs[ClientId].points.push_back(msgs_p1);
            vMsgs[ClientId].points.push_back(msgs_o);
            vMsgs[ClientId].points.push_back(msgs_p2);
            vMsgs[ClientId].points.push_back(msgs_o);
            vMsgs[ClientId].points.push_back(msgs_p3);
            vMsgs[ClientId].points.push_back(msgs_o);
            vMsgs[ClientId].points.push_back(msgs_p4);
            vMsgs[ClientId].points.push_back(msgs_p1);
            vMsgs[ClientId].points.push_back(msgs_p2);
            vMsgs[ClientId].points.push_back(msgs_p2);
            vMsgs[ClientId].points.push_back(msgs_p3);
            vMsgs[ClientId].points.push_back(msgs_p3);
            vMsgs[ClientId].points.push_back(msgs_p4);
            vMsgs[ClientId].points.push_back(msgs_p4);
            vMsgs[ClientId].points.push_back(msgs_p1);
        }
    }

    for(vector<visualization_msgs::Marker>::const_iterator vit = vMsgs.begin();vit!=vMsgs.end();++vit)
    {
        visualization_msgs::Marker msg = *vit;
        if(!msg.points.empty())
        {
            if(mCurVisBundle.mNativeId == 0)
                mPubMarker0.publish(msg);
            else if(mCurVisBundle.mNativeId == 1)
                mPubMarker1.publish(msg);
            else if(mCurVisBundle.mNativeId == 2)
                mPubMarker2.publish(msg);
            else if(mCurVisBundle.mNativeId == 3)
                mPubMarker3.publish(msg);
            else
                cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << ": mCurVisBundle.mNativeId not in [0,3]" << endl;
        }
    }
}

void Viewer::PubMapPointsAsCloud()
{
    pcl::PointCloud<pcl::PointXYZRGB> Cloud;

    mvNumPoints[mCurVisBundle.mNativeId] = 0;

    for(map<idpair,mpptr>::iterator mit = mCurVisBundle.mmpMPs.begin();mit!=mCurVisBundle.mmpMPs.end();++mit)
    {
        mpptr pMPi = mit->second;

        if((pMPi->IsEmpty()) || (pMPi->isBad())) continue;

        size_t ClientId = pMPi->mId.second;

        if(ClientId < 0 || ClientId > 3)
        {
            cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Map::PubMapPointsAsPclMsg(): pMPi->mId.second not in [0,3] -- mClientId: " << pMPi->mId.second << endl;
            throw infrastructure_ex();
        }

        pcl::PointXYZRGB p;
        cv::Mat Tworld = pMPi->GetWorldPos();
        p.x = (params::vis::mfScaleFactor)*((double)(Tworld.at<float>(0,0)));
        p.y = (params::vis::mfScaleFactor)*((double)(Tworld.at<float>(0,1)));
        p.z = (params::vis::mfScaleFactor)*((double)(Tworld.at<float>(0,2)));

        if(pMPi->mbFromServer && pMPi->mbMultiUse)
        {
            p.r = Colors::rgbRed().r();
            p.g = Colors::rgbRed().g();
            p.b = Colors::rgbRed().b();
        }
        if(mpCC->mSysState == eSystemState::SERVER && pMPi->mbMultiUse)
        {
            p.r = Colors::rgbRed().r();
            p.g = Colors::rgbRed().g();
            p.b = Colors::rgbRed().b();
        }
        else if(ClientId == 0)
        {
            p.r = static_cast<uint8_t>(params::colors::mc0.mu8R);
            p.g = static_cast<uint8_t>(params::colors::mc0.mu8G);
            p.b = static_cast<uint8_t>(params::colors::mc0.mu8B);
        }
        else if(ClientId == 1)
        {
            p.r = static_cast<uint8_t>(params::colors::mc1.mu8R);
            p.g = static_cast<uint8_t>(params::colors::mc1.mu8G);
            p.b = static_cast<uint8_t>(params::colors::mc1.mu8B);
        }
        else if(ClientId == 2)
        {
            p.r = static_cast<uint8_t>(params::colors::mc2.mu8R);
            p.g = static_cast<uint8_t>(params::colors::mc2.mu8G);
            p.b = static_cast<uint8_t>(params::colors::mc2.mu8B);
        }
        else if(ClientId == 3)
        {
            p.r = static_cast<uint8_t>(params::colors::mc3.mu8R);
            p.g = static_cast<uint8_t>(params::colors::mc3.mu8G);
            p.b = static_cast<uint8_t>(params::colors::mc3.mu8B);
        }

        Cloud.points.push_back(p);

        ++mvNumPoints[mCurVisBundle.mNativeId];
    }

    if(!Cloud.points.empty())
    {
        sensor_msgs::PointCloud2 pclMsg;
        pcl::toROSMsg(Cloud,pclMsg);
        pclMsg.header.frame_id = msCurFrame;
        pclMsg.header.stamp = ros::Time::now();

        if(mCurVisBundle.mNativeId == 0)
            mPubPcl0.publish(pclMsg);
        else if(mCurVisBundle.mNativeId == 1)
            mPubPcl1.publish(pclMsg);
        else if(mCurVisBundle.mNativeId == 2)
            mPubPcl2.publish(pclMsg);
        else if(mCurVisBundle.mNativeId == 3)
            mPubPcl3.publish(pclMsg);
        else
            cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << ": mCurVisBundle.mNativeId not in [0,3]" << endl;
    }
}

pcl::PointXYZRGB Viewer::CreateMP(Mat p3D, size_t nClientId)
{
    pcl::PointXYZRGB p;
    p.x = (params::vis::mfScaleFactor)*((double)(p3D.at<float>(0)));
    p.y = (params::vis::mfScaleFactor)*((double)(p3D.at<float>(1)));
    p.z = (params::vis::mfScaleFactor)*((double)(p3D.at<float>(2)));

    if(nClientId == 0)
    {
        p.r = static_cast<uint8_t>(params::colors::mc0.mu8R);
        p.g = static_cast<uint8_t>(params::colors::mc0.mu8G);
        p.b = static_cast<uint8_t>(params::colors::mc0.mu8B);
    }
    else if(nClientId == 1)
    {
        p.r = static_cast<uint8_t>(params::colors::mc1.mu8R);
        p.g = static_cast<uint8_t>(params::colors::mc1.mu8G);
        p.b = static_cast<uint8_t>(params::colors::mc1.mu8B);
    }
    else if(nClientId == 2)
    {
        p.r = static_cast<uint8_t>(params::colors::mc2.mu8R);
        p.g = static_cast<uint8_t>(params::colors::mc2.mu8G);
        p.b = static_cast<uint8_t>(params::colors::mc2.mu8B);
    }
    else if(nClientId == 3)
    {
        p.r = static_cast<uint8_t>(params::colors::mc3.mu8R);
        p.g = static_cast<uint8_t>(params::colors::mc3.mu8G);
        p.b = static_cast<uint8_t>(params::colors::mc3.mu8B);
    }

    return p;
}

void Viewer::PubTrajectories()
{
    vector<vector<kfptr>> vvTraj;
    vector<size_t> vMaxId = vector<size_t>(4,0);

    vvTraj.resize(4,vector<kfptr>(mCurVisBundle.mMaxKfId+1,nullptr));

    for(map<idpair,kfptr>::iterator mit = mCurVisBundle.mmpKFs.begin();mit!=mCurVisBundle.mmpKFs.end();++mit)
    {
        kfptr pKFi = mit->second;

        if((pKFi->IsEmpty()) || (pKFi->isBad())) continue;

        size_t ClientId = pKFi->mId.second;

        if(pKFi->mId.first > mCurVisBundle.mMaxKfId)
        {
            cout << "\033[1;33m!!! WARN !!!\033[0m" << __func__ << __LINE__ << ": pKFi->mId.first > miMaxKFid" << endl;
            continue;
        }

        if(mpCC->mSysState == eSystemState::CLIENT)
        {
            if(ClientId != mCurVisBundle.mNativeId)
                continue;
        }

        vvTraj[ClientId][pKFi->mId.first] = pKFi;
        if(vMaxId[ClientId] < pKFi->mId.first)
            vMaxId[ClientId] = pKFi->mId.first;
    }

    vector<visualization_msgs::Marker> vMsgs;
    std::stringstream* ss;

    for(int i=0;i<4;++i)
    {
        visualization_msgs::Marker Traj;

        Traj.header.frame_id = msCurFrame;
        Traj.header.stamp = ros::Time::now();
        ss = new stringstream;
        *ss << mSysType << "Traj" << i;
        Traj.ns = ss->str();
        Traj.id=0;
        Traj.type = visualization_msgs::Marker::LINE_STRIP;
        Traj.scale.x=params::vis::mfTrajMarkerSize;
        Traj.action=visualization_msgs::Marker::ADD;

        if(i == 0)
        {
            Traj.color = params::colors::mc0.mColMsg;
        }
        else if(i == 1)
        {
            Traj.color = params::colors::mc1.mColMsg;
        }
        else if(i == 2)
        {
            Traj.color = params::colors::mc2.mColMsg;
        }
        else if(i == 3)
        {
            Traj.color = params::colors::mc3.mColMsg;
        }
        Traj.color.a = 1.0;

        vMsgs.push_back(Traj);
    }

    delete ss;

    float fScale = static_cast<float>(params::vis::mfScaleFactor);

    for(int ito = 0;ito<4;++ito)
    {
        vector<kfptr> vTraj = vvTraj[ito];

        if(vTraj.empty())
            continue;

        geometry_msgs::Point p0;
        bool bP0 = false;

        for(int iti = 0;iti<=vMaxId[ito];++iti)
        {
            kfptr pKFi = vTraj[iti];

            if(!pKFi)
                continue;

            if(pKFi->mbFromServer)
                continue;

            cv::Mat T = pKFi->GetPoseInverse();

            geometry_msgs::Point p;

            p.x = fScale*(T.at<float>(0,3));
            p.y = fScale*(T.at<float>(1,3));
            p.z = fScale*(T.at<float>(2,3));

            if(pKFi->mId.first == 0)
            {
                p0 = p;
                bP0 = true;
            }
            else if(pKFi->mId.first == 1 && bP0)
            {
                vMsgs[ito].points.push_back(p0);
                vMsgs[ito].points.push_back(p);
            }
            else
                vMsgs[ito].points.push_back(p);
        }
    }

    for(vector<visualization_msgs::Marker>::const_iterator vit = vMsgs.begin();vit!=vMsgs.end();++vit)
    {
        visualization_msgs::Marker msg = *vit;
        if(!msg.points.empty())
        {
            if(mCurVisBundle.mNativeId == 0)
                mPubMarker0.publish(msg);
            else if(mCurVisBundle.mNativeId == 1)
                mPubMarker1.publish(msg);
            else if(mCurVisBundle.mNativeId == 2)
                mPubMarker2.publish(msg);
            else if(mCurVisBundle.mNativeId == 3)
                mPubMarker3.publish(msg);
            else
                cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << ": mCurVisBundle.mNativeId not in [0,3]" << endl;
        }
    }
}

void Viewer::PubCovGraph()
{
    if(mCurVisBundle.mmpKFs.size() < 5) return;

    visualization_msgs::Marker CovGraphMsg;
    visualization_msgs::Marker SharedEdgeMsg;
    std::stringstream* ss;

    CovGraphMsg.header.frame_id = msCurFrame;
    CovGraphMsg.header.stamp = ros::Time::now();
    ss = new stringstream;
    *ss << mSysType << "CovGraph" << mCurVisBundle.mNativeId;
    CovGraphMsg.ns = ss->str();
    CovGraphMsg.type = visualization_msgs::Marker::LINE_LIST;

    CovGraphMsg.color = params::colors::mcCovGraph.mColMsg;

    CovGraphMsg.action = visualization_msgs::Marker::ADD;
    CovGraphMsg.scale.x = params::vis::mfCovGraphMarkerSize;
    CovGraphMsg.id = 0;

    SharedEdgeMsg = CovGraphMsg;

    ss = new stringstream;
    *ss << mSysType << "CovGraphSharedEdges" << mCurVisBundle.mNativeId;
    SharedEdgeMsg.ns = ss->str();
    SharedEdgeMsg.color = Colors::msgRed();

    delete ss;

    size_t MaxVal;

    if(mpCC->mSysState == eSystemState::CLIENT)
        MaxVal = 2*max(mCurVisBundle.mMaxKfId,mCurVisBundle.mmpKFs.size())+1; //num of KFs is not necessarily equal to highest ID
    else
        MaxVal = 2*mCurVisBundle.mMaxKfIdUnique + 1; //sometimes for client, KF is not already entered in map --> KF->mUniqueId > mnMaxKFidUnique

    vector<vector<bool>> CovMat(MaxVal,vector<bool>(MaxVal,false));

    size_t ConId=KFRANGE,KfId=KFRANGE;
    size_t count = 0;

    float fScale = static_cast<float>(params::vis::mfScaleFactor);

    for(map<idpair,kfptr>::iterator mit = mCurVisBundle.mmpKFs.begin();mit!=mCurVisBundle.mmpKFs.end();++mit)
    {
        kfptr pKFi = mit->second;

        if((pKFi->IsEmpty()) || (pKFi->isBad())) continue;

        vector<kfptr> vConKFs = pKFi->GetCovisiblesByWeight(params::vis::miCovGraphMinFeats);
        set<kfptr> sConKFs;
        sConKFs.insert(vConKFs.begin(),vConKFs.end());

        for(set<kfptr>::iterator sit=sConKFs.begin();sit!=sConKFs.end();++sit)
        {
            kfptr pKFcon = *sit;
            if(pKFcon->isBad()) continue;

            if(mpCC->mSysState == eSystemState::CLIENT)
            {
                if(!mpMap->GetKfPtr(pKFcon->mId.first,pKFcon->mId.second) || !mpMap->GetKfPtr(pKFi->mId.first,pKFi->mId.second))
                    continue;
            }

            if(mpCC->mSysState == eSystemState::CLIENT)
            {
                if(pKFi->mVisId == -1) pKFi->mVisId = count++;
                if(pKFcon->mVisId == -1) pKFcon->mVisId = count++;

                KfId = pKFi->mVisId;
                ConId = pKFcon->mVisId;
            }
            else if(mpCC->mSysState == eSystemState::SERVER)
            {
                KfId = pKFi->mUniqueId;
                ConId = pKFcon->mUniqueId;
            }

            if(max(KfId,ConId) > (MaxVal-1)) //starts with 0 -> CovMat[MaxVal][MaxVal] does not exist. Yet, this should not happen...
            {
                cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  Map::GetCovGraphAsMarkerMsg(...): KF ID out of bounds" << endl;
                cout << "MaxVal: " << MaxVal << endl;
                cout << "pKFi->Id: " << pKFi->mId.first << "|" << pKFi->mId.second << "|" << pKFi->mUniqueId << endl;
                cout << "pKFcon->Id: " << pKFcon->mId.first << "|" << pKFcon->mId.second << "|" << pKFcon->mUniqueId << endl;
                continue;
            }

            if(CovMat[KfId][ConId]==true || CovMat[ConId][KfId]==true) continue;

            cv::Mat T1 = pKFi->GetPoseInverse();
            cv::Mat T2 = pKFcon->GetPoseInverse();

            geometry_msgs::Point p1;
            geometry_msgs::Point p2;

            p1.x = fScale*((double)(T1.at<float>(0,3)));
            p1.y = fScale*((double)(T1.at<float>(1,3)));
            p1.z = fScale*((double)(T1.at<float>(2,3)));

            p2.x = fScale*((double)(T2.at<float>(0,3)));
            p2.y = fScale*((double)(T2.at<float>(1,3)));
            p2.z = fScale*((double)(T2.at<float>(2,3)));

            if(pKFi->mId.second == pKFcon->mId.second)
            {
                CovGraphMsg.points.push_back(p1);
                CovGraphMsg.points.push_back(p2);
            }
            else
            {
                SharedEdgeMsg.points.push_back(p1);
                SharedEdgeMsg.points.push_back(p2);
            }

            CovMat[KfId][ConId]=true;
            CovMat[ConId][KfId]=true;
        }
    }

    if(mpCC->mSysState == eSystemState::CLIENT)
    {
        //reset pKF->mVisId
        for(map<idpair,kfptr>::iterator mit = mCurVisBundle.mmpKFs.begin();mit!=mCurVisBundle.mmpKFs.end();++mit)
        {
            kfptr pKFi = mit->second;

            if((pKFi->IsEmpty()) || (pKFi->isBad())) continue;

            if(pKFi->mVisId != -1) pKFi->mVisId = -1;

            set<kfptr> sConKFs = pKFi->GetConnectedKeyFrames();

            for(set<kfptr>::iterator sit=sConKFs.begin();sit!=sConKFs.end();++sit)
            {
                kfptr pKFcon = *sit;
                if(pKFcon->isBad()) continue;

                if(pKFcon->mVisId != -1) pKFcon->mVisId = -1;
            }
        }
    }

    if(!CovGraphMsg.points.empty())
    {
        if(mCurVisBundle.mNativeId == 0)
        {
            mPubMarker0.publish(CovGraphMsg);
            mPubMarker0.publish(SharedEdgeMsg);
        }
        else if(mCurVisBundle.mNativeId == 1)
        {
            mPubMarker1.publish(CovGraphMsg);
            mPubMarker1.publish(SharedEdgeMsg);
        }
        else if(mCurVisBundle.mNativeId == 2)
        {
            mPubMarker2.publish(CovGraphMsg);
            mPubMarker2.publish(SharedEdgeMsg);
        }
        else if(mCurVisBundle.mNativeId == 3)
        {
            mPubMarker3.publish(CovGraphMsg);
            mPubMarker3.publish(SharedEdgeMsg);
        }
        else
            cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << ": mCurVisBundle.mNativeId not in [0,3]" << endl;
    }
}

void Viewer::ClearCovGraph(size_t MapId)
{
    visualization_msgs::Marker MarkerMsg;

    MarkerMsg.header.frame_id = "world";
    MarkerMsg.header.stamp = ros::Time::now();
    MarkerMsg.id = 0;
    MarkerMsg.action = 3;

    visualization_msgs::MarkerArray MArray;
    MArray.markers.push_back(MarkerMsg);

    stringstream* ss;

    for(int it=0;it<4;++it)
    {
        ss = new stringstream;
        *ss << mSysType << "CovGraph" << MapId;
        MarkerMsg.ns = ss->str();

        if(MapId == 0)
        {
            mPubMarker0.publish(MarkerMsg);
        }
        else if(MapId == 1)
        {
            mPubMarker1.publish(MarkerMsg);
        }
        else if(MapId == 2)
        {
            mPubMarker2.publish(MarkerMsg);
        }
        else if(MapId == 3)
        {
            mPubMarker3.publish(MarkerMsg);
        }
        else
            cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << ": mCurVisBundle.mNativeId not in [0,3]" << endl;
    }

    //clear points
    this->ClearMapPoints(mvNumPoints[MapId],MapId);

    msBlockedMaps.insert(MapId);

    delete ss;
}

void Viewer::ClearMarkers(size_t MapId)
{
    visualization_msgs::Marker MarkerMsg;

    MarkerMsg.header.frame_id = "world";
    MarkerMsg.header.stamp = ros::Time::now();
    MarkerMsg.id = 0;
    MarkerMsg.action = 3;

    visualization_msgs::MarkerArray MArray;
    MArray.markers.push_back(MarkerMsg);

    stringstream* ss;

    {
        visualization_msgs::Marker mark1,mark2,mark3,markuav;

        MarkerMsg.action = 2;

        ss = new stringstream;
        *ss << mSysType << "KFsFromServMap" << mCurVisBundle.mNativeId;
        MarkerMsg.ns = ss->str();
        mark2 = MarkerMsg;

        if(mpCC->mSysState == eSystemState::SERVER)
        {
            ss = new stringstream;
            *ss << mSysType << "CurKfMap" << mCurVisBundle.mNativeId;
            MarkerMsg.ns = ss->str();
            mark3 = MarkerMsg;
        }

        ss = new stringstream;
        *ss << mSysType << "frameUAV" << mCurVisBundle.mNativeId;
        MarkerMsg.ns = ss->str();
        markuav = MarkerMsg;


        if(MapId == 0)
        {
            mPubMarker0.publish(mark2);
            mPubMarker0.publish(mark3);
            mPubMarker0.publish(markuav);
        }
        else if(MapId == 1)
        {
            mPubMarker1.publish(mark2);
            mPubMarker1.publish(mark3);
            mPubMarker1.publish(markuav);
        }
        else if(MapId == 2)
        {
            mPubMarker2.publish(mark2);
            mPubMarker2.publish(mark3);
            mPubMarker2.publish(markuav);
        }
        else if(MapId == 3)
        {
            mPubMarker3.publish(mark2);
            mPubMarker3.publish(mark3);
            mPubMarker3.publish(markuav);
        }

        for(int it=0;it<4;++it)
        {
            ss = new stringstream;
            *ss << mSysType << "KFs" << it << "Map" << MapId;
            MarkerMsg.ns = ss->str();
            mark1 = MarkerMsg;

            if(MapId == 0)
            {
                mPubMarker0.publish(mark1);
            }
            else if(MapId == 1)
            {
                mPubMarker1.publish(mark1);
            }
            else if(MapId == 2)
            {
                mPubMarker2.publish(mark1);
            }
            else if(MapId == 3)
            {
                mPubMarker3.publish(mark1);
            }
        }
    }

    delete ss;
}

void Viewer::ClearMapPoints(size_t n, size_t ClientId)
{
    pcl::PointCloud<pcl::PointXYZ> pclCloud;

    for(size_t it = 0;it < n;++it)
    {
        pcl::PointXYZ p;
        p.x = 0.0;
        p.y = 0.0;
        p.z = 0.0;
        pclCloud.points.push_back(p);
    }

    sensor_msgs::PointCloud2 pclMsg;
    pcl::toROSMsg(pclCloud,pclMsg);
    pclMsg.header.frame_id = "world";
    pclMsg.header.stamp = ros::Time::now();

    if(ClientId == 0)
        mPubPcl0.publish(pclMsg);
    else if(ClientId == 1)
        mPubPcl1.publish(pclMsg);
    else if(ClientId == 2)
        mPubPcl2.publish(pclMsg);
    else if(ClientId == 3)
        mPubPcl3.publish(pclMsg);
    else
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << ": mCurVisBundle.mNativeId not in [0,3]" << endl;
}

void Viewer::RequestReset()
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
                break;
        }
        if(mpCC->mSysState == eSystemState::CLIENT)
            usleep(params::timings::client::miViewerRate);
        else if(mpCC->mSysState == eSystemState::SERVER)
            usleep(params::timings::server::miViewerRate);
        else KILLSYS
    }
}

void Viewer::ResetIfRequested()
{
    unique_lock<mutex> lockReset(mMutexReset);
    if(mbResetRequested)
    {
        unique_lock<mutex> lockMap(mMutexDrawMap,defer_lock);
        unique_lock<mutex> lockFrame(mMutexFrameDraw,defer_lock);

        lock(lockMap,lockFrame);

        mbDrawFrame = false;
        mmVisData.clear();

        mbResetRequested=false;
    }
}

void Viewer::PubFramePoseAsFrustum()
{
    if(mpCC->mSysState != eSystemState::CLIENT)
    {
        cout << COUTFATAL << " System Type Mismatch" << endl;
        throw infrastructure_ex();
    }

    if(!(mpTracker->mState == Tracking::eTrackingState::OK))
        return;

    float fScale = static_cast<float>(params::vis::mfScaleFactor);
    float d = static_cast<float>(params::vis::mfCamSize);

    visualization_msgs::Marker Frame;

    Frame.header.frame_id = msCurFrame;
    Frame.header.stamp = ros::Time::now();

    stringstream* ss;
    ss = new stringstream;
    *ss << mSysType << "CurKfMap" << mpCC->mClientId;
    Frame.ns = ss->str();

    Frame.id=0;
    Frame.type = visualization_msgs::Marker::LINE_LIST;
    Frame.scale.x=MUL*params::vis::mfCamLineSize;
    Frame.pose.orientation.w=1.0;
    Frame.action=visualization_msgs::Marker::ADD;

    Frame.color = Colors::msgGold();

    cv::Mat o = (cv::Mat_<float>(4,1) << 0, 0, 0, 1);
    cv::Mat p1 = (cv::Mat_<float>(4,1) << MUL*d, MUL*d*0.8, MUL*d*0.5, 1);
    cv::Mat p2 = (cv::Mat_<float>(4,1) << MUL*d, -MUL*d*0.8, MUL*d*0.5, 1);
    cv::Mat p3 = (cv::Mat_<float>(4,1) << -MUL*d, -MUL*d*0.8, MUL*d*0.5, 1);
    cv::Mat p4 = (cv::Mat_<float>(4,1) << -MUL*d, MUL*d*0.8, MUL*d*0.5, 1);

    cv::Mat Tcw = mpTracker->mCurrentFrame->mTcw;
    cv::Mat Twc = Tcw.inv();
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    cv::Mat ow = -Rcw.t()*tcw;
    cv::Mat p1w = Twc*p1;
    cv::Mat p2w = Twc*p2;
    cv::Mat p3w = Twc*p3;
    cv::Mat p4w = Twc*p4;

    geometry_msgs::Point msgs_o,msgs_p1, msgs_p2, msgs_p3, msgs_p4;
    msgs_o.x=(fScale)*ow.at<float>(0);
    msgs_o.y=(fScale)*ow.at<float>(1);
    msgs_o.z=(fScale)*ow.at<float>(2);
    msgs_p1.x=(fScale)*p1w.at<float>(0);
    msgs_p1.y=(fScale)*p1w.at<float>(1);
    msgs_p1.z=(fScale)*p1w.at<float>(2);
    msgs_p2.x=(fScale)*p2w.at<float>(0);
    msgs_p2.y=(fScale)*p2w.at<float>(1);
    msgs_p2.z=(fScale)*p2w.at<float>(2);
    msgs_p3.x=(fScale)*p3w.at<float>(0);
    msgs_p3.y=(fScale)*p3w.at<float>(1);
    msgs_p3.z=(fScale)*p3w.at<float>(2);
    msgs_p4.x=(fScale)*p4w.at<float>(0);
    msgs_p4.y=(fScale)*p4w.at<float>(1);
    msgs_p4.z=(fScale)*p4w.at<float>(2);

    Frame.points.push_back(msgs_o);
    Frame.points.push_back(msgs_p1);
    Frame.points.push_back(msgs_o);
    Frame.points.push_back(msgs_p2);
    Frame.points.push_back(msgs_o);
    Frame.points.push_back(msgs_p3);
    Frame.points.push_back(msgs_o);
    Frame.points.push_back(msgs_p4);
    Frame.points.push_back(msgs_p1);
    Frame.points.push_back(msgs_p2);
    Frame.points.push_back(msgs_p2);
    Frame.points.push_back(msgs_p3);
    Frame.points.push_back(msgs_p3);
    Frame.points.push_back(msgs_p4);
    Frame.points.push_back(msgs_p4);
    Frame.points.push_back(msgs_p1);

    if(mpCC->mClientId == 0)
        mPubMarker0.publish(Frame);
    else if(mpCC->mClientId == 1)
        mPubMarker1.publish(Frame);
    else if(mpCC->mClientId == 2)
        mPubMarker2.publish(Frame);
    else if(mpCC->mClientId == 3)
        mPubMarker3.publish(Frame);
    else
        cout << COUTERROR << ": mCurVisBundle.mNativeId not in [0,3]" << endl;

    delete ss;
}

} //end ns
