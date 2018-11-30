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

#ifndef CSLAM_ESTD_H_
#define CSLAM_ESTD_H_

#include <vector>
#include <thread>
#include <mutex>
#include <boost/shared_ptr.hpp>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <numeric>

//ROS
#include <std_msgs/ColorRGBA.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//CSLAM
#include <cslam/config.h>

#define MAPRANGE std::numeric_limits<uint8_t>::max()
#define KFRANGE std::numeric_limits<uint16_t>::max()
#define MPRANGE std::numeric_limits<uint32_t>::max()
#define UIDRANGE std::numeric_limits<uint32_t>::max()

#define defpair make_pair(KFRANGE,MAPRANGE) //default pair
#define defid -1 //default id

#define LLL cout << __LINE__ << " --- " << ros::Time::now() << endl;
#define FFF cout << __func__ << endl;
#define LLLS if(mSysState == eSystemState::SERVER) cout << __LINE__ << " --- " << ros::Time::now() << endl;
#define FFLL cout << __func__ << ":" << __LINE__ << endl;

#define COUTERROR "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << ":" << __LINE__ << ": "
#define COUTFATAL "\033[1;31m!!!!! FATAL ERROR !!!!!\033[0m " << __func__ << ":"  << __LINE__ << ": "
#define COUTWARN "\033[1;33m!!! WARN !!!\033[0m " << __func__ << ":" << __LINE__ << ": "
#define COUTNOTICE "\033[1;34m!!! NOTICE !!!\033[0m " << __func__ << ":" << __LINE__ << ": "
#define KILLSYS {cout << COUTFATAL << endl; throw infrastructure_ex();}

using namespace std;
namespace estd{

typedef pair<size_t,size_t> idpair;
typedef boost::shared_ptr<std::thread> threadptr;
class UniqueIdDispenser;
typedef boost::shared_ptr<UniqueIdDispenser> uidptr;

class infrastructure_ex : public std::exception
{
public:
    virtual const char* what() const throw()
      {
        return "EXCEPTION: Bad Infrastructure";
      }
};

class UniqueIdDispenser
{
public:
    UniqueIdDispenser() : mLastId(0) {}

    size_t GetId()
    {
        unique_lock<mutex> lock(mMutexId);
        ++mLastId;

        return mLastId;
    }

    size_t GetLastId()
    {
        unique_lock<mutex> lock(mMutexId);
        return mLastId;
    }

    void SetLastId(size_t id)
    {
        mLastId = id;
    }

private:
    size_t mLastId;
    mutex mMutexId;
};

class Colors
{
public:
    struct RGB255
    {
        RGB255(int r,int g,int b)
            : miR(r),miG(g),miB(b)
            {}
    public:
        RGB255 rgb(){return RGB255(miR,miG,miB);}
        int r(){return miR;}
        int g(){return miG;}
        int b(){return miB;}
        uint32_t rgbpcl(){return ((uint32_t)static_cast<uint8_t>(miR) << 16 | (uint32_t)static_cast<uint8_t>(miG) << 8 | (uint32_t)static_cast<uint8_t>(miB));}
    private:
        const int miR;
        const int miG;
        const int miB;
    };

public:

    //Black
    static RGB255 rgbBlack(){RGB255 black(0,0,0); return black.rgb();}
    static uint32_t pclBlack(){RGB255 black(0,0,0); return black.rgbpcl();}
    static std_msgs::ColorRGBA msgBlack(){std_msgs::ColorRGBA black; black.r = 0.0; black.g = 0.0; black.b = 0.0; black.a = 1.0; return black;}

    //White
    static RGB255 rgbWhite(){RGB255 white(255,255,255); return white.rgb();}
    static uint32_t pclWhite(){RGB255 white(255,255,255); return white.rgbpcl();}
    static std_msgs::ColorRGBA msgWhite(){std_msgs::ColorRGBA white; white.r = 1.0; white.g = 1.0; white.b = 1.0; white.a = 1.0; return white;}

    //Red
    static RGB255 rgbRed(){RGB255 red(255,0,0); return red.rgb();}
    static uint32_t pclRed(){RGB255 red(255,0,0); return red.rgbpcl();}
    static std_msgs::ColorRGBA msgRed(){std_msgs::ColorRGBA red; red.r = 1.0; red.g = 0.0; red.b = 0.0; red.a = 1.0; return red;}

    //Green
    static RGB255 rgbGreen(){RGB255 green(0,255,0); return green.rgb();}
    static uint32_t pclGreen(){RGB255 green(0,255,0); return green.rgbpcl();}
    static std_msgs::ColorRGBA msgGreen(){std_msgs::ColorRGBA green; green.r = 0.0; green.g = 1.0; green.b = 0.0; green.a = 1.0; return green;}

    //DarkGreen
    static RGB255 rgbDarkGreen(){RGB255 darkgreen(0,255,0); return darkgreen.rgb();}
    static uint32_t pclDarkGreen(){RGB255 darkgreen(0,255,0); return darkgreen.rgbpcl();}
    static std_msgs::ColorRGBA msgDarkGreen(){std_msgs::ColorRGBA darkgreen; darkgreen.r = 0.0; darkgreen.g = 0.8; darkgreen.b = 0.0; darkgreen.a = 1.0; return darkgreen;}

    //Blue
    static RGB255 rgbBlue(){RGB255 blue(0,0,255); return blue.rgb();}
    static uint32_t pclBlue(){RGB255 blue(0,0,255); return blue.rgbpcl();}
    static std_msgs::ColorRGBA msgBlue(){std_msgs::ColorRGBA blue; blue.r = 0.0; blue.g = 0.0; blue.b = 1.0; blue.a = 1.0; return blue;}

    //Magenta
    static RGB255 rgbMagenta(){RGB255 magenta(255,0,255); return magenta.rgb();}
    static uint32_t pclMagenta(){RGB255 magenta(255,0,255); return magenta.rgbpcl();}
    static std_msgs::ColorRGBA msgMagenta(){std_msgs::ColorRGBA magenta; magenta.r = 1.0; magenta.g = 0.0; magenta.b = 1.0; magenta.a = 1.0; return magenta;}

    //DarkMagenta
    static RGB255 rgbDarkMagenta(){RGB255 markmagenta(255,0,255); return markmagenta.rgb();}
    static uint32_t pclDarkMagenta(){RGB255 markmagenta(255,0,255); return markmagenta.rgbpcl();}
    static std_msgs::ColorRGBA msgDarkMagenta(){std_msgs::ColorRGBA markmagenta; markmagenta.r = 1.0; markmagenta.g = 0.0; markmagenta.b = 1.0; markmagenta.a = 1.0; return markmagenta;}

    //Cyan
    static RGB255 rgbCyan(){RGB255 cyan(0,255,255); return cyan.rgb();}
    static uint32_t pclCyan(){RGB255 cyan(0,255,255); return cyan.rgbpcl();}
    static std_msgs::ColorRGBA msgCyan(){std_msgs::ColorRGBA cyan; cyan.r = 0.0; cyan.g = 1.0; cyan.b = 1.0; cyan.a = 1.0; return cyan;}

    //Yellow
    static RGB255 rgbYellow(){RGB255 yellow(255,255,0); return yellow.rgb();}
    static uint32_t pclYellow(){RGB255 yellow(255,255,0); return yellow.rgbpcl();}
    static std_msgs::ColorRGBA msgYellow(){std_msgs::ColorRGBA yellow; yellow.r = 1.0; yellow.g = 1.0; yellow.b = 0.0; yellow.a = 1.0; return yellow;}

    //Orange
    static RGB255 rgbOrange(){RGB255 orange(255,128,0); return orange.rgb();}
    static uint32_t pclOrange(){RGB255 orange(255,128,0); return orange.rgbpcl();}
    static std_msgs::ColorRGBA msgOrange(){std_msgs::ColorRGBA orange; orange.r = 1.0; orange.g = 0.5; orange.b = 0.0; orange.a = 1.0; return orange;}

    //Gold
    static RGB255 rgbGold(){RGB255 gold(212,175,55); return gold.rgb();}
    static uint32_t pclGold(){RGB255 gold(212,175,55); return gold.rgbpcl();}
    static std_msgs::ColorRGBA msgGold(){std_msgs::ColorRGBA gold; gold.r = 0.831; gold.g = 0.686; gold.b = 0.216; gold.a = 1.0; return gold;}

    //Dark Gray
    static RGB255 rgbDarkGray(){RGB255 darkgray(105,105,105); return darkgray.rgb();}
    static uint32_t pclDarkGray(){RGB255 darkgray(105,105,105); return darkgray.rgbpcl();}
    static std_msgs::ColorRGBA msgDarkGray(){std_msgs::ColorRGBA darkgray; darkgray.r = 0.412; darkgray.g = 0.412; darkgray.b = 0.412; darkgray.a = 1.0; return darkgray;}

    //Light Gray
    static RGB255 rgbLightGray(){RGB255 lightgray(105,105,105); return lightgray.rgb();}
    static uint32_t pclLightGray(){RGB255 lightgray(105,105,105); return lightgray.rgbpcl();}
    static std_msgs::ColorRGBA msgLightGray(){std_msgs::ColorRGBA lightgray; lightgray.r = 0.6; lightgray.g = 0.6; lightgray.b = 0.6; lightgray.a = 1.0; return lightgray;}
};

#ifdef LOGGING
class mylog
{
public:
    mylog(double dTimeOutInSec, int iSleep)
        : mdTimeOutInSec(dTimeOutInSec),miSleep(iSleep)
    {
        mvMappingState.resize(4,0);
        mvCommState.resize(4,0);
        mvKFState.resize(4,0);
        mvMPState.resize(4,0);
        mvLoopState.resize(4,0);
        mvMatchState.resize(4,0);
        mvMergeState.resize(4,0);
        mvOptState.resize(4,0);

        mvMappingLock.resize(4,0);

        mdStartTime = ros::Time::now().toSec();

        mvdLastTimeMapping.resize(4,mdStartTime);
        mvdLastTimeComm.resize(4,mdStartTime);
        mvdLastTimeLoop.resize(4,mdStartTime);

//        mdLastTimeMapping = mdStartTime;
//        mdLastTimeComm = mdStartTime;
//        mdLastTimeLoop = mdStartTime;
        mdLastTimeMatch = mdStartTime;

        mvbFinished.resize(4,false);
    }

    void run(){
        while(1)
        {
            bool bOkMapping = false;
            bool bOkComm = false;
            bool bOkLoop = false;
            bool bOkMatch = false;

            double dNow = ros::Time::now().toSec();

            {
                std::unique_lock<std::mutex> lock(mMutexMapping);

                bool b0,b1,b2,b3;

                if(mvbFinished[0])
                    b0 = true;
                else if(mvdLastTimeMapping[0] != mdStartTime)
                    b0 = (dNow - mvdLastTimeMapping[0]) < mdTimeOutInSec;
                else
                    b0 = true;

                if(mvbFinished[1])
                    b1 = true;
                else if(mvdLastTimeMapping[1] != mdStartTime)
                    b1 = (dNow - mvdLastTimeMapping[1]) < mdTimeOutInSec;
                else
                    b1 = true;

                if(mvbFinished[2])
                    b2 = true;
                else if(mvdLastTimeMapping[2] != mdStartTime)
                    b2 = (dNow - mvdLastTimeMapping[2]) < mdTimeOutInSec;
                else
                    b2 = true;

                if(mvbFinished[3])
                    b3 = true;
                else if(mvdLastTimeMapping[3] != mdStartTime)
                    b3 = (dNow - mvdLastTimeMapping[3]) < mdTimeOutInSec;
                else
                    b3 = true;

                bOkMapping = b0 && b1 && b2 && b3;
            }

            {
                std::unique_lock<std::mutex> lock(mMutexComm);

                bool b0,b1,b2,b3;

                if(mvbFinished[0])
                    b0 = true;
                else if(mvdLastTimeComm[0] != mdStartTime)
                    b0 = (dNow - mvdLastTimeComm[0]) < mdTimeOutInSec;
                else
                    b0 = true;

                if(mvbFinished[1])
                    b1 = true;
                else if(mvdLastTimeComm[1] != mdStartTime)
                    b1 = (dNow - mvdLastTimeComm[1]) < mdTimeOutInSec;
                else
                    b1 = true;

                if(mvbFinished[2])
                    b2 = true;
                else if(mvdLastTimeComm[2] != mdStartTime)
                    b2 = (dNow - mvdLastTimeComm[2]) < mdTimeOutInSec;
                else
                    b2 = true;

                if(mvbFinished[3])
                    b3 = true;
                else if(mvdLastTimeComm[3] != mdStartTime)
                    b3 = (dNow - mvdLastTimeComm[3]) < mdTimeOutInSec;
                else
                    b3 = true;

                bOkComm = b0 && b1 && b2 && b3;
            }

            {
                std::unique_lock<std::mutex> lock(mMutexLoop);

                bool b0,b1,b2,b3;

                if(mvbFinished[0])
                    b0 = true;
                else if(mvdLastTimeLoop[0] != mdStartTime)
                    b0 = (dNow - mvdLastTimeLoop[0]) < mdTimeOutInSec;
                else
                    b0 = true;

                if(mvbFinished[1])
                    b1 = true;
                else if(mvdLastTimeLoop[1] != mdStartTime)
                    b1 = (dNow - mvdLastTimeLoop[1]) < mdTimeOutInSec;
                else
                    b1 = true;

                if(mvbFinished[2])
                    b2 = true;
                else if(mvdLastTimeLoop[2] != mdStartTime)
                    b2 = (dNow - mvdLastTimeLoop[2]) < mdTimeOutInSec;
                else
                    b2 = true;

                if(mvbFinished[3])
                    b3 = true;
                else if(mvdLastTimeLoop[3] != mdStartTime)
                    b3 = (dNow - mvdLastTimeLoop[3]) < mdTimeOutInSec;
                else
                    b3 = true;

                bOkLoop = b0 && b1 && b2 && b3;
            }

            {
                std::unique_lock<std::mutex> lock(mMutexMatch);
                bOkMatch = (ros::Time::now().toSec() - mdLastTimeMatch) < mdTimeOutInSec;
            }

            bool bOk = bOkMapping && bOkComm && bOkLoop && bOkMatch;

            if(!bOk)
            {
                //reading not covered by mutex, but we take the risk to avoid risking that the logger causes a deadlock

                std::cout << "\033[1;33m!!! ----- Potential Deadlock ----- !!!\033[0m " << std::endl;
//                std::cout << "Current Time: " <<dNow << std::endl;
                std::cout << "Mapping 0|1|2|3: " << mvMappingState[0] << "|" << mvMappingState[1] << "|" << mvMappingState[2] << "|" << mvMappingState[3] << std::endl;
                std::cout << "Comm 0|1|2|3: " << mvCommState[0] << "|" << mvCommState[1] << "|" << mvCommState[2] << "|" << mvCommState[3] << std::endl;
                std::cout << "KF 0|1|2|3: " << mvKFState[0] << "|" << mvKFState[1] << "|" << mvKFState[2] << "|" << mvKFState[3] << std::endl;
                std::cout << "MP 0|1|2|3: " << mvMPState[0] << "|" << mvMPState[1] << "|" << mvMPState[2] << "|" << mvMPState[3] << std::endl;
                std::cout << "Loop 0|1|2|3: " << mvLoopState[0] << "|" << mvLoopState[1] << "|" << mvLoopState[2] << "|" << mvLoopState[3] << std::endl;
//                std::cout << "Match 0|1|2|3: " << mvMatchState[0] << "|" << mvMatchState[1] << "|" << mvMatchState[2] << "|" << mvMatchState[3] << std::endl;
//                std::cout << "Merge 0|1|2|3: " << mvMergeState[0] << "|" << mvMergeState[1] << "|" << mvMergeState[2] << "|" << mvMergeState[3] << std::endl;
                std::cout << "Match 0|1|2|3: " << mvMatchState[0] << std::endl;
                std::cout << "Merge 0|1|2|3: " << mvMergeState[0] << std::endl;
//                std::cout << "Opt 0|1|2|3: " << mvOptState[0] << "|" << mvOptState[1] << "|" << mvOptState[2] << "|" << mvOptState[3] << std::endl;
                std::cout << "Mapping Mutex 0|1|2|3: " << mvMappingLock[0] << "|" << mvMappingLock[1] << "|" << mvMappingLock[2] << "|" << mvMappingLock[3] << std::endl;
                std::cout << "\033[1;33m!!! ------------------------------ !!!\033[0m " << std::endl;
            }

            usleep(miSleep);
        }
    }

    void SetMapping(int iLine, int iClientId){std::unique_lock<std::mutex> lock(mMutexMapping); mvMappingState[iClientId]=iLine; mvdLastTimeMapping[iClientId] = ros::Time::now().toSec();}
    void SetComm(int iLine, int iClientId){std::unique_lock<std::mutex> lock(mMutexComm); mvCommState[iClientId]=iLine; mvdLastTimeComm[iClientId] = ros::Time::now().toSec();}
    void SetKF(int iLine, int iClientId){std::unique_lock<std::mutex> lock(mMutexKF); mvKFState[iClientId]=iLine;}
    void SetMP(int iLine, int iClientId){std::unique_lock<std::mutex> lock(mMutexMP); mvMPState[iClientId]=iLine;}
    void SetLoop(int iLine, int iClientId){std::unique_lock<std::mutex> lock(mMutexLoop); mvLoopState[iClientId]=iLine; mvdLastTimeLoop[iClientId] = ros::Time::now().toSec();}
    void SetMatch(int iLine, int iClientId){std::unique_lock<std::mutex> lock(mMutexMatch); mvMatchState[iClientId]=iLine; mdLastTimeMatch = ros::Time::now().toSec();}
    void SetMerge(int iLine, int iClientId){std::unique_lock<std::mutex> lock(mMutexMerge); mvMergeState[iClientId]=iLine;}
    void SetOpt(int iLine, int iClientId){std::unique_lock<std::mutex> lock(mMutexOpt); mvOptState[iClientId]=iLine;}

    void SetMappingLock(int iLine, int iClientId){std::unique_lock<std::mutex> lock(mMutexMappingLock); mvMappingLock[iClientId]=iLine;}

    void SetFinished(int iClientId){std::unique_lock<std::mutex> lock(mMutexFinished); mvbFinished[iClientId]=true;}

private:
    double mdTimeOutInSec;
    int miSleep;
    double mdStartTime;

    std::vector<int> mvMappingState;
    std::vector<int> mvCommState;
    std::vector<int> mvKFState;
    std::vector<int> mvMPState;
    std::vector<int> mvLoopState;
    std::vector<int> mvMatchState;
    std::vector<int> mvMergeState;
    std::vector<int> mvOptState;

    std::vector<int> mvMappingLock;

    std::vector<double> mvdLastTimeMapping;
    std::vector<double> mvdLastTimeComm;
    std::vector<double> mvdLastTimeLoop;

//    double mdLastTimeMapping;
//    double mdLastTimeComm;
//    double mdLastTimeLoop;
    double mdLastTimeMatch;

    std::mutex mMutexMapping;
    std::mutex mMutexComm;
    std::mutex mMutexKF;
    std::mutex mMutexMP;
    std::mutex mMutexLoop;
    std::mutex mMutexMatch;
    std::mutex mMutexMerge;
    std::mutex mMutexOpt;

    std::mutex mMutexMappingLock;

    std::mutex mMutexFinished;

    std::vector<bool> mvbFinished;
};
#endif

} //end ns

#endif
