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

#ifndef CSLAM_CONFIG_H_
#define CSLAM_CONFIG_H_

//C++
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <ctime>

//ROS
#include <std_msgs/ColorRGBA.h>

namespace cslam {
typedef double fptype;
}

//#define LOGGING
//#define TRACELOCK
//#define DEBUGGING2

#define INTERRUPTBA //Comm interrupts GBA when new data arrives from agent
#define DONOTINTERRUPTMERGE //Do not interrupt the map merging process
#define FINALBA

#define HIDEBUFFERLIMITS
#define SERVERCURKFSEARCHITS 3

namespace params {

typedef cslam::fptype fptype;

const std::string s0 (__FILE__);
const std::size_t p0 = s0.find("include");
const std::string s1 (s0.substr(0,p0));
const std::string s2 ("conf/config.yaml");
const std::string s3 = s1 + s2;
const std::string conf (s3);

const std::string out0 (__FILE__);
const std::size_t iout0 = out0.find("include");
const std::string out1 (out0.substr(0,iout0));
const std::string out2 ("output/");
const std::string out3 = out1 + out2;
const std::string outpath (out3);

template<typename T> inline static T GetVal(std::string path, std::string paramname)
{
    cv::FileStorage fSettings(path, cv::FileStorage::READ);

    if(!fSettings.isOpened())
    {
       std::cerr << "Failed to open config file at: " << path << std::endl;
       exit(-1);
    }

    const double val = fSettings[paramname];
    return (T)val;
}

inline std::string GetTimeAsString()
{
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );

    std::string year,mon,day,hour,min,sec,stringtime;

    std::stringstream* ss;
    ss = new std::stringstream;
    *ss << now->tm_year + 1900;
    year = ss->str();

    ss = new std::stringstream;
    *ss << now->tm_mon + 1;
    mon = ss->str();
    if(mon.size() == 1)
    {
        ss = new std::stringstream;
        *ss << 0 << mon;
        mon = ss->str();
    }

    ss = new std::stringstream;
    *ss << now->tm_mday;
    day = ss->str();
    if(day.size() == 1)
    {
        ss = new std::stringstream;
        *ss << 0 << day;
        day = ss->str();
    }

    ss = new std::stringstream;
    *ss << now->tm_hour;
    hour = ss->str();
    if(hour.size() == 1)
    {
        ss = new std::stringstream;
        *ss << 0 << hour;
        hour = ss->str();
    }

    ss = new std::stringstream;
    *ss << now->tm_min;
    min = ss->str();
    if(min.size() == 1)
    {
        ss = new std::stringstream;
        *ss << 0 << min;
        min = ss->str();
    }

    ss = new std::stringstream;
    *ss << now->tm_sec;
    sec = ss->str();
    if(sec.size() == 1)
    {
        ss = new std::stringstream;
        *ss << 0 << sec;
        sec = ss->str();
    }

    ss = new std::stringstream;
    *ss << year << mon << day << "_" << hour << min << sec << "_";

    stringtime = ss->str();

    delete ss;

    return stringtime;
}

inline std_msgs::ColorRGBA MakeColorMsg(float fR,float fG, float fB)
{
    std_msgs::ColorRGBA msg;
    msg.r = fR;
    msg.g = fG;
    msg.b = fB;
    msg.a = 1.0;
    return msg;
}

struct VisColorRGB
{
public:
    VisColorRGB(float fR,float fG, float fB)
        : mfR(fR),mfG(fG),mfB(fB),
          mu8R((u_int8_t)(fR*255)),mu8G((u_int8_t)(fG*255)),mu8B((u_int8_t)(fB*255)),mColMsg(MakeColorMsg(fR,fG,fB))
        {
            //...
        }

    const float mfR,mfG,mfB;
    const u_int8_t mu8R,mu8G,mu8B;
    const std_msgs::ColorRGBA mColMsg;
};

namespace sys {
    const int mbStrictLock = 1;
    const std::string msTime = GetTimeAsString();
}

namespace stats {
    const bool mbWriteKFsToFile = GetVal<bool>(conf,"Stats.WriteKFsToFile");
    const std::string  msOutputDir = outpath;
    const int miTrajectoryFormat = GetVal<int>(conf,"Stats.trajectory_format");
}

namespace timings {
    const int miLockSleep = GetVal<int>(conf,"Timing.LockSleep");

    namespace client {
        const int miRosRate = GetVal<int>(conf,"Timing.Client.RosRate");
        const int miViewerRate = GetVal<int>(conf,"Timing.Client.ViewerRate");
        const int miMappingRate = GetVal<int>(conf,"Timing.Client.MappingRate");
        const int miCommRate = GetVal<int>(conf,"Timing.Client.CommRate");
    }

    namespace server {
        const int miRosRate = GetVal<int>(conf,"Timing.Server.RosRate");
        const int miViewerRate = GetVal<int>(conf,"Timing.Server.ViewerRate");
        const int miMappingRate = GetVal<int>(conf,"Timing.Server.MappingRate");
        const int miCommRate = GetVal<int>(conf,"Timing.Server.CommRate");
        const int miPlaceRecRateRate = GetVal<int>(conf,"Timing.Server.PlaceRecRate");
    }
}

namespace extractor {
    const int miNumFeatures = GetVal<int>(conf,"ORBextractor.nFeatures");
    const fptype mfScaleFactor = GetVal<fptype>(conf,"ORBextractor.scaleFactor");
    const int miNumLevels = GetVal<int>(conf,"ORBextractor.nLevels");
    const int miIniThFAST = GetVal<int>(conf,"ORBextractor.iniThFAST");
    const int miNumThFAST = GetVal<int>(conf,"ORBextractor.minThFAST");
}

namespace tracking {
    const int miInitKFs = GetVal<int>(conf,"Tracking.iInitKFs");
    //KF Creation Params
    const int miMinFrames = GetVal<int>(conf,"Tracking.MinFrames");
    const int miMaxFrames = GetVal<int>(conf,"Tracking.MaxFrames");
    const int miMatchesInliersThres = GetVal<int>(conf,"Tracking.nMatchesInliersThres");
    const fptype mfThRefRatio = GetVal<fptype>(conf,"Tracking.thRefRatio");
    //Tracking Functions Inlier Thresholds
    const int miTrackWithRefKfInlierThresSearch = GetVal<int>(conf,"Tracking.TrackWithRefKfInlierThresSearch");
    const int miTrackWithRefKfInlierThresOpt = GetVal<int>(conf,"Tracking.TrackWithRefKfInlierThresOpt");
    const int miTrackWithMotionModelInlierThresSearch = GetVal<int>(conf,"Tracking.TrackWithMotionModelInlierThresSearch");
    const int miTrackWithMotionModelInlierThresOpt = GetVal<int>(conf,"Tracking.TrackWithMotionModelInlierThresOpt");
    const int miTrackLocalMapInlierThres = GetVal<int>(conf,"Tracking.TrackLocalMapInlierThres");
}

namespace mapping {
    const fptype mfRedundancyThres = GetVal<fptype>(conf,"Mapping.RedThres");
    const int miLocalMapSize = GetVal<int>(conf,"Mapping.LocalMapSize");
    const int miLocalMapBuffer = GetVal<int>(conf,"Mapping.LocalMapBuffer");
    const int miNumRecentKFs = GetVal<int>(conf,"Mapping.RecentKFWindow");
}

namespace comm {

    namespace client {
        const fptype mfPubFreq = GetVal<fptype>(conf,"Comm.Client.PubFreq");
        const fptype mfPubPeriodicTime = 1/mfPubFreq;

        const int miKfItBound = GetVal<int>(conf,"Comm.Client.KfItBound");
        const int miMpItBound = GetVal<int>(conf,"Comm.Client.MpItBound");
        const int miPubMapBufferSize = GetVal<int>(conf,"Comm.Client.PubMapBuffer");
        const int miSubMapBufferSize = GetVal<int>(conf,"Comm.Client.SubMapBuffer");
        const int miKfPubMax = GetVal<int>(conf,"Comm.Client.PubMaxKFs");
        const int miMpPubMax = GetVal<int>(conf,"Comm.Client.PubMaxMPs");
    }

    namespace server {
        const fptype mfPubFreq = GetVal<fptype>(conf,"Comm.Server.PubFreq");
        const fptype mfPubPeriodicTime = 1/mfPubFreq;

        const int miKfLimitToClient = GetVal<int>(conf,"Comm.Server.KfsToClient");

        const int miKfItBound = GetVal<int>(conf,"Comm.Server.KfItBound");
        const int miMpItBound = GetVal<int>(conf,"Comm.Server.MpItBound");
        const int miPubMapBufferSize = GetVal<int>(conf,"Comm.Server.PubMapBuffer");
        const int miSubMapBufferSize = GetVal<int>(conf,"Comm.Server.SubMapBuffer");
    }

}

namespace placerec {
    const int miNewLoopThres = GetVal<int>(conf,"Placerec.NewLoopThres");
    const int miStartMapMatchingAfterKf = GetVal<int>(conf,"Placerec.StartMapMatchingAfterKf");
    const int miCovisibilityConsistencyTh = GetVal<int>(conf,"Placerec.CovisibilityConsistencyTh");
}

namespace opt {
    //Loop Closure
    const int mSolverIterations = GetVal<int>(conf,"Opt.SolverIterations");
    const int mMatchesThres = GetVal<int>(conf,"Opt.MatchesThres"); //matches that need to be found by SearchByBoW()
    const int mInliersThres = GetVal<int>(conf,"Opt.InliersThres"); //inliers after pose optimization
    const int mTotalMatchesThres = GetVal<int>(conf,"Opt.TotalMatchesThres"); //total matches SearchByProjection
    //RANSAC params
    const fptype mProbability = GetVal<fptype>(conf,"Opt.Probability");
    const int mMinInliers = GetVal<int>(conf,"Opt.MinInliers");
    const int mMaxIterations = GetVal<int>(conf,"Opt.MaxIterations");
    //Map Merger
    const int mGBAIterations = GetVal<int>(conf,"Opt.GBAIterations");
    const int miEssGraphMinFeats = GetVal<int>(conf,"Opt.EssGraphMinFeats");
}

namespace vis {
    const bool mbActive = GetVal<bool>(conf,"Viewer.Active");

    const bool mbShowCovGraph = GetVal<bool>(conf,"Viewer.ShowCovGraph");
    const bool mbShowMPs = GetVal<bool>(conf,"Viewer.ShowMapPoints");
    const bool mbShowTraj = GetVal<bool>(conf,"Viewer.ShowTraj");
    const bool mbShowKFs = GetVal<bool>(conf,"Viewer.ShowKFs");
    const int miCovGraphMinFeats = GetVal<int>(conf,"Viewer.CovGraphMinFeats");

    const fptype mfScaleFactor = GetVal<fptype>(conf,"Viewer.ScaleFactor");
    const fptype mfTrajMarkerSize = GetVal<fptype>(conf,"Viewer.TrajMarkerSize");
    const fptype mfCovGraphMarkerSize = GetVal<fptype>(conf,"Viewer.CovGraphMarkerSize");
    const fptype mfLoopMarkerSize = GetVal<fptype>(conf,"Viewer.LoopMarkerSize");
    const fptype mfMarkerSphereDiameter = GetVal<fptype>(conf,"Viewer.MarkerSphereDiameter");
    const fptype mfCamSize = GetVal<fptype>(conf,"Viewer.CamSize");
    const fptype mfCamLineSize = GetVal<fptype>(conf,"Viewer.CamLineSize");
}

namespace colors {
    const VisColorRGB mc0 = VisColorRGB(GetVal<float>(conf,"Viewer.ColorR0"),GetVal<float>(conf,"Viewer.ColorG0"),GetVal<float>(conf,"Viewer.ColorB0"));
    const VisColorRGB mc1 = VisColorRGB(GetVal<float>(conf,"Viewer.ColorR1"),GetVal<float>(conf,"Viewer.ColorG1"),GetVal<float>(conf,"Viewer.ColorB1"));
    const VisColorRGB mc2 = VisColorRGB(GetVal<float>(conf,"Viewer.ColorR2"),GetVal<float>(conf,"Viewer.ColorG2"),GetVal<float>(conf,"Viewer.ColorB2"));
    const VisColorRGB mc3 = VisColorRGB(GetVal<float>(conf,"Viewer.ColorR3"),GetVal<float>(conf,"Viewer.ColorG3"),GetVal<float>(conf,"Viewer.ColorB3"));
    const VisColorRGB mcCovGraph = VisColorRGB(GetVal<float>(conf,"Viewer.ColorRcov"),GetVal<float>(conf,"Viewer.ColorGcov"),GetVal<float>(conf,"Viewer.ColorBcov"));
}

void ShowParams();

} //end ns

#endif
