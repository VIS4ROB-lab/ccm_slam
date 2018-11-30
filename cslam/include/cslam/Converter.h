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

#ifndef CSLAM_CONVERTER_H
#define CSLAM_CONVERTER_H

//C++
#include <opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include <Eigen/Dense>

//CSLAM
#include <cslam/config.h>
#include <cslam/estd.h>

//Msgs
#include <ccmslam_msgs/CvKeyPoint.h>

//Thirdparty
#include "thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

using namespace std;

namespace cslam
{

class Converter
{
public:
    static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

    static g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);
    static g2o::SE3Quat toSE3Quat(const g2o::Sim3 &gSim3);

    static cv::Mat toCvMat(const g2o::SE3Quat &SE3);
    static cv::Mat toCvMat(const g2o::Sim3 &Sim3);
    static cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m);
    static cv::Mat toCvMat(const Eigen::Matrix3d &m);
    static cv::Mat toCvMat(const Eigen::Matrix<double,3,1> &m);
    static cv::Mat toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t);

    static Eigen::Matrix<double,3,1> toVector3d(const cv::Mat &cvVector);
    static Eigen::Matrix<double,3,1> toVector3d(const cv::Point3f &cvPoint);
    static Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3);
    static Eigen::Matrix<double,4,4> toMatrix4d(const cv::Mat &cvMat4);

    static std::vector<float> toQuaternion(const cv::Mat &M);

    static ccmslam_msgs::CvKeyPoint toCvKeyPointMsg(const cv::KeyPoint &kp);
    static cv::KeyPoint fromCvKeyPointMsg(const ccmslam_msgs::CvKeyPoint &Msg);

    template<typename TMes, typename TMat> static void CvMatToMsgArrayFixedSize(cv::Mat &InMat, TMes &MsgArray)
    {
        if(!(InMat.rows || InMat.cols)) //0D --> fill with zeros
        {
            for(int idx=0;idx<MsgArray.size();++idx) MsgArray[idx] = static_cast<TMat>(0.0);
        }
        else if(InMat.cols == 1) //1D
        {
            for(int idx=0;idx<MsgArray.size();++idx) MsgArray[idx] = InMat.at<TMat>(idx);
        }
        else
        {
            for(int idx=0;idx<InMat.rows;++idx)
                for(int idy=0;idy<InMat.cols;++idy)
                    MsgArray[idx*InMat.rows+idy] = InMat.at<TMat>(idx,idy);
        }
    }

    template<typename TMes, typename TMat> static void MsgArrayFixedSizeToCvMat(cv::Mat &InMat, TMes &MsgArray)
    {
        if(InMat.cols == 1) //1D
        {
            for(int idx=0;idx<MsgArray.size();++idx) InMat.at<TMat>(idx) = MsgArray[idx];
        }
        else
        {
            for(int idx=0;idx<InMat.rows;++idx)
                for(int idy=0;idy<InMat.cols;++idy)
                    InMat.at<TMat>(idx,idy)=MsgArray[idx*InMat.rows+idy];
        }
    }
};

}// ns

#endif // CONVERTER_H
