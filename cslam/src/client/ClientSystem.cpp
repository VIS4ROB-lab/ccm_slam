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

#include <cslam/client/ClientSystem.h>

namespace cslam{

ClientSystem::ClientSystem(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, const string &strVocFile, const string &strCamFile)
    : mNh(Nh), mNhPrivate(NhPrivate), mstrCamFile(strCamFile), mpUID(new estd::UniqueIdDispenser())
{    
    params::ShowParams();

    //+++++ load params +++++

//    std::string TopicNameCamSub;
    int ClientId;

    mNhPrivate.param("ClientId",ClientId,-1);
    mClientId = static_cast<size_t>(ClientId);

    //+++++ Check settings files +++++

    cv::FileStorage fsSettingsCam(strCamFile.c_str(), cv::FileStorage::READ);
    if(!fsSettingsCam.isOpened())
    {
       cerr << "Failed to open cam file at: " << strCamFile << endl;
       exit(-1);
    }

    //+++++ load vocabulary +++++

    this->LoadVocabulary(strVocFile);

    //+++++ Create KeyFrame Database +++++
    mpKFDB.reset(new KeyFrameDatabase(mpVoc));

    //+++++ Create the Map +++++
    mpMap.reset(new Map(mNh,mNhPrivate,mClientId,eSystemState::CLIENT));
    usleep(10000); //wait to avoid race conditions
    //+++++ Initialize Agent +++++
    mpAgent.reset(new ClientHandler(mNh,mNhPrivate,mpVoc,mpKFDB,mpMap,mClientId,mpUID,eSystemState::CLIENT,strCamFile,nullptr));
    usleep(10000); //wait to avoid race conditions
    mpAgent->InitializeThreads();
    usleep(10000); //wait to avoid race conditions

    //++++++++++
    cout << endl << "Clientsystem initialized (Client ID: " << mClientId << ")" << endl;
}

void ClientSystem::LoadVocabulary(const string &strVocFile)
{
    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVoc.reset(new ORBVocabulary());
    bool bVocLoad = mpVoc->loadFromTextFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Failed to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;
}

} //end namespace
