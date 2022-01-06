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

#include <cslam/Map.h>

#include <cslam/Database.h>

namespace cslam {

Map::Map(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, size_t MapId, eSystemState SysState)
    : mnMaxKFid(0), mnMaxMPid(0),
      mnMaxKFidUnique(0),mnMaxMPidUnique(0),
      mNh(Nh), mNhPrivate(NhPrivate),
      mMapId(MapId),mbOutdated(false),
      mSysState(SysState),
      mbLockMapUpdate(false),mbLockPointCreation(false)
      ,mnLastKfIdUnique(0)
      ,mbStopGBA(false),mbRunningGBA(false),mbFinishedGBA(false),
    #ifdef FINALBA
    mbGBAinterrupted(false),
    #endif
    mbNoStartGBA(false),mbStopGBAInProgress(false)
    #ifdef DONOTINTERRUPTMERGE
    ,mbMergeStepGBA(false)
    #endif
{
    uint myseed = time(NULL);
    srand (myseed);
    std::cout << "Map " << mMapId << " rand seed: " << myseed << std::endl;

    string SysType;
    if(mSysState == eSystemState::CLIENT)
    {
        SysType = "Client";
    }
    else if(mSysState == eSystemState::SERVER)
    {
        SysType = "Server";
    }
    else
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Map::Map(): invalid systems state: " << mSysState << endl;
        throw infrastructure_ex();
    }

    cout << "+++++ Map " << mMapId << " Initialized +++++" << endl;
}

Map::Map(const mapptr &pMapTarget, const mapptr &pMapToFuse)
    : mNh(pMapTarget->mNh),mNhPrivate(pMapTarget->mNhPrivate),
      mMapId(pMapTarget->mMapId),mbOutdated(false),
      mbLockMapUpdate(false),mbLockPointCreation(false)
      ,mnLastKfIdUnique(pMapToFuse->GetLastKfIdUnique())
    ,mbStopGBA(false),mbRunningGBA(false),mbFinishedGBA(false),
    #ifdef FINALBA
    mbGBAinterrupted(false),
    #endif
    mbNoStartGBA(false),mbStopGBAInProgress(false)
    #ifdef DONOTINTERRUPTMERGE
    ,mbMergeStepGBA(false)
    #endif
{

    mSysState = pMapTarget->mSysState;

    string SysType;
    if(mSysState == eSystemState::CLIENT)
    {
        SysType = "Client";
    }
    else if(mSysState == eSystemState::SERVER)
    {
        SysType = "Server";
    }
    else
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Map::Map(): invalid systems state: " << mSysState << endl;
        throw infrastructure_ex();
    }

    //data Map A
    set<size_t> msuAssClientsA = pMapTarget->msuAssClients;
    set<size_t> msnFinishedAgentsA = pMapTarget->msnFinishedAgents;
    vector<kfptr> mvpKeyFrameOriginsA = pMapTarget->mvpKeyFrameOrigins;
    long unsigned int mnMaxKFidA = pMapTarget->GetMaxKFid();
    long unsigned int mnMaxMPidA = pMapTarget->GetMaxMPid();
    long unsigned int mnMaxKFidUniqueA = pMapTarget->GetMaxKFidUnique();
    long unsigned int mnMaxMPidUniqueA = pMapTarget->GetMaxKFidUnique();
    std::map<idpair,mpptr> mmpMapPointsA = pMapTarget->GetMmpMapPoints();
    std::map<idpair,kfptr> mmpKeyFramesA = pMapTarget->GetMmpKeyFrames();
    std::map<idpair,mpptr> mmpErasedMapPointsA = pMapTarget->GetMmpErasedMapPoints();
    std::map<idpair,kfptr> mmpErasedKeyFramesA = pMapTarget->GetMmpErasedKeyFrames();
    set<ccptr> spCCA = pMapTarget->GetCCPtrs();

    //data Map B
    set<size_t> msuAssClientsB = pMapToFuse->msuAssClients;
    set<size_t> msnFinishedAgentsB = pMapToFuse->msnFinishedAgents;
    vector<kfptr> mvpKeyFrameOriginsB = pMapToFuse->mvpKeyFrameOrigins;
    long unsigned int mnMaxKFidB = pMapToFuse->GetMaxKFid();
    long unsigned int mnMaxMPidB = pMapToFuse->GetMaxMPid();
    long unsigned int mnMaxKFidUniqueB = pMapToFuse->GetMaxKFidUnique();
    long unsigned int mnMaxMPidUniqueB = pMapToFuse->GetMaxKFidUnique();
    std::map<idpair,mpptr> mmpMapPointsB = pMapToFuse->GetMmpMapPoints();
    std::map<idpair,kfptr> mmpKeyFramesB = pMapToFuse->GetMmpKeyFrames();
    std::map<idpair,mpptr> mmpErasedMapPointsB = pMapToFuse->GetMmpErasedMapPoints();
    std::map<idpair,kfptr> mmpErasedKeyFramesB = pMapToFuse->GetMmpErasedKeyFrames();
    set<ccptr> spCCB = pMapToFuse->GetCCPtrs();

    //fill new map
    mOdomFrame = pMapTarget->mOdomFrame;

    msuAssClients.insert(msuAssClientsA.begin(),msuAssClientsA.end());
    msuAssClients.insert(msuAssClientsB.begin(),msuAssClientsB.end());
    msnFinishedAgents.insert(msnFinishedAgentsA.begin(),msnFinishedAgentsA.end());
    msnFinishedAgents.insert(msnFinishedAgentsB.begin(),msnFinishedAgentsB.end());
    mvpKeyFrameOrigins.insert(mvpKeyFrameOrigins.end(),mvpKeyFrameOriginsA.begin(),mvpKeyFrameOriginsA.end());
    mvpKeyFrameOrigins.insert(mvpKeyFrameOrigins.end(),mvpKeyFrameOriginsB.begin(),mvpKeyFrameOriginsB.end());
    mnMaxKFid = std::max(mnMaxKFidA,mnMaxKFidB);
    mnMaxMPid = std::max(mnMaxMPidA,mnMaxMPidB);
    mnMaxKFidUnique = std::max(mnMaxKFidUniqueA,mnMaxKFidUniqueB);
    mnMaxMPidUnique = std::max(mnMaxMPidUniqueA,mnMaxMPidUniqueB);
    mmpMapPoints.insert(mmpMapPointsA.begin(),mmpMapPointsA.end());
    mmpMapPoints.insert(mmpMapPointsB.begin(),mmpMapPointsB.end());
    mmpKeyFrames.insert(mmpKeyFramesA.begin(),mmpKeyFramesA.end());
    mmpKeyFrames.insert(mmpKeyFramesB.begin(),mmpKeyFramesB.end());
    mmpErasedMapPoints.insert(mmpErasedMapPointsA.begin(),mmpErasedMapPointsA.end());
    mmpErasedMapPoints.insert(mmpErasedMapPointsB.begin(),mmpErasedMapPointsB.end());
    mmpErasedKeyFrames.insert(mmpErasedKeyFramesA.begin(),mmpErasedKeyFramesA.end());
    mmpErasedKeyFrames.insert(mmpErasedKeyFramesB.begin(),mmpErasedKeyFramesB.end());
    mspCC.insert(spCCA.begin(),spCCA.end());
    mspCC.insert(spCCB.begin(),spCCB.end());

    #ifdef FINALBA
    if(pMapTarget->isGBAinterrupted() || pMapToFuse->isGBAinterrupted())
        this->mbGBAinterrupted = true;
    #endif

    for(set<ccptr>::const_iterator sit = mspCC.begin();sit!=mspCC.end();++sit)
    {
        ccptr pCC = *sit;
        mspComm.insert(pCC->mpCH->GetCommPtr());
    }

    for(set<size_t>::iterator sit = msnFinishedAgentsA.begin();sit!=msnFinishedAgentsA.end();++sit)
        cout << "Target Map Finished Agents: " << *sit << endl;

    for(set<size_t>::iterator sit = msnFinishedAgents.begin();sit!=msnFinishedAgents.end();++sit)
        cout << "Merged Map Finished Agents: " << *sit << endl;

    //----------------------------
}

void Map::UpdateAssociatedData()
{

    //replace associated maps
    for(std::map<idpair,kfptr>::iterator mit = mmpKeyFrames.begin();mit!=mmpKeyFrames.end();++mit)
    {
        kfptr pKF = mit->second;
        pKF->ReplaceMap(this->shared_from_this());
        for(set<commptr>::const_iterator sit2 = mspComm.begin();sit2!=mspComm.end();++sit2)
        {
            commptr pComm = *sit2;
            pKF->AddCommPtr(pComm);
        }
    }

    for(std::map<idpair,mpptr>::iterator mit = mmpMapPoints.begin();mit!=mmpMapPoints.end();++mit)
    {
        mpptr pMP = mit->second;
        pMP->ReplaceMap(this->shared_from_this());
        for(set<commptr>::const_iterator sit2 = mspComm.begin();sit2!=mspComm.end();++sit2)
        {
            commptr pComm = *sit2;
            pMP->AddCommPtr(pComm);
        }
    }

    for(map<idpair,kfptr>::iterator mit = mmpErasedKeyFrames.begin();mit!=mmpErasedKeyFrames.end();++mit)
    {
        kfptr pKF = mit->second;
        pKF->ReplaceMap(this->shared_from_this());
        for(set<commptr>::const_iterator sit2 = mspComm.begin();sit2!=mspComm.end();++sit2)
        {
            commptr pComm = *sit2;
            pKF->AddCommPtr(pComm);
        }
    }

    for(map<idpair,mpptr>::iterator mit = mmpErasedMapPoints.begin();mit!=mmpErasedMapPoints.end();++mit)
    {
        mpptr pMP = mit->second;
        pMP->ReplaceMap(this->shared_from_this());
        for(set<commptr>::const_iterator sit2 = mspComm.begin();sit2!=mspComm.end();++sit2)
        {
            commptr pComm = *sit2;
            pMP->AddCommPtr(pComm);
        }
    }
}

void Map::AddKeyFrame(kfptr pKF)
{
    {
        unique_lock<mutex> lock(mMutexMap);

        if(mSysState == eSystemState::CLIENT)
        {
            std::map<idpair,kfptr>::iterator mit = mmpKeyFrames.find(pKF->mId);
            if(mit != mmpKeyFrames.end())
            {
                return;
            }
            else
            {
                commptr pComm = *(mspComm.begin());
                if(!pKF->mbFromServer)
                    pComm->PassKftoComm(pKF);
            }
        }
        else if(mSysState == eSystemState::SERVER)
        {
            for(set<commptr>::const_iterator sit = mspComm.begin();sit!=mspComm.end();++sit)
            {
                pKF->AddCommPtr(*sit);
            }

            mnLastKfIdUnique = pKF->mUniqueId;
        }

        if(pKF->mId.first>mnMaxKFid)
            mnMaxKFid=pKF->mId.first;
        if(pKF->mUniqueId>mnMaxKFidUnique)
            mnMaxKFidUnique=pKF->mUniqueId;

        mmpKeyFrames[pKF->mId] = pKF;

        if(mSysState == SERVER)
            if(mmpKeyFrames.size() % 50 == 0)
                cout << "KFs in Map: " << mmpKeyFrames.size() << endl;
    }
}

void Map::AddMapPoint(mpptr pMP)
{
    unique_lock<mutex> lock(mMutexMap);

    if(mSysState == eSystemState::CLIENT)
    {
        std::map<idpair,mpptr>::iterator mit = mmpMapPoints.find(pMP->mId);
        if(mit != mmpMapPoints.end())
        {
            return;
        }
        else
        {
            commptr pComm = *(mspComm.begin());
            if(!pMP->mbFromServer)
            {
                pComm->PassMptoComm(pMP);
            }
        }
    }
    else if(mSysState == eSystemState::SERVER)
    {
        for(set<commptr>::const_iterator sit = mspComm.begin();sit!=mspComm.end();++sit)
        {
            pMP->AddCommPtr(*sit);
        }
    }

    if(pMP->mId.first>mnMaxMPid)
        mnMaxMPid=pMP->mId.first;
    if(pMP->mUniqueId>mnMaxMPidUnique)
        mnMaxMPidUnique=pMP->mUniqueId;

    mmpMapPoints[pMP->mId] = pMP;
}

void Map::EraseMapPoint(mpptr pMP)
{
    unique_lock<mutex> lock(mMutexMap);

    std::map<idpair,mpptr>::iterator mit = mmpMapPoints.find(pMP->mId);
    if(mit != mmpMapPoints.end()) mmpMapPoints.erase(mit);

    if(msuAssClients.count(pMP->mId.second))
    {
        unique_lock<mutex> lock2(mMutexErased);
        mmpErasedMapPoints[pMP->mId] = pMP;
    }
}

void Map::EraseKeyFrame(kfptr pKF)
{
    if(pKF->mId.first == 0)
    {
        cout << COUTFATAL << " cannot erase Origin-KF" << endl;
        throw infrastructure_ex();
    }

    unique_lock<mutex> lock(mMutexMap);

    std::map<idpair,kfptr>::iterator mit = mmpKeyFrames.find(pKF->mId);
    if(mit != mmpKeyFrames.end()) mmpKeyFrames.erase(mit);

    if(msuAssClients.count(pKF->mId.second))
    {
        unique_lock<mutex> lock2(mMutexErased);
        mmpErasedKeyFrames[pKF->mId] = pKF;
    }
}

void Map::SaveMap(const string &path_name) {
    std::cout << "+++ Save Map to File +++" << std::endl;

    std::string kf_tmp = "/keyframes";
    std::string mp_tmp = "/mappoints";

    char cstr0[path_name.size()+1];
    char cstr[path_name.size() + kf_tmp.size()+1];
    char cstr2[path_name.size() + mp_tmp.size()+1];
    strcpy(cstr0,path_name.c_str());
    strcpy(cstr, (path_name+kf_tmp).c_str());
    strcpy(cstr2, (path_name+mp_tmp).c_str());
    std::cout << cstr0 << std::endl;
    std::cout << mkdir(cstr0,  0777);
    std::cout << mkdir(cstr,  0777);
    std::cout << mkdir(cstr2,  0777);
    std::cout << std::endl;

    std::cout << "--> Writing Keyframes to file" << std::endl;
    auto keyframes = this->GetAllKeyFrames();
    for(unsigned long int i = 0; i < keyframes.size(); i++) {
        std::ofstream fs;
        fs.open(path_name+"/keyframes/keyframes"+std::to_string(i)+".txt");
        if(fs.is_open()) {
            kfptr kfi = keyframes[i];
            std::stringstream kf_ss;
            cereal::BinaryOutputArchive oarchive(kf_ss);
            oarchive(*kfi);

            fs << kf_ss.str();
            fs.close();
        }
    }

    std::cout << "--> Writing Landmarks to file" << std::endl;
    auto mappoints = this->GetAllMapPoints();
    for(unsigned long int i = 0; i < mappoints.size(); i++) {
        std::ofstream fs;
        fs.open(path_name+"/mappoints/mappoints"+std::to_string(i)+".txt");
        if(fs.is_open()) {
            mpptr mpi = mappoints[i];
            std::stringstream mp_ss;
            cereal::BinaryOutputArchive oarchive(mp_ss);
            oarchive(*mpi);

            fs << mp_ss.str();
            fs.close();
        }
    }

    std::cout << "+++ DONE +++" << std::endl;
}

void Map::SetReferenceMapPoints(const vector<mpptr> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

vector<Map::kfptr> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);

    vector<kfptr> vpKFs;
    for(std::map<idpair,kfptr>::iterator mit_set = mmpKeyFrames.begin();mit_set!=mmpKeyFrames.end();++mit_set)
        vpKFs.push_back(mit_set->second);
    return vpKFs;
}

vector<Map::mpptr> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);

    vector<mpptr> vpMPs;
    for(std::map<idpair,mpptr>::iterator mit_set = mmpMapPoints.begin();mit_set!=mmpMapPoints.end();++mit_set)
        vpMPs.push_back(mit_set->second);
    return vpMPs;
}

void Map::LoadMap(const string &path_name, vocptr voc, commptr comm, dbptr kfdb, uidptr uid) {
    std::cout << "+++ Load Map from File +++" << std::endl;

    if(!voc) {
        std::cout << COUTFATAL << "invalid vocabulary ptr" << std::endl;
        exit(-1);
    }

    std::vector<std::string> filenames_kf, filenames_mp;
    std::vector<kfptr> keyframes;
    std::vector<mpptr> mappoints;

    map<idpair,idpair> saved_kf_ids_to_sys_ids; //maps the IDs of the saved KFs/MPs to new IDs -- systems assumes after load, all client IDs are 0.
    map<idpair,idpair> saved_mp_ids_to_sys_ids;
    size_t next_kf_id0 = 0;
    size_t next_mp_id0 = 0;

    std::string kf_tmp = "/keyframes/";
    std::string mp_tmp = "/mappoints/";
    char cstr0[path_name.size()+mp_tmp.size()+1];
    char cstr1[path_name.size()+kf_tmp.size()+1];
    strcpy(cstr0, (path_name+kf_tmp).c_str());
    strcpy(cstr1, (path_name+mp_tmp).c_str());

    // getting all filenames for the keyframes
    struct dirent *entry;
    DIR *dir = opendir(cstr0);

    if (dir == nullptr) {
        std::cout << "Directory is empty." << std::endl;
        return;
    }

    while ((entry = readdir(dir)) != nullptr) {

        if ( !strcmp(entry->d_name, ".") || !strcmp(entry->d_name, "..") ) {
            continue;
        } else {
            std::stringstream path;
            path << path_name+"/keyframes/";
            path << entry->d_name;
            filenames_kf.push_back(path.str());
        }
    }
    closedir(dir);

    // getting all filenames for the mappoints
    struct dirent *entry2;
    DIR *dir2 = opendir(cstr1);
    if (dir2 == nullptr) {
        std::cout << "Directory is empty." << std::endl;
        return;
    }

     while ((entry2 = readdir(dir2)) != nullptr) {

        if ( !strcmp(entry2->d_name, ".") || !strcmp(entry2->d_name, "..") ) {
            continue;
        } else {
            std::stringstream path;
            path << path_name+"/mappoints/";
            path << entry2->d_name;
            filenames_mp.push_back(path.str());
        }
    }
    closedir(dir);

    std::cout << "--> Loading Keyframes" << std::endl;
    for(unsigned long int i = 0; i < filenames_kf.size(); i++) {
        kfptr kf(new KeyFrame(voc,shared_from_this(),kfdb,comm,eSystemState::SERVER,uid->GetId()));
        std::stringstream buf;
        std::ifstream fs;
        fs.open(filenames_kf[i]);
        if(fs.is_open()) {
            buf << fs.rdbuf();
            cereal::BinaryInputArchive iarchive(buf);
            iarchive(*kf);
            keyframes.push_back(kf);
            if(kf->mId.second == 0) next_kf_id0 = max(next_kf_id0,kf->mId.first+1);
//            this->AddKeyFrame(kf);
            if(kf->mId.first == 0) mvpKeyFrameOrigins.push_back(kf);
            fs.close();
        } else {
            std::cout << filenames_kf[i] << std::endl;
            exit(-1);
        }
    }

    std::sort(keyframes.begin(),keyframes.end(),kftimecmpsmaller());

    std::cout << "--> Loading MapPoints" << std::endl;
    for(unsigned long int i = 0; i < filenames_mp.size(); i++) {
        mpptr mp(new MapPoint(shared_from_this(),comm,eSystemState::SERVER,uid->GetId()));
        std::stringstream buf;
        std::ifstream fs;
        fs.open(filenames_mp[i]);
        if(fs.is_open()) {
            buf << fs.rdbuf();
            cereal::BinaryInputArchive iarchive(buf);
            iarchive(*mp);
            mappoints.push_back(mp);
            if(mp->mId.second == 0) next_mp_id0 = max(next_mp_id0,mp->mId.first+1);
//            this->AddMapPoint(mp);
            fs.close();
        } else {
            std::cout << filenames_mp[i] << std::endl;
            exit(-1);
        }
    }

    std::cout << "Map consists of " << keyframes.size() << " keyframes" << std::endl;
    std::cout << "Map consists of " << mappoints.size() << " mappoints" << std::endl;

    // Re-Map if necessary
    for(auto kf : keyframes) {
        if(kf->mId.second != 0) {
            idpair new_id = make_pair(next_kf_id0++,0);
            saved_kf_ids_to_sys_ids[kf->mId] = new_id;
            kf->mId = new_id;
        }
    }
    for(auto lm : mappoints) {
        if(lm->mId.second != 0) {
            idpair new_id = make_pair(next_mp_id0++,0);
            saved_mp_ids_to_sys_ids[lm->mId] = new_id;
            lm->mId = new_id;
        }
    }
    // -------------------

    std::cout << "--> Building Connections" << std::endl;

    for(auto kf : keyframes) {
        this->AddKeyFrame(kf);
        kf->ProcessAfterLoad(saved_kf_ids_to_sys_ids);
        if(!this->msuAssClients.count(kf->mId.second))
            msuAssClients.insert(kf->mId.second);
    }

    std::cout << "----> Landmarks" << std::endl;
    for(auto lm : mappoints) {
        for(auto mit = lm->mmObservations_minimal.begin(); mit!=lm->mmObservations_minimal.end();++mit){
            size_t feat_id = mit->second;
            idpair kf_id = mit->first;

            if(kf_id.second != 0) {
                if(!saved_kf_ids_to_sys_ids.count(kf_id)) {
                    std::cout << COUTERROR << "ID ERROR" << std::endl;
                    exit(-1);
                }
                kf_id = saved_kf_ids_to_sys_ids[kf_id];
            }

            kfptr kf = this->GetKfPtr(kf_id);
            if(!kf){
                std::cout << COUTWARN << "cannot find KF" << std::endl;
                continue;
            }
            lm->AddObservation(kf,feat_id);
        }
        idpair kf_ref_id = lm->mRefKfId;

        if(kf_ref_id.second != 0) {
            if(!saved_kf_ids_to_sys_ids.count(kf_ref_id)) {
                std::cout << COUTERROR << "ID ERROR" << std::endl;
                exit(-1);
            }
            kf_ref_id = saved_kf_ids_to_sys_ids[kf_ref_id];
        }

        auto kf_ref = this->GetKfPtr(kf_ref_id);
        if(!kf_ref){
            std::cout << COUTWARN << "cannot find KF" << std::endl;
            continue;
        }
        lm->SetReferenceKeyFrame(kf_ref);
        lm->ComputeDistinctiveDescriptors();
        lm->UpdateNormalAndDepth();
        this->AddMapPoint(lm);
    }

    std::cout << "----> Keyframes" << std::endl;
    for(auto kf : keyframes) {
        for(auto mit = kf->mmMapPoints_minimal.begin(); mit!=kf->mmMapPoints_minimal.end();++mit){
            size_t feat_id = mit->first;
            idpair lm_id = mit->second;

            if(lm_id.second != 0) {
                if(!saved_mp_ids_to_sys_ids.count(lm_id)) {
                    std::cout << COUTERROR << "ID ERROR -- MP:" << lm_id.first << "|" << lm_id.second << std::endl;
                    exit(-1);
                }
                lm_id = saved_mp_ids_to_sys_ids[lm_id];
            }

            mpptr lm = this->GetMpPtr(lm_id);
            if(!lm) std::cout << COUTWARN << "requested LM " << lm_id.first << "|" << lm_id.second << " does not exist" << std::endl;
            kf->AddMapPoint(lm,feat_id);
        }
//        kf->ProcessAfterLoad();
        kf->UpdateConnections();
//        if(!kf->GetParent()) {
//            std::cout << COUTWARN << "KF " << kf->mId.first << "|" << kf->mId.second << " has no parent" << std::endl;
//        }
    }

    std::cout << "+++ DONE +++" << std::endl;
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);

    return mmpMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);

    return mmpKeyFrames.size();
}

vector<Map::mpptr> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

long unsigned int Map::GetMaxMPid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxMPid;
}

long unsigned int Map::GetMaxKFidUnique()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFidUnique;
}

long unsigned int Map::GetMaxMPidUnique()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxMPidUnique;
}

long unsigned int Map::GetLastKfIdUnique()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnLastKfIdUnique;
}

void Map::clear()
{
    mmpMapPoints.clear();
    mmpKeyFrames.clear();
    mmpErasedMapPoints.clear();
    mmpErasedKeyFrames.clear();
    mnMaxKFid = 0;
    mnMaxMPid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

Map::kfptr Map::GetKfPtr(size_t KfId, size_t ClientId, bool bIgnoreMutex) //Performance: find a better implementation for this method
{
    if(!bIgnoreMutex)
        unique_lock<mutex> lock(mMutexMap);

    idpair idp = make_pair(KfId,ClientId);
    std::map<idpair,kfptr>::iterator mit = mmpKeyFrames.find(idp);
    if(mit != mmpKeyFrames.end()) return mit->second;
    else return nullptr;
}

Map::kfptr Map::GetRandKfPtr()
{
    ccptr pCC = *(mspCC.begin());

    if(mnMaxKFid < (params::mapping::miNumRecentKFs))
        return nullptr;

    int cnt = 0;
    int MaxIts = 1;
    kfptr pKF;

    while(!pKF && cnt < MaxIts)
    {
        //KF ID
        size_t min = 0; //this KF is the query KF, it's neighbors are candidates for culling -- so 0 and 1 can be considered
        size_t max = mnMaxKFid;
        size_t id = min + (rand() % (size_t)(max - min + 1));

        //Client ID
        size_t cid = MAPRANGE;
        if(msuAssClients.size() > 1)
        {
            min = 0;
            max = msuAssClients.size() - 1;
            size_t temp = min + (rand() % (size_t)(max - min + 1));

            set<size_t>::iterator sit = msuAssClients.begin();
            for(int it = 0;it < msuAssClients.size();++it)
            {
                if(it == temp)
                {
                    cid = *sit;
                    break;
                }
                else
                    ++sit;
            }
        }
        else
        {
            cid = *(msuAssClients.begin());
        }

        pKF = this->GetKfPtr(id,cid);
        ++cnt;
    }

    return pKF;
}

Map::mpptr Map::GetMpPtr(size_t MpId, size_t ClientId)
{
    unique_lock<mutex> lock(mMutexMap);

    idpair idp = make_pair(MpId,ClientId);
    std::map<idpair,mpptr>::iterator mit = mmpMapPoints.find(idp);
    if(mit != mmpMapPoints.end()) return mit->second;
    else return nullptr;
}

Map::kfptr Map::GetErasedKfPtr(size_t KfId, size_t ClientId)
{
    unique_lock<mutex> lock(mMutexMap);

    idpair idp = make_pair(KfId,ClientId);
    std::map<idpair,kfptr>::iterator mit = mmpErasedKeyFrames.find(idp);
    if(mit != mmpErasedKeyFrames.end()) return mit->second;
    else return nullptr;
}

Map::mpptr Map::GetErasedMpPtr(size_t MpId, size_t ClientId)
{
    unique_lock<mutex> lock(mMutexMap);

    idpair idp = make_pair(MpId,ClientId);
    std::map<idpair,mpptr>::iterator mit = mmpErasedMapPoints.find(idp);
    if(mit != mmpErasedMapPoints.end()) return mit->second;
    else return nullptr;
}

bool Map::IsKfDeleted(size_t KfId, size_t ClientId)
{
    unique_lock<mutex> lock2(mMutexMap);
    unique_lock<mutex> lock(mMutexErased);

    idpair idp = make_pair(KfId,ClientId);
    std::map<idpair,kfptr>::iterator mit = mmpErasedKeyFrames.find(idp);
    if(mit != mmpErasedKeyFrames.end()) return true;
    else return false;
}

bool Map::IsMpDeleted(size_t MpId, size_t ClientId)
{
    unique_lock<mutex> lock2(mMutexMap);
    unique_lock<mutex> lock(mMutexErased);

    idpair idp = make_pair(MpId,ClientId);
    std::map<idpair,mpptr>::iterator mit = mmpErasedMapPoints.find(idp);
    if(mit != mmpErasedMapPoints.end()) return true;
    else return false;
}

void Map::AddCCPtr(ccptr pCC)
{
    unique_lock<mutex> lock(this->mMutexCC);
    mspCC.insert(pCC);

    if(pCC->mpCH->GetCommPtr()) //when this is called during init procedure, mpCH->GetCommPtr() is still nullptr
        mspComm.insert(pCC->mpCH->GetCommPtr());
}

set<Map::ccptr> Map::GetCCPtrs()
{
    unique_lock<mutex> lock(this->mMutexCC);
    return mspCC;
}

bool Map::kftimecmp::operator ()(const kfptr pA, const kfptr pB) const
{
    return pA->mdInsertStamp > pB->mdInsertStamp;
}

bool Map::kftimecmpsmaller::operator ()(const kfptr pA, const kfptr pB) const
{
    return pA->mdInsertStamp < pB->mdInsertStamp;
}

void Map::FindLocalKFsByTime(kfptr pKFcur, set<kfptr> &sKfVicinity, priority_queue<int> &pqNativeKFs, list<kfptr> &lForeignKFs, int nLocalKFs)
{
    set<kfptr,kftimecmp> spKFsort;

    for(std::map<idpair,kfptr>::iterator mit_set = mmpKeyFrames.begin();mit_set!=mmpKeyFrames.end();++mit_set)
        spKFsort.insert(mit_set->second);

    //keep n newest KFs
    {
        int nMax = spKFsort.size(); //cannot use 'spKFsort.size()' in loop header because size continuously reduces in loop itself
        for(int it=0;it<nMax;++it)
        {
            if(spKFsort.empty())
                break;

            kfptr pKFi = *(spKFsort.begin());
            spKFsort.erase(pKFi);

            if(!pKFi || pKFi->isBad())
                continue;

            sKfVicinity.insert(pKFi);

            if(pKFi->mId.second == mMapId)
                pqNativeKFs.push(pKFi->mId.first);
            else
                lForeignKFs.push_back(pKFi);

            if(sKfVicinity.size() >= nLocalKFs)
                break;
        }
    }
}

void Map::MapTrimming(kfptr pKFcur)
{
    unique_lock<mutex> lock(mMutexMap);
    unique_lock<mutex> lock2(mMutexErased);

    int nLocalKFs = params::mapping::miLocalMapSize;
    int KfLimit = params::mapping::miLocalMapSize + params::mapping::miLocalMapBuffer;

    if(mmpKeyFrames.size() <= nLocalKFs)
        return;

    set<kfptr> sKfVicinity;
    set<mpptr> sMpVicinity;
    sKfVicinity.insert(pKFcur);

    //for KF Limit
    priority_queue<int> pqNativeKFs;
    list<kfptr> lForeignKFs;

    pKFcur->UpdateConnections(true);

    this->FindLocalKFsByTime(pKFcur,sKfVicinity,pqNativeKFs,lForeignKFs,nLocalKFs);

    //add MPs included by the KFs
    for(set<kfptr>::iterator sit = sKfVicinity.begin();sit!=sKfVicinity.end();++sit)
    {
        kfptr pKFi = *sit;
        vector<mpptr> vMPs = pKFi->GetMapPointMatches();

        sMpVicinity.insert(vMPs.begin(),vMPs.end());
    }

    //find & erase KFs

    for(map<idpair,kfptr>::iterator mit = mmpKeyFrames.begin();mit!=mmpKeyFrames.end();)
    {
        kfptr pKFi = mit->second;
        bool bErase = false;

        if(!sKfVicinity.count(pKFi))
        {
            if(pKFi->CanBeForgotten() || pKFi->mId.second != this->mMapId || pKFi->isBad())
                bErase = true;
        }

        if(bErase)
        {
            if(!pKFi->isBad())
                pKFi->SetBadFlag(true);

            if(!pKFi->isBad())
            {
                cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << ":" << __LINE__ << " SetBadFlag() called, but KF not set to bad" << endl;
                throw infrastructure_ex();
            }

            if(pKFi->mId.second == mMapId) mmpErasedKeyFrames[pKFi->mId] = pKFi;

            mit = mmpKeyFrames.erase(mit);
        }
        else
            ++mit;
    }

    //find & erase MPs
    for(map<idpair,mpptr>::iterator mit = mmpMapPoints.begin();mit!=mmpMapPoints.end();)
    {
        mpptr pMPi = mit->second;
        bool bErase = false;

        if(!sMpVicinity.count(pMPi))
        {
            if(pMPi->CanBeForgotten() || pMPi->mId.second != this->mMapId || pMPi->isBad())
                bErase = true;
        }

        if(bErase || pMPi->isBad())
        {
            if(!pMPi->isBad())
                pMPi->SetBadFlag(true);

            if(pMPi->mId.second == mMapId) mmpErasedMapPoints[pMPi->mId] = pMPi;

            mit = mmpMapPoints.erase(mit);

            if(mspMPsToErase.count(pMPi))
                mspMPsToErase.erase(pMPi);
        }
        else
        {
            ++mit;
        }
    }

    for(set<mpptr>::iterator sit = mspMPsToErase.begin();sit != mspMPsToErase.end();)
    {
        mpptr pMPi = *sit;

        if(pMPi)
        {
            std::map<idpair,mpptr>::iterator mit2 = mmpMapPoints.find(pMPi->mId);
            if(mit2 != mmpMapPoints.end())
                mmpMapPoints.erase(mit2);
        }

        sit = mspMPsToErase.erase(sit);
    }

    if(!mspMPsToErase.empty())
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << ":" << __LINE__ << " mspMPsToErase !empty()" << endl;
        throw infrastructure_ex();
    }

    //------------------------
    //--- Enforce KF Limit ---
    //------------------------

    if(mmpKeyFrames.size() > KfLimit)
    {
        cout << "+++++ Enforcing KF upper limit +++++" << ros::Time::now().toSec() << endl;

        set<kfptr,kftimecmpsmaller>spKFsmintime;
        for(std::map<idpair,kfptr>::iterator mit_set = mmpKeyFrames.begin();mit_set!=mmpKeyFrames.end();++mit_set)
            spKFsmintime.insert(mit_set->second);

        while(mmpKeyFrames.size() > KfLimit)
        {
            if(!lForeignKFs.empty())
            {
                kfptr pKFi = lForeignKFs.front();

                //can always be forgotten, comes from server
                if(pKFi->mId.second == mMapId)
                    mmpErasedKeyFrames[pKFi->mId] = pKFi;

                std::map<idpair,kfptr>::iterator mit = mmpKeyFrames.find(pKFi->mId);
                if(mit != mmpKeyFrames.end()) mmpKeyFrames.erase(mit);

                lForeignKFs.pop_front();

                spKFsmintime.erase(pKFi);
            }
            else if(!mmpKeyFrames.empty())
            {
                kfptr pKFi = *(spKFsmintime.begin());

                if(pKFi->mId.first == 0)
                {
                    //do nothing -- never delete 0
                    spKFsmintime.erase(pKFi);
                }
                else
                {
                    pKFi->SetBadFlag(true);

                    if(pKFi->mId.second == mMapId) mmpErasedKeyFrames[pKFi->mId] = pKFi;

                    std::map<idpair,kfptr>::iterator mit = mmpKeyFrames.find(pKFi->mId);
                    if(mit != mmpKeyFrames.end())
                        mmpKeyFrames.erase(mit);
                    else
                        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << ":" << __LINE__ << " MP ins mspKeyFrames, but not in mspKeyFrames" << endl;

                    spKFsmintime.erase(pKFi);
                }
            }
            else
                cout << "\033[1;33m!!! WARN !!!\033[0m " << __func__ << ": only current KF left, and the ones that cannot be erased" << endl;
        }

        //find MPs included by the KFs
        set<mpptr> spMPsToKeep;
        for(map<idpair,kfptr>::iterator mit = mmpKeyFrames.begin();mit!=mmpKeyFrames.end();++mit)
        {
            kfptr pKFi = mit->second;

            vector<mpptr> vMPs = pKFi->GetMapPointMatches();

            for(vector<mpptr>::iterator vit = vMPs.begin();vit!=vMPs.end();++vit)
            {
                mpptr pMPi = *vit;
                if(!pMPi || pMPi->isBad()) continue;

                spMPsToKeep.insert(pMPi);
            }
        }

        //delete other MPs
        for(map<idpair,mpptr>::iterator mit = mmpMapPoints.begin();mit!=mmpMapPoints.end();)
        {
            mpptr pMPi = mit->second;

            if(!spMPsToKeep.count(pMPi))
            {
                pMPi->SetBadFlag(true);

                if(pMPi->mId.second == mMapId) mmpErasedMapPoints[pMPi->mId] = pMPi;
                mit = mmpMapPoints.erase(mit);
            }
            else
                ++mit;
        }

//        std::cout << "KFs after erasing: " << mmpKeyFrames.size() << std::endl;
    }
}

void Map::PackVicinityToMsg(kfptr pKFcur, ccmslam_msgs::Map &msgMap, ccptr pCC)
{
    if(pKFcur->mId.first == 0)
        return; //we do not send the origin KFs
    //max == -1 means there is no upper limit
    int max = params::comm::server::miKfLimitToClient;

    unique_lock<mutex> lock(mMutexMap);
    unique_lock<mutex> lock2(mMutexErased);

    set<kfptr> sKfVicinity;
    set<mpptr> sMpVicinity;
    sKfVicinity.insert(pKFcur);
    set<kfptr> spKFsAddedLastIteration;
    spKFsAddedLastIteration.insert(pKFcur);

    pKFcur->UpdateConnections(true);

    //add KFs from CovGraph
    int depth = max;
    for(int it = 1;it<=depth;++it)
    {
        set<kfptr> spAdd; //do not insert into sKfVicinity while iterating it

        for(set<kfptr>::iterator sit = spKFsAddedLastIteration.begin();sit!=spKFsAddedLastIteration.end();++sit)
        {
            kfptr pKFi = *sit;
            if(pKFi->mId.first == 0) continue; //we do not send the origin KFs

            vector<KeyFrame::kfptr> vCovKfs = pKFi->GetVectorCovisibleKeyFrames();

            for(vector<kfptr>::iterator vit = vCovKfs.begin();vit!=vCovKfs.end();++vit)
            {
                kfptr pKFj = *vit;
                if(!pKFj || pKFj->isBad()) continue;
                if(pKFj->mId.first == 0) continue; //we do not send the origin KFs
                spAdd.insert(pKFj);

                if((max != -1) && (sKfVicinity.size() + spAdd.size()) >= max)
                {
                    break;
                }
            }

            if((max != -1) && (sKfVicinity.size() + spAdd.size()) >= max)
                break;
        }

        sKfVicinity.insert(spKFsAddedLastIteration.begin(),spKFsAddedLastIteration.end());
        spKFsAddedLastIteration = spAdd;

        if((max != -1) && (sKfVicinity.size() >= max))
            break;

        if(spKFsAddedLastIteration.empty())
            break;
    }

    //add MPs included by the KFs
    for(set<kfptr>::iterator sit = sKfVicinity.begin();sit!=sKfVicinity.end();++sit)
    {
        kfptr pKFi = *sit;
        vector<mpptr> vMPs = pKFi->GetMapPointMatches();

        for(vector<mpptr>::iterator vit = vMPs.begin();vit!=vMPs.end();++vit)
        {
            mpptr pMPi = *vit;
            if(!pMPi || pMPi->isBad()) continue;

            sMpVicinity.insert(pMPi);
        }
    }

    pKFcur->ConvertToMessage(msgMap,pCC->mg2oS_wcurmap_wclientmap,pKFcur); //make sure this is first KF in vector

    for(set<kfptr>::iterator sit = sKfVicinity.begin();sit!=sKfVicinity.end();++sit)
    {
        kfptr pKFi = *sit;

        if(pKFi->mId == pKFcur->mId)
            continue; //do not enter twice

        pKFi->ConvertToMessage(msgMap,pCC->mg2oS_wcurmap_wclientmap,pKFcur);
    }

    for(set<mpptr>::iterator sit = sMpVicinity.begin();sit != sMpVicinity.end();++sit)
    {
        mpptr pMPi = *sit;
        pMPi->ConvertToMessage(msgMap,pKFcur,pCC->mg2oS_wcurmap_wclientmap);
    }
}

void Map::HandleMissingParent(size_t QueryId, size_t QueryCId, Mat &T_cref_cquery, kfptr pRefKf = nullptr)
{
    //If a KF/MP gets a relative pos/pose to a KF from a msg, and it cannot find this KF, this method searches another KF that can be used

    idpair QId = make_pair(QueryId,QueryCId);
    kfptr pParent,pNewParent;

    unique_lock<mutex> lock2(mMutexMap);
    unique_lock<mutex> lock(mMutexErased);

    std::map<idpair,kfptr>::iterator mit = mmpErasedKeyFrames.find(QId);

    if(mit == mmpErasedKeyFrames.end())
    {
        //this would be a problem...
    }
    else
    {
        pParent = mit->second;
    }

    if(!pParent)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << ": Cannot find parent from msg" << endl;
        cout << "parent KF: " << QueryId << "|" << QueryCId << endl;
        cout << "KFs in map: " << mmpKeyFrames.size() << endl;
        throw estd::infrastructure_ex();
    }

    pNewParent = pParent->GetParent();
    cv::Mat Tcq_cnewparent = pParent->GetTcp();
    while(pNewParent->isBad())
    {
        if(!pNewParent)
        {
            cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << ": Cannot find parent from msg" << endl;
            throw estd::infrastructure_ex();
        }

        Tcq_cnewparent = Tcq_cnewparent * pNewParent->GetTcp();
        pNewParent = pNewParent->GetParent();
    }

    if(!pRefKf)
    {
        //called from constructor - needs RefKf
        pRefKf = pNewParent;
        T_cref_cquery = Tcq_cnewparent.inv();
    }
    else
    {
        //has already RefKf, an needs TF from parent specified in msg

        T_cref_cquery = pRefKf->GetPose() * pNewParent->GetPoseInverse() * Tcq_cnewparent.inv();
    }
}

Map::kfptr Map::GetPredecessor(kfptr pKF)
{
    kfptr pPred;
    size_t kfid = pKF->mId.first;
    while(!pPred)
    {
        kfid--;

        if(kfid == -1)
        {
            cout << "\033[1;31m!!!!! FATAL !!!!!\033[0m " << __func__ << __LINE__ << " cannot find predecessor" << endl;
            cout << "KF ID: " << pKF->mId.first << "|" << pKF->mId.second << endl;
            cout << "In map: " << this->mnMaxKFid << endl;
            cout << "Workaround: take first KF (id 0)" << endl;
            pPred = this->GetKfPtr(0,mMapId);
            cout << "get KF: 0|" << mMapId <<" -- nullptr? " << (int)!pPred << endl;
        }
        else
        {
            pPred = this->GetKfPtr(kfid,pKF->mId.second);
        }
    }

    return pPred;
}

void Map::ClearBadMPs()
{
    unique_lock<mutex> lock(mMutexMap);
    unique_lock<mutex> lock2(mMutexErased);

    for(set<mpptr>::iterator sit = mspMPsToErase.begin();sit != mspMPsToErase.end();)
    {
        mpptr pMPi = *sit;

        if(pMPi)
        {
            map<idpair,mpptr>::iterator mit2 = mmpMapPoints.find(pMPi->mId);
            if(mit2 != mmpMapPoints.end())
                mmpMapPoints.erase(mit2);
        }

        sit = mspMPsToErase.erase(sit);
    }
}

Map::ccptr Map::GetCCPtr(size_t nClientId)
{
    for(set<ccptr>::iterator sit = mspCC.begin();sit != mspCC.end(); ++sit)
    {
        ccptr pCC = *sit;
        if(pCC->mClientId == nClientId)
            return pCC;
    }

    cout << COUTERROR << "no ccptr found for query-ID" << endl;
    return nullptr;
}

void Map::StopGBA()
{
    {
        //prevent two handlers from stopping GBA at the same time
        unique_lock<mutex> lockStopGBAInProgress(mMutexStopGBAInProgess);
        if(mbStopGBAInProgress)
            return;
        else
            mbStopGBAInProgress = true;
    }

    cout << "Map " << mMapId << ": Stop GBA" << endl;
    if(this->isRunningGBA())
    {
        #ifdef DONOTINTERRUPTMERGE
        if(this->isMergeStepGBA())
        {
            cout << "Map " << mMapId << ": GBA stop declined -- MergeGBA" << endl;
            unique_lock<mutex> lockStopGBAInProgress(mMutexStopGBAInProgess);
            mbStopGBAInProgress = false;
            return;
        }
        #endif

        this->mbStopGBA = true;

        while(!this->isFinishedGBA())
            usleep(5000);

        this->mpThreadGBA->join();
        delete this->mpThreadGBA;
    }
    else
    {
        cout << COUTERROR << "called w/o GBA running -- Map " << mMapId << endl;
    }

    {
        unique_lock<mutex> lockStopGBAInProgress(mMutexStopGBAInProgess);
        mbStopGBAInProgress = false;
    }
    cout << "Map " << mMapId << ": GBA Stopped" << endl;
}

void Map::RequestBA(size_t nClientId)
{
    if(this->isRunningGBA())
    {
        cout << "Denied -- GBA running" << endl;
        return;
    }

    if(this->isNoStartGBA())
    {
        cout << "Denied -- NoStartGBA" << endl;
        return;
    }

    #ifdef FINALBA
    if(!this->isGBAinterrupted())
        cout << COUTERROR << "Agent " << nClientId << " requesting BA, but was not interrupted" << endl;
    #endif

    msnFinishedAgents.insert(nClientId);

    if(msnFinishedAgents.size() == msuAssClients.size())
    {
        bool b0 = false;
        bool b1 = false;
        bool b2 = false;
        bool b3 = false;

        for(set<ccptr>::iterator sit = mspCC.begin();sit!=mspCC.end();++sit)
        {
            ccptr pCC = *sit;

            if(pCC->mClientId > 3) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId out of bounds (" << pCC->mClientId << ")" << endl;
            if(!(this->msuAssClients.count(pCC->mClientId))) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId in pCC but not in msuAssClients" << endl;
            switch(pCC->mClientId)
            {
                case(static_cast<size_t>(0)):
                    if(b0) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId found twice" << endl;
                    b0 = true;
                    #ifdef LOGGING
//                    pCC->mpLogger->SetMappingLock(__LINE__,pCC->mClientId);
                    #endif
                    while(!pCC->LockMapping()){usleep(params::timings::miLockSleep);}
                    break;
                case(static_cast<size_t>(1)):
                    if(b1) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId found twice" << endl;
                    b1 = true;
                    #ifdef LOGGING
//                    pCC->mpLogger->SetMappingLock(__LINE__,pCC->mClientId);
                    #endif
                    while(!pCC->LockMapping()){usleep(params::timings::miLockSleep);}
                    break;
                case(static_cast<size_t>(2)):
                    if(b2) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId found twice" << endl;
                    b2 = true;
                    #ifdef LOGGING
//                    pCC->mpLogger->SetMappingLock(__LINE__,pCC->mClientId);
                    #endif
                    while(!pCC->LockMapping()){usleep(params::timings::miLockSleep);}
                    break;
                case(static_cast<size_t>(3)):
                    if(b3) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId found twice" << endl;
                    b3 = true;
                    #ifdef LOGGING
//                    pCC->mpLogger->SetMappingLock(__LINE__,pCC->mClientId);
                    #endif
                    while(!pCC->LockMapping()){usleep(params::timings::miLockSleep);}
                    break;
                default: cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId out of bounds" << endl;
            }

            pCC->mbOptActive = true;
        }

        idpair nLoopKF = make_pair(mnLastKfIdUnique,mMapId);

        this->setRunningGBA();
        this->setFinishedGBA();
        this->mbStopGBA = false;

        #ifdef DEBUGGING2
        this->CheckStructure();
        #endif

        // Launch a new thread to perform Global Bundle Adjustment
        this->mpThreadGBA = new thread(&Map::RunGBA,this,nLoopKF);

        std::cout << "Map: Wait for GBA to finish" << std::endl;
        while(this->isRunningGBA()) {
            usleep(10000);
        }
        std::cout << "Map: GBA finished - continue" << std::endl;
    }
    else
    {
        cout << "msuAssClient: " << endl;
        for(set<size_t>::iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit;
        cout << endl;

        cout << "msuFinishedAgents: " << endl;
        for(set<size_t>::iterator sit = msnFinishedAgents.begin();sit!=msnFinishedAgents.end();++sit)
            cout << *sit;
        cout << endl;
    }
}

void Map::RunGBA(idpair nLoopKF)
{
    cout << "-> Starting Global Bundle Adjustment" << endl;

    Optimizer::MapFusionGBA(shared_from_this(),this->mMapId,params::opt::mGBAIterations,&(this->mbStopGBA),nLoopKF,true);

    #ifdef FINALBA
    if(!this->mbStopGBA)
    #endif
    {
        unique_lock<mutex> lock(this->mMutexGBA);

        this->LockMapUpdate();

        cout << "-> Global Bundle Adjustment finished" << endl;
        cout << "-> Updating map ..." << endl;

        // Correct keyframes starting at map first keyframe
        list<kfptr> lpKFtoCheck(this->mvpKeyFrameOrigins.begin(),this->mvpKeyFrameOrigins.end());

        #ifdef DEBUGGING2
        std::cout << "Map Origins: " << std::endl;
        for(list<kfptr>::iterator litcheck=lpKFtoCheck.begin();litcheck!=lpKFtoCheck.end();++litcheck)
        {
            kfptr pKFcheck = *litcheck;
            std::cout << "KF " << pKFcheck->mId.first << "|" << pKFcheck->mId.second << std::endl;
        }
        #endif

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
            lpKFtoCheck.pop_front();
        }

        cout << "--> Updating MPs ..." << endl;

        // Correct MapPoints
        const vector<mpptr> vpMPs = this->GetAllMapPoints();

        for(size_t i=0; i<vpMPs.size(); i++)
        {
            mpptr pMP = vpMPs[i];

            if(pMP->isBad())
                continue;

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
                    std::cout << COUTERROR << " KF" << pRefKF->mId.first << "|" << pRefKF->mId.second << ": !(pRefKF->mTcwBefGBA.dims >= 2)" << " -- bad: " << (int)pRefKF->isBad() << std::endl;
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
        this->unsetGBAinterrupted();
        #endif

        this->UnLockMapUpdate();
    }
    #ifdef FINALBA
    else
    {
        cout << COUTNOTICE << "GBA interrupted" << endl;
        this->setGBAinterrupted();
    }
    #endif

    if(params::stats::mbWriteKFsToFile)
    {
        for(int it=0;it<4;++it)
        {
            std::stringstream ss;
            ss << params::stats::msOutputDir << "KF_GBA_" << it << ".csv";
            this->WriteStateToCsv(ss.str(),it);
        }
    }

    this->setFinishedGBA();
    this->unsetRunningGBA();

    for(set<ccptr>::iterator sit = mspCC.begin();sit!=mspCC.end();++sit)
    {
        ccptr pCC = *sit;
        pCC->UnLockMapping();

        pCC->mbOptActive = false;
    }

    cout << "-> Leave Thread" << endl;
}

void Map::WriteStateToCsv(const std::string& filename,
                          const size_t clientId) {
  std::vector<kfptr> foundKFs;
  foundKFs.reserve(mmpKeyFrames.size());
  // Get all frames from the required client
  for (std::map<idpair,kfptr>::const_iterator itr = mmpKeyFrames.begin();
       itr != mmpKeyFrames.end(); ++itr) {
    kfptr pKF = itr->second;
    idpair currID = itr->first;
    if (currID.second == clientId) {
      foundKFs.push_back(pKF);
    }
  }

  if(foundKFs.empty()) //would overwrite files from other maps with empty files
      return;

  // Sort the keyframes by timestamp
  std::sort(foundKFs.begin(), foundKFs.end(), KeyFrame::compKFstamp);

  // Write out the keyframe data
  std::ofstream keyframesFile;
  keyframesFile.open(filename);
  for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); ++itr) {
    kfptr pKF = (*itr);
    const double stamp = pKF->mTimeStamp;
    Eigen::Vector3d bA = Eigen::Vector3d::Zero();
    Eigen::Vector3d bG = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    const cv::Mat T_wc = pKF->GetPoseInverse();
    const Eigen::Matrix4d eT_wc = Converter::toMatrix4d(T_wc);
    ccptr pCC = *(mspCC.begin());
    Eigen::Matrix4d T_SC;
    if(mSysState == SERVER)
        T_SC = pKF->mT_SC;
    else
        T_SC = pCC->mT_SC;

    const Eigen::Matrix4d Tws = eT_wc * T_SC.inverse();
    const Eigen::Quaterniond q(Tws.block<3,3>(0,0));

    if(params::stats::miTrajectoryFormat == 0) {
        keyframesFile << std::setprecision(25) << stamp * 1e9f << ",";
        keyframesFile << Tws(0,3) << "," << Tws(1,3) << "," << Tws(2,3) << ",";
        keyframesFile << q.w() << "," << q.x() << "," << q.y() << "," << q.z() << ",";
        keyframesFile << vel[0] << "," << vel[1] << "," << vel[2] << ",";
        keyframesFile << bG[0] << "," << bG[1] << "," << bG[2] << ",";
        keyframesFile << bA[0] << "," << bA[1] << "," << bA[2] << std::endl;
    } else if(params::stats::miTrajectoryFormat == 1) {
        keyframesFile << std::setprecision(25) << stamp << " ";
        keyframesFile << Tws(0,3) << " " << Tws(1,3) << " " << Tws(2,3) << " ";
        keyframesFile << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    } else {
        std::cout << COUTFATAL << "miTrajectoryFormat '" << params::stats::miTrajectoryFormat << "' not in { 0=EUROC | 1=TUM }" << std::endl;
        exit(-1);
    }
  }
  keyframesFile.close();

  std::cout << "KFs written to file" << std::endl;
}

#ifdef DEBUGGING2
void Map::CheckStructure()
{
    std::cout << "+++++ Check Map Structure +++++" << std::endl;

    std::cout << "--- KFs ---" << std::endl;

    for(set<kfptr>::iterator sit = mspKeyFrames.begin();sit!=mspKeyFrames.end();++sit)
    {
        kfptr pKF = *sit;

        if(pKF->isBad())
            std::cout << COUTWARN << "KF " << pKF->GetId() << ": bad but in map" << std::endl;

        kfptr pKFp = pKF->GetParent();

        if(!pKFp)
        {
            if(pKF->mId.first == 0)
                continue; //ok
            else
            {
                std::cout << "KF " << pKF->GetId() << ": no parent" << std::endl;

                kfptr pNewParent;
                size_t newid = pKF->mId.first-1;

                pNewParent = this->GetKfPtr(newid,pKF->mId.second);

                while(!pNewParent)
                {
                    newid += -1;
                    pNewParent = this->GetKfPtr(newid,pKF->mId.second);
                }

                if(!pNewParent)
                {
                    std::cout << COUTFATAL << "KF " << pKF->GetId() << ": Cannot find new parent - quitting" << std::endl;
                    throw estd::infrastructure_ex();
                }

                pKF->ChangeParent(pNewParent);
            }
        }
        else
        {
            if(pKFp->mId == pKF->mId)
            {
                std::cout << COUTERROR << "KF " << pKF->GetId() << ": is its own parent" << std::endl;
                std::cout << "Same ptr? " << (int)(pKF == pKFp) << std::endl;

                kfptr pNewParent;
                size_t newid = pKF->mId.first-1;

                pNewParent = this->GetKfPtr(newid,pKF->mId.second);

                while(!pNewParent)
                {
                    newid += -1;
                    pNewParent = this->GetKfPtr(newid,pKF->mId.second);
                }

                if(!pNewParent)
                {
                    std::cout << COUTFATAL << "KF " << pKF->GetId() << ": Cannot find new parent - quitting" << std::endl;
                    throw estd::infrastructure_ex();
                }

                pKF->ChangeParent(pNewParent);
            }
            else
                continue; //ok
        }

    }

    std::cout << "--- MPs ---" << std::endl;

    for(set<mpptr>::iterator sit = mspMapPoints.begin();sit!=mspMapPoints.end();++sit)
    {
        mpptr pMP = *sit;

        if(pMP->isBad())
        {
            std::cout << COUTWARN << "MP " << pMP->GetId() << ": bad but in map" << std::endl;

            this->EraseMapPoint(pMP);
        }

        kfptr pKFp = pMP->GetReferenceKeyFrame();

        if(!pKFp)
        {
                std::cout << "MP " << pMP->GetId() << ": no parent" << std::endl;

                map<kfptr,size_t> observations = pMP->GetObservations();

                if(observations.size() < 2)
                {
                    std::cout << "MP " << pMP->GetId() << ": only " << observations.size() << " observation" << std::endl;

                    pMP->SetBadFlag();

                    continue;
                }

                kfptr pNewParent;

                for(map<kfptr,size_t>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
                {
                    pNewParent = mit->first;

                    if(pNewParent)
                        break;
                }

                if(!pNewParent)
                {
                    std::cout << COUTFATAL << "MP " << pMP->GetId() << ": Cannot find new parent - quitting" << std::endl;
                    throw estd::infrastructure_ex();
                }

                pMP->SetReferenceKeyFrame(pNewParent);
        }
    }

    std::cout << "--- DONE ---" << std::endl;
}
#endif

} //end ns
