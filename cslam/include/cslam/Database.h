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

#ifndef CSLAM_DATABASE_H_
#define CSLAM_DATABASE_H_

//C++
#include <boost/shared_ptr.hpp>
#include <vector>
#include <list>
#include <set>
#include <mutex>

//CSLAM
#include <cslam/config.h>
#include <cslam/estd.h>
#include <cslam/Datatypes.h>
#include <cslam/ORBVocabulary.h>
#include <cslam/KeyFrame.h>
#include <cslam/Map.h>
#include <cslam/Frame.h>

#include <cslam/MapPoint.h>

//Thirdparty
#include <thirdparty/DBoW2/DBoW2/BowVector.h>

using namespace std;
using namespace estd;

namespace cslam {

//forward decs
class Map;
class Frame; //new
//------------

class KeyFrameDatabase
{
public:
    typedef boost::shared_ptr<KeyFrameDatabase> dbptr;
    typedef boost::shared_ptr<KeyFrame> kfptr;
    typedef boost::shared_ptr<Map> mapptr;
public:

   KeyFrameDatabase(const vocptr pVoc);

    void add(kfptr pKF);

    void erase(kfptr pKF);

    void clear();

   // Loop Detection
    vector<kfptr> DetectLoopCandidates(kfptr pKF, float minScore);
    vector<kfptr> DetectMapMatchCandidates(kfptr pKF, float minScore, mapptr pMap);

   // Relocalization
   std::vector<kfptr> DetectRelocalizationCandidates(Frame& F);

   //Debug
   typedef boost::shared_ptr<MapPoint> mpptr;
   map<idpair,mpptr> mmpMPs;
   map<idpair,bool> mmbDirectBad;
   void AddMP(mpptr pMP);
   void AddDirectBad(size_t id, size_t cid);
   bool FindMP(size_t id, size_t cid);
   bool FindDirectBad(size_t id, size_t cid);
   mutex mMutexMPs;
   void ResetMPs();

protected:

  // Associated vocabulary
  const vocptr mpVoc;

  // Inverted file
  std::vector<list<kfptr> > mvInvertedFile;

  // Mutex
  std::mutex mMutex;
};

} //end namespace

#endif
