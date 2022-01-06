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

#include <cslam/config.h>

using namespace std;

namespace params {

void ShowParams()
{
    cout << "++++++++++ System ++++++++++" << endl;
    cout << "miLockSleep: " << params::sys::mbStrictLock << endl;
    cout << endl;
    std::cout << "++++++++++ Stats ++++++++++" << std::endl;
    std::cout << "mbWriteKFsToFile: " << (int)params::stats::mbWriteKFsToFile << std::endl;
    std::cout << "msOutputDir: " << params::stats::msOutputDir << std::endl;
    std::cout << "miTrajectoryFormat: " << params::stats::miTrajectoryFormat << std::endl;
    std::cout << std::endl;
    cout << "++++++++++ Timings ++++++++++" << endl;
    cout << "miLockSleep: " << params::timings::miLockSleep << endl;
    cout << "----- Client -----" << endl;
    cout << "miRosRate: " << params::timings::client::miRosRate << endl;
    cout << "miViewerRate: " << params::timings::client::miViewerRate << endl;
    cout << "miMappingRate: " << params::timings::client::miMappingRate << endl;
    cout << "miCommRate: " << params::timings::client::miCommRate << endl;
    cout << "----- Server -----" << endl;
    cout << "miRosRate: " << params::timings::server::miRosRate << endl;
    cout << "miViewerRate: " << params::timings::server::miViewerRate << endl;
    cout << "miMappingRate: " << params::timings::server::miMappingRate << endl;
    cout << "miCommRate: " << params::timings::server::miCommRate << endl;
    cout << "miPlaceRecRateRate: " << params::timings::server::miPlaceRecRateRate << endl;
    cout << endl;
    cout << "++++++++++ Tracking ++++++++++" << endl;
    cout << "miInitKFs: " << params::tracking::miInitKFs << endl;
    cout << "miMinFrames: " << params::tracking::miMinFrames << endl;
    cout << "miMaxFrames: " << params::tracking::miMaxFrames << endl;
    cout << "miMatchesInliersThres: " << params::tracking::miMatchesInliersThres << endl;
    cout << "mfThRefRatio: " << params::tracking::mfThRefRatio << endl;
    //Tracking Functions Inlier Thresholds
    std::cout << "miTrackWithRefKfInlierThresSearch: " << params::tracking::miTrackWithRefKfInlierThresSearch << std::endl;
    std::cout << "miTrackWithRefKfInlierThresOpt: " << params::tracking::miTrackWithRefKfInlierThresOpt << std::endl;
    std::cout << "miTrackWithMotionModelInlierThresSearch: " << params::tracking::miTrackWithMotionModelInlierThresSearch << std::endl;
    std::cout << "miTrackWithMotionModelInlierThresOpt: " << params::tracking::miTrackWithMotionModelInlierThresOpt << std::endl;
    std::cout << "miTrackLocalMapInlierThres: " << params::tracking::miTrackLocalMapInlierThres << std::endl;
    cout << endl;
    cout << "++++++++++ Mapping ++++++++++" << endl;
    cout << "mfRedundancyThres: " << params::mapping::mfRedundancyThres << endl;
    cout << "miKfLimitClient: " << params::mapping::miLocalMapSize << endl;
    cout << "miKfBufferClient: " << params::mapping::miLocalMapBuffer << endl;
    cout << "miNumRecentKFs: " << params::mapping::miNumRecentKFs << endl;
    cout << endl;
    cout << "++++++++++ Comm ++++++++++" << endl;
    cout << "----- Client -----" << endl;
    cout << "mfPubFreq: " << params::comm::client::mfPubFreq << endl;
    cout << "mfPubPeriodicTime: " << params::comm::client::mfPubPeriodicTime << endl;
    cout << "miPubMapBufferSize: " << params::comm::client::miPubMapBufferSize << endl;
    cout << "miSubMapBufferSize: " << params::comm::client::miSubMapBufferSize << endl;
    cout << "miKfItBound: " << params::comm::client::miKfItBound << endl;
    cout << "miMpItBound: " << params::comm::client::miMpItBound << endl;
    cout << "miKfPubMax: " << params::comm::client::miKfPubMax << endl;
    cout << "miMpPubMax: " << params::comm::client::miMpPubMax << endl;
    cout << "----- Server -----" << endl;
    cout << "mfPubFreq: " << params::comm::server::mfPubFreq << endl;
    cout << "mfPubPeriodicTime: " << params::comm::server::mfPubPeriodicTime << endl;
    cout << "miKfLimitToClient: " << params::comm::server::miKfLimitToClient << endl;
    cout << "miPubMapBufferSize: " << params::comm::server::miPubMapBufferSize << endl;
    cout << "miSubMapBufferSize: " << params::comm::server::miSubMapBufferSize << endl;
    cout << "miKfItBound: " << params::comm::server::miKfItBound << endl;
    cout << "miMpItBound: " << params::comm::server::miMpItBound << endl;
    cout << endl;
    cout << "++++++++++ PlaceRec ++++++++++" << endl;
    cout << "miNewLoopThres: " << params::placerec::miNewLoopThres << endl;
    cout << "miStartMapMatchingAfterKf: " << params::placerec::miStartMapMatchingAfterKf << endl;
    cout << "miCovisibilityConsistencyTh: " << params::placerec::miCovisibilityConsistencyTh << endl;
    cout << endl;
    cout << "++++++++++ Vis ++++++++++" << endl;
    cout << "mbVisMode: " << (int)params::vis::mbActive << endl;
    cout << "mbShowCovGraph: " << (int)params::vis::mbShowCovGraph << endl;
    cout << "mbShowMPs: " << (int)params::vis::mbShowMPs << endl;
    cout << "mbShowTraj: " << (int)params::vis::mbShowTraj << endl;
    cout << "mbShowKFs: " << (int)params::vis::mbShowKFs << endl;
    cout << "miCovGraphMinFeats: " << params::vis::miCovGraphMinFeats << endl;
    cout << "mfScaleFactor: " << params::vis::mfScaleFactor << endl;
    cout << "mfTrajMarkerSize: " << params::vis::mfTrajMarkerSize << endl;
    cout << "mfCovGraphMarkerSize: " << params::vis::mfCovGraphMarkerSize << endl;
    cout << "mfLoopMarkerSize: " << params::vis::mfLoopMarkerSize << endl;
    cout << "mfMarkerSphereDiameter: " << params::vis::mfMarkerSphereDiameter << endl;
    cout << "mfCamSize: " << params::vis::mfCamSize << endl;
    cout << "mfCamLineSize: " << params::vis::mfCamLineSize << endl;
    cout << endl;
    cout << "++++++++++ Opt ++++++++++" << endl;
    cout << "mSolverIterations: " << params::opt::mSolverIterations << endl;
    cout << "mMatchesThres: " << params::opt::mMatchesThres << endl;
    cout << "mInliersThres: " << params::opt::mInliersThres << endl;
    cout << "mTotalMatchesThres: " << params::opt::mTotalMatchesThres << endl;
    cout << "mProbability: " << params::opt::mProbability << endl;
    cout << "mMinInliers: " << params::opt::mMinInliers << endl;
    cout << "mMaxIterations: " << params::opt::mMaxIterations << endl;
    cout << "mGBAIterations: " << params::opt::mGBAIterations << endl;
    cout << "miEssGraphMinFeats: " << params::opt::miEssGraphMinFeats << endl;
    cout << endl;
    cout << "++++++++++ Extractor ++++++++++" << endl;
    cout << "miNumFeatures: " << params::extractor::miNumFeatures << endl;
    cout << "mfScaleFactor: " << params::extractor::mfScaleFactor << endl;
    cout << "miNumLevels: " << params::extractor::miNumLevels << endl;
    cout << "miIniThFAST: " << params::extractor::miIniThFAST << endl;
    cout << "miNumThFAST: " << params::extractor::miNumThFAST << endl;
    cout << endl;
    cout << "++++++++++ Colors ++++++++++" << endl;
    cout << "R0: " << params::colors::mc0.mfR << endl;
    cout << "G0: " << params::colors::mc0.mfG << endl;
    cout << "B0: " << params::colors::mc0.mfB << endl;
    cout << "R1: " << params::colors::mc1.mfR << endl;
    cout << "G1: " << params::colors::mc1.mfG << endl;
    cout << "B1: " << params::colors::mc1.mfB << endl;
    cout << "R2: " << params::colors::mc2.mfR << endl;
    cout << "G2: " << params::colors::mc2.mfG << endl;
    cout << "B2: " << params::colors::mc2.mfB << endl;
    cout << "R3: " << params::colors::mc3.mfR << endl;
    cout << "G3: " << params::colors::mc3.mfG << endl;
    cout << "B3: " << params::colors::mc3.mfB << endl;
    cout << "Rcov: " << params::colors::mcCovGraph.mfR << endl;
    cout << "Gcov: " << params::colors::mcCovGraph.mfG << endl;
    cout << "Bcov: " << params::colors::mcCovGraph.mfB << endl;
    cout << endl;
}

} //end ns
