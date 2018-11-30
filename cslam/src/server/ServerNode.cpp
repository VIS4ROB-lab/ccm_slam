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


#include <cslam/server/ServerSystem.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "CSLAM server node");

    if(argc != 2)
    {
        cerr << endl << "Usage: rosrun cslam clientnode path_to_vocabulary" << endl;
        ros::shutdown();
        return 1;
    }

    ros::NodeHandle Nh;
    ros::NodeHandle NhPrivate("~");

    boost::shared_ptr<cslam::ServerSystem> pSSys{new cslam::ServerSystem(Nh,NhPrivate,argv[1])};
    pSSys->InitializeClients();


    ROS_INFO("started CSLAM server node...");

    ros::MultiThreadedSpinner MSpin(2);

        MSpin.spin();

    ros::waitForShutdown();

    return 0;
}
