##List of Known Dependencies
###CCM-SLAM version 1.0

In this document we list, to our best knowledge, all the pieces of code included by CCM-SLAM and linked libraries which are not property of the authors of CCM-SLAM.

#####Code in **src** and **include** folders

* *ORB-SLAM2*  
Parts of CMM-SLAM are based on modules of [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2), which are re-used in modified or partially in unmodified condition. The original code is GPLv3 licensed.

* *ORBextractor.cc*.  
This is a modified version of orb.cpp of OpenCV library. The original code is BSD licensed.

* *PnPsolver.h, PnPsolver.cc*.  
This is a modified version of the epnp.h and epnp.cc of Vincent Lepetit. 
This code can be found in popular BSD licensed computer vision libraries as [OpenCV](https://github.com/Itseez/opencv/blob/master/modules/calib3d/src/epnp.cpp) and [OpenGV](https://github.com/laurentkneip/opengv/blob/master/src/absolute_pose/modules/Epnp.cpp). The original code is FreeBSD.

* Function *ORBmatcher::DescriptorDistance* in *ORBmatcher.cc*.  
The code is from: http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel.
The code is in the public domain.

#####Code in Thirdparty folder

* All code in **DBoW2** folder.  
This is a modified version of [DBoW2](https://github.com/dorian3d/DBoW2) and [DLib](https://github.com/dorian3d/DLib) library. All files included are BSD licensed.

* All code in **g2o** folder.  
This is a modified version of [g2o](https://github.com/RainerKuemmerle/g2o). All files included are BSD licensed.

#####Library dependencies 

* **OpenCV**.  
BSD license.

* **Eigen3**.  
For versions greater than 3.1.1 is MPL2, earlier versions are LGPLv3.

* **PCL**.  
BSD license.

* **ROS**.  
BSD license. 
CCM-SLAM Messages dependencies: catkin, roscpp, std_msgs, geometry_msgs, sensor_msgs, message_generation, message_runtime -- all BSD licensed.
CCM-SLAM dependencies: catkin, cmake_modules, roscpp, cv_bridge, tf, pcl_ros, tf_conversions, image_transport -- all BSD licensed.
