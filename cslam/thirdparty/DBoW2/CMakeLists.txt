cmake_minimum_required(VERSION 2.8)
project(DBoW2)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Set both to FALSE for System OpenCV.
set(USE_THIRDPARTY_OPENCV3 0)
set(USE_THIRDPARTY_OPENCV2 0)

if(${USE_THIRDPARTY_OPENCV3} AND ${USE_THIRDPARTY_OPENCV2})
  message( FATAL_ERROR "ill-defined external OpenCV dependencies." )
endif()

set(HDRS_DBOW2
  DBoW2/BowVector.h
  DBoW2/FORB.h 
  DBoW2/FClass.h       
  DBoW2/FeatureVector.h
  DBoW2/ScoringObject.h   
  DBoW2/TemplatedVocabulary.h)
set(SRCS_DBOW2
  DBoW2/BowVector.cpp
  DBoW2/FORB.cpp      
  DBoW2/FeatureVector.cpp
  DBoW2/ScoringObject.cpp)

set(HDRS_DUTILS
  DUtils/Random.h
  DUtils/Timestamp.h)
set(SRCS_DUTILS
  DUtils/Random.cpp
  DUtils/Timestamp.cpp)

if(${USE_THIRDPARTY_OPENCV3})
set(OpenCV_DIR ${PROJECT_SOURCE_DIR}/../opencv-3.3.0/build)
set(OpenCV_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/../opencv-3.3.0/build/include)
find_package(OpenCV 3.3.0 REQUIRED)
elseif(${USE_THIRDPARTY_OPENCV2})
set(OpenCV_DIR ${PROJECT_SOURCE_DIR}/../opencv-2.4.13/build)
set(OpenCV_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/../opencv-2.4.13/build/include)
find_package(OpenCV 2.4.13







 REQUIRED)
else()
find_package(OpenCV REQUIRED)
endif()

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${OpenCV_INCLUDE_DIRS})
add_library(DBoW2 SHARED ${SRCS_DBOW2} ${SRCS_DUTILS})
target_link_libraries(DBoW2 ${OpenCV_LIBS})

