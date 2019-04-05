CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

project(openeage)

set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11 -Wall -O2 -march=native" )

Set(EIGEN3_INCLUDE_DIR "/usr/local/include/eigen3")
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(OpenMVG REQUIRED)
include_directories(${OPENMVG_INCLUDE_DIRS})

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
SET( G2O_ROOT /usr/local/include/g2o )

find_package( G2O REQUIRED )
find_package( OpenCV REQUIRED )
find_package( Cholmod REQUIRED )

include_directories(${CHOLMOD_INCLUDE_DIR})

add_subdirectory(openMVG_utils)
add_subdirectory(utils)
  
###
# Image format converter 
###
ADD_EXECUTABLE(Image_Converter ./utils/image_converter.cpp)
TARGET_LINK_LIBRARIES(Image_Converter
  ${OpenCV_LIBS}
  )
  

###
# Intrinsic image analysis and SfM_Data container initialization
###
ADD_EXECUTABLE(openMVG_main_SfMInit_ImageListing ./openMVG_utils/main_SfMInit_ImageListing.cpp)
TARGET_LINK_LIBRARIES(openMVG_main_SfMInit_ImageListing
  openMVG_system
  openMVG_image
  openMVG_features
  openMVG_sfm
  easyexif
  )


###
# Add executable that computes:
# - openMVG_main_ComputeFeatures: features and descriptors
# - openMVG_main_ComputeMatches: putative matches + geometric filtered matches
###

ADD_EXECUTABLE(openMVG_main_ComputeFeatures ./openMVG_utils/main_ComputeFeatures.cpp)
TARGET_LINK_LIBRARIES(openMVG_main_ComputeFeatures
  openMVG_system
  openMVG_image
  openMVG_features
  openMVG_multiview
  openMVG_sfm
  stlplus
  vlsift
  )

ADD_EXECUTABLE(openMVG_main_ComputeMatches ./openMVG_utils/main_ComputeMatches.cpp)
TARGET_LINK_LIBRARIES(openMVG_main_ComputeMatches
  openMVG_system
  openMVG_features
  openMVG_multiview
  openMVG_sfm
  openMVG_matching_image_collection
  stlplus
  )


###
# SfM Pipelines
# - Incremental/Sequential
# - Global
###
# - Convert SfM_Data format (from one to another)
# - Pair filtering (from frustum)
# - Compute structure from known camera poses
# - Compute structure color
###
ADD_EXECUTABLE(openMVG_main_GlobalSfM main_GlobalSfM.cpp G2OEngine.cpp sfm_data_BA.cpp)
TARGET_LINK_LIBRARIES(openMVG_main_GlobalSfM
  openMVG_system
  openMVG_image
  openMVG_features
  openMVG_sfm
  stlplus
  ${OpenCV_LIBS}
  g2o_core 
  g2o_types_slam3d 
  g2o_solver_csparse 
  g2o_stuff 
  g2o_csparse_extension 
  g2o_types_sba
  ${CHOLMOD_LIBRARIES}
  )
  
###
# G2O BA process
###
# - Read SfM Data
# - Perform BA with G2O
# - Output SfM Data
###
ADD_EXECUTABLE(OpenEDGE_main_G2O_BA_process G2O_BA_process.cpp G2OEngine.cpp sfm_data_BA.cpp)
TARGET_LINK_LIBRARIES(OpenEDGE_main_G2O_BA_process
  openMVG_system
  openMVG_image
  openMVG_features
  openMVG_sfm
  stlplus
  ${OpenCV_LIBS}
  g2o_core 
  g2o_types_slam3d 
  g2o_solver_csparse 
  g2o_stuff 
  g2o_csparse_extension 
  g2o_types_sba
  ${CHOLMOD_LIBRARIES}
  )

###
# SfM tools to visualize feature tracking data
###

# - View extracted Keypoints
#
ADD_EXECUTABLE(openMVG_main_exportKeypoints ./openMVG_utils/main_exportKeypoints.cpp)
TARGET_LINK_LIBRARIES(openMVG_main_exportKeypoints
  openMVG_system
  openMVG_features
  openMVG_sfm
  stlplus
  )


# - View computed matches (putatives, geometrics) per image pair
#
ADD_EXECUTABLE(openMVG_main_exportMatches ./openMVG_utils/main_exportMatches.cpp)
TARGET_LINK_LIBRARIES(openMVG_main_exportMatches
  openMVG_system
  openMVG_features
  openMVG_sfm
  stlplus
  )

# - View tracks per image pair
#
ADD_EXECUTABLE(openMVG_main_exportTracks ./openMVG_utils/main_exportTracks.cpp)
TARGET_LINK_LIBRARIES(openMVG_main_exportTracks
  openMVG_system
  openMVG_features
  openMVG_sfm
  stlplus
  )

# - Export undistorted images related to a sfm_data file
#
ADD_EXECUTABLE(openMVG_main_ExportUndistortedImages ./openMVG_utils/main_ExportUndistortedImages.cpp)
TARGET_LINK_LIBRARIES(openMVG_main_ExportUndistortedImages
  openMVG_system
  openMVG_image
  openMVG_features
  openMVG_sfm
  stlplus
  )


###
# SfM export to X
# - PMVS + bundler data format
# - OpenMVS
# - CMPMVS
# - Meshlab
# - MVE (File format v2)
###

# - Export a SfM openMVG scene to openMVS format
#
ADD_EXECUTABLE(openMVG_main_openMVG2openMVS ./openMVG_utils/main_openMVG2openMVS.cpp)
TARGET_LINK_LIBRARIES(openMVG_main_openMVG2openMVS
  openMVG_system
  openMVG_image
  openMVG_features
  openMVG_sfm
  stlplus
  )

###
# SfM Research tools
#
# - Quality comparison against a GT camera path (MultiView Evaluation dataset)
###
ADD_EXECUTABLE(openMVG_main_evalQuality ./openMVG_utils/main_evalQuality.cpp)
TARGET_LINK_LIBRARIES(openMVG_main_evalQuality
  openMVG_system
  openMVG_features
  openMVG_sfm
  stlplus
  )


