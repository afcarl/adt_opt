cmake_minimum_required(VERSION 2.8.3)
project(adt_opt)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
roscpp
sensor_msgs
tf
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lcp_val_simulator
   CATKIN_DEPENDS 
       sensor_msgs
       tf
#  DEPENDS system_lib
)

###########
## Build ##
###########
# After installing the sejong library via sudo make install, we have to find the installed libraries and all the included files

set(Sejong_Library_Path /usr/local/lib/SDC_Toolkit)
set(Sejong_Include_Path /usr/local/include/SDC_Toolkit/)
set(Sejong_Eigen_Path ~/Sejong_Dynamic_Control_Toolkit/ExternalSource)

find_library(SJUtils NAMES libSJutils.so PATHS ${Sejong_Library_Path}  REQUIRED)
find_library(SJurdf NAMES libSJurdf.so PATHS ${Sejong_Library_Path}  REQUIRED)
find_library(SJrbdl NAMES libSJrbdl.so PATHS ${Sejong_Library_Path}  REQUIRED)
find_library(SJMobyLCP NAMES libSJMobyLCP.so PATHS ${Sejong_Library_Path}  REQUIRED)
find_library(SJValkyrie_Controller NAMES libValkyrie_Controller.so PATHS ${Sejong_Library_Path}  REQUIRED)
find_library(SJsnopt NAMES libSJsnopt.so PATHS ${Sejong_Library_Path}  REQUIRED)

find_library(Valkyrie_model NAMES libValkyrie_model.so PATHS ${Sejong_Library_Path} REQUIRED)
include_directories(${Sejong_Include_Path})
include_directories(${Sejong_Eigen_Path})

include_directories(
# include
#${catkin_INCLUDE_DIRS}
)

#include_directories(include)
include_directories(src/draco_dynamic_model)

set(draco_sim_model_sources src/draco_dynamic_model/DracoP1Rot_Definition.h
						  src/draco_dynamic_model/Draco_Dyn_Model.hpp						  
						  src/draco_dynamic_model/Draco_Dyn_Model.cpp
						  src/draco_dynamic_model/Draco_Kin_Model.hpp						  
						  src/draco_dynamic_model/Draco_Kin_Model.cpp						  
						  src/draco_dynamic_model/DracoModel.hpp
						  src/draco_dynamic_model/DracoModel.cpp)								  			

add_executable(test_draco_model  src/small_tests/test_draco_model.cpp	
								 ${draco_sim_model_sources})

target_link_libraries(test_draco_model  ${SJUtils}
  									    ${SJurdf}
									    ${SJrbdl}
									    ${SJsnopt})								 