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

include_directories(include)
include_directories(src/draco_dynamic_model)

set(draco_dyn_model_sources src/draco_dynamic_model/DracoP1Rot_Definition.h
						  src/draco_dynamic_model/Draco_Dyn_Model.hpp						  
						  src/draco_dynamic_model/Draco_Dyn_Model.cpp
						  src/draco_dynamic_model/Draco_Kin_Model.hpp						  
						  src/draco_dynamic_model/Draco_Kin_Model.cpp						  
						  src/draco_dynamic_model/DracoModel.hpp
						  src/draco_dynamic_model/DracoModel.cpp)					

set(draco_act_model_sources src/draco_actuator_model/DracoActuatorModel.cpp) 		
set(draco_combined_dynamics_sources src/draco_combined_dynamics_model/draco_combined_dynamics_model.cpp) 			  			



set(adt_container_sources src/adt/containers/adt_opt_variable.cpp
						  src/adt/containers/adt_opt_variable_manager.cpp
						  src/adt/containers/adt_constraint_list.cpp
						  src/adt/containers/adt_contact_list.cpp)

set(adt_contact_sources src/adt/contacts/adt_draco_contact_toe.cpp
						src/adt/contacts/adt_draco_contact_heel.cpp)

set(adt_optimization_var_limits_sources src/adt/optimization_variable_limits/adt_jump_variable_limits.cpp)

set(adt_optimization_problem_sources src/adt/optimization_problems/adt_opt_jump_problem.cpp)

set(adt_hard_constraints_sources src/adt/hard_constraints/adt_floor_2d_contact_lcp_constraint.cpp
								 src/adt/hard_constraints/adt_friction_cone_2d_constraint.cpp
								 src/adt/hard_constraints/adt_linear_back_euler_time_integration_constraint.cpp
								 src/adt/hard_constraints/adt_position_kinematic_constraint.cpp
								 src/adt/hard_constraints/adt_com_kinematic_constraint.cpp
								 src/adt/hard_constraints/adt_dynamics_constraint.cpp
								 src/adt/hard_constraints/adt_timestep_constraint.cpp)

set(adt_objective_function_sources src/adt/objective_functions/adt_jump_objective_function.cpp)

set(adt_snopt_wrapper_sources src/adt/adt_snopt_wrapper.cpp)

#--------------------------------------------
# Test Draco Model 
#--------------------------------------------
add_executable(test_draco_model  src/small_tests/test_draco_model.cpp  ${draco_dyn_model_sources} 	
																	   ${draco_act_model_sources}
																	   ${draco_combined_dynamics_sources} 
																	   ${adt_contact_sources})
target_link_libraries(test_draco_model  ${SJUtils} ${SJurdf} ${SJrbdl} ${SJsnopt})								 
#--------------------------------------------

#--------------------------------------------
# Test Variable Containers Model 
#--------------------------------------------
add_executable(test_adt_containers  src/small_tests/test_adt_containers.cpp  ${adt_container_sources})
target_link_libraries(test_adt_containers  ${SJUtils} ${SJurdf} ${SJrbdl} ${SJsnopt})					 
#--------------------------------------------

#--------------------------------------------
# Test Hard Constraints Object 
#--------------------------------------------
add_executable(test_hard_constraints_obj  src/small_tests/test_draco_constraints_obj.cpp
										${adt_container_sources}
										${draco_dyn_model_sources}
										${draco_act_model_sources}		
										${draco_combined_dynamics_sources}					
										${adt_contact_sources}
										${adt_optimization_var_limits_sources}
										${adt_optimization_problem_sources}									
										${adt_objective_function_sources}									
										${adt_hard_constraints_sources}
										) 
target_link_libraries(test_hard_constraints_obj  ${SJUtils} ${SJurdf} ${SJrbdl} ${SJsnopt})

#--------------------------------------------
# Test Optimization Problem Object 
#--------------------------------------------
add_executable(test_jump_prob_object  src/small_tests/test_jump_prob_object.cpp  
										${draco_dyn_model_sources}
										${draco_act_model_sources}
										${draco_combined_dynamics_sources}
										${adt_container_sources}
										${adt_optimization_problem_sources}
										${adt_contact_sources}
										${adt_optimization_var_limits_sources}
										${adt_hard_constraints_sources}
										${adt_objective_function_sources}	
										${adt_snopt_wrapper_sources}								
										) 
target_link_libraries(test_jump_prob_object  ${SJUtils} ${SJurdf} ${SJrbdl} ${SJsnopt})

#--------------------------------------------
# Test Optimization Problem Object 
#--------------------------------------------
add_executable(adt_problem  src/optimizations/main_jump_opt.cpp  
										${draco_dyn_model_sources}
										${draco_act_model_sources}
										${draco_combined_dynamics_sources}
										${adt_container_sources}
										${adt_optimization_problem_sources}
										${adt_contact_sources}
										${adt_optimization_var_limits_sources}
										${adt_hard_constraints_sources}
										${adt_objective_function_sources}	
										${adt_snopt_wrapper_sources}								
										) 
target_link_libraries(adt_problem  ${SJUtils} ${SJurdf} ${SJrbdl} ${SJsnopt})



		 
