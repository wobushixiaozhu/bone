cmake_minimum_required(VERSION 3.0.2)
project(diana_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf
)
find_package(Ceres REQUIRED)

find_package(Qt5 REQUIRED Core Widgets)
find_package(yaml-cpp REQUIRED)

qt5_wrap_cpp(MOC src/SurgRobotROS.cpp)
qt5_wrap_ui(UIC src/SurgRobotROS.ui)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
  #   FILES
  #   Message1.msg
  #   Message2.msg
  # )
  
  ## Generate services in the 'srv' folder
  # add_service_files(
    #   FILES
    #   Service1.srv
    #   Service2.srv
    # )
    
    ## Generate actions in the 'action' folder
    # add_action_files(
      #   FILES
      #   Action1.action
      #   Action2.action
      # )
      
      ## Generate added messages and services with any dependencies listed here
      # generate_messages(
        #   DEPENDENCIES
        #   std_msgs
        # )
        
        ################################################
        ## Declare ROS dynamic reconfigure parameters ##
        ################################################
        
        ## To declare and build dynamic reconfigure parameters within this
        ## package, follow these steps:
        ## * In the file package.xml:
        ##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
        ## * In this file (CMakeLists.txt):
        ##   * add "dynamic_reconfigure" to
        ##     find_package(catkin REQUIRED COMPONENTS ...)
        ##   * uncomment the "generate_dynamic_reconfigure_options" section below
        ##     and list every .cfg file to be processed
        
        ## Generate dynamic reconfigure parameters in the 'cfg' folder
        # generate_dynamic_reconfigure_options(
          #   cfg/DynReconf1.cfg
          #   cfg/DynReconf2.cfg
          # )
          
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
            #  LIBRARIES diana_driver
            #  CATKIN_DEPENDS roscpp rospy std_msgs
            #  DEPENDS system_lib
            )
            
            ###########
            ## Build ##
            ###########
            
            #set(DIANA_API_DIR DianaApi/lib)
            
            ## Specify additional locations of header files
            ## Your package locations should be listed before other locations
            SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__USE_LONG_INTEGERS__")

            include_directories(
              include
              ${catkin_INCLUDE_DIRS}
              "/usr/include/eigen3"
              DianaApi/include
              ${CERES_INCLUDE_DIRS}
              )
              
              ## Declare a C++ library
              # add_library(${PROJECT_NAME}
              #   src/${PROJECT_NAME}/diana_driver.cpp
              # )
              link_directories(DianaApi/lib)
              link_directories(qpOASES)
              
              add_library(utility_med src/utility_med.cpp)

              target_link_libraries(utility_med
              ${catkin_LIBRARIES}
              )

add_library(dualFoSen src/DualFoSenDriver.cpp)

add_library(CameraSen src/CameraComm.cpp)

add_library(DianaDriver src/diana_driver.cpp)

add_library(TD_lib src/TD2Order.cpp include/TD2Order.hpp)



target_link_libraries(CameraSen
${catkin_LIBRARIES}
utility_med
  #dualFoSen
  #DianaApi/lib/libDianaApi.so
)




target_link_libraries(DianaDriver
  libDianaApi.so
  #dualFoSen
  #DianaApi/lib/libDianaApi.so
)

add_library(Controllers_lib  src/Controllers.cpp)
target_link_libraries(Controllers_lib
  utility_med
  TD_lib
)


add_library(Spline2DCurve_lib src/Spline2DCurve.cpp)
target_link_libraries(Spline2DCurve_lib
  ${catkin_LIBRARIES}
  libqpOASES.so
  utility_med
  ${CERES_LIBRARIES}
)

add_library(PathFollowTask_lib src/PathFollowTask.cpp)
target_link_libraries(PathFollowTask_lib
  ${catkin_LIBRARIES}
  libqpOASES.so
  utility_med
  ${CERES_LIBRARIES}
  Spline2DCurve_lib
)




add_library(MillingTask_lib src/MillingTask.cpp)
target_link_libraries(MillingTask_lib
  PathFollowTask_lib
  TD_lib
  Controllers_lib
  yaml-cpp.so
)


add_library(${PROJECT_NAME}_admittance_lib src/DianaAdmitControl.cpp)
target_link_libraries(${PROJECT_NAME}_admittance_lib
  ${catkin_LIBRARIES}
  utility_med
  libDianaApi.so
  DianaDriver
  dualFoSen
  CameraSen
  PathFollowTask_lib
  MillingTask_lib
  yaml-cpp.so
)

add_library(${PROJECT_NAME}_admittance_libp src/DianaAdmitControlPlaneCut.cpp
src/PlaneCuttingTask.cpp)
target_link_libraries(${PROJECT_NAME}_admittance_libp
  ${catkin_LIBRARIES}
  utility_med
  libDianaApi.so
  DianaDriver
  dualFoSen
  CameraSen
  yaml-cpp.so
)


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

add_executable(Spline2DCurve_node src/Spline2DCurve.cpp)

add_executable(dualFoSen_node src/DualFoSenDriver.cpp)

add_executable(CameraSen_node src/CameraComm.cpp)

add_executable(CameraSen_node_test src/CameraComm_Server.cpp)

add_executable(${PROJECT_NAME}_node src/diana_driver.cpp)

add_executable(${PROJECT_NAME}_admittance src/DianaAdmitControl.cpp)

add_executable(utility1 src/utility_med.cpp)

add_executable(PathFollowTask_node src/PathFollowTask.cpp)

add_executable(SurgRobotROS src/SurgRobotROS.cpp  include/SurgRobotROS.hpp src/SurgRobotROS.ui)

add_executable(SurgRobotROSP src/SurgRobotROSP.cpp  include/SurgRobotROSP.hpp src/SurgRobotROS.ui)

add_executable(TD src/TD2Order.cpp include/TD2Order.hpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(utility1
${catkin_LIBRARIES}
)



target_link_libraries(Spline2DCurve_node
  ${catkin_LIBRARIES}
  libqpOASES.so
  utility_med
  ${CERES_LIBRARIES}
)

target_link_libraries(PathFollowTask_node
  ${catkin_LIBRARIES}
  libqpOASES.so
  utility_med
  ${CERES_LIBRARIES}
  Spline2DCurve_lib
)

target_link_libraries(dualFoSen_node
  ${catkin_LIBRARIES}
)

target_link_libraries(CameraSen_node
  ${catkin_LIBRARIES}
  utility_med
  yaml-cpp.so
)

target_link_libraries(CameraSen_node_test
  ${catkin_LIBRARIES}
  utility_med
  yaml-cpp.so
)



target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  libDianaApi.so
  #dualFoSen
  #DianaApi/lib/libDianaApi.so
)

target_link_libraries(${PROJECT_NAME}_admittance
  ${catkin_LIBRARIES}
  libDianaApi.so
  DianaDriver
  dualFoSen
  CameraSen
  utility_med
  PathFollowTask_lib
  MillingTask_lib
)


target_link_libraries(SurgRobotROS
  ${catkin_LIBRARIES}
  Qt5::Widgets
  ${PROJECT_NAME}_admittance_lib
  # yaml-cpp.so
)

target_link_libraries(SurgRobotROSP
  ${catkin_LIBRARIES}
  Qt5::Widgets
  ${PROJECT_NAME}_admittance_libp
  # yaml-cpp.so
)






#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_diana_driver.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
