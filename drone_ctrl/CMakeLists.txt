cmake_minimum_required(VERSION 3.1.0)

project(drone_ctrl LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

if(CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        dji_sdk
        std_msgs
        sensor_msgs
        image_transport
        cv_bridge
        message_generation
        pcl_conversions
        pcl_ros)

find_package(djiosdk-core QUIET)

if(NOT djiosdk-core_FOUND)
    MESSAGE(STATUS "DO NOT FIND THE djiosdk-core LIB, WILL TRY TO DOWNLOAD FORM GITHUB, COMPILE, INSTALL")
    execute_process(COMMAND mkdir ${CMAKE_CURRENT_SOURCE_DIR}/../cmake-modules/build)
    execute_process(
            COMMAND cmake ..
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../cmake-modules/build)

    execute_process(
            COMMAND make
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../cmake-modules/build)
endif()

find_package(DJIOSDK REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.2 REQUIRED)

find_package(Qt5 COMPONENTS Widgets REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp rospy image_transport std_msgs sensor_msgs geometry_msgs message_runtime cv_bridge
)

include_directories(
        ${DJIOSDK_INCLUDE_DIRS}
        ${DJIOSDK_LIBRARIES}
        ${CMAKE_CURRENT_BINARY_DIR}/..
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)
###########
## Eigen ##
###########
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
link_directories(${EIGEN3_LIBRARY_DIRS})
#########
## CCD ##
#########
find_package(CCD REQUIRED)
include_directories(${CCD_INCLUDE_DIRS})
link_directories(${CCD_LIBRARY_DIRS})
#########
## FCL ##
#########
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(FCL REQUIRED)
include_directories(${FCL_INCLUDE_DIRS})
link_directories(${FCL_LIBRARY_DIRS})
#########
## OCTO ##
#########
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})
################# fixed_flight_demo #####

add_executable(fixed_flight_demo
        src/fixed_flight_demo.cpp
        src/drone_object_ros.cpp)

target_link_libraries(fixed_flight_demo ${catkin_LIBRARIES}  ${DJIOSDK_LIBRARIES})

################# Drone_Auto #####

add_executable(drone_auto
            src/drone_auto.cpp
        src/modules/environment_processing.cpp
            src/drone_object_ros.cpp src/modules/abstract_module.cpp src/modules/abstract_module.h src/modules/drone_decision.cpp src/modules/drone_decision.h)

target_link_libraries(drone_auto ${catkin_LIBRARIES} ${OpenCV_LIBS}  ${DJIOSDK_LIBRARIES})

################# Drone_Pathfinder #####

#qt5_wrap_cpp(MOC src/qt_ros_test.h) ##for real environment do not need
qt5_wrap_ui(UIC src/qt_ros_test.ui)
add_executable(drone_Pathfinder
        src/drone_Pathfinder.cpp
        src/modules/pointcloud_processing.cpp src/modules/pointcloud_processing.h src/modules/CollisionDetection.h src/modules/CollisionDetection.cpp
        src/drone_object_ros.cpp src/modules/abstract_module.cpp src/modules/abstract_module.h src/modules/direction_decision.cpp src/modules/direction_decision.h
        src/qnode.cpp src/qnode.h src/qt_ros_test.cpp
#        src/modules/img_process.cpp src/modules/img_process.h
        #${MOC} ##for real environment do not need
        ${UIC}
        )

target_link_libraries(drone_Pathfinder Qt5::Widgets ${catkin_LIBRARIES} ${PCL_LIBRARIES}  ${DJIOSDK_LIBRARIES} ${FCL_LIBRARIES} ${OpenCV_LIBS} ${EIGEN3_LIBRARIES} ${CCD_LIBRARIES} ${OCTOMAP_LIBRARIES})

################# Drone_keyboard #####


qt5_wrap_cpp(QT_MOC include/DialogKeyboard.h)
qt5_add_resources(RESOURCES include/drone_keyboard.qrc)


add_executable( drone_keyboard


        src/DialogKeyboard.ui	
        src/DialogKeyboard.cpp
        src/drone_keyboard.cpp
        src/drone_object_ros.cpp

        ${RESOURCES}
        ${QT_MOC}
        )

target_link_libraries( drone_keyboard Qt5::Widgets ${catkin_LIBRARIES}  ${DJIOSDK_LIBRARIES})

################# keyboard_rc #####

qt5_wrap_cpp(QT_MOC_KEYBOARD_RC src/modules/keyboard_rc/keyboard_rc_gui.h)
qt5_add_resources(RESOURCES src/modules/keyboard_rc/keyboard_rc_gui.qrc)

add_executable( keyboard_rc

        src/modules/keyboard_rc/keyboard_rc_gui.ui
        src/modules/keyboard_rc/keyboard_rc_gui.cpp
        src/modules/keyboard_rc/keyboard_rc.cpp
        ${RESOURCES}
        ${QT_MOC_KEYBOARD_RC}
        )
target_link_libraries( keyboard_rc Qt5::Widgets ${catkin_LIBRARIES} )
################# viewer #####

add_executable(viewer
        src/viewer.cpp)
target_link_libraries(viewer ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
################# viewer_pc #####

add_executable(viewer_pc
        src/viewer_pc.cpp)
target_link_libraries(viewer_pc ${catkin_LIBRARIES} ${PCL_LIBRARIES})

