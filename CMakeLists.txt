cmake_minimum_required(VERSION 3.0.2)
project(md_camera)

find_package(
        catkin REQUIRED COMPONENTS
        cv_bridge
        dynamic_reconfigure
        roscpp
        sensor_msgs
)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

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
#   sensor_msgs
# )


## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )
catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES md_camera
        #  CATKIN_DEPENDS cv_bridge dynamic_reconfigure roscpp sensor_msgs
        #  DEPENDS system_lib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIR}
        ${Boost_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_node src/main.cpp src/MDCamera.cpp src/resolution.cpp)

target_link_libraries(
        ${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRAIRES}
        MVSDK
)
