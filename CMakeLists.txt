cmake_minimum_required(VERSION 3.0.2)
project(md_camera)

find_package(
        catkin REQUIRED COMPONENTS
        topic_tools
        cv_bridge
        dynamic_reconfigure
        roscpp
        sensor_msgs
        message_generation
)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(yaml-cpp REQUIRED)

add_definitions(-DCONFIG_PATH="${PROJECT_SOURCE_DIR}/params/")
add_definitions(-DRECORD_PATH="${PROJECT_SOURCE_DIR}/record/")

add_service_files(
        FILES
        GetCameraInfo.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
        sensor_msgs
)

generate_dynamic_reconfigure_options(
        config/Camera.cfg
)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES md_camera
        #  CATKIN_DEPENDS cv_bridge dynamic_reconfigure roscpp sensor_msgs
        #  DEPENDS system_lib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIR}
        ${Boost_INCLUDE_DIR}
)

add_executable(
        ${PROJECT_NAME}_node
        src/main.cpp
        src/MDCamera.cpp
        src/resolution.cpp
        src/rosCamera.cpp
        src/cameraMatrix.cpp
        src/LockFrame.cpp
        src/Recorder.cpp
        src/TopicRecorder.cpp
)

add_executable(
        video_pub_node
        src/videoPubNode.cpp
        src/cameraMatrix.cpp
        src/TopicRecorder.cpp
        src/Player.cpp
)

add_dependencies(
        ${PROJECT_NAME}_node
        md_camera_gencpp
)

add_dependencies(
        video_pub_node
        md_camera_gencpp
)

target_link_libraries(
        ${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRAIRES}
        ${YAML_CPP_LIBRARIES}
        MVSDK
)

target_link_libraries(
        video_pub_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRAIRES}
        ${YAML_CPP_LIBRARIES}
)
