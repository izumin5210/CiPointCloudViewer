cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(CiPointCloudViewerApp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -W -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CINDER_VERBOSE ON)

find_package(Qt5 COMPONENTS Core Widgets)

if(APPLE)
  set(OpenCV_DIR "/usr/local/opt/opencv3/share/OpenCV")
  include_directories(/usr/local/include/ni2)
#  set(OpenCV_DIR "/usr/local/opt/opencv3/")
#  set(OpenCV_INCLUDE_DIRS "${OpenCV_DIR}/include")
#  set(OpenCV_LIBS "${OpenCV_DIR}/lib")
elseif(LINUX)
#  find_package(OpenCV 3.0 REQUIRED)
  include_directories(/usr/include/openni2)
endif()

find_package(OpenCV 3.0 REQUIRED)

find_package(PCL 1.8 REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package        (CUDA QUIET)
include_directories (/usr/local/cuda/include)

find_package(Boost COMPONENTS system filesystem program_options thread REQUIRED)

get_filename_component(CINDER_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../../cinder/Cinder" ABSOLUTE)
get_filename_component(PROJECT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

set(CINDER_IMGUI_DIR ${PROJECT_DIR}/blocks/ImGui)

include_directories(${CINDER_IMGUI_DIR}/lib/imgui)
include_directories(${CINDER_IMGUI_DIR}/include)

set(CINDER_IMGUI_SOURCES
    ${CINDER_IMGUI_DIR}/lib/imgui/imgui.cpp
    ${CINDER_IMGUI_DIR}/lib/imgui/imgui_draw.cpp
    ${CINDER_IMGUI_DIR}/lib/imgui/imgui_demo.cpp
    ${CINDER_IMGUI_DIR}/src/CinderImGui.cpp
)

set(PROJECT_SOURCES
    ${PROJECT_DIR}/src/CiPointCloudViewerApp.cpp
    ${PROJECT_DIR}/src/Dispatcher.cpp
    ${PROJECT_DIR}/src/FpsCounter.cpp
    ${PROJECT_DIR}/src/impl/AppGuiImpl.cpp
    ${PROJECT_DIR}/src/impl/CloudsImpl.cpp
    ${PROJECT_DIR}/src/impl/ViewParamsImpl.cpp
    ${PROJECT_DIR}/src/impl/SavingVerticesWorkerImpl.cpp
    ${PROJECT_DIR}/src/impl/YamlConfigure.cpp
    ${PROJECT_DIR}/src/impl/io/CloudDataSourcesImpl.cpp
    ${PROJECT_DIR}/src/impl/io/SensorDeviceManagerImpl.cpp
    ${PROJECT_DIR}/src/io/CalibrationParamsManager.cpp
    ${PROJECT_DIR}/src/io/SequentialPcdPlayer.cpp
    ${CINDER_IMGUI_SOURCES}
)

include_directories(${PROJECT_DIR}/blocks/nod/include)

include("${CINDER_PATH}/proj/cmake/modules/cinderMakeApp.cmake")

ci_make_app(
    SOURCES     ${PROJECT_SOURCES}
    INCLUDES    ${PROJECT_DIR}/include ${PCL_INCLUDE_DIRS} /usr/local/fruit
    CINDER_PATH ${CINDER_PATH}
    LIBRARIES   ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} freenect2 yaml-cpp fruit
)
