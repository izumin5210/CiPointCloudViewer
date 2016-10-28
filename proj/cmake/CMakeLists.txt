cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(CiPointCloudViewerApp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -W -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CINDER_VERBOSE ON)

if(APPLE)
  include_directories(/usr/local/include/ni2)
  set(OpenCV_DIR /usr/local/opt/opencv3/share/OpenCV)
  set(Qt5_DIR /usr/local/opt/qt5/lib/cmake/Qt5)
elseif(LINUX)
  include_directories(/usr/include/openni2)
endif()

find_package        (CUDA QUIET)
include_directories (/usr/local/cuda/include)

find_package(PCL 1.8 REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
find_package(OpenCV 3.0 REQUIRED)
find_package(Boost COMPONENTS system filesystem program_options thread REQUIRED)

set(NITE2 TRUE)
if(NITE2)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_NITE2")
  set (NiTE2_LIBRARIES "NiTE2")
  include_directories (/usr/local/include/nite2)
  link_directories(/usr/local/lib/ni2)
endif()

get_filename_component(CINDER_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../../cinder/Cinder" ABSOLUTE)
get_filename_component(PROJECT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

set(CINDER_IMGUI_DIR ${PROJECT_DIR}/blocks/ImGui)

include_directories(${CINDER_IMGUI_DIR}/lib/imgui)
include_directories(${CINDER_IMGUI_DIR}/include)

set(CINDER_IMGUI_SOURCES
    ${CINDER_IMGUI_DIR}/lib/imgui/imgui.cpp
    ${CINDER_IMGUI_DIR}/lib/imgui/imgui_draw.cpp
    ${CINDER_IMGUI_DIR}/src/CinderImGui.cpp
)

execute_process(
  COMMAND find ${PROJECT_DIR}/src -name "*.cpp" -type f
  COMMAND paste -s -d ";" -
  OUTPUT_VARIABLE cppfiles
)
foreach(cppfile ${cppfiles})
  string(STRIP ${cppfile} cppfile)
  list(APPEND PROJECT_SOURCES "${cppfile}")
endforeach(cppfile)

list(APPEND PROJECT_SOURCES ${CINDER_IMGUI_SOURCES})

include_directories(${PROJECT_DIR}/blocks/nod/include)
include_directories(${PROJECT_DIR}/blocks/msgpack/include)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DMSGPACK_DISABLE_LEGACY_NIL")

include("${CINDER_PATH}/proj/cmake/modules/cinderMakeApp.cmake")

ci_make_app(
    SOURCES     ${PROJECT_SOURCES}
    INCLUDES    ${PROJECT_DIR}/include ${PCL_INCLUDE_DIRS}
    CINDER_PATH ${CINDER_PATH}
    LIBRARIES   ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} freenect2 ${NiTE2_LIBRARIES} yaml-cpp
)
