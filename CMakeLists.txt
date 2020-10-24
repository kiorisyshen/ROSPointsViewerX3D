cmake_minimum_required(VERSION 3.10)
project(ROSPointsViewerX3D)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
)

# Add glfw
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

add_subdirectory(thirdparty/glfw)

find_package(PCL REQUIRED)
find_package(PkgConfig REQUIRED)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${CMAKE_CURRENT_LIST_DIR}/X3DLib/include
)

catkin_package(
    CATKIN_DEPENDS roscpp std_msgs
    DEPENDS PCL
)


add_executable(pt2_X3D_viewer 
    src/pt2_X3D_viewer.cpp 
    src/imgui_impl_glfw.cpp
)
target_link_libraries(pt2_X3D_viewer
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    glfw
    ${CMAKE_CURRENT_LIST_DIR}/X3DLib/libX3D.a
)