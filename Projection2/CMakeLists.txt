cmake_minimum_required(VERSION 3.5)
project(test1)
add_compile_options(-std=c++11)

set(src_list 3dto2dConverter.cpp)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem chrono thread)
find_package(PCL REQUIRED COMPONENT common io visualization)
find_package(VTK REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
add_definitions(${PCL_DEFINITIONS})



add_executable(test1 test.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${Pangolin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})



#${EIGEN3_LIBRARIES} ${Eigen_INCLUDE_DIRS}