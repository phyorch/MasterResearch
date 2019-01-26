cmake_minimum_required(VERSION 3.5)
SET (SRC_LIST test1_experiment_old.cpp Disparity.cpp ImageUtils.cpp SimilarityMeasure.cpp StereoGC.cpp Disparity.h ImageUtils.h SimilarityMeasure.h StereoGC.h)
add_compile_options(-std=c++11)

#add_subdirectory(../../../Install_Lib/HesaiLidarSDK/build/bin)

#SET(OpenCV_DIR /home/phyorch/opencv-3.4.0)
SET(OpenCV_DIR /opt/ros/kinetic)
SET(PCL_DIR /usr/include/pcl-1.7/pcl)

find_package(OpenCV REQUIRED COMPONENTS highgui imgproc calib3d ximgproc) #
find_package(Eigen3 REQUIRED)
find_package(VTK REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem chrono thread)
find_package(PCL REQUIRED COMPONENTS common io visualization filters)
find_package(PythonLibs 2.7)

find_package(ZED 2 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(myproject PRIVATE ${PYTHON_INCLUDE_DIRS})

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})
link_libraries(/usr/local/lib/libhesaiLidarSDK.so)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
add_definitions(${PCL_DEFINITIONS})

add_executable(test0 test0.cpp Disparity.cpp RealEquipment.cpp ImageUtils.cpp SimilarityMeasure.cpp Disparity.h RealEquipment.h ImageUtils.h SimilarityMeasure.h)
target_link_libraries(test0 cmaes
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        ${ZED_LIBRARIES}
        ${Boost_LIBRARIES})

add_executable(test1 ${SRC_LIST})
target_link_libraries(test1 ${OpenCV_LIBS}
                            ${PCL_LIBRARIES}
                            ${Boost_LIBRARIES})

add_executable(test2 test2_data_capturing.cpp Disparity.cpp RealEquipment.cpp Disparity.h RealEquipment.h)
target_link_libraries(test2 ${ZED_LIBRARIES}
                            ${Boost_LIBRARIES}
                            ${PCL_LIBRARIES}
                            ${OpenCV_LIBRARIES}
                            ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_NPP_LIBRARIES_ZED})

add_executable(test3 test3_hesai_capturing.cpp)
target_link_libraries(test3 hesaiLidarSDK
                            ${Boost_LIBRARIES}
                            ${PCL_LIBRARIES})

add_executable(test4 test4_point_cloud_transfer.cpp Disparity.cpp RealEquipment.cpp RealEquipment.h Disparity.h)
target_link_libraries(test4 hesaiLidarSDK
                            ${ZED_LIBRARIES}
                            ${OpenCV_LIBRARIES}
                            ${Boost_LIBRARIES}
                            ${PCL_LIBRARIES})

add_executable(test5 test5_calibration_old.cpp Disparity.cpp RealEquipment.cpp ImageUtils.cpp SimilarityMeasure.cpp Disparity.h RealEquipment.h ImageUtils.h SimilarityMeasure.h)
target_link_libraries(test5 hesaiLidarSDK
                            ${OpenCV_LIBS}
                            ${PCL_LIBRARIES}
                            ${ZED_LIBRARIES}
                            ${Boost_LIBRARIES}
                            ${PYTHON_LIBRARIES})

add_executable(test6 test6_calibration.cpp Disparity.cpp RealEquipment.cpp ImageUtils.cpp SimilarityMeasure.cpp Disparity.h RealEquipment.h ImageUtils.h SimilarityMeasure.h)
target_link_libraries(test6 cmaes
                            ${OpenCV_LIBS}
                            ${PCL_LIBRARIES}
                            ${ZED_LIBRARIES}
                            ${Boost_LIBRARIES})

add_executable(test7 test7_pcl_transformation.cpp Disparity.cpp RealEquipment.cpp ImageUtils.cpp SimilarityMeasure.cpp Disparity.h RealEquipment.h ImageUtils.h SimilarityMeasure.h)
target_link_libraries(test7 cmaes
                            ${OpenCV_LIBS}
                            ${PCL_LIBRARIES}
                            ${ZED_LIBRARIES}
                            ${Boost_LIBRARIES})

add_executable(test8 test8_pcl_icp.cpp Disparity.cpp RealEquipment.cpp ImageUtils.cpp SimilarityMeasure.cpp Disparity.h RealEquipment.h ImageUtils.h SimilarityMeasure.h)
target_link_libraries(test8 cmaes
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        ${ZED_LIBRARIES}
        ${Boost_LIBRARIES})