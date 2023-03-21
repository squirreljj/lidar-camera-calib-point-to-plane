project( lidar_camera_p_plane )
find_package( OpenCV REQUIRED )
find_package(PCL 1.3 REQUIRED )
find_package(Ceres REQUIRED)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(${PCL_INCLUDE_DIRS} include)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable( lidar_camera_p_plane ./src/lidar_camera_p_plane.cpp )


target_link_libraries(lidar_camera_p_plane ${PCL_LIBRARIES})
target_link_libraries(lidar_camera_p_plane ${OpenCV_LIBS} )
target_link_libraries(lidar_camera_p_plane ${CERES_LIBRARIES})
