project( point_cloud_filter )
find_package( OpenCV REQUIRED )
find_package(PCL 1.3 REQUIRED )
find_package(Ceres REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable( point_cloud_filter point_cloud_filter.cpp )


target_link_libraries(point_cloud_filter ${PCL_LIBRARIES})
target_link_libraries(point_cloud_filter ${OpenCV_LIBS} )
target_link_libraries(point_cloud_filter ${CERES_LIBRARIES})
