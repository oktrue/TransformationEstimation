cmake_minimum_required (VERSION 3.8)

add_library (TransformationEstimation SHARED "TransformationEstimation.cpp" "TransformationEstimation.h")
# add_executable (TransformationEstimation "TransformationEstimation.cpp" "TransformationEstimation.h")
target_link_libraries(TransformationEstimation ${PCL_LIBRARIES})
