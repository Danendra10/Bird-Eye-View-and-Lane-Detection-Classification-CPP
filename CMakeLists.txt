cmake_minimum_required(VERSION 3.0.2)
project(IRCAR-VISION)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(NumCpp REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  # ${NumCpp_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/road_sign_detector.cc)
target_link_libraries(${PROJECT_NAME}_node ${OpenCV_LIBRARIES} ${EIGEN3_LIBRARIES})


add_executable(get_image src/get_image.cc)
target_link_libraries(get_image ${OpenCV_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(dummy src/dummy.cc)
target_link_libraries(dummy ${OpenCV_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(generate_aruco src/generate_aruco.cc)
target_link_libraries(generate_aruco ${OpenCV_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(lane_detect src/lane_detect.cc)
target_link_libraries(lane_detect ${OpenCV_LIBRARIES} ${EIGEN3_LIBRARIES})