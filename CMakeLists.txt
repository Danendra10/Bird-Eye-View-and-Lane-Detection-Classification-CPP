cmake_minimum_required(VERSION 3.0.2)
project(IRCAR-VISION)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(NumCpp REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${NumCpp_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/vision.cpp)
target_link_libraries(${PROJECT_NAME}_node ${OpenCV_LIBRARIES} ${EIGEN3_LIBRARIES} NumCpp::NumCpp)