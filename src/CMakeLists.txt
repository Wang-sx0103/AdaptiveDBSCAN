#################
##  configure  ##
#################

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/${CMAKE_BUILD_TYPE})

find_package(Eigen3 REQUIRED)
#find_package(OpenCV REQUIRED)
include_directories(
    
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    #    ${OpenCV_INCLUDE_DIRS}
)

#############
##  build  ##
#############
