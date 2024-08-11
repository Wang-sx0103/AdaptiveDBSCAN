#################
##  configure  ##
#################

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/${CMAKE_BUILD_TYPE})

# find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(Sophus REQUIRED)
# find_package(OpenCV REQUIRED)
# find_package(PCL REQUIRED)
# find_package(Pangolin REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    # ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    # ${Pangolin_INCLUDE_DIRS}
)

#################
##    build    ##
#################

add_executable(Cluster ./Cluster.cxx)
target_link_libraries(Cluster
    PRIVATE ${PCL_LIBRARIES}
#    PRIVATE libTemplate
)

add_executable(ClusterP ./ClusterP.cxx)
target_link_libraries(ClusterP
    PRIVATE ${PCL_LIBRARIES}
#    PRIVATE libTemplate
)

add_executable(ShowCluster ./ShowCluster.cxx)
target_link_libraries(ShowCluster
    PRIVATE ${PCL_LIBRARIES}
#    PRIVATE libTemplate
)
