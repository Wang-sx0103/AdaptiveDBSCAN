cmake_minimum_required(VERSION 3.27)
# cmake_policy(SET CMP0144 NEW)
project(AdaptiveDBSCAN
        VERSION        0.1.0
        DESCRIPTION    "  "
        HOMEPAGE_URL   "  "
        LANGUAGES      C CXX)
message(STATUS "Start building project!")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")

#################
##  configure  ##
#################

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# 如果是 MSVC 编译器
if(MSVC)
    # 设置 MSVC 的优化选项
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
    set(CMAKE_CXX_FLAGS_DEBUG "/Zi /Od /W3")
    set(CMAKE_CXX_FLAGS_RELEASE "/Ob2")
# 如果是 GCC 编译器
elseif(CMAKE_COMPILER_IS_GNUCXX)
    # 设置 GCC 的优化选项
    # set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O2")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
    set(CMAKE_CXX_FLAGS_DEBUG  "-g -O0")
    set(CMAKE_CXX_FLAGS_RELEASE  "-O3")
endif()

find_package(PCL REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
)

#############
##  build  ##
#############

add_subdirectory(src)
add_subdirectory(tests)
add_subdirectory(apps)
