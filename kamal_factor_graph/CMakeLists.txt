cmake_minimum_required(VERSION 3.15)

project(MyGTSAM)

set(CMAKE_CXX_STANDARD 17)

find_package(GTSAM REQUIRED)
find_package(Python3 COMPONENTS Development NumPy REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Python3_NumPy_INCLUDE_DIRS})
include_directories(/usr/include/sym)

add_executable(${PROJECT_NAME} main.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE ${Python3_INCLUDE_DIRS})

target_link_libraries(
    ${PROJECT_NAME}
    PRIVATE gtsam    # Link GTSAM
    PRIVATE ${Python3_LIBRARIES}  # Link Python libraries
    PRIVATE ${Python3_NumPy_LIBRARIES}  # Link NumPy libraries
    #/usr/lib/libsymforce_gen.so
    #/usr/lib/libsymforce_opt.so
    #/usr/lib/libsymforce_slam.so
    #/usr/lib/libsymforce_cholesky.so
)

