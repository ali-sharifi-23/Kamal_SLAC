cmake_minimum_required(VERSION 3.15)

project(MyGTSAM)

set(CMAKE_CXX_STANDARD 17)

# Find GTSAM and Python dependencies
find_package(GTSAM REQUIRED)
find_package(Python3 COMPONENTS Development NumPy REQUIRED)

# Include the matplotlib-cpp directory and Eigen headers
include_directories(/home/ali/matplotlib-cpp)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Python3_NumPy_INCLUDE_DIRS})

# Add the executable
add_executable(${PROJECT_NAME} main.cpp)

# Link the Python3 include directories to the project
target_include_directories(${PROJECT_NAME} PRIVATE ${Python3_INCLUDE_DIRS})

# Link against GTSAM, Python, and Eigen libraries
target_link_libraries(
    ${PROJECT_NAME}
    PRIVATE gtsam    # Link GTSAM
    PRIVATE ${Python3_LIBRARIES}  # Link Python libraries
    PRIVATE ${Python3_NumPy_LIBRARIES}  # Link NumPy libraries
)

