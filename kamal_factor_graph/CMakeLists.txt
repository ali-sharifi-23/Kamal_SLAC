cmake_minimum_required(VERSION 3.15)

project(MyGTSAM)

set(CMAKE_CXX_STANDARD 17)

find_package(GTSAM REQUIRED)

add_executable(${PROJECT_NAME} main.cpp)



# Link GTSAM libraries to your executable
target_link_libraries(
        ${PROJECT_NAME}
        gtsam      # GTSAM library
)
