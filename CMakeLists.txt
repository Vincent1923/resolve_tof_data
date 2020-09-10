# 设置CMAKE最低版本
cmake_minimum_required(VERSION 2.8)
# 设置项目名称
set(PROJECT_NAME resolve_tof_data)
# 建立项目
project(${PROJECT_NAME})

# 添加C++ 11标准支持
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11")
#set(CMAKE_CXX_STANDARD 11)

# 寻找PCL库
find_package(PCL 1.2 REQUIRED)

# 寻找OpenCV库
find_package(OpenCV 4 REQUIRED)

# 导入需要使用的boost库，如regex filesystem thread program_options
find_package(Boost REQUIRED COMPONENTS filesystem program_options thread)
if (NOT Boost_FOUND)
    message("Not found Boost")
endif ()

# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

# 添加头文件
include_directories("/usr/include/eigen3")
include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} .)
message("${Boost_INCLUDE_DIRS}")
message("${OpenCV_INCLUDE_DIRS}")
message("/usr/include/eigen3")
message(${PCL_INCLUDE_DIRS})

add_subdirectory(input_source)
add_subdirectory(frame_utils)

add_executable(resolve_tof_data resolve_tof_data_main.cc)
target_link_libraries(resolve_tof_data ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} input_source frame_utils)
