cmake_minimum_required(VERSION 3.10)

# 项目名称
project(quner_vision)

# 设置ROS环境
set(ROS_DISTRO humble)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH})


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -w")

# 查找OpenCV
find_package(OpenCV REQUIRED)

# 查找InferenceEngine
find_package(InferenceEngine REQUIRED)

# 查找ROS Package
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(gary_msgs REQUIRED)

# 添加可执行文件，现在只需要main.cpp
add_executable(${PROJECT_NAME} ./src/qunerVISIONode.cpp)

ament_target_dependencies(quner_vision PUBLIC rclcpp std_msgs gary_msgs)

install(TARGETS
  quner_vision
  DESTINATION lib/${PROJECT_NAME})


# 链接OpenCV和InferenceEngine库，以及需要的GxIAPI和其他相关库
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS} ${InferenceEngine_LIBRARIES} -lgxiapi -lpthread)
target_link_libraries(${PROJECT_NAME} PRIVATE ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES} )

# 设置OpenCV和InferenceEngine的头文件路径
include_directories(${OpenCV_INCLUDE_DIRS} ${InferenceEngine_INCLUDE_DIRS})

# 设置ROS Package的头文件路径
include_directories(${rclcpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})

# 设置额外的头文件路径，以确保可以找到GxIAPI.h和DxImageProc.h
include_directories(./src /usr/include)

# 设置链接的库文件路径
link_directories(/usr/lib ./src)

# 如果需要，可以为目标添加特定的编译选项
target_compile_options(${PROJECT_NAME} PRIVATE -L/usr/lib -L./ -Wl,-rpath=/usr/lib:./)

ament_package()
