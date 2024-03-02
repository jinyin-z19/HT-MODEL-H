# CMakeLists.txt
cmake_minimum_required(VERSION 3.0)

project(liveltbot_driver_sdk)

# 设置测试动态链接库应用程序的名字
set(EXAMPLE example)
 
#设置动态库的名字,最终生成libhightorque_biped_sdk.so
set(SDK_DYNAMIC_LIB liveltbot_driver_sdk)

SET(CMAKE_CXX_FLAGS "-std=c++0x")

# 设置动态链接库输出的指定路径
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/lib)

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_LIST_DIR}/build)

# 根据库文件代码生成动态库
add_library(${SDK_DYNAMIC_LIB}
            SHARED 
            src/Livelybot_Driver.cpp
            src/Motor_Control.cpp
)
 
# 包含指定头文件所在的目录
target_include_directories(${SDK_DYNAMIC_LIB}
                            PUBLIC
                            ${PROJECT_SOURCE_DIR}/include
)
 
# 创建测试动态链接库应用程序
add_executable(${EXAMPLE} 
    ${PROJECT_SOURCE_DIR}/example/example.cpp
)
 
# 链接动态库文件
target_link_libraries(${EXAMPLE} ${SDK_DYNAMIC_LIB})
