set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(PCL 1.2 REQUIRED)

#指定头文件目录
include_directories(${PROJECT_SOURCE_DIR}/include ${PCL_INCLUDE_DIRS})

set(CMAKE_BUILD_TYPE DEBUG)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#指定库文件输出路径
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#生成动态库
add_library(extract_point_cloud_so SHARED extract_point_cloud.cpp)
set_target_properties(extract_point_cloud_so PROPERTIES OUTPUT_NAME "extract_point_cloud")

#生成静态库
add_library(extract_point_cloud_a STATIC extract_point_cloud.cpp)
set_target_properties(extract_point_cloud_a PROPERTIES OUTPUT_NAME "extract_point_cloud")

#执行文件存放路径
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
add_executable(test test.cpp)

target_link_libraries(test ${PCL_LIBRARIES} ${LIBRARY_OUTPUT_PATH}/libextract_point_cloud.a)