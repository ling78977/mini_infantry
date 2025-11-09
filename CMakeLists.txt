cmake_minimum_required(VERSION 3.16)

project(mini_infantry
    LANGUAGES C CXX
)

# 基本设置
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
# 查找 WiringPi 库
find_package(WiringPi REQUIRED)
find_package(YAML-CPP REQUIRED)
find_package(PahoMqttCpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/control
    ${Boost_INCLUDE_DIRS}
)
# 可选设置
# option(BUILD_SHARED_LIBS "Build shared libraries" OFF)
# option(ENABLE_WARNINGS "Enable common warnings" ON)

# if(ENABLE_WARNINGS)
#     if(MSVC)
#         add_compile_options(/W4 /permissive-)
#     else()
#         add_compile_options(-Wall -Wextra -Wpedantic)
#     endif()
# endif()

# 源文件收集（支持增量构建）
# file(GLOB_RECURSE PROJECT_SOURCES CONFIGURE_DEPENDS
#     "${CMAKE_SOURCE_DIR}/src/*.c"
#     "${CMAKE_SOURCE_DIR}/src/*.cpp"
# )

# # 如果没有源文件，生成一个最小的 main 用于首次配置（放在 build 目录）
# if(NOT PROJECT_SOURCES)
#     message(STATUS "No sources found in src/ — generating a minimal main in the build directory.")
#     set(PROJECT_SOURCES "${CMAKE_BINARY_DIR}/mini_infantry_dummy_main.cpp")
#     file(WRITE "${PROJECT_SOURCES}" "int main() { return 0; }\n")
# endif()

add_executable(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/src/control/test1.cpp)
add_executable(debug_pid ${CMAKE_CURRENT_SOURCE_DIR}/src/control/debug_pid.cpp)

target_link_libraries(debug_pid PRIVATE
    WiringPi::WiringPi
    yaml-cpp
    PahoMqttCpp::paho-mqttpp3-shared
    nlohmann_json::nlohmann_json
    Boost::system
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    WiringPi::WiringPi
    yaml-cpp
)
# # include 目录（如果存在）
# if(EXISTS "${CMAKE_SOURCE_DIR}/include")
#     target_include_directories(${PROJECT_NAME} PRIVATE "${CMAKE_SOURCE_DIR}/include")
# endif()

# # 常用目标属性
# target_compile_definitions(${PROJECT_NAME} PRIVATE PROJECT_VERSION_MAJOR=${PROJECT_VERSION_MAJOR} PROJECT_VERSION_MINOR=${PROJECT_VERSION_MINOR})
# set_target_properties(${PROJECT_NAME} PROPERTIES
#     OUTPUT_NAME "${PROJECT_NAME}"
#     POSITION_INDEPENDENT_CODE ON
# )

# # 安装规则
# install(TARGETS ${PROJECT_NAME}
#     RUNTIME DESTINATION bin
# )

# # 可选：启用 CTest 测试框架
# option(ENABLE_TESTING "Enable testing with CTest" OFF)
# if(ENABLE_TESTING)
#     include(CTest)
# endif()