
# 最低CMake版本要求（需要支持C++20配置）
cmake_minimum_required(VERSION 3.20)

# 项目设置
project(Tester
    VERSION 1.0.0
    LANGUAGES CXX
)

# 设置C++20标准并强制要求
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)  # 禁用编译器扩展（保持标准合规）

# 添加可执行文件
add_executable(${PROJECT_NAME}
    src/main.cpp    # 你的源文件路径
    # 添加其他源文件...
)

add_subdirectory(SOEM)


target_include_directories(
    ${PROJECT_NAME}
    PRIVATE
    "D:/WpdPack/Include"
)

target_link_directories(
    ${PROJECT_NAME}
    PRIVATE
    "D:/WpdPack/Lib/x64/"
)
target_link_libraries(
    ${PROJECT_NAME}
    PRIVATE
    soem
)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)

