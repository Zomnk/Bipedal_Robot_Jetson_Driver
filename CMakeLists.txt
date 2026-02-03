cmake_minimum_required(VERSION 3.25)
project(sim2sim)

set(CMAKE_CXX_STANDARD 17)

# 添加LCM库
set(LCM_DIR "/usr/local/lib/lcm/cmake")
find_package(lcm REQUIRED)

set(LCM_INCLUDE_DIRS "/usr/local/include")
include_directories(${LCM_INCLUDE_DIRS})
set(LCM_LIBRARY_DIRS "/usr/local/lib")
link_directories(${LCM_LIBRARY_DIRS})

# 找到 CUDA
set(CMAKE_CUDA_COMPILER "/usr/local/cuda-10.2/bin/nvcc")
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-10.2)
set(CUDA_INCLUDE_DIRS "/usr/local/cuda-10.2/include")
set(CUDA_LIBRARY_DIRS "/usr/local/cuda-10.2/lib64")
set(cuBLAS_LIBRARY_DIRS "/usr/lib/aarch64-linux-gnu")
find_package(CUDA REQUIRED)

include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${cuBLAS_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})

# 设置 LibTorch 的路径
set(Torch_DIR "/home/jetson/.local/lib/python3.6/site-packages/torch/share/cmake/Torch")
set(CMAKE_PREFIX_PATH "/home/jetson/.local/lib/python3.6/site-packages/torch")  # 将此路径替换为您解压 LibTorch 的路径
find_package(Torch REQUIRED)
include_directories(${Torch_DIR}/include)
include_directories(${CMAKE_BINARY_DIR}/lcm_types)
include_directories(${PROJECT_SOURCE_DIR}/include)

# LCM类型目录已在上面包含
# include_directories("lcm_types/my_lcm/Request.hpp")  # 错误：这是文件不是目录
# include_directories("lcm_types/my_lcm/Response.hpp")  # 错误：这是文件不是目录

add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/lcm_types/my_lcm/request.hpp
    OUTPUT ${CMAKE_BINARY_DIR}/lcm_types/my_lcm/response.hpp
    COMMAND lcm-gen -x ${CMAKE_SOURCE_DIR}/lcm_types/request.lcm
    COMMAND lcm-gen -x ${CMAKE_SOURCE_DIR}/lcm_types/response.lcm
    COMMAND lcm-gen -p ${CMAKE_SOURCE_DIR}/lcm_types/request.lcm
    COMMAND lcm-gen -p ${CMAKE_SOURCE_DIR}/lcm_types/response.lcm
    DEPENDS ${CMAKE_SOURCE_DIR}/lcm_types/request.lcm
    DEPENDS ${CMAKE_SOURCE_DIR}/lcm_types/response.lcm
    COMMENT "Generating LCM types"
)

add_custom_target(lcm_types ALL
    DEPENDS ${PROJECT_SOURCE_DIR}/lcm_types/my_lcm/Request.hpp
    DEPENDS ${PROJECT_SOURCE_DIR}/lcm_types/my_lcm/Response.hpp
)

add_executable(lcm_service src/lcm_service.cpp)
add_executable(lcm_publisher src/lcm_publisher.cpp)
add_executable(udp_publisher src/udp_publisher.cpp)
add_executable(udp_publisher_tinker src/udp_publisher_tinker.cpp)
add_executable(udp_test src/udp_test.cpp)  # UDP通信测试程序（不需要PyTorch）
add_executable(jetson_full_test src/jetson_full_test.cpp)  # 完整数据流测试（不需要PyTorch）
add_executable(calibrate_robot src/calibrate_robot.cpp)  # 机器人标定工具（不需要PyTorch）

add_dependencies(lcm_service lcm_types)
add_dependencies(lcm_publisher lcm_types)
add_dependencies(udp_publisher lcm_types)
add_dependencies(udp_publisher lcm_types)
set(LCM_LIBRARIES "/usr/local/lib/liblcm.so.1.5.0")
target_link_libraries(lcm_service ${LCM_LIBRARIES})
target_link_libraries(lcm_publisher ${LCM_LIBRARIES})
target_link_libraries(udp_publisher ${LCM_LIBRARIES})

target_link_libraries(lcm_service ${TORCH_LIBRARIES})
target_link_libraries(lcm_publisher ${TORCH_LIBRARIES})
target_link_libraries(udp_publisher ${TORCH_LIBRARIES})
target_link_libraries(udp_publisher_tinker ${TORCH_LIBRARIES})
# udp_test, jetson_full_test, calibrate_robot 不需要链接PyTorch库，只需要标准C++库
