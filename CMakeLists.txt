cmake_minimum_required(VERSION 3.10)

# 设置libc类型为uclibc
set(LIBC_TYPE "uclibc")
message(STATUS "Using libc type: ${LIBC_TYPE}")

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

# 原方案：使用外部 Luckfox SDK（需要时取消下方注释，并注释“当前方案”中的编译器设置）。
# set(SDK_PATH "/home/mingxing/luckfox-pico")
# message(STATUS "Using SDK path: ${SDK_PATH}")
# set(CMAKE_C_COMPILER "${SDK_PATH}/tools/linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf/bin/arm-rockchip830-linux-uclibcgnueabihf-gcc")
# set(CMAKE_CXX_COMPILER "${SDK_PATH}/tools/linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf/bin/arm-rockchip830-linux-uclibcgnueabihf-g++")

# 当前方案：使用项目内置的 RV1106/uClibc 交叉工具链。
set(TOOLCHAIN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/toolchain/arm-rockchip830-linux-uclibcgnueabihf")
set(TOOLCHAIN_PREFIX "${TOOLCHAIN_ROOT}/bin/arm-rockchip830-linux-uclibcgnueabihf-")
if(NOT EXISTS "${TOOLCHAIN_PREFIX}gcc" OR NOT EXISTS "${TOOLCHAIN_PREFIX}g++")
    message(FATAL_ERROR "项目内交叉工具链不完整: ${TOOLCHAIN_ROOT}")
endif()
set(CMAKE_C_COMPILER "${TOOLCHAIN_PREFIX}gcc")
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_PREFIX}g++")
message(STATUS "Using project toolchain: ${TOOLCHAIN_ROOT}")

# 固定项目名称和源代码目录
set(PROJECT_NAME "Intelligent_Recognition_Client")
set(EXAMPLE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/code")
message(STATUS "Fixed project name: ${PROJECT_NAME}")
message(STATUS "Fixed source directory: ${EXAMPLE_DIR}")

# 设置项目名称
project(${PROJECT_NAME})

# 设置运行时库路径和定义RV1106_1103宏
set(CMAKE_INSTALL_RPATH "$ORIGIN/lib")
add_definitions(-DRV1106_1103)

# 链接库目录
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/${LIBC_TYPE})

# 查找OpenCV库
set(OpenCV_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/${LIBC_TYPE}/lib/cmake/opencv4")
find_package(OpenCV REQUIRED)
# 查找线程库
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# 获取源文件
file(GLOB SRC_CONTROL "${EXAMPLE_DIR}/modules/control/*.cc")
file(GLOB SRC_SERVO "${EXAMPLE_DIR}/modules/servo/*.cc")
file(GLOB SRC_TCP "${EXAMPLE_DIR}/modules/tcp/*.cc")
file(GLOB SRC_VIDEO "${EXAMPLE_DIR}/modules/video/*.cc")
file(GLOB SRC_YOLO "${EXAMPLE_DIR}/modules/yolo/*.cc")
file(GLOB SRC_UDP_DISCOVERY "${EXAMPLE_DIR}/modules/udp_discovery/*.cc")  # UDP发现模块
file(GLOB SRC_MAIN "${EXAMPLE_DIR}/src/*.cc")
# DMA缓冲区分配器（RGA硬件加速所需，提供dma_buf_alloc/dma_buf_free）
set(SRC_DMA_ALLOC "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/allocator/dma/dma_alloc.cpp")
set(SRC_MP4_MUX "${EXAMPLE_DIR}/modules/video/mp4_mux.c")

set(SRC_FILES
    ${SRC_CONTROL}
    ${SRC_SERVO}
    ${SRC_TCP}
    ${SRC_VIDEO}
    ${SRC_YOLO}
    ${SRC_UDP_DISCOVERY}  # 添加UDP发现模块
    ${SRC_DMA_ALLOC}      # DMA缓冲区分配器
    ${SRC_MP4_MUX}        # 无外部运行时依赖的 MP4 封装器
    ${SRC_MAIN}
)
add_executable(${PROJECT_NAME} ${SRC_FILES})

# 设置编译选项
add_compile_options(-g -Wall
                    -DISP_HW_V30 -DRKPLATFORM=ON -DARCH64=OFF
                    -DROCKIVA -DUAPI2
                    -D_LARGEFILE_SOURCE -D_LARGEFILE64_SOURCE -D_FILE_OFFSET_BITS=64 
                    )

# 设置链接选项
set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath-link,${CMAKE_CURRENT_SOURCE_DIR}/lib:/usr/lib")

# 链接库
target_link_libraries(${PROJECT_NAME}
                    ${OpenCV_LIBS}        # OpenCV库
                    rknnmrt              # RKNN神经网络推理库
                    Threads::Threads      # 线程库
                    rockiva              # 智能视频分析库
                    sample_comm          # 示例公共代码库
                    rockit               # 瑞芯微媒体处理接口
                    rockchip_mpp         # 瑞芯微媒体处理平台
                    rkaiq                # 瑞芯微图像质量调整库
                    pthread              # POSIX线程库
                    rtsp                 # RTSP流媒体库
                    rga                  # 瑞芯微图形加速器
                    )

# 设置包含目录
target_include_directories(${PROJECT_NAME} PRIVATE                        
                            ${OpenCV_INCLUDE_DIRS}
                            ${EXAMPLE_DIR}/include
                            ${EXAMPLE_DIR}/modules/control 
                            ${EXAMPLE_DIR}/modules/servo
                            ${EXAMPLE_DIR}/modules/tcp
                            ${EXAMPLE_DIR}/modules/video
                            ${EXAMPLE_DIR}/modules/yolo
                            ${EXAMPLE_DIR}/modules/udp_discovery  # UDP发现模块头文件
                            ${CMAKE_CURRENT_SOURCE_DIR}
                            ${CMAKE_CURRENT_SOURCE_DIR}/utils
                            ${CMAKE_CURRENT_SOURCE_DIR}/common 
                            ${CMAKE_CURRENT_SOURCE_DIR}/common/isp3.x   
                            ${CMAKE_CURRENT_SOURCE_DIR}/include
                            ${CMAKE_CURRENT_SOURCE_DIR}/include/rknn
                            ${CMAKE_CURRENT_SOURCE_DIR}/include/librga 
                            ${CMAKE_CURRENT_SOURCE_DIR}/include/rkaiq
                            ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/allocator/dma
                            ${CMAKE_CURRENT_SOURCE_DIR}/include/rkaiq/uAPI2
                            ${CMAKE_CURRENT_SOURCE_DIR}/include/rkaiq/common
                            ${CMAKE_CURRENT_SOURCE_DIR}/include/rkaiq/xcore
                            ${CMAKE_CURRENT_SOURCE_DIR}/include/rkaiq/algos
                            ${CMAKE_CURRENT_SOURCE_DIR}/include/rkaiq/iq_parser
                            ${CMAKE_CURRENT_SOURCE_DIR}/include/rkaiq/iq_parser_v2
                            ${CMAKE_CURRENT_SOURCE_DIR}/include/rkaiq/smartIr
                          )
message(STATUS "model dir PATH: ${EXAMPLE_DIR}/model")


# 设置安装前缀，所有安装内容将被放置到该目录下
set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/install/")
# 安装可执行文件到指定的安装目录
install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX})
# 设置模型文件所在目录
SET(MODEL_DIR "${EXAMPLE_DIR}/model")
# 搜索模型目录下的所有文件，保存到MODEL_FILES变量中
file(GLOB MODEL_FILES "${MODEL_DIR}/*")  
# 安装所有模型文件到安装目录下的model子目录
install(FILES ${MODEL_FILES} DESTINATION ${CMAKE_INSTALL_PREFIX}/model)
