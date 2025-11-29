# toolchain.cmake

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# 指定工具链前缀
set(TOOLCHAIN_PREFIX aarch64-linux-gnu-)

# 指定编译器
# 假设编译器已经添加到 PATH 环境变量中。如果没有，请修改为编译器的绝对路径。
set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++)

# 指定 Sysroot 路径
set(CMAKE_SYSROOT /home/lese/ti-processor-sdk-linux-am62lxx-evm-11.01.16.13/linux-devkit/sysroots/aarch64-oe-linux)

# 设置查找路径
set(CMAKE_FIND_ROOT_PATH ${CMAKE_SYSROOT})

# 交叉编译时的查找规则
# 查找程序（如 make, python）时，不在 Sysroot 中查找（使用宿主机的工具）
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# 查找库、头文件、包时，只在 Sysroot 中查找
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
