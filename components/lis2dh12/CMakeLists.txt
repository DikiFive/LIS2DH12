# 定义 ESP-IDF 组件的注册信息
# SRCS: 指定组件的源文件
# INCLUDE_DIRS: 指定组件的头文件目录
idf_component_register(SRCS "lis2dh12.c"
                        INCLUDE_DIRS include)

# 如果配置了包含 IMU 传感器（通过 Kconfig 选项 CONFIG_SENSOR_INCLUDED_IMU）
if(CONFIG_SENSOR_INCLUDED_IMU)
    # 将 imu_lis2dh12_init 函数添加到链接库中，确保其不会被链接器优化掉
    # INTERFACE 关键字表示这个库是接口库，其依赖项会传递给使用它的目标
    target_link_libraries(${COMPONENT_LIB} INTERFACE "-u imu_lis2dh12_init")
endif()

# 包含 package_manager 模块，用于处理组件依赖和版本管理
include(package_manager)
# 定义当前组件的版本信息，通常从 idf_component.yml 中读取
cu_pkg_define_version(${CMAKE_CURRENT_LIST_DIR})
