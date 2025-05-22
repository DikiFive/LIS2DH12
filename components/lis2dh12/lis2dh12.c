/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file lis2dh12.c
 * @brief LIS2DH12 加速度传感器驱动实现
 *
 * 本文件包含了 LIS2DH12 传感器的 I2C 驱动函数，
 * 用于初始化、配置、读取设备ID和加速度数据等操作。
 */

#include <stdio.h>          // 标准输入输出库
#include "lis2dh12.h"       // LIS2DH12 传感器驱动头文件
#include "esp_log.h"        // ESP-IDF 日志库
#include "iot_sensor_hub.h" // IoT 传感器中心库

static const char *TAG = "lis2dh12"; // 用于日志输出的标签

// 宏定义：用于检查条件并返回错误
#define IOT_CHECK(tag, a, ret)                                         \
    if (!(a))                                                          \
    {                                                                  \
        ESP_LOGE(tag, "%s:%d (%s)", __FILE__, __LINE__, __FUNCTION__); \
        return (ret);                                                  \
    }

// 宏定义：断言表达式为 ESP_OK，否则返回 ESP_FAIL
#define ERR_ASSERT(tag, param) IOT_CHECK(tag, (param) == ESP_OK, ESP_FAIL)
// 宏定义：断言指针不为空，否则返回 ESP_FAIL
#define POINT_ASSERT(tag, param) IOT_CHECK(tag, (param) != NULL, ESP_FAIL)
// 宏定义：断言 FreeRTOS 资源操作成功，否则返回指定错误码
#define RES_ASSERT(tag, res, ret) IOT_CHECK(tag, (res) != pdFALSE, ret)

/**
 * @brief LIS2DH12 设备结构体
 */
typedef struct
{
    i2c_bus_device_handle_t i2c_dev; // I2C 总线设备句柄
    uint8_t dev_addr;                // LIS2DH12 设备的 I2C 地址
} lis2dh12_dev_t;

/**
 * @brief 创建 LIS2DH12 传感器设备句柄
 *
 * @param bus I2C 总线句柄
 * @param dev_addr LIS2DH12 设备的 I2C 地址
 * @return lis2dh12_handle_t LIS2DH12 传感器句柄，失败返回 NULL
 */
lis2dh12_handle_t lis2dh12_create(i2c_bus_handle_t bus, uint8_t dev_addr)
{
    // 分配 LIS2DH12 设备结构体内存并清零
    lis2dh12_dev_t *sens = (lis2dh12_dev_t *)calloc(1, sizeof(lis2dh12_dev_t));
    if (sens == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for LIS2DH12 device");
        return NULL;
    }

    // 创建 I2C 总线设备
    sens->i2c_dev = i2c_bus_device_create(bus, dev_addr, i2c_bus_get_current_clk_speed(bus));
    if (sens->i2c_dev == NULL)
    {
        free(sens); // 创建失败则释放已分配的内存
        ESP_LOGE(TAG, "Failed to create I2C bus device for LIS2DH12");
        return NULL;
    }
    sens->dev_addr = dev_addr;      // 保存设备地址
    return (lis2dh12_handle_t)sens; // 返回传感器句柄
}

/**
 * @brief 删除 LIS2DH12 传感器设备句柄
 *
 * @param sensor 指向 LIS2DH12 传感器句柄的指针
 * @return esp_err_t ESP_OK 成功，其他失败
 */
esp_err_t lis2dh12_delete(lis2dh12_handle_t *sensor)
{
    // 如果传感器句柄为空，直接返回成功
    if (*sensor == NULL)
    {
        return ESP_OK;
    }

    lis2dh12_dev_t *sens = (lis2dh12_dev_t *)(*sensor); // 获取设备结构体指针
    i2c_bus_device_delete(&sens->i2c_dev);              // 删除 I2C 总线设备
    free(sens);                                         // 释放设备结构体内存
    *sensor = NULL;                                     // 将句柄置空
    return ESP_OK;
}

/**
 * @brief 获取 LIS2DH12 设备ID
 *
 * @param sensor LIS2DH12 传感器句柄
 * @param deviceid 用于存储设备ID的指针
 * @return esp_err_t ESP_OK 成功，其他失败
 */
esp_err_t lis2dh12_get_deviceid(lis2dh12_handle_t sensor, uint8_t *deviceid)
{
    lis2dh12_dev_t *sens = (lis2dh12_dev_t *)sensor; // 获取设备结构体指针
    uint8_t tmp;                                     // 临时变量用于存储读取的字节
    // 从 LIS2DH12_WHO_AM_I_REG 寄存器读取设备ID
    ERR_ASSERT(TAG, i2c_bus_read_byte(sens->i2c_dev, LIS2DH12_WHO_AM_I_REG, &tmp));
    *deviceid = tmp; // 将读取到的设备ID赋值给指针
    return ESP_OK;
}

/**
 * @brief 设置 LIS2DH12 传感器配置
 *
 * @param sensor LIS2DH12 传感器句柄
 * @param lis2dh12_config 指向 LIS2DH12 配置结构体的指针
 * @return esp_err_t ESP_OK 成功，其他失败
 */
esp_err_t lis2dh12_set_config(lis2dh12_handle_t sensor, lis2dh12_config_t *lis2dh12_config)
{
    lis2dh12_dev_t *sens = (lis2dh12_dev_t *)sensor; // 获取设备结构体指针
    uint8_t buffer[6];                               // 缓冲区用于读写多个寄存器

    // 配置温度传感器使能状态 (TEMP_CFG_REG)
    ERR_ASSERT(TAG, i2c_bus_read_byte(sens->i2c_dev, LIS2DH12_TEMP_CFG_REG, buffer));     // 读取当前温度配置寄存器
    buffer[0] &= ~LIS2DH12_TEMP_EN_MASK;                                                  // 清除温度使能位
    buffer[0] |= 0xC0;                                                                    // 设置温度使能位为0xC0
    ERR_ASSERT(TAG, i2c_bus_write_byte(sens->i2c_dev, LIS2DH12_TEMP_CFG_REG, buffer[0])); // 写入温度配置寄存器

    // 等待温度传感器启动
    vTaskDelay(pdMS_TO_TICKS(100));

    // 配置 CTRL_REG1 到 CTRL_REG6 (多字节读写)
    ERR_ASSERT(TAG, i2c_bus_read_bytes(sens->i2c_dev, LIS2DH12_CTRL_REG1 | LIS2DH12_I2C_MULTI_REG_ONCE, 6, buffer)); // 读取 CTRL_REG1 到 CTRL_REG6

    // 配置 CTRL_REG1 (ODR, LP_EN, XYZ_EN)
    buffer[0] &= (uint8_t)~(LIS2DH12_ODR_MASK | LIS2DH12_LP_EN_MASK | LIS2DH12_Z_EN_MASK | LIS2DH12_Y_EN_MASK | LIS2DH12_X_EN_MASK);     // 清除相关位
    buffer[0] |= ((uint8_t)lis2dh12_config->odr) << LIS2DH12_ODR_BIT;                                                                    // 设置输出数据速率
    buffer[0] |= ((uint8_t)((lis2dh12_config->opt_mode == LIS2DH12_OPT_LOW_POWER) ? 1 : 0) << LIS2DH12_LP_EN_BIT) & LIS2DH12_LP_EN_MASK; // 设置低功耗使能
    buffer[0] |= ((uint8_t)lis2dh12_config->z_enable) << LIS2DH12_Z_EN_BIT;                                                              // 设置Z轴使能
    buffer[0] |= ((uint8_t)lis2dh12_config->y_enable) << LIS2DH12_Y_EN_BIT;                                                              // 设置Y轴使能
    buffer[0] |= ((uint8_t)lis2dh12_config->x_enable) << LIS2DH12_X_EN_BIT;                                                              // 设置X轴使能

    // 配置 CTRL_REG4 (BDU, FS, HR)
    buffer[3] &= ~(LIS2DH12_BDU_MASK | LIS2DH12_FS_MASK | LIS2DH12_HR_MASK);                                                      // 清除相关位
    buffer[3] |= ((uint8_t)lis2dh12_config->bdu_status) << LIS2DH12_BDU_BIT;                                                      // 设置块数据更新状态
    buffer[3] |= ((uint8_t)lis2dh12_config->fs) << LIS2DH12_FS_BIT;                                                               // 设置满量程
    buffer[3] |= ((uint8_t)((lis2dh12_config->opt_mode == LIS2DH12_OPT_HIGH_RES) ? 1 : 0) << LIS2DH12_HR_BIT) & LIS2DH12_HR_MASK; // 设置高分辨率模式
    ERR_ASSERT(TAG, i2c_bus_write_bytes(sens->i2c_dev, LIS2DH12_CTRL_REG1 | LIS2DH12_I2C_MULTI_REG_ONCE, 6, buffer));             // 写入 CTRL_REG1 到 CTRL_REG6
    return ESP_OK;
}

/**
 * @brief 获取 LIS2DH12 传感器当前配置
 *
 * @param sensor LIS2DH12 传感器句柄
 * @param lis2dh12_config 指向 LIS2DH12 配置结构体的指针，用于存储读取到的配置
 * @return esp_err_t ESP_OK 成功，其他失败
 */
esp_err_t lis2dh12_get_config(lis2dh12_handle_t sensor, lis2dh12_config_t *lis2dh12_config)
{
    lis2dh12_dev_t *sens = (lis2dh12_dev_t *)sensor; // 获取设备结构体指针
    uint8_t buffer[6];                               // 缓冲区用于读写多个寄存器

    // 读取温度传感器使能状态 (TEMP_CFG_REG)
    ERR_ASSERT(TAG, i2c_bus_read_bytes(sens->i2c_dev, LIS2DH12_TEMP_CFG_REG, 1, buffer));
    lis2dh12_config->temp_enable = (lis2dh12_temp_en_t)((buffer[0] & LIS2DH12_TEMP_EN_MASK) >> LIS2DH12_TEMP_EN_BIT);

    // 读取 CTRL_REG1 到 CTRL_REG6 (多字节读写)
    ERR_ASSERT(TAG, i2c_bus_read_bytes(sens->i2c_dev, LIS2DH12_CTRL_REG1 | LIS2DH12_I2C_MULTI_REG_ONCE, 6, buffer));
    lis2dh12_config->odr = (lis2dh12_odr_t)((buffer[0] & LIS2DH12_ODR_MASK) >> LIS2DH12_ODR_BIT);          // 获取输出数据速率
    lis2dh12_config->z_enable = (lis2dh12_state_t)((buffer[0] & LIS2DH12_Z_EN_MASK) >> LIS2DH12_Z_EN_BIT); // 获取Z轴使能状态
    lis2dh12_config->y_enable = (lis2dh12_state_t)((buffer[0] & LIS2DH12_Y_EN_MASK) >> LIS2DH12_Y_EN_BIT); // 获取Y轴使能状态
    lis2dh12_config->x_enable = (lis2dh12_state_t)((buffer[0] & LIS2DH12_X_EN_MASK) >> LIS2DH12_X_EN_BIT); // 获取X轴使能状态
    lis2dh12_config->bdu_status = (lis2dh12_state_t)((buffer[3] & LIS2DH12_BDU_MASK) >> LIS2DH12_BDU_BIT); // 获取块数据更新状态
    lis2dh12_config->fs = (lis2dh12_fs_t)((buffer[3] & LIS2DH12_FS_MASK) >> LIS2DH12_FS_BIT);              // 获取满量程
    // 获取操作模式 (结合低功耗和高分辨率位)
    lis2dh12_config->opt_mode = (lis2dh12_opt_mode_t)((((buffer[0] & LIS2DH12_LP_EN_MASK) << 1) >> LIS2DH12_LP_EN_BIT) | ((buffer[3] & LIS2DH12_HR_MASK) >> LIS2DH12_HR_BIT));
    return ESP_OK;
}

/**
 * @brief 设置温度传感器使能状态
 *
 * @param sensor LIS2DH12 传感器句柄
 * @param temp_en 温度使能状态
 * @return esp_err_t ESP_OK 成功，其他失败
 */
esp_err_t lis2dh12_set_temp_enable(lis2dh12_handle_t sensor, lis2dh12_temp_en_t temp_en)
{
    lis2dh12_dev_t *sens = (lis2dh12_dev_t *)sensor;                                // 获取设备结构体指针
    uint8_t tmp;                                                                    // 临时变量
    ERR_ASSERT(TAG, i2c_bus_read_byte(sens->i2c_dev, LIS2DH12_TEMP_CFG_REG, &tmp)); // 读取温度配置寄存器
    tmp &= ~LIS2DH12_TEMP_EN_MASK;                                                  // 清除温度使能位
    tmp |= ((uint8_t)temp_en) << LIS2DH12_TEMP_EN_BIT;                              // 设置新的温度使能位
    ERR_ASSERT(TAG, i2c_bus_write_byte(sens->i2c_dev, LIS2DH12_TEMP_CFG_REG, tmp)); // 写入温度配置寄存器
    return ESP_OK;
}

/**
 * @brief 设置输出数据速率 (ODR)
 *
 * @param sensor LIS2DH12 传感器句柄
 * @param odr 输出数据速率
 * @return esp_err_t ESP_OK 成功，其他失败
 */
esp_err_t lis2dh12_set_odr(lis2dh12_handle_t sensor, lis2dh12_odr_t odr)
{
    lis2dh12_dev_t *sens = (lis2dh12_dev_t *)sensor;                             // 获取设备结构体指针
    uint8_t tmp;                                                                 // 临时变量
    ERR_ASSERT(TAG, i2c_bus_read_byte(sens->i2c_dev, LIS2DH12_CTRL_REG1, &tmp)); // 读取 CTRL_REG1
    tmp &= ~LIS2DH12_ODR_MASK;                                                   // 清除 ODR 位
    tmp |= ((uint8_t)odr) << LIS2DH12_ODR_BIT;                                   // 设置新的 ODR
    ERR_ASSERT(TAG, i2c_bus_write_byte(sens->i2c_dev, LIS2DH12_CTRL_REG1, tmp)); // 写入 CTRL_REG1
    return ESP_OK;
}

/**
 * @brief 设置 Z 轴使能状态
 *
 * @param sensor LIS2DH12 传感器句柄
 * @param status Z 轴使能状态
 * @return esp_err_t ESP_OK 成功，其他失败
 */
esp_err_t lis2dh12_set_z_enable(lis2dh12_handle_t sensor, lis2dh12_state_t status)
{
    lis2dh12_dev_t *sens = (lis2dh12_dev_t *)sensor;                             // 获取设备结构体指针
    uint8_t tmp;                                                                 // 临时变量
    ERR_ASSERT(TAG, i2c_bus_read_byte(sens->i2c_dev, LIS2DH12_CTRL_REG1, &tmp)); // 读取 CTRL_REG1
    tmp &= ~LIS2DH12_Z_EN_MASK;                                                  // 清除 Z 轴使能位
    tmp |= ((uint8_t)status) << LIS2DH12_Z_EN_BIT;                               // 设置新的 Z 轴使能状态
    ERR_ASSERT(TAG, i2c_bus_write_byte(sens->i2c_dev, LIS2DH12_CTRL_REG1, tmp)); // 写入 CTRL_REG1
    return ESP_OK;
}

/**
 * @brief 设置 Y 轴使能状态
 *
 * @param sensor LIS2DH12 传感器句柄
 * @param status Y 轴使能状态
 * @return esp_err_t ESP_OK 成功，其他失败
 */
esp_err_t lis2dh12_set_y_enable(lis2dh12_handle_t sensor, lis2dh12_state_t status)
{
    lis2dh12_dev_t *sens = (lis2dh12_dev_t *)sensor;                             // 获取设备结构体指针
    uint8_t tmp;                                                                 // 临时变量
    ERR_ASSERT(TAG, i2c_bus_read_byte(sens->i2c_dev, LIS2DH12_CTRL_REG1, &tmp)); // 读取 CTRL_REG1
    tmp &= ~LIS2DH12_Y_EN_MASK;                                                  // 清除 Y 轴使能位
    tmp |= ((uint8_t)status) << LIS2DH12_Y_EN_BIT;                               // 设置新的 Y 轴使能状态
    ERR_ASSERT(TAG, i2c_bus_write_byte(sens->i2c_dev, LIS2DH12_CTRL_REG1, tmp)); // 写入 CTRL_REG1
    return ESP_OK;
}

/**
 * @brief 设置 X 轴使能状态
 *
 * @param sensor LIS2DH12 传感器句柄
 * @param status X 轴使能状态
 * @return esp_err_t ESP_OK 成功，其他失败
 */
esp_err_t lis2dh12_set_x_enable(lis2dh12_handle_t sensor, lis2dh12_state_t status)
{
    lis2dh12_dev_t *sens = (lis2dh12_dev_t *)sensor;                             // 获取设备结构体指针
    uint8_t tmp;                                                                 // 临时变量
    ERR_ASSERT(TAG, i2c_bus_read_byte(sens->i2c_dev, LIS2DH12_CTRL_REG1, &tmp)); // 读取 CTRL_REG1
    tmp &= ~LIS2DH12_X_EN_MASK;                                                  // 清除 X 轴使能位
    tmp |= ((uint8_t)status) << LIS2DH12_X_EN_BIT;                               // 设置新的 X 轴使能状态
    ERR_ASSERT(TAG, i2c_bus_write_byte(sens->i2c_dev, LIS2DH12_CTRL_REG1, tmp)); // 写入 CTRL_REG1
    return ESP_OK;
}

/**
 * @brief 设置块数据更新 (BDU) 模式
 *
 * @param sensor LIS2DH12 传感器句柄
 * @param status BDU 状态
 * @return esp_err_t ESP_OK 成功，其他失败
 */
esp_err_t lis2dh12_set_bdumode(lis2dh12_handle_t sensor, lis2dh12_state_t status)
{
    lis2dh12_dev_t *sens = (lis2dh12_dev_t *)sensor;                             // 获取设备结构体指针
    uint8_t tmp;                                                                 // 临时变量
    ERR_ASSERT(TAG, i2c_bus_read_byte(sens->i2c_dev, LIS2DH12_CTRL_REG4, &tmp)); // 读取 CTRL_REG4
    tmp &= ~LIS2DH12_BDU_MASK;                                                   // 清除 BDU 位
    tmp |= ((uint8_t)status) << LIS2DH12_BDU_BIT;                                // 设置新的 BDU 状态
    ERR_ASSERT(TAG, i2c_bus_write_byte(sens->i2c_dev, LIS2DH12_CTRL_REG4, tmp)); // 写入 CTRL_REG4
    return ESP_OK;
}

/**
 * @brief 设置满量程 (Full Scale)
 *
 * @param sensor LIS2DH12 传感器句柄
 * @param fs 满量程值
 * @return esp_err_t ESP_OK 成功，其他失败
 */
esp_err_t lis2dh12_set_fs(lis2dh12_handle_t sensor, lis2dh12_fs_t fs)
{
    lis2dh12_dev_t *sens = (lis2dh12_dev_t *)sensor;                             // 获取设备结构体指针
    uint8_t tmp;                                                                 // 临时变量
    ERR_ASSERT(TAG, i2c_bus_read_byte(sens->i2c_dev, LIS2DH12_CTRL_REG4, &tmp)); // 读取 CTRL_REG4
    tmp &= ~LIS2DH12_FS_MASK;                                                    // 清除 FS 位
    tmp |= ((uint8_t)fs) << LIS2DH12_FS_BIT;                                     // 设置新的 FS
    ERR_ASSERT(TAG, i2c_bus_write_byte(sens->i2c_dev, LIS2DH12_CTRL_REG4, tmp)); // 写入 CTRL_REG4
    return ESP_OK;
}

/**
 * @brief 获取满量程 (Full Scale)
 *
 * @param sensor LIS2DH12 传感器句柄
 * @param fs 指向满量程值的指针
 * @return esp_err_t ESP_OK 成功，其他失败
 */
esp_err_t lis2dh12_get_fs(lis2dh12_handle_t sensor, lis2dh12_fs_t *fs)
{
    lis2dh12_dev_t *sens = (lis2dh12_dev_t *)sensor;                             // 获取设备结构体指针
    uint8_t tmp;                                                                 // 临时变量
    ERR_ASSERT(TAG, i2c_bus_read_byte(sens->i2c_dev, LIS2DH12_CTRL_REG4, &tmp)); // 读取 CTRL_REG4
    *fs = (lis2dh12_fs_t)((tmp & LIS2DH12_FS_MASK) >> LIS2DH12_FS_BIT);          // 解析 FS 位
    return ESP_OK;
}

/**
 * @brief 设置操作模式 (Operation Mode)
 *
 * @param sensor LIS2DH12 传感器句柄
 * @param opt_mode 操作模式
 * @return esp_err_t ESP_OK 成功，其他失败
 */
esp_err_t lis2dh12_set_opt_mode(lis2dh12_handle_t sensor, lis2dh12_opt_mode_t opt_mode)
{
    lis2dh12_dev_t *sens = (lis2dh12_dev_t *)sensor;                                  // 获取设备结构体指针
    uint8_t tmp1, tmp2;                                                               // 临时变量
    ERR_ASSERT(TAG, i2c_bus_read_byte(sens->i2c_dev, LIS2DH12_CTRL_REG1, &tmp1));     // 读取 CTRL_REG1
    ERR_ASSERT(TAG, i2c_bus_read_byte(sens->i2c_dev, LIS2DH12_CTRL_REG4, &tmp2));     // 读取 CTRL_REG4
    tmp1 &= ~LIS2DH12_LP_EN_MASK;                                                     // 清除低功耗使能位
    tmp1 |= ((uint8_t)((opt_mode >> 1) << LIS2DH12_LP_EN_BIT) & LIS2DH12_LP_EN_MASK); // 设置低功耗使能位
    tmp2 &= ~LIS2DH12_HR_MASK;                                                        // 清除高分辨率位
    tmp2 |= ((uint8_t)opt_mode & LIS2DH12_HR_MASK) << LIS2DH12_HR_BIT;                // 设置高分辨率位
    ERR_ASSERT(TAG, i2c_bus_write_byte(sens->i2c_dev, LIS2DH12_CTRL_REG1, tmp1));     // 写入 CTRL_REG1
    ERR_ASSERT(TAG, i2c_bus_write_byte(sens->i2c_dev, LIS2DH12_CTRL_REG4, tmp2));     // 写入 CTRL_REG4
    return ESP_OK;
}

/**
 * @brief 获取 X 轴加速度原始值
 *
 * @param sensor LIS2DH12 传感器句柄
 * @param x_acc 指向 X 轴加速度原始值的指针
 * @return esp_err_t ESP_OK 成功，其他失败
 */
esp_err_t lis2dh12_get_x_acc(lis2dh12_handle_t sensor, uint16_t *x_acc)
{
    lis2dh12_dev_t *sens = (lis2dh12_dev_t *)sensor; // 获取设备结构体指针
    uint8_t buffer[2];                               // 缓冲区用于存储读取的字节
    // 从 LIS2DH12_OUT_X_L_REG 寄存器开始读取 2 字节（X轴低字节和高字节）
    ERR_ASSERT(TAG, i2c_bus_read_bytes(sens->i2c_dev, LIS2DH12_OUT_X_L_REG | LIS2DH12_I2C_MULTI_REG_ONCE, 2, buffer));
    // 将两个字节组合成 16 位有符号整数
    *x_acc = (int16_t)((((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0]);
    return ESP_OK;
}

/**
 * @brief 获取 Y 轴加速度原始值
 *
 * @param sensor LIS2DH12 传感器句柄
 * @param y_acc 指向 Y 轴加速度原始值的指针
 * @return esp_err_t ESP_OK 成功，其他失败
 */
esp_err_t lis2dh12_get_y_acc(lis2dh12_handle_t sensor, uint16_t *y_acc)
{
    lis2dh12_dev_t *sens = (lis2dh12_dev_t *)sensor; // 获取设备结构体指针
    uint8_t buffer[2];                               // 缓冲区用于存储读取的字节
    // 从 LIS2DH12_OUT_Y_L_REG 寄存器开始读取 2 字节（Y轴低字节和高字节）
    ERR_ASSERT(TAG, i2c_bus_read_bytes(sens->i2c_dev, LIS2DH12_OUT_Y_L_REG | LIS2DH12_I2C_MULTI_REG_ONCE, 2, buffer));
    // 将两个字节组合成 16 位有符号整数
    *y_acc = (int16_t)((((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0]);
    return ESP_OK;
}

/**
 * @brief 获取 Z 轴加速度原始值
 *
 * @param sensor LIS2DH12 传感器句柄
 * @param z_acc 指向 Z 轴加速度原始值的指针
 * @return esp_err_t ESP_OK 成功，其他失败
 */
esp_err_t lis2dh12_get_z_acc(lis2dh12_handle_t sensor, uint16_t *z_acc)
{
    lis2dh12_dev_t *sens = (lis2dh12_dev_t *)sensor; // 获取设备结构体指针
    uint8_t buffer[2];                               // 缓冲区用于存储读取的字节
    // 从 LIS2DH12_OUT_Z_L_REG 寄存器开始读取 2 字节（Z轴低字节和高字节）
    ERR_ASSERT(TAG, i2c_bus_read_bytes(sens->i2c_dev, LIS2DH12_OUT_Z_L_REG | LIS2DH12_I2C_MULTI_REG_ONCE, 2, buffer));
    // 将两个字节组合成 16 位有符号整数
    *z_acc = (int16_t)((((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0]);
    return ESP_OK;
}

/**
 * @brief 获取 LIS2DH12 温度数据
 *
 * @param sensor LIS2DH12 传感器句柄
 * @param temperature 指向温度值的指针 (单位：摄氏度)
 * @return esp_err_t ESP_OK 成功，其他失败
 */
esp_err_t lis2dh12_get_temp(lis2dh12_handle_t sensor, float *temperature, int16_t *raw_temperature)
{
    lis2dh12_dev_t *sens = (lis2dh12_dev_t *)sensor; // 获取设备结构体指针
    uint8_t buffer[2];                               // 缓冲区用于存储读取的字节
    int16_t raw_temp;                                // 原始温度值
    uint8_t status;                                  // 状态寄存器值

    // 读取状态寄存器，检查温度数据是否可用
    ERR_ASSERT(TAG, i2c_bus_read_byte(sens->i2c_dev, LIS2DH12_STATUS_REG_AUX_REG, &status));
    if (!(status & LIS2DH12_TDA_MASK))
    {
        ESP_LOGW(TAG, "Temperature data not available yet");
        return ESP_ERR_NOT_FOUND;
    }

    // 从 LIS2DH12_OUT_TEMP_L_REG 寄存器开始读取 2 字节（温度低字节和高字节）
    ERR_ASSERT(TAG, i2c_bus_read_bytes(sens->i2c_dev, LIS2DH12_OUT_TEMP_L_REG | LIS2DH12_I2C_MULTI_REG_ONCE, 2, buffer));
    // 将两个字节组合成 16 位有符号整数
    raw_temp = (int16_t)((((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0]);

    if (raw_temperature)
    {
        *raw_temperature = raw_temp;
    }

    // 将原始温度值转换为摄氏度
    *temperature = LIS2DH12_FROM_LSB_TO_degC_HR(raw_temp);
    return ESP_OK;
}

/**
 * @brief 获取 LIS2DH12 原始加速度值 (X, Y, Z 三轴)
 *
 * @param sensor LIS2DH12 传感器句柄
 * @param raw_acce_value 指向原始加速度值结构体的指针
 * @return esp_err_t ESP_OK 成功，其他失败
 */
esp_err_t lis2dh12_get_raw_acce(lis2dh12_handle_t sensor, lis2dh12_raw_acce_value_t *raw_acce_value)
{
    lis2dh12_dev_t *sens = (lis2dh12_dev_t *)sensor; // 获取设备结构体指针
    uint8_t buffer[6];                               // 缓冲区用于存储读取的 6 字节原始数据 (X, Y, Z 各 2 字节)
    // 从 LIS2DH12_OUT_X_L_REG 寄存器开始读取 6 字节
    ERR_ASSERT(TAG, i2c_bus_read_bytes(sens->i2c_dev, LIS2DH12_OUT_X_L_REG | LIS2DH12_I2C_MULTI_REG_ONCE, 6, buffer));
    // 将读取到的字节直接转换为 16 位有符号整数
    raw_acce_value->raw_acce_x = *(int16_t *)buffer;
    raw_acce_value->raw_acce_y = *(int16_t *)(buffer + 2);
    raw_acce_value->raw_acce_z = *(int16_t *)(buffer + 4);
    return ESP_OK;
}

/**
 * @brief 获取 LIS2DH12 加速度值 (转换为 g 单位)
 *
 * @param sensor LIS2DH12 传感器句柄
 * @param acce_value 指向加速度值结构体的指针 (包含 X, Y, Z 轴的 g 单位值)
 * @return esp_err_t ESP_OK 成功，其他失败
 */
esp_err_t lis2dh12_get_acce(lis2dh12_handle_t sensor, lis2dh12_acce_value_t *acce_value)
{
    lis2dh12_fs_t fs;                             // 满量程
    lis2dh12_raw_acce_value_t raw_acce_value;     // 原始加速度值
    esp_err_t ret = lis2dh12_get_fs(sensor, &fs); // 获取当前满量程设置

    if (ret != ESP_OK)
    {
        return ret; // 获取满量程失败则返回错误
    }

    ret = lis2dh12_get_raw_acce(sensor, &raw_acce_value); // 获取原始加速度值

    // 根据不同的满量程设置，将原始值转换为 g 单位
    if (fs == LIS2DH12_FS_2G)
    {
        acce_value->acce_x = LIS2DH12_FROM_FS_2g_HR_TO_mg(raw_acce_value.raw_acce_x) / 1000.0;
        acce_value->acce_y = LIS2DH12_FROM_FS_2g_HR_TO_mg(raw_acce_value.raw_acce_y) / 1000.0;
        acce_value->acce_z = LIS2DH12_FROM_FS_2g_HR_TO_mg(raw_acce_value.raw_acce_z) / 1000.0;
    }
    else if (fs == LIS2DH12_FS_4G)
    {
        acce_value->acce_x = LIS2DH12_FROM_FS_4g_HR_TO_mg(raw_acce_value.raw_acce_x) / 1000.0;
        acce_value->acce_y = LIS2DH12_FROM_FS_4g_HR_TO_mg(raw_acce_value.raw_acce_y) / 1000.0;
        acce_value->acce_z = LIS2DH12_FROM_FS_4g_HR_TO_mg(raw_acce_value.raw_acce_z) / 1000.0;
    }
    else if (fs == LIS2DH12_FS_8G)
    {
        acce_value->acce_x = LIS2DH12_FROM_FS_8g_HR_TO_mg(raw_acce_value.raw_acce_x) / 1000.0;
        acce_value->acce_y = LIS2DH12_FROM_FS_8g_HR_TO_mg(raw_acce_value.raw_acce_y) / 1000.0;
        acce_value->acce_z = LIS2DH12_FROM_FS_8g_HR_TO_mg(raw_acce_value.raw_acce_z) / 1000.0;
    }
    else if (fs == LIS2DH12_FS_16G)
    {
        acce_value->acce_x = LIS2DH12_FROM_FS_16g_HR_TO_mg(raw_acce_value.raw_acce_x) / 1000.0;
        acce_value->acce_y = LIS2DH12_FROM_FS_16g_HR_TO_mg(raw_acce_value.raw_acce_y) / 1000.0;
        acce_value->acce_z = LIS2DH12_FROM_FS_16g_HR_TO_mg(raw_acce_value.raw_acce_z) / 1000.0;
    }

    return ret;
}

/***sensors hal interface****/
#ifdef CONFIG_SENSOR_INCLUDED_IMU

static lis2dh12_handle_t lis2dh12 = NULL; // LIS2DH12 传感器句柄
static bool is_init = false;              // 传感器是否已初始化标志

/**
 * @brief IMU LIS2DH12 传感器初始化函数 (HAL 接口)
 *
 * @param i2c_bus I2C 总线句柄
 * @param addr LIS2DH12 设备的 I2C 地址
 * @return esp_err_t ESP_OK 成功，其他失败
 */
esp_err_t imu_lis2dh12_init(i2c_bus_handle_t i2c_bus, uint8_t addr)
{
    // 如果已经初始化或者 I2C 总线句柄无效，则返回失败
    if (is_init || !i2c_bus)
    {
        return ESP_FAIL;
    }

    // 创建 LIS2DH12 传感器设备
    lis2dh12 = lis2dh12_create(i2c_bus, addr);

    // 如果创建失败，返回失败
    if (!lis2dh12)
    {
        return ESP_FAIL;
    }

    uint8_t lis2dh12_deviceid;         // 设备ID
    lis2dh12_config_t lis2dh12_config; // 配置结构体
    // 获取设备ID并打印
    esp_err_t ret = lis2dh12_get_deviceid(lis2dh12, &lis2dh12_deviceid);
    ESP_LOGI(TAG, "lis2dh12 device address is: 0x%02x\n", lis2dh12_deviceid);

    // 设置 LIS2DH12 默认配置
    lis2dh12_config.temp_enable = LIS2DH12_TEMP_DISABLE;   // 禁用温度传感器
    lis2dh12_config.odr = LIS2DH12_ODR_100HZ;              // 设置输出数据速率为 100Hz
    lis2dh12_config.opt_mode = LIS2DH12_OPT_NORMAL;        // 设置为正常模式
    lis2dh12_config.z_enable = LIS2DH12_ENABLE;            // 使能Z轴
    lis2dh12_config.y_enable = LIS2DH12_ENABLE;            // 使能Y轴
    lis2dh12_config.x_enable = LIS2DH12_ENABLE;            // 使能X轴
    lis2dh12_config.bdu_status = LIS2DH12_DISABLE;         // 禁用块数据更新
    lis2dh12_config.fs = LIS2DH12_FS_4G;                   // 设置满量程为 ±4G
    ret = lis2dh12_set_config(lis2dh12, &lis2dh12_config); // 应用配置

    // 如果配置成功，设置初始化标志为 true
    if (ret == ESP_OK)
    {
        is_init = true;
    }

    return ret;
}

/**
 * @brief IMU LIS2DH12 传感器去初始化函数 (HAL 接口)
 *
 * @return esp_err_t ESP_OK 成功，其他失败
 */
esp_err_t imu_lis2dh12_deinit(void)
{
    // 如果未初始化，直接返回成功
    if (!is_init)
    {
        return ESP_OK;
    }

    // 删除 LIS2DH12 传感器设备
    esp_err_t ret = lis2dh12_delete(&lis2dh12);

    // 如果删除成功，设置初始化标志为 false
    if (ret == ESP_OK)
    {
        is_init = false;
    }

    return ret;
}

/**
 * @brief IMU LIS2DH12 传感器测试函数 (HAL 接口)
 *
 * @return esp_err_t ESP_OK 成功，其他失败
 */
esp_err_t imu_lis2dh12_test(void)
{
    // 如果未初始化，返回失败
    if (!is_init)
    {
        return ESP_FAIL;
    }

    return ESP_OK; // 返回成功
}

/**
 * @brief IMU LIS2DH12 获取加速度数据函数 (HAL 接口)
 *
 * @param acce_x 指向 X 轴加速度值的指针
 * @param acce_y 指向 Y 轴加速度值的指针
 * @param acce_z 指向 Z 轴加速度值的指针
 * @return esp_err_t ESP_OK 成功，其他失败
 */
esp_err_t imu_lis2dh12_acquire_acce(float *acce_x, float *acce_y, float *acce_z)
{
    // 检查初始化状态和指针有效性
    if (!is_init || !acce_x || !acce_y || !acce_z)
    {
        return ESP_FAIL;
    }

    lis2dh12_acce_value_t acce = {0, 0, 0}; // 加速度值结构体

    // 获取 LIS2DH12 加速度数据
    if (ESP_OK == lis2dh12_get_acce(lis2dh12, &acce))
    {
        *acce_x = acce.acce_x; // 赋值 X 轴加速度
        *acce_y = acce.acce_y; // 赋值 Y 轴加速度
        *acce_z = acce.acce_z; // 赋值 Z 轴加速度
        return ESP_OK;
    }

    // 获取失败则将值置零并返回失败
    *acce_x = 0;
    *acce_y = 0;
    *acce_z = 0;
    return ESP_FAIL;
}

/**
 * @brief 空的获取陀螺仪数据函数 (HAL 接口，表示不支持)
 *
 * @param x 指向 X 轴陀螺仪值的指针
 * @param y 指向 Y 轴陀螺仪值的指针
 * @param z 指向 Z 轴陀螺仪值的指针
 * @return esp_err_t 总是返回 ESP_ERR_NOT_SUPPORTED
 */
static esp_err_t imu_lis2dh12_null_acquire_function(float *x, float *y, float *z)
{
    return ESP_ERR_NOT_SUPPORTED; // 表示不支持此功能
}

/**
 * @brief IMU 传感器实现结构体
 *
 * 包含了 LIS2DH12 传感器 HAL 接口的函数指针。
 */
static imu_impl_t lis2dh12_impl = {
    .init = imu_lis2dh12_init,                          // 初始化函数
    .deinit = imu_lis2dh12_deinit,                      // 去初始化函数
    .test = imu_lis2dh12_test,                          // 测试函数
    .acquire_acce = imu_lis2dh12_acquire_acce,          // 获取加速度函数
    .acquire_gyro = imu_lis2dh12_null_acquire_function, // 获取陀螺仪函数 (不支持)
};

// 传感器中心检测函数，用于注册 LIS2DH12 IMU 传感器
SENSOR_HUB_DETECT_FN(IMU_ID, lis2dh12, &lis2dh12_impl);

#endif /* CONFIG_SENSOR_INCLUDED_IMU */
