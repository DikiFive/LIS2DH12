/* LIS2DH12 实时数据读取示例

   这是一个简单的 I2C 示例，展示了如何初始化 I2C
   以及如何通过 I2C 实时读取 LIS2DH12 加速度传感器的数据。

   此示例代码属于公共领域（或根据您的选择，采用 CC0 许可）。
*/
#include <stdio.h>             // 标准输入输出库
#include <math.h>              // 数学库，用于 atan2f 和 sqrtf
#include "esp_log.h"           // ESP-IDF 日志库
#include "driver/i2c.h"        // ESP-IDF I2C 驱动
#include "freertos/FreeRTOS.h" // FreeRTOS 实时操作系统
#include "freertos/task.h"     // FreeRTOS 任务管理
#include "lis2dh12.h"          // LIS2DH12 传感器驱动

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define I2C_MASTER_SCL_IO GPIO_NUM_5 /*!< I2C 主机时钟线 GPIO 号 */
#define I2C_MASTER_SDA_IO GPIO_NUM_4 /*!< I2C 主机数据线 GPIO 号 */
#define I2C_MASTER_NUM I2C_NUM_0     /*!< I2C 主机端口号 */
#define I2C_MASTER_TX_BUF_DISABLE 0  /*!< I2C 主机不需要发送缓冲区 */
#define I2C_MASTER_RX_BUF_DISABLE 0  /*!< I2C 主机不需要接收缓冲区 */
#define I2C_MASTER_FREQ_HZ 100000    /*!< I2C 主机时钟频率 */

static i2c_bus_handle_t i2c_bus = NULL;   // I2C 总线句柄
static lis2dh12_handle_t lis2dh12 = NULL; // LIS2DH12 传感器句柄

/**
 * @brief I2C 主机和 LIS2DH12 传感器初始化
 */
static void lis2dh12_init()
{
    // I2C 配置结构体
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,                // I2C 工作模式：主机模式
        .sda_io_num = I2C_MASTER_SDA_IO,        // SDA 引脚号
        .sda_pullup_en = GPIO_PULLUP_ENABLE,    // 启用 SDA 上拉电阻
        .scl_io_num = I2C_MASTER_SCL_IO,        // SCL 引脚号
        .scl_pullup_en = GPIO_PULLUP_ENABLE,    // 启用 SCL 上拉电阻
        .master.clk_speed = I2C_MASTER_FREQ_HZ, // I2C 主机时钟频率
    };
    // 创建 I2C 总线和 LIS2DH12 传感器设备
    i2c_bus = i2c_bus_create(I2C_MASTER_NUM, &conf);
    lis2dh12 = lis2dh12_create(i2c_bus, LIS2DH12_I2C_ADDRESS);

    // 配置 LIS2DH12 传感器
    lis2dh12_config_t lis2dh12_config = {
        .temp_enable = LIS2DH12_TEMP_ENABLE, // 启用温度传感器
        .odr = LIS2DH12_ODR_25HZ,            // 设置输出数据速率为 25Hz
        .opt_mode = LIS2DH12_OPT_HIGH_RES,   // 设置为高分辨率模式
        .z_enable = LIS2DH12_ENABLE,         // 使能Z轴
        .y_enable = LIS2DH12_ENABLE,         // 使能Y轴
        .x_enable = LIS2DH12_ENABLE,         // 使能X轴
        .bdu_status = LIS2DH12_ENABLE,       // 启用块数据更新
        .fs = LIS2DH12_FS_16G,               // 设置满量程为 ±16G
    };
    lis2dh12_set_config(lis2dh12, &lis2dh12_config);
}

/**
 * @brief LIS2DH12 传感器去初始化
 */
static void lis2dh12_deinit()
{
    lis2dh12_delete(&lis2dh12); // 删除 LIS2DH12 传感器设备
    i2c_bus_delete(&i2c_bus);   // 删除 I2C 总线
}

/**
 * @brief 应用程序主函数
 */
void app_main(void)
{
    printf("LIS2DH12 实时数据读取示例\n\n");

    // 设置日志级别
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("LIS2DH12", ESP_LOG_DEBUG);
    esp_log_level_set("I2C_BUS", ESP_LOG_DEBUG);

    // 初始化 LIS2DH12 传感器
    lis2dh12_init();

    // 持续读取数据
    float temperature = 0.0f; // 温度值
    int16_t raw_temp = 0;     // 原始温度值

    // 读取设备ID并打印
    uint8_t deviceid;
    if (lis2dh12_get_deviceid(lis2dh12, &deviceid) == ESP_OK)
    {
        printf("Device ID: 0x%02x\n\n", deviceid);
    }

    lis2dh12_acce_value_t acce_value = {0}; // 加速度值结构体
    while (1)
    {
        // 获取温度数据
        esp_err_t err = lis2dh12_get_temp(lis2dh12, &temperature, &raw_temp);
        if (err == ESP_ERR_NOT_FOUND)
        {
            printf("Waiting for temperature data...\n");
        }
        else if (err == ESP_OK)
        {
            // 获取加速度数据
            lis2dh12_get_acce(lis2dh12, &acce_value);

            // 打印所有数据
            printf("Temperature: %.2f degC (Raw: %d, 0x%04x)\n", temperature, raw_temp, (uint16_t)raw_temp);
            float pitch_rad = atan2f(-acce_value.acce_x, sqrtf(acce_value.acce_y * acce_value.acce_y + acce_value.acce_z * acce_value.acce_z));
            float roll_rad = atan2f(acce_value.acce_y, sqrtf(acce_value.acce_x * acce_value.acce_x + acce_value.acce_z * acce_value.acce_z));

            float pitch_deg = pitch_rad * 180.0f / M_PI;
            float roll_deg = roll_rad * 180.0f / M_PI;

            printf("Acceleration: X=%.2f g, Y=%.2f g, Z=%.2f g\n",
                   acce_value.acce_x, acce_value.acce_y, acce_value.acce_z);
            printf("Pitch: %.2f deg, Roll: %.2f deg\n", pitch_deg, roll_deg);
            printf("\n");
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // 延时100ms
    }

    // 注意：由于是无限循环，以下代码通常不会被执行
    // lis2dh12_deinit();
}
