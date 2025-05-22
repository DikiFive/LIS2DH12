# LIS2DH12 加速度计和温度传感器示例

(有关示例的更多信息，请参阅上层“examples”目录中的 README.md 文件。)

## 概述

此示例演示了如何使用 I2C 驱动程序从 LIS2DH12 加速度传感器读取三轴加速度和温度数据，并计算设备的静态姿态角度（俯仰角和横滚角）。

如果您需要开发基于 I2C 接口的加速度计或温度传感器应用，此示例可作为基本模板。

## 如何使用示例

### 所需硬件

要运行此示例，您需要一块基于 ESP32、ESP32-S 或 ESP32-C 的开发板，以及一个 LIS2DH12 加速度传感器。

#### 引脚分配：

**注意：** 以下引脚分配为默认使用，您可以在 `menuconfig` 中更改它们。

|                  | SDA             | SCL           |
| ---------------- | -------------- | -------------- |
| ESP I2C Master   | I2C_MASTER_SDA | I2C_MASTER_SCL |
| LIS2DH12 Sensor  | SDA            | SCL            |

有关 `I2C_MASTER_SDA` 和 `I2C_MASTER_SCL` 的实际默认值，请参阅 `menuconfig` 中的 `Example Configuration`。

**注意：** SDA/SCL 引脚无需添加外部上拉电阻，因为驱动程序将启用内部上拉电阻。

### 构建和烧录

输入 `idf.py -p PORT flash monitor` 来构建、烧录和监控项目。

（要退出串口监控，请键入 `Ctrl-]`。）

有关配置和使用 ESP-IDF 构建项目的完整步骤，请参阅 [入门指南](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html)。

## 示例输出

```bash
LIS2DH12 实时数据读取示例

Device ID: 0x33

Temperature: 25.00 degC (Raw: 0, 0x0000)
Acceleration: X=0.05 g, Y=0.00 g, Z=0.43 g
Pitch: 0.00 deg, Roll: 0.00 deg

Temperature: 25.00 degC (Raw: 0, 0x0000)
Acceleration: X=0.05 g, Y=0.00 g, Z=0.43 g
Pitch: 0.00 deg, Roll: 0.00 deg
...
```