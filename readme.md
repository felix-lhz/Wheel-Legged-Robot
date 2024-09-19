# Stack Force

## 一、安装说明

https://www.kdocs.cn/l/cdPyalz8EbqI

###### 校准说明

[舵机校准文档](舵机校准教程.docx)

## 二、遥控说明

<img src="D:\DeskTop\HW\Projects\wheel-legged-robot\StackForce\遥控说明.jpg" style="zoom: 25%;" />

## 三、电控设备说明

##### 1、主控板

[主控板原理图](StackForce主控板.pdf)

ESP32双主控芯片（按键切换）：S1型号（黄色灯亮）：`ESP32 Dev Moudle`，S3型号（绿色灯亮）：`ESP32S3 Dev Moudle`。

<img src="D:\DeskTop\HW\Projects\wheel-legged-robot\StackForce\Pictures\主控板.jpg" style="zoom:50%;" />

##### 2、双路无刷电机驱动板（小功率）

[双路无刷电机小电流驱动板原理图](双路无刷电机小功率驱动.pdf)

<img src="D:\DeskTop\HW\Projects\wheel-legged-robot\StackForce\Pictures\驱动板（小功率）.jpg" style="zoom:50%;" />

###### 规格

| 工作电压         | 12V ~ 24V | 支持最大电流         | 5A（双路） |
| ---------------- | --------- | -------------------- | ---------- |
| 采样电阻阻值     | 10mR      | 电极电流采集基准电压 | 1.65V      |
| 报错指示         | 支持      | 电源端子规格         | XT30       |
| 支持控制电机数量 | 2         | 电机端子规格         | MR30       |
| 尺寸             | 6.5*4.0CM | 净重                 | 16.3g      |

##### 3、舵机驱动板+IMU模块

[多路舵机驱动板+IMU模块原理图](多路舵机+IMU模块.pdf)

<img src="D:\DeskTop\HW\Projects\wheel-legged-robot\StackForce\Pictures\舵机驱动板.jpg" style="zoom:50%;" />

###### 功能

- 输入电压：`12V~ 24V`
- 最大输出电流：`5A`
- 12路舵机启动端口
- 航模遥控器支持协议：`PPM / SBUS`
- IMU模块：`MPU6050`
- 3pin端口规格：`3P 2.54排针`

##### 4、磁编码器（MT6701）

<img src="D:\DeskTop\HW\Projects\wheel-legged-robot\StackForce\Pictures\磁编码器.jpg" style="zoom:50%;" />

- 14为高精度
- 最高测量转速`55000RPM`
- 支持`SSI/I2C`协议

##### 5、无刷电机（2208）



## 四、机械结构说明

<img src="D:\DeskTop\HW\Projects\wheel-legged-robot\StackForce\Pictures\整机.jpg" style="zoom:50%;" />

###### 1、倒立摆结构

机器人上层机构为机体，驱动轮轴与腿部机构转轴的连杆为摆杆，得到倒立摆结构模型。

<img src="D:\DeskTop\HW\Projects\wheel-legged-robot\StackForce\Pictures\倒立摆结果.jpg" style="zoom: 67%;" />

###### 2、五连杆结构

该机器人以倒立摆结构为基本原理，实现五连杆的二阶倒立摆结构，便于实现机器人的自平衡。

![](D:\DeskTop\HW\Projects\wheel-legged-robot\StackForce\Pictures\五连杆结果.jpg)

## 五、调试软件说明

###### 1、VScode

###### 2、`Platform IO`插件

简易使用在安装文档里有展示。

## 六、算法控制

##### 1、LQR（RM轮腿式机器人）

[轮腿式平衡机器人控制_陈阳](轮腿式平衡机器人控制_陈阳.pdf)

[RoboMaster平衡步兵机器人控制系统设计 - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/563048952)

##### 2、IDA-PBC（Ollie轮腿机器人）

[Ollie](Ollie.pdf)

[腾讯Ollie轮腿机器人论文阅读及分享 - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/619764891)

