# 大 yaw 自稳与编码器行进方向

## 当前控制结构

1. 底盘内置 BMI088 实测 Z 角速度作为反向速度前馈。
2. 云台板载 BMI088 的 EKF 航向/角速度经 0x0404 反馈给底盘，修正大 yaw 实际偏转。
3. 大 yaw 绝对编码器与固定车头零位独立决定行进坐标，不对两套 IMU 航向作差。

```
e_heading = wrap(target_heading - measured_gimbal_heading)
w_world = wheel_rate + clamp(2 * e_heading, -1.5, 1.5)
e_rate = w_world - measured_gimbal_rate
w_relative = w_world - measured_chassis_rate + 0.25*e_rate + integral(0.5*e_rate)
w_motor = installation_direction * external_ratio * w_relative
```

所有角度单位 rad、角速度 rad/s，物理方向逆时针为正。安装方向 -1，外部传动比 1，电机限速 ±10 rad/s。电机内部 Gr=10 不重复乘入外部传动比。航向误差限幅 ±pi/2，速度 PI 的防积分累积包含前馈，避免因底盘速度前馈饱和而继续积累。

启用后重捕当前实测航向；滚轮推进目标，松手时捕获当时航向并保持。云台 IMU 角度按其样本年龄做短时预测，不用旧样本反复积分航向。禁用、任何必需 IMU 丢失/过期、反馈故障时清除目标/积分；恢复时重新捕获当前航向。云台启动 ID、板载 EKF 零位代数变化后也重新捕获。

云台 IMU 超过 50 ms、数据非有限或底盘 IMU 无效时，大 yaw 停止失能，不静默退回旧开环速度。编码器行进坐标不依赖云台 IMU 有效性，所以该项不会直接停止四轮平移；若编码器也缺失/过期或电机报错，则仍按原保护停止云台坐标平移。

## 通信与固件

CAN3、基 ID 6、速度帧 ID 0x206、反馈 ID 0x11。速度载荷为 4 字节小端 float；使能/失能/清故障特殊帧保持 8 字节。按厂家手册的速度模式 D0..D3 定义修正此前发送 8 字节速度载荷的问题：
https://docs.openarm.dev/assets/files/dm4310-5d089e0b013d8e2fc203c5b670008a66.pdf

用户上位机显示硬件为模式 3（速度模式）。没有切换模式、保存电机零点或烧录。

需要底盘新固件与前面已生成的板载 IMU 云台固件配套；旧外置 IMU 云台固件不保证采用用户现在指定的板载来源。

## 行进零位与限制

车头零位按用户确认取 `kRobotBigYawForwardZeroRad=-1.132792 rad`。当前绝对编码器角减去该固定零位后做安装方向与外部传动比转换，得到云台相对底盘角。不重新设置电机内部零点，不在使能时改软件零位。

本方案修正的是相对 IMU 所报告航向的偏转。六轴 IMU 的长期绝对航向漂移仍不可观，需要视觉或其他外部航向约束；编码器坐标转换不能消除实际云台朝向漂移、轮胎打滑或轮组误差。

## 验证

- `python3 tests/test_big_yaw_gyro.py`：真实控制函数与原生速度帧、双向前馈、航向跨 pi、滚轮松手、IMU 会话/零位切换、丢失/非有限/过期数据、故障保护、含电机 20 ms 滞后/10% 速度增益误差/底盘 gyro 0.03 rad/s 偏差的两分钟模拟，覆盖 2～3.4 rad/s、4 s 周期余弦旋转、反向、滚轮转向与松手锁定。
- `python3 tests/test_gimbal_translation.py`：编码器整圈、不同安装符号/零位、模拟 IMU 漂移下方向不变、反馈失效和四舵轮解算。
- 云台 `bash tests/interboard/run_dm_imu_tests.sh`：板载 IMU 数据填充、0x0404 编码和底盘实际解析。
- 两块板 Debug 构建通过。未完成实车闭环测试。
