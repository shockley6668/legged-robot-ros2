# Legged Robot ROS2 project

这是一个基于 ROS2 的双足机器人控制项目，旨在实现高性能的步态控制与模型推理。

## 1. 项目架构与功能包

项目包含两个主要的 ROS2 功能包：

- **`robot_bringup` (C++)**:
  - **功能**: 负责底层通讯与硬件抽象层。
  - **核心节点**: `bridge_node`。
  - **职责**: 建立与下位机的通讯桥梁，负责接受下位机数据以及电机控制命令的下发。

- **`robot_control` (Python)**:
  - **功能**: 负责高级控制逻辑与深度学习模型推理。
  - **核心节点**: `inference_node`。
  - **职责**: 加载 ONNX 模型进行动作推理，处理控制算法，并将计算出的目标姿态通过 ROS2 话题发送给 `robot_bringup`。

## 2. 交互与通讯

- **手柄控制**:
  - 项目深度集成了 ROS2 的标准手柄接口，通过监听 `/joy` 话题将遥控输入转换为机器人的控制指令。
- **下位机通讯**:
  - 采用USB虚拟串口进行高效传输。
  - 通讯协议支持实时反馈电机状态（位置、速度、电流）以及 IMU 数据，确保闭环控制的精度。

## 3. 当前版本与开发路线

- **当前版本 (v1.0)**:
  - **双模型设计**: 目前的逻辑中，**走路（Walking）** 和 **静止（Standing/Static）** 是分开推理的。系统会根据状态切换调用两个不同的 ONNX 模型。

- **未来规划 (Upcoming Branch)**:
  - 即将发布一个**单模型集成版本**分支。
  - **新特性**: 将通过单一模型同时完成“踏步（Stepping）”和“静止（Static）”的推理。
  - **优势**: 减少模型切换延迟，提升动作转换的平滑度和泛化能力。

## 4. 快速启动

```bash
# 构建项目
colcon build --symlink-install

# 启动底层桥接
ros2 launch robot_bringup bridge.launch.py

# 启动控制推理
ros2 launch robot_control robot_control.launch.py
```
