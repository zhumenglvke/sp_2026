# ljf2026_sentry
缝合sp的自瞄跟深北莫的导航决策的尝试

---

# sp_vision_25 视觉模块说明

## 1. 项目简介

`sp_vision_25` 是哨兵机器人视觉与自瞄模块，负责从相机图像中识别目标、估计目标空间位置、跟踪目标运动状态，并生成云台瞄准和开火相关指令。

本工程将视觉模块接入 ROS 2，并与哨兵导航、决策、云台和控制板通信模块协同工作。当前推荐的 ROS 2 双相机入口为：

```text
src/sentry_dual_cameras_ros.cpp
```

核心视觉算法不集中在 `src` 中。`src` 主要存放可执行程序入口，具体的检测、解算、跟踪和预测算法位于 `tasks` 目录。

## 2. 系统处理流程

```text
相机采集图像
    ↓
YOLO 或传统视觉检测装甲板
    ↓
数字分类与目标属性识别
    ↓
PnP/坐标变换解算目标三维位置
    ↓
Tracker + EKF 跟踪目标运动状态
    ↓
Aimer 预测弹丸飞行期间的目标位置
    ↓
Shooter 判断是否满足开火条件
    ↓
通过 ROS 2 或控制板接口发送控制结果
```

## 3. 目录结构

```text
sp_vision_25/
├── assets/                  # 神经网络模型和识别模板，需要自行准备
├── configs/                 # 相机、检测器、跟踪器和弹道参数
├── io/                      # 相机、云台、控制板和 ROS 2 通信
├── src/                     # 各机器人及不同运行方式的程序入口
├── tasks/
│   ├── auto_aim/            # 装甲板检测、解算、跟踪、预测和瞄准
│   ├── auto_buff/           # 能量机关识别和预测
│   └── omniperception/      # 多相机感知结果选择与决策
├── tests/                   # 测试和调试程序
├── tools/                   # 日志、数学、绘图、队列等通用工具
├── CMakeLists.txt
└── package.xml
```

## 4. 核心模块

### 4.1 自动瞄准

核心代码位于 `tasks/auto_aim/`：

| 模块 | 作用 |
|---|---|
| `detector.cpp` | 使用传统图像处理方法检测装甲板 |
| `yolo.cpp` | 使用 OpenVINO 加载 YOLO 模型并完成目标检测 |
| `classifier.cpp` | 使用分类模型识别装甲板数字或类别 |
| `solver.cpp` | 完成 PnP、坐标变换和目标三维位置解算 |
| `tracker.cpp` | 管理目标匹配、目标选择和跟踪状态 |
| `target.cpp` | 使用 EKF 估计目标位置、速度和旋转状态 |
| `aimer.cpp` | 根据目标运动和弹道时间计算瞄准位置 |
| `shooter.cpp` | 根据瞄准误差和目标状态判断是否开火 |
| `planner/` | MPC 等预测与规划算法 |

### 4.2 能量机关

`tasks/auto_buff/` 用于能量机关目标的检测、位姿解算、运动建模和击打点预测。

### 4.3 多相机感知

`tasks/omniperception/` 负责处理多路相机的检测结果，并根据目标质量、相机方向及决策条件选择有效结果。

## 5. `src` 中的主要入口

| 入口 | 用途 |
|---|---|
| `standard.cpp` | 基础单相机自瞄流程，适合阅读整体调用关系 |
| `standard_mpc.cpp` | 使用 Planner/MPC 的预测控制版本 |
| `mt_standard.cpp` | 多线程标准机器人入口，同时支持自瞄和能量机关 |
| `sentry.cpp` | 哨兵单相机运行入口 |
| `sentry_debug.cpp` | 带图像显示、日志和绘图的哨兵调试入口 |
| `sentry_multithread.cpp` | 哨兵多线程处理入口 |
| `sentry_dual_cameras.cpp` | 非 ROS 2 的双相机入口 |
| `sentry_dual_cameras_ros.cpp` | ROS 2 双相机入口，当前推荐使用 |
| `sentry_dual_sn.cpp` | 前后相机独立检测的双相机入口 |
| `sentry_dual_sn_ros.cpp` | 上述方案的 ROS 2 版本 |
| `auto_aim_debug_mpc.cpp` | 自瞄 MPC 调试入口 |
| `auto_buff_debug.cpp` | 能量机关调试入口 |
| `auto_buff_debug_mpc.cpp` | 能量机关 MPC 调试入口 |
| `uav.cpp` | 无人机视觉入口 |
| `uav_debug.cpp` | 无人机视觉调试入口 |

`async_detect_worker.hpp` 封装了异步 YOLO 检测线程，可在后台接收图像并输出检测结果。

## 6. 运行环境

当前工程使用的主要环境如下：

- Ubuntu 22.04
- ROS 2 Humble
- OpenVINO 2024.6.0
- OpenCV
- Eigen3
- yaml-cpp
- fmt
- nlohmann-json
- `auto_aim_interfaces`
- `pb_rm_interfaces`

使用 OpenVINO 时不仅需要运行库，还需要 C++ 开发文件和 CMake 配置文件。安装后，CMake 应当能够直接找到 `OpenVINO`，不应在 `CMakeLists.txt` 中写死某一台电脑的安装路径。

## 7. 模型文件

模型资源应放在：

```text
sp_vision_25/assets/
```

当前 `configs/demo.yaml` 中引用了以下资源：

```text
assets/tiny_resnet.onnx
assets/yolo11.xml
assets/yolov8.xml
assets/yolov5.xml
assets/0526.xml
assets/best.xml
assets/standard_fanblade.jpg
```

注意事项：

- `assets` 不会由 OpenVINO 自动生成。
- `.onnx` 是 ONNX 模型文件。
- OpenVINO IR 模型通常由同名 `.xml` 和 `.bin` 两个文件组成。
- 模型名称和实际使用模型由 YAML 配置决定，不需要的模型可以不配置。
- 模型文件缺失时，工程可能可以通过编译，但视觉程序无法正常完成推理。
- 模型应从原项目、队伍模型仓库或训练导出结果中获取。

建议目录结构：

```text
assets/
├── tiny_resnet.onnx
├── yolo11.xml
├── yolo11.bin
├── yolov8.xml
├── yolov8.bin
├── yolov5.xml
├── yolov5.bin
└── standard_fanblade.jpg
```

## 8. 配置文件

配置文件位于 `configs/`。运行时需要重点检查：

- `enemy_color`：敌方颜色。
- `yolo_name`：当前选择的检测模型类型。
- `*_model_path`：模型文件路径。
- `device`：OpenVINO 推理设备，例如 `CPU` 或 `GPU`。
- 相机曝光、增益、设备标识及内参。
- 装甲板检测阈值。
- Tracker 连续检测与丢失帧参数。
- 云台和相机外参。
- 弹速、延迟及瞄准补偿参数。

模型相对路径以程序运行目录为基准。通过 ROS 2 启动文件运行时，应确保工作目录或传入的配置路径能够正确定位 `assets/`。

### ROS 2 双相机入口的时序与开火配置

`sentry_dual_cameras_ros` 只处理前摄的新帧，超过 `max_frame_age_ms` 的前后摄结果不会参与控制。前摄断帧超时后会重建 Tracker，并发出一次停火命令；后摄仍按原有的接管等待时间和冷却时间工作。后摄采集与 YOLO 检测在独立线程中运行，只有前摄失去目标且允许接管时才启用后摄推理。

可在 `configs/demo.yaml` 中设置：

- `imu_delay_ms`：前摄帧时间戳与 IMU 姿态查询的固定偏移，默认 2 ms；此值需要结合实际相机和 IMU 时间戳标定。
- `max_frame_age_ms`：检测结果的最大帧龄，默认 100 ms。
- 弹速：使用 `GimbalROS` 从 ROS `/BulletSpeed` 更新的值；其回调已经处理异常弹速。
- `pitch_tolerance_deg`：云台实际俯仰角与瞄准命令角的开火误差上限，默认 2°。启用 `tracking_only_fire` 时，仅 Tracker 处于 `tracking` 状态允许开火；当前帧没有装甲板时也不触发开火。
- `target_whitelist`：静态目标白名单，名称取自 `ARMOR_NAMES`，例如 `[one, three, sentry, outpost]`。空列表保持原来的敌方颜色和 5 号装甲板过滤规则。该配置同时作用于前、后摄。导航发送的动态目标列表目前没有接到这个 ROS 2 入口。
- `back_camera_config`：后摄相机配置路径，默认 `configs/cam2.yaml`。

## 9. 编译方法

本项目使用 zsh。首先进入工作空间并加载 ROS 2 环境：

```zsh
cd /home/wlw/Desktop/RM/lf_sentry
source /opt/ros/humble/setup.zsh
```

首次编译时，先编译消息接口，再编译视觉模块：

```zsh
colcon build --symlink-install \
  --packages-select auto_aim_interfaces pb_rm_interfaces sp_vision
```

编译完成后加载工作空间：

```zsh
source install/setup.zsh
```

如果修改了 CMake 配置或依赖查找方式，可以清理该包的 CMake 缓存后重新构建：

```zsh
colcon build --symlink-install \
  --packages-select sp_vision \
  --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 10. 运行方法

### 10.1 直接运行 ROS 2 双相机程序

```zsh
cd /home/wlw/Desktop/RM/lf_sentry
source /opt/ros/humble/setup.zsh
source install/setup.zsh

ros2 run sp_vision sentry_dual_cameras_ros \
  "$(ros2 pkg prefix --share sp_vision)/configs/demo.yaml"
```

### 10.2 通过哨兵启动文件运行

完整工作空间编译完成后，可以通过 bringup 包启动：

```zsh
cd /home/wlw/Desktop/RM/lf_sentry
source /opt/ros/humble/setup.zsh
source install/setup.zsh

ros2 launch pb2025_sentry_bringup bringup_sp.launch.py
```

启动文件负责查找 `sp_vision` 的包共享目录，并从安装后的 `configs/demo.yaml` 启动 `sentry_dual_cameras_ros`。

## 11. 开发建议

- 新的检测、跟踪和预测算法放入 `tasks/`，不要直接堆积在 `src` 的 `main()` 中。
- 新的硬件或 ROS 2 通信接口放入 `io/`。
- 可复用的数学和线程工具放入 `tools/`。
- `src` 只负责创建模块、组织线程和连接数据流。
- 参数优先写入 YAML，避免在源码中硬编码绝对路径、弹速或设备编号。
- ROS 2 包内资源通过 `ament_index_cpp` 或 Python 的 package share API 查找，避免依赖某个用户的 `/home/...` 路径。

## 12. 常见问题

### 12.1 CMake 找不到 OpenVINO

确认已经安装 OpenVINO 开发组件，并删除原先写死的 `OpenVINO_DIR`。重新加载终端环境后，使用 `--cmake-clean-cache` 构建。

### 12.2 提示找不到模型文件

检查 `assets/` 是否存在，并确认 YAML 中的模型文件名与实际文件完全一致。Linux 文件名区分大小写。

### 12.3 OpenVINO 能读取 `.xml`，但推理初始化失败

检查同目录下是否存在匹配的 `.bin` 文件，并确认模型导出版本与当前 OpenVINO 兼容。

### 12.4 找不到 ROS 2 可执行程序

重新加载环境：

```zsh
source /opt/ros/humble/setup.zsh
source /home/wlw/Desktop/RM/lf_sentry/install/setup.zsh
```

然后检查：

```zsh
ros2 pkg executables sp_vision
```

### 12.5 相机无法打开

检查相机是否连接、设备标识是否正确、当前用户是否具有设备访问权限，以及 YAML 中的相机参数是否对应实际硬件。

### 12.6 双相机有图像但没有识别结果

依次检查：

1. 两台相机是否都有新帧。
2. 相机曝光和敌方颜色是否正确。
3. 模型路径是否有效。
4. OpenVINO `device` 是否可用。
5. 检测置信度阈值是否过高。
6. 相机内参与外参是否正确。

### 12.7 出现 `No tf data` 或 `base_link does not exist`

这是机器人 TF/描述启动链路问题，不是 YOLO 模型问题。需要确认 `robot_state_publisher` 已启动、机器人描述已加载，并且 RViz 的 Fixed Frame 与实际发布的 TF 一致。

## 13. 当前集成说明

当前版本已经完成以下 ROS 2 集成调整：

- 使用 `sp_vision` 替代不存在的旧视觉 bringup 包。
- 启动文件通过 ROS 2 包共享目录定位配置文件。
- 移除了开发者电脑上的硬编码绝对路径。
- CMake 自动查找 OpenVINO。
- 安装 `sentry_dual_cameras_ros` 可执行程序和 `configs/`。
- `assets/` 存在时会随包安装；缺失时会给出警告。

在正式运行视觉程序前，仍需准备与当前 YAML 配置相匹配的模型文件，并完成双相机、云台和坐标系参数标定。
