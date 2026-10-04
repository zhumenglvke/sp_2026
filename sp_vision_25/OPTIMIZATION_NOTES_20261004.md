# PnP 与未来命中面优化说明

本次修改在保留原有 `YOLO → Solver → Tracker → Aimer → Shooter` 框架的前提下，完成两项改进：

1. 在 PnP 结果进入 Tracker/EKF 前增加输入、数值、距离和重投影质量检查。
2. 将 Aimer 迭代结果显式保存为“未来命中面”，只有飞行时间和物理装甲面同时收敛时才允许 Shooter 开火。

## 1. PnP 健壮性和重投影检查

### 处理顺序

`Solver::solve()` 现在返回 `bool`。处理顺序如下：

```text
检查四个图像角点数量和有限性
    ↓
执行 SOLVEPNP_IPPE
    ↓
检查 solvePnP 返回值、rvec 和 tvec
    ↓
检查正深度与距离范围
    ↓
重新投影四个物理角点并计算像素 RMSE
    ↓
检查坐标变换后的三维位置和姿态
    ↓
通过后才允许 Tracker 使用
```

`Armor` 新增：

```cpp
bool pnp_valid;
double pnp_reprojection_error_px;
```

Tracker 初始化目标时会按原有排序寻找第一个 PnP 有效观测；已有目标更新时会跳过 PnP 无效候选。重投影误差通过硬门限后，还以较小权重加入候选代价，使几何拟合更好的框优先。

### 初始配置

```yaml
pnp_max_reprojection_error_px: 8.0
pnp_min_distance_m: 0.1
pnp_max_distance_m: 30.0
```

`8 px` 是便于首次运行的宽松起点，不代表实车最佳值。调参时记录正确检测框的重投影 RMSE 分布：

- 正确框频繁被拒绝：逐步放宽，例如 `8 → 10 → 12 px`。
- 明显错误角点仍被接纳：逐步收紧，例如 `8 → 6 → 4 px`。
- 应分别统计近距离、远距离、运动模糊、装甲板侧视和画面边缘场景。
- 建议让 99% 左右的正确观测通过，再结合关联位置/yaw门限排除错误目标。

## 2. 未来命中面选择

原代码已经按飞行时间预测目标，但收敛条件只检查飞行时间，`converged` 结果也没有进入开火门控。因此切板附近可能出现：飞行时间变化很小，但迭代选择的物理装甲面仍在变化。

本次新增 `ShotCandidate`：

```cpp
struct ShotCandidate
{
  bool valid;
  bool trajectory_converged;
  int armor_id;
  Eigen::Vector4d hit_xyza;
  double view_angle;
  double fly_time;
};
```

每次 Aimer 处理的逻辑为：

```text
预测到当前时刻和系统延迟之后
    ↓
选择初始装甲面并解第一次弹道
    ↓
预测到“当前时刻 + 系统延迟 + 子弹飞行时间”
    ↓
重新展开整车所有物理装甲面并选面
    ↓
重新计算飞行时间
    ↓
飞行时间差小于门限，并且装甲面编号稳定
    ↓
生成有效 ShotCandidate
```

如果在最大迭代次数内没有联合收敛，Aimer 仍输出最后一次云台跟随角，但 `ShotCandidate::valid=false`，Shooter 禁止该帧开火。

Shooter 现在使用未来命中面的 `view_angle` 判断装甲板朝向。近距离放宽只作用于云台跟随误差，不能绕过未来命中面的朝向条件。

### 初始配置

```yaml
prediction_max_iterations: 10
fly_time_convergence_ms: 1.0
```

调参建议：

- 先保持 `1 ms / 10次` 收集日志和录像回放结果。
- 若绝大多数帧 2～3 次即可收敛，可将上限降到 4～6 次减少计算量。
- 高速旋转目标经常不收敛时，应先检查 EKF 角速度、系统延迟和弹速，不应直接无限放宽收敛门限。
- 若命中面在相邻两块之间频繁切换，应联合调整 Aimer 的 8°切换迟滞和高速进入/离开窗口。

## 3. 安全行为

- Aimer 每帧开始时都会清空上一帧 `ShotCandidate`，提前返回不会遗留旧开火许可。
- PnP 失败只让当前观测失效，不改变 Tracker 状态机结构。
- 未收敛的未来命中面只禁止开火，不停止云台继续跟随。
- 现有帧龄检查、相机超时停火、关联门限、目标白名单和实际弹速逻辑保持不变。

## 4. 主要修改位置

| 文件 | 修改内容 |
|---|---|
| `tasks/auto_aim/armor.hpp` | 保存 PnP 有效性与重投影 RMSE |
| `tasks/auto_aim/solver.hpp/.cpp` | PnP 返回状态、输入/数值/距离/重投影检查 |
| `tasks/auto_aim/tracker.cpp` | 丢弃无效 PnP，重投影误差参与候选选择 |
| `tasks/auto_aim/aimer.hpp/.cpp` | `ShotCandidate`、未来命中面与飞行时间联合收敛 |
| `tasks/auto_aim/shooter.cpp` | 使用未来命中面决定是否允许开火 |
| `configs/demo.yaml`、`configs/sentry.yaml` | 新增可调参数和初始值 |

