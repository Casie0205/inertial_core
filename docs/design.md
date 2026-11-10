# inertial_core 设计

## 目录

- [Frames and units](#frames-and-units)
- [State and input](#state-and-input)
- [Mechanization sequence](#mechanization-sequence)
- [Earth model](#earth-model)
- [Baseline definitions](#baseline-definitions)
- [Failure handling](#failure-handling)
- [Validation plan](#validation-plan)

本文定义数值约定、单步机械编排和地球模型边界。

## Frames and units

- NED navigation frame: x north, y east, z down.
- FRD body frame: x forward, y right, z down.
- `q_bn`: body-to-navigation quaternion.
- `position_rad_m`: latitude, longitude, ellipsoidal height.
- `velocity_ned_mps`: north, east, down velocity.
- `ImuIncrement`: interval-end `dt`, angular increment in radians and velocity increment in m/s.

## State and input

`NavState` 只包含时间、位置、速度和姿态。IMU 零偏、比例因子、杆臂、噪声状态和高度辅助
均不属于该库。

`MechanizationRunner` 保存上一帧 IMU 增量，以计算相邻采样的圆锥和划桨交叉项。首次调用时
交叉项为零。无状态调用接口可由调用方显式传入上一帧增量。双样本修正按等采样间隔推导，
当前实现要求相邻 `dt` 差值不超过 `RuntimeConfig::equal_interval_tolerance_s`。
`RuntimeConfig::enable_coning_sculling` 可关闭全部圆锥和划桨二阶修正。

`RuntimeConfig` 与 `preintegration_core` 使用相同字段名和默认值。
`runtime_config_yaml.hpp` 将 `config/runtime.yaml` 映射到同一个结构；YAML 缺失字段
保留默认值，未知重力模型会抛出 `std::invalid_argument`。

## Mechanization sequence

每个区间执行以下顺序：

1. 用上一状态计算当地 `RM/RN`、`omega_ie`、`omega_en` 和重力；
2. 对比力增量和速度增量作圆锥、划桨补偿；
3. 将比力增量投影到 NED；
4. 用重力和哥氏项更新速度；
5. 用中点速度更新纬经高；
6. 用姿态、地球自转和运输率更新 `q_bn`。

地球模型开启时，导航系旋转同时进入速度投影和姿态更新。运输率与地球自转由不同开关控制。

## Earth model

`EarthModel::evaluate()` 返回一个显式参数包：

```text
RM, RN, omega_ie_ned, omega_en_ned, omega_in_ned, gravity
```

正常重力使用纬度高阶项和高度修正。关闭椭球半径时，`RM=RN=fallback_sphere_radius_m`，
但经纬高坐标定义保持不变。

建议实验一次只改变一个开关：

| Baseline | Rotation | Transport | Coriolis | Gravity |
| --- | --- | --- | --- | --- |
| no-Earth | off | off | off | normal or constant |
| rotation-only | on | off | on | normal or constant |
| full-basic | on | on | on | normal |

对应字段依次为 `enable_earth_rotation`、`enable_transport_rate`、
`enable_coriolis` 和 `gravity_model`。`enable_ellipsoid_radii` 独立控制
WGS84 半径或球半径；`enable_coning_sculling` 独立控制二阶增量修正。

## Baseline definitions

“no-Earth” 在本库中表示不计算地球自转、运输率和地球哥氏项。重力仍存在，否则系统处于
自由落体，不能与导航机械编排比较。

“full-basic” 表示当前已实现的完整基础机械编排，不包含潮汐、极移、高阶重力场或地球
曲率之外的专门误差修正。

## Failure handling

- `dt <= 0`、非有限输入、非法纬度和奇异 `cos(latitude)` 会抛出
  `std::invalid_argument`；
- 传播出现非有限状态时抛出 `std::runtime_error`；
- 库不静默修复非法输入。

## Validation plan

- 地球参数与 KF/PSINS 数量级和符号对照；
- 地球开启时静止理想 IMU 的状态保持；
- no-Earth 自由落体和支撑静止；
- 水平常速运动；
- 圆锥/划桨开关和等间隔容差；
- 后续增加纯北向、纯东向、转弯和纬度高差场景。
