# inertial_core

## 目录

- [Purpose](#purpose)
- [Scope](#scope)
- [Build](#build)
- [API](#api)
- [Earth switches](#earth-switches)
- [YAML configuration](#yaml-configuration)
- [Validation](#validation)
- [References](#references)
- [License](#license)

最小纯惯性机械编排库，用于独立验证 NED/FRD 惯性导航和地球模型开关。

## Purpose

`inertial_core` 从 KF/PSINS 类机械编排中抽取最小的纯惯性路径：

```text
IMU increment + previous navigation state
  -> velocity update
  -> position update
  -> attitude update
  -> next navigation state
```

库不读取 GNSS，不包含卡尔曼滤波、因子图、滑窗、零偏估计或高度阻尼。

## Scope

共享约定：

- navigation frame: NED, `[north, east, down]`;
- body frame: FRD;
- attitude: `q_bn`, body-to-navigation;
- position: `[latitude, longitude, height]`, radians and metres;
- IMU input: interval-end angular and velocity increments, radians and m/s.

`include/inertial_core/mechanization.hpp` 提供无状态单步函数和带上一采样缓存的
`MechanizationRunner`。`include/inertial_core/earth_model.hpp` 提供独立的地球参数计算。
相邻采样圆锥/划桨修正要求 `dt` 相等，默认容差为 `1e-9 s`。
配置结构 `RuntimeConfig` 与 `preintegration_core` 使用相同字段名和默认值。

## Build

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build -j 4
ctest --test-dir build --output-on-failure
```

依赖只有 C++17 和 Eigen3。

## API

```cpp
#include "inertial_core/mechanization.hpp"

inertial_core::NavState initial;
inertial_core::RuntimeConfig options;
inertial_core::MechanizationRunner runner(initial, options);

inertial_core::ImuIncrement imu;
imu.dt_s = 0.005;
imu.delta_angle_rad = gyro_rate * imu.dt_s;
imu.delta_velocity_mps = specific_force * imu.dt_s;

const inertial_core::NavState next = runner.step(imu);
```

`MechanizationDiagnostics` 输出每一步的当地地球参数、比力投影增量和重力/哥氏组合增量，
便于检查开关是否真正作用于动力学。

## Earth switches

| Option | Meaning |
| --- | --- |
| `enable_earth_rotation` | 计算 `omega_ie_ned` 和相应的导航系旋转 |
| `enable_transport_rate` | 计算 `omega_en_ned` |
| `enable_coriolis` | 在速度方程中计算地球哥氏项 `2*omega_ie x v` |
| `enable_ellipsoid_radii` | 使用 WGS84 `RM/RN`；关闭后使用球半径 |
| `gravity_model` | 常值重力或随纬度、高度变化的正常重力 |
| `enable_coning_sculling` | 是否计算圆锥和划桨二阶修正 |
| `equal_interval_tolerance_s` | 相邻 IMU 采样间隔允许误差 |

常用实验模式可直接使用 `inertial_core/presets.hpp` 中的
`noEarthConfig()`、`rotationOnlyConfig()`、`fullBasicConfig()` 和
`constantGravityConfig()`。

关闭 `enable_earth_rotation` 不会自动关闭重力。关闭全部旋转项仍保留重力，
这是导航机械编排的 no-Earth 基线，不是自由落体。

## YAML configuration

逐项注释的默认配置文件位于 [`config/runtime.yaml`](config/runtime.yaml)。

启用 YAML 加载目标：

```bash
cmake -S . -B build -G Ninja -DINERTIAL_CORE_BUILD_YAML_CONFIG=ON
```

```cpp
#include "inertial_core/runtime_config_yaml.hpp"

const inertial_core::RuntimeConfig config =
    inertial_core::loadRuntimeConfigFile("config/runtime.yaml");
```

加载器支持 `runtime:` 根节点，也接受直接放在文档根节点下的同名字段。
缺失字段保留 `RuntimeConfig` 默认值。

## Validation

当前测试覆盖：

- WGS84 半径和正常重力数量级；
- NED 地球自转符号；
- no-Earth 常值重力自由落体和支撑静止；
- 地球自转开启时理想静止 IMU 的姿态、速度和高度保持；
- 水平常速运动的位置更新；
- `config/runtime.yaml` 默认值、字段覆盖和非法重力模型；
- 非法时间间隔拒绝。

## References

- `D:/Code/KF-GINS/src/kf-gins/insmech.cpp`
- `D:/Code/OB_GINS/src/preintegration`
- `D:/Code/psins260705/base/base1/insupdate.m`
- 机械编排方程和实现约定见 [docs/design.md](docs/design.md)。

## License

本库包含基于 KF-GINS/OB_GINS 机械编排方法整理和重写的实现，发布为 GPL-3.0。
