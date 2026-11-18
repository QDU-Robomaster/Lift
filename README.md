# Lift

底盘抬升机构控制模块，管理两个前轴电机和一个后轴电机。模块通过 `CMD`、统一
`Motor` 接口、姿态 Topic 与 `EventBinder` 接入；不要求新增 CMD 命令或 Topic。

Chassis lift controller for two front motors and one rear motor. It uses the `CMD` and
`Motor` interfaces, chassis attitude Topics and `EventBinder`; no new CMD command or Topic
is required.

## 已实现功能 / Implemented Behavior

- `RELAX` 清除前后轴动作、复位 PID，并对三个电机调用 `Relax()`。
- `AUTOBALANCE` 用 IMU pitch 误差生成后轴速度目标，再经后轴速度环输出；左右前轴
  可独立接收 `DEPLOY`、`RETRACT` 预设，并通过各自的位置环和速度环控制。
- 后轴 `REAR_DEPLOY` / `REAR_RETRACT` 按配置速度向相应机械方向运动。代码不使用后轴
  绝对位置或机械零点判断端点是否到达。
- 后轴受阻检测仅在速度和扭矩阈值均大于零时启用。运动请求期间，若输出轴速度低于
  配置阈值且电机扭矩超过配置阈值持续 50 ms，模块锁存当时的后轴相对角度，并用位置
  环和速度环保持该位置。受阻只表示运动受阻，不代表已确认到达机械硬挡块。
- 再次请求 `AUTOBALANCE` 或后轴端点动作会清除受阻状态并重试；`CMD` 上线与失控事件
  都会请求 `RELAX`。

- `RELAX` clears both axes' actions, resets the PIDs and calls `Relax()` on all three motors.
- `AUTOBALANCE` turns IMU pitch error into a rear speed target, then uses the rear speed
  loop to produce the motor command. Each front motor can independently receive a `DEPLOY`
  or `RETRACT` preset and is controlled by its position and speed loops.
- Rear `REAR_DEPLOY` and `REAR_RETRACT` move toward the corresponding mechanical direction
  at the configured speed. Rear absolute position and a mechanical zero are not used to
  determine whether an endpoint has been reached.
- Rear blockage detection is enabled only when both speed and torque thresholds are
  positive. During a motion request, output-shaft speed below the configured threshold and
  motor torque above its threshold for 50 ms latch the current relative rear angle. Position
  and speed loops then hold that angle. A blockage is not confirmation of a hard stop.
- A new `AUTOBALANCE` or rear endpoint request clears the blocked state and retries. Both
  `CMD` link-start and link-loss events request `RELAX`.

## 已知未完成项 / Known Limitations

- `LimitOutput()` 目前为空，不会对各轴组合输出再做轴级限速、限流或变化率限制。各 PID
  的 `out_limit` 可独立配置，但本模块不会为它们填入机器人专用限值。前轴配置中的
  `*_min_angle` / `*_max_angle` 尚未生效。
- 后轴端点动作没有位置到位判断或硬挡块确认。如果受阻阈值未配置，动作不会因受阻检测
  自动停止；即使配置阈值，受阻也无法区分机械端点与其他卡阻。
- 当前没有统一的反馈失效输出门控：后轴反馈更新无效时会清除累计初始化标志，但控制
  循环仍会继续，可能使用上次反馈值计算输出。
- `gyro_topic_name` 被订阅，但当前控制逻辑不使用角速度反馈。
- PID 参数、输出上限、正反方向及机构动作仍需在目标机器人上配置和验证。该模块尚未
  接入本地机器人配置，也未完成实车验证。

- `LimitOutput()` is empty, so no additional axis-level speed, current or slew-rate limits
  are applied. Each PID can clamp its output through `out_limit`, but this module does not
  provide robot-specific values. The front `*_min_angle` and `*_max_angle` parameters are
  not active.
- Rear endpoint actions have no position-based completion or hard-stop confirmation. If
  blockage thresholds are not configured, blockage detection will not stop the action; even
  when enabled, it cannot distinguish a mechanical endpoint from another obstruction.
- There is no general output gate for invalid feedback. An invalid rear feedback update
  clears the relative-angle initialization flag, but the control loop continues and may
  calculate output using the previous feedback values.
- `gyro_topic_name` is subscribed, but angular-rate feedback is not currently used by the
  control logic.
- PID parameters, output limits, direction and mechanism behavior must be configured and
  validated on the target robot. This module is not yet integrated into the local robot
  configuration and has not been validated on hardware.

## 事件与线程 / Events and Thread

`GetEvent()` 提供以下事件，供 `EventBinder` 绑定遥控器或键盘输入：

`GetEvent()` exposes these events for controller or keyboard bindings through `EventBinder`:

| 事件 / Event | 行为 / Behavior |
| --- | --- |
| `SET_MODE_RELAX` | 放松全部电机 / Relax all motors |
| `SET_MODE_AUTOBALANCE` | 启用或重试后轴调平 / Enable or retry rear leveling |
| `FRONT_DEPLOY` | 左右前轴分别到下拨预设 / Move each front axis to its deploy preset |
| `FRONT_RETRACT` | 左右前轴分别到收起预设 / Move each front axis to its retract preset |
| `REAR_DEPLOY` | 后轴向配置的下拨方向运动 / Move rear axis in deploy direction |
| `REAR_RETRACT` | 后轴向配置的收起方向运动 / Move rear axis in retract direction |

事件回调只缓存请求，控制线程每 2 ms 读取反馈、处理请求、检查受阻状态并执行控制。
同一轴保留最新请求；`RELAX` 优先；同周期后轴端点请求优先于调平请求。

Callbacks only cache requests. Every 2 ms, the control thread updates feedback, processes
requests, checks rear blockage and runs the controllers. The latest request is kept per
axis; `RELAX` has priority, and a rear endpoint request overrides leveling in the same
cycle.

```text
姿态 Topic + 电机反馈 / attitude Topic + motor feedback
       ↓
Update → ProcessEvents → CheckBlocked → Control
```

## 构造与参数 / Construction and Parameters

```cpp
Lift(CMD& cmd, Motor& front_left_motor, Motor& front_right_motor,
     Motor& back_motor, const Param& param);
```

前轴角度单位为 rad，速度单位为 rad/s。前轴参数包括位置/速度 PID、配置零点和动作
预设；后轴参数包括调平 PID、速度 PID、方向标志、运动速度以及可选受阻阈值。姿态与
角速度 Topic 名称可配置，默认分别为 `chassis_euler` 和 `chassis_gyro`。

Front angles use rad and angular speeds use rad/s. Front parameters include position and
speed PIDs, configured zeros and action presets. Rear parameters include leveling and speed
PIDs, direction, motion speed and optional blockage thresholds. Attitude and gyro Topic
names are configurable and default to `chassis_euler` and `chassis_gyro`.

创建顺序应为 `CMD`、三个电机、姿态 Topic 发布者、`Lift`，再创建 `EventBinder`。
在 EventBinder 的模块列表中加入 `{"lift", lift}` 后，绑定形式例如：

Construct `CMD`, the three motors and attitude Topic publishers before `Lift`, then create
`EventBinder`. Add `{"lift", lift}` to its module list; a binding has this form:

```cpp
{"dr16", DR16::SwitchPos::DR16_SW_R_POS_TOP,
 "lift", LiftEvent::SET_MODE_RELAX}
```

此处仅展示绑定语法，未预设具体遥控器或键盘分配。模块依赖 `QDU-Robomaster/CMD`、
`QDU-Robomaster/Motor` 与 LibXR。目标机构为前轴两台 DM4340、后轴 RM3508 P19 + C620
及底盘 IMU。

This is binding syntax only; no controller or keyboard assignment is prescribed. The module
depends on `QDU-Robomaster/CMD`, `QDU-Robomaster/Motor` and LibXR. Intended hardware is two
front DM4340 motors, a rear RM3508 P19 with C620, and a chassis IMU.

## 反馈坐标 / Feedback Coordinates

- 前轴角度为 DM 电机输出轴角度减去配置零点，不再次除以 DM 内部减速比。若机构工作行程
  跨越单圈角差边界，需要另行实现角度展开。
- 后轴用 RM 转子反馈的角度差累计相对位移，再除以 P19 减速比 `3591 / 187`。首次有效
  反馈仅建立本次跟踪起点，不代表处于机械最低位；重新初始化后累计值重新从零开始。
- 后轴跨圈累计假设相邻有效采样间转子位移小于半圈。该相对位置只供本次跟踪和受阻保持，
  不代表机械绝对位置，也不作为端点判据。

- Front angle is the DM output-shaft angle minus its configured zero; do not divide by the
  DM's internal reduction again. A mechanism travel crossing the single-turn boundary needs
  a separate unwrap implementation.
- Rear relative travel accumulates RM rotor-angle deltas and divides by the P19 ratio
  `3591 / 187`. The first valid feedback seeds the tracking origin and does not indicate the
  mechanical lower stop; after reinitialization, accumulated travel starts from zero again.
- Rear unwrapping assumes less than half a rotor turn between valid samples. This relative
  angle is only for the current tracking session and blocked-position hold, not an absolute
  mechanical position or endpoint criterion.
