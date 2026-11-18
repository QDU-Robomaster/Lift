#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 底盘抬升、姿态调平与受阻重试控制框架 / Chassis lift, attitude-leveling and blocked-motion retry scaffold
depends:
- id: QDU-Robomaster/CMD
  ref: same-or-dev
- id: QDU-Robomaster/Motor
  ref: same-or-dev
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "CMD.hpp"
#include "Motor.hpp"
#include "event.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "message.hpp"
#include "mutex.hpp"
#include "pid.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "transform.hpp"

/**
 * @brief 抬升模式、前轴预设与后轴端点动作事件，在 GetEvent() 上注册。
 *        Lift mode, front preset and rear endpoint-action events on GetEvent().
 */
enum class LiftEvent : uint8_t
{
  SET_MODE_RELAX,        ///< 放松 / Relax
  SET_MODE_AUTOBALANCE,  ///< 启用或重试后轴调平 / Enable or retry rear-axis leveling
  FRONT_DEPLOY,          ///< 前轴下拨到预设位置 / Deploy the front axis to its preset
  FRONT_RETRACT,         ///< 前轴收起到预设位置 / Retract the front axis to its preset
  REAR_DEPLOY,   ///< 后轴向下降端点运动 / Move the rear    axis toward the lower stop
  REAR_RETRACT,  ///< 后轴向收起端点运动 / Move the rear axis toward the upper stop
};

/**
 * @brief 两个前轴电机、一个后轴电机的事件驱动抬升控制框架。
 *        Event-driven lift scaffold with two front motors and one rear motor.
 * @note 已实现前轴位置/速度串级控制、后轴调平与受阻位置保持；输出限幅仍待实现。
 *       Front cascaded control, rear leveling and blocked-position hold are implemented;
 *       output limiting remains TODO.
 */
class Lift
{
 public:
  enum class Mode : uint8_t
  {
    RELAX,        ///< 放松 / Relax
    AUTOBALANCE,  ///< 自动调平，允许独立升降动作 / Leveling with independent axis actions
  };

  /**
   * @brief 抬升配置，角度单位 rad；前轴预设与限位相对配置零点，后轴无需回零。
   *        Lift parameters in rad. Front presets and limits use configured offsets;
   *        rear leveling and endpoint actions do not require homing.
   */
  struct Param
  {
    uint32_t task_stack_depth;                      ///< 线程栈深 / Thread stack depth
    LibXR::PID<float>::Param pid_front_left_angle;  ///< 左前位置环 / Front-left position
    LibXR::PID<float>::Param pid_front_left_omega;  ///< 左前速度环 / Front-left speed
    LibXR::PID<float>::Param pid_front_right_angle;  ///< 右前位置环 / Front-right position
    LibXR::PID<float>::Param pid_front_right_omega;  ///< 右前速度环 / Front-right speed
    LibXR::PID<float>::Param pid_back_angle;         ///< 后轴位置环 / Rear position loop
    LibXR::PID<float>::Param pid_back_omega;         ///< 后轴速度环 / Rear speed
    LibXR::PID<float>::Param pid_balance;            ///< 车体 pitch 调平环 / Body pitch

    float front_left_max_angle;   ///< 左前软上限 / Front-left upper soft limit
    float front_left_min_angle;   ///< 左前软下限 / Front-left lower soft limit
    float front_right_max_angle;  ///< 右前软上限 / Front-right upper soft limit
    float front_right_min_angle;  ///< 右前软下限 / Front-right lower soft limit
    float front_left_zero;        ///< 左前位置零点 / Front-left position zero
    float front_right_zero;       ///< 右前位置零点 / Front-right position zero

    float front_left_deploy_angle;    ///< 左前下拨预设 / Front-left deployment preset
    float front_left_retract_angle;   ///< 左前收起预设 / Front-left retraction preset
    float front_right_deploy_angle;   ///< 右前下拨预设 / Front-right deployment preset
    float front_right_retract_angle;  ///< 右前收起预设 / Front-right retraction preset
    bool reverse_flag;  ///< 后轴角度与 pitch 同向为 true / Rear angle follows pitch
    float back_deploy_omega;  ///< 后轴下拨速度 / Rear deployment speed
    float back_block_speed_threshold;  ///< 受阻低速阈值，rad/s；0 关闭检测 / Low speed threshold; 0 disables
    float back_block_torque_threshold;  ///< 电机侧扭矩阈值，N·m；0 关闭检测 / Motor torque threshold; 0 disables

    LibXR::Thread::Priority thread_priority;  ///< 线程优先级 / Thread priority
    const char* euler_topic_name;  ///< 底盘欧拉角 Topic / Chassis Euler angle Topic
    const char* gyro_topic_name;   ///< 底盘角速度 Topic / Chassis angular velocity Topic
  };

  /**
   * @brief 保存配置、注册抬升与 CMD 上下线事件，最后启动控制线程。
   *        Store parameters, register lift and CMD link events, then start the thread.
   * @param cmd 命令模块，提供上下线事件 / Command Module providing link events
   * @param front_left_motor 左前 DM 电机 / Front-left DM motor
   * @param front_right_motor 右前 DM 电机 / Front-right DM motor
   * @param back_motor 后轴 RM 电机 / Rear RM motor
   * @param param 抬升配置参数 / Lift parameters
   */
  Lift(CMD& cmd, Motor& front_left_motor, Motor& front_right_motor, Motor& back_motor,
       const Param& param = {.task_stack_depth = 2048,
                             .pid_front_left_angle = {},
                             .pid_front_left_omega = {},
                             .pid_front_right_angle = {},
                             .pid_front_right_omega = {},
                             .pid_back_angle = {},
                             .pid_back_omega = {},
                             .pid_balance = {},
                             .front_left_max_angle = 0.0f,
                             .front_left_min_angle = 0.0f,
                             .front_right_max_angle = 0.0f,
                             .front_right_min_angle = 0.0f,
                             .front_left_zero = 0.0f,
                             .front_right_zero = 0.0f,
                             .front_left_deploy_angle = 0.0f,
                             .front_left_retract_angle = 0.0f,
                             .front_right_deploy_angle = 0.0f,
                             .front_right_retract_angle = 0.0f,
                             .reverse_flag = false,
                             .back_deploy_omega = 0.0f,
                             .back_block_speed_threshold = 0.0f,
                             .back_block_torque_threshold = 0.0f,
                             .thread_priority = LibXR::Thread::Priority::MEDIUM,
                             .euler_topic_name = "chassis_euler",
                             .gyro_topic_name = "chassis_gyro"})
      : param_(param),
        front_left_motor_(&front_left_motor),
        front_right_motor_(&front_right_motor),
        back_motor_(&back_motor),
        pid_front_left_angle_(param.pid_front_left_angle),
        pid_front_left_omega_(param.pid_front_left_omega),
        pid_front_right_angle_(param.pid_front_right_angle),
        pid_front_right_omega_(param.pid_front_right_omega),
        pid_back_angle_(param.pid_back_angle),
        pid_back_omega_(param.pid_back_omega),
        pid_balance_(param.pid_balance)
  {
    auto callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Lift* lift, uint32_t event_id)
        {
          UNUSED(in_isr);
          lift->EventHandler(static_cast<LiftEvent>(event_id));
        },
        this);

    lift_event_.Register(static_cast<uint32_t>(LiftEvent::SET_MODE_RELAX), callback);
    lift_event_.Register(static_cast<uint32_t>(LiftEvent::SET_MODE_AUTOBALANCE),
                         callback);
    lift_event_.Register(static_cast<uint32_t>(LiftEvent::FRONT_DEPLOY), callback);
    lift_event_.Register(static_cast<uint32_t>(LiftEvent::FRONT_RETRACT), callback);
    lift_event_.Register(static_cast<uint32_t>(LiftEvent::REAR_DEPLOY), callback);
    lift_event_.Register(static_cast<uint32_t>(LiftEvent::REAR_RETRACT), callback);

    auto link_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Lift* lift, uint32_t event_id)
        {
          UNUSED(in_isr);
          UNUSED(event_id);
          lift->EventHandler(LiftEvent::SET_MODE_RELAX);
        },
        this);
    cmd.GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, link_callback);
    cmd.GetEvent().Register(CMD::CMD_EVENT_START_CTRL, link_callback);

    thread_.Create(this, ThreadFunc, "LiftThread", param.task_stack_depth,
                   param.thread_priority);
  }

  /**
   * @brief 订阅底盘姿态与角速度，每 2 ms 刷新反馈、处理请求并调用控制入口。
   *        Subscribe to chassis attitude and angular velocity, then update feedback,
   *        process requests and call the control entry every 2 ms.
   */
  static void ThreadFunc(Lift* lift)
  {
    LibXR::Topic::ASyncSubscriber<LibXR::EulerAngle<float>> euler_suber(
        lift->param_.euler_topic_name);
    LibXR::Topic::ASyncSubscriber<Eigen::Matrix<float, 3, 1>> gyro_suber(
        lift->param_.gyro_topic_name);
    euler_suber.StartWaiting();
    gyro_suber.StartWaiting();

    auto last_wakeup = LibXR::Timebase::GetMilliseconds();
    lift->last_update_time_ = LibXR::Timebase::GetMicroseconds();
    while (true)
    {
      if (euler_suber.Available())
      {
        lift->euler_ = euler_suber.GetData();
        euler_suber.StartWaiting();
      }
      if (gyro_suber.Available())
      {
        lift->gyro_data_ = gyro_suber.GetData();
        gyro_suber.StartWaiting();
      }

      lift->Update();
      lift->ProcessEvents();
      lift->CheckBlocked();
      lift->Control();
      LibXR::Thread::SleepUntil(last_wakeup, 2);
    }
  }

  /// 获取事件对象，供 EventBinder 绑定 / Event object for EventBinder bindings
  LibXR::Event& GetEvent() { return lift_event_; }

 private:
  enum class Preset : uint8_t
  {
    NONE,
    DEPLOY,   ///< 下拨到预设位置 / Deploy to preset
    RETRACT,  ///< 收起到预设位置 / Retract to preset
  };

  /// 后轴内部动作；受阻暂停另用标志表示 / Rear action; blocking is a separate flag
  enum class BackAction : uint8_t
  {
    IDLE,         ///< 无动作请求 / No action requested
    AUTOBALANCE,  ///< 姿态环生成速度目标 / Attitude loop generates a speed target
    DEPLOY,       ///< 向下降端点运动 / Move toward the lower stop
    RETRACT,      ///< 向收起端点运动 / Move toward the upper stop
  };

  struct PendingEvents
  {
    bool relax = false;
    bool autobalance = false;
    Preset front = Preset::NONE;
    Preset back = Preset::NONE;
  };

  /**
   * @brief 在线程上下文记录请求；同一轴保留最新请求，放松优先。
   *        Record requests in thread context; keep the latest per axis, with relax
   *        taking priority. Motor operations are deferred to the control thread.
   */
  void EventHandler(LiftEvent event)
  {
    LibXR::Mutex::LockGuard lock(event_mutex_);
    if (event == LiftEvent::SET_MODE_RELAX)
    {
      pending_events_ = {};
      pending_events_.relax = true;
      return;
    }
    if (pending_events_.relax)
    {
      return;
    }

    switch (event)
    {
      case LiftEvent::SET_MODE_AUTOBALANCE:
        pending_events_.autobalance = true;
        break;
      case LiftEvent::FRONT_DEPLOY:
        pending_events_.front = Preset::DEPLOY;
        break;
      case LiftEvent::FRONT_RETRACT:
        pending_events_.front = Preset::RETRACT;
        break;
      case LiftEvent::REAR_DEPLOY:
        pending_events_.back = Preset::DEPLOY;
        break;
      case LiftEvent::REAR_RETRACT:
        pending_events_.back = Preset::RETRACT;
        break;
      default:
        break;
    }
  }

  /// 在控制线程分发模式与预设请求 / Dispatch mode and preset requests in the thread
  void ProcessEvents()
  {
    PendingEvents events;
    {
      LibXR::Mutex::LockGuard lock(event_mutex_);
      events = pending_events_;
      pending_events_ = {};
    }

    if (events.relax)
    {
      SetMode(Mode::RELAX);
      return;
    }
    if (events.autobalance)
    {
      SetMode(Mode::AUTOBALANCE);
    }
    if (current_mode_ == Mode::RELAX)
    {
      return;
    }
    if (events.front != Preset::NONE)
    {
      SetFrontPreset(events.front);
    }
    if (events.back != Preset::NONE)
    {
      // 同周期后轴端点请求优先于调平 / Rear endpoint requests override leveling here
      SetBackPreset(events.back);
    }
  }

  /**
   * @brief 刷新反馈与周期，计算后轴本次跟踪的相对输出轴位移和角速度。
   *        Refresh feedback and the time step; unwrap the rear rotor angle and
   *        convert travel and speed to the gearbox output shaft.
   */
  void Update()
  {
    const float LAST_BACK_ANGLE = back_feedback_.abs_angle;

    front_left_motor_->Update();
    front_right_motor_->Update();
    const auto BACK_UPDATE_RESULT = back_motor_->Update();
    front_left_feedback_ = front_left_motor_->GetFeedback();
    front_right_feedback_ = front_right_motor_->GetFeedback();
    back_feedback_ = back_motor_->GetFeedback();

    auto now = LibXR::Timebase::GetMicroseconds();
    dt_ = (now - last_update_time_).ToSecondf();
    last_update_time_ = now;

    // DM 已反馈输出轴角度；相对零点的行程须在 [-pi, pi) 内。
    // DM reports output-shaft angles; travel relative to zero must stay in [-pi, pi).
    abs_front_left_angle_ = front_left_feedback_.abs_angle - param_.front_left_zero;
    abs_front_right_angle_ = front_right_feedback_.abs_angle - param_.front_right_zero;

    // RMMotor 解码过反馈后 state 才为 1；OK 不保证本周期收到新帧。
    // RMMotor sets state to 1 after decoding; OK does not guarantee a new frame.
    if (BACK_UPDATE_RESULT != LibXR::ErrorCode::OK || back_feedback_.state != 1 ||
        !std::isfinite(static_cast<float>(back_feedback_.abs_angle)) ||
        !std::isfinite(back_feedback_.omega))
    {
      back_angle_initialized_ = false;
      return;
    }

    if (!back_angle_initialized_)
    {
      // 首帧只建立相对位移起点；重连后重新计数，无需机械回零。
      // Seed relative travel on the first sample and after reconnecting; no homing.
      back_rotor_angle_ = 0.0f;
      back_angle_initialized_ = true;
    }
    else
    {
      // CycleValue 差值自动跨圈；相邻有效反馈间转子位移必须小于半圈。
      // CycleValue unwraps the delta; rotor travel between samples must be < pi.
      back_rotor_angle_ += back_feedback_.abs_angle - LAST_BACK_ANGLE;
    }
    back_relative_angle_ = back_rotor_angle_ / BACK_REDUCTION_RATIO;
    back_omega_ = back_feedback_.omega / BACK_REDUCTION_RATIO;
  }

  void SetMode(Mode mode)
  {
    current_mode_ = mode;
    back_blocked_ = false;
    blocked_condition_duration_ = 0.0f;
    if (mode == Mode::RELAX)
    {
      front_preset_ = Preset::NONE;
      back_action_ = BackAction::IDLE;
      ResetPID();
    }
    else
    {
      // 同模式事件也重启后轴调平，保留前轴动作 / Retry rear leveling, keep front action
      back_action_ = BackAction::AUTOBALANCE;
      pid_balance_.Reset();
      pid_back_angle_.Reset();
      pid_back_omega_.Reset();
    }
    // TODO: 控制输出接入时增加平滑过渡。
  }

  void SetFrontPreset(Preset preset)
  {
    front_preset_ = preset;
    pid_front_left_angle_.Reset();
    pid_front_left_omega_.Reset();
    pid_front_right_angle_.Reset();
    pid_front_right_omega_.Reset();
    if (preset != Preset::NONE)
    {
      front_left_motor_->Enable();
      front_right_motor_->Enable();
    }
  }

  void SetBackPreset(Preset preset)
  {
    if (preset == Preset::NONE)
    {
      return;
    }
    back_action_ = preset == Preset::DEPLOY ? BackAction::DEPLOY : BackAction::RETRACT;
    back_blocked_ = false;
    pid_balance_.Reset();
    pid_back_angle_.Reset();
    pid_back_omega_.Reset();
    // TODO: 按指定方向限速、限流寻端点；不使用断电前的位置或自动恢复调平。
  }

  /// 受阻检测入口，不把受阻当作端点成功 / Block detection, not endpoint confirmation
  void CheckBlocked()
  {
    if (back_blocked_ || !back_angle_initialized_ ||
        param_.back_block_speed_threshold <= 0.0f ||
        param_.back_block_torque_threshold <= 0.0f)
    {
      return;
    }

    const bool MOTION_REQUESTED =
        back_action_ != BackAction::IDLE &&
        std::abs(target_back_omega_) > param_.back_block_speed_threshold;
    const bool LOW_SPEED = std::abs(back_omega_) < param_.back_block_speed_threshold;
    const bool HIGH_TORQUE = std::abs(back_feedback_.torque) >
                             param_.back_block_torque_threshold;

    // 时间滤波：连续满足受阻条件一定时长才判定为受阻
    // Time filtering: blocked only after conditions persist for duration threshold
    if (MOTION_REQUESTED && LOW_SPEED && HIGH_TORQUE)
    {
      blocked_condition_duration_ += dt_;
      constexpr float BLOCK_CONFIRM_DURATION = 0.05f;  // 50ms
      if (blocked_condition_duration_ >= BLOCK_CONFIRM_DURATION)
      {
        LatchBackBlocked();
        blocked_condition_duration_ = 0.0f;
      }
    }
    else
    {
      // 条件不满足时重置计时器 / Reset timer when conditions not met
      blocked_condition_duration_ = 0.0f;
    }
  }

  /// 受阻确认时锁存当前后轴位置 / Latch current rear position when blocked
  void LatchBackBlocked()
  {
    if (back_blocked_ || !back_angle_initialized_)
    {
      return;
    }

    back_hold_angle_ = back_relative_angle_;
    pid_back_angle_.Reset();
    pid_back_omega_.Reset();
    back_blocked_ = true;
  }

  void Control()
  {
    if (current_mode_ == Mode::RELAX)
    {
      Relax();
      return;
    }

    ControlFront();
    ControlBack();
    LimitOutput();

    if (front_preset_ == Preset::NONE)
    {
      front_left_motor_->Relax();
      front_right_motor_->Relax();
    }
    else
    {
      front_left_motor_->Control(
          {.mode = Motor::ControlMode::MODE_TORQUE, .torque = front_left_torque_});
      front_right_motor_->Control(
          {.mode = Motor::ControlMode::MODE_TORQUE, .torque = front_right_torque_});
    }
    back_motor_->Control(
        {.mode = Motor::ControlMode::MODE_CURRENT, .velocity = back_output_});
  }

  void ControlFront()
  {
    switch (front_preset_)
    {
      case Preset::DEPLOY:
        target_front_left_angle_ = param_.front_left_deploy_angle;
        target_front_right_angle_ = param_.front_right_deploy_angle;
        break;
      case Preset::RETRACT:
        target_front_right_angle_ = param_.front_right_retract_angle;
        target_front_left_angle_ = param_.front_left_retract_angle;
        break;
      case Preset::NONE:
        front_left_torque_ = 0.0f;
        front_right_torque_ = 0.0f;
        return;
    }

    target_front_left_omega_ = pid_front_left_angle_.Calculate(
        target_front_left_angle_, abs_front_left_angle_, dt_);
    target_front_right_omega_ = pid_front_right_angle_.Calculate(
        target_front_right_angle_, abs_front_right_angle_, dt_);
    front_left_torque_ = pid_front_left_omega_.Calculate(target_front_left_omega_,
                                                         front_left_feedback_.omega, dt_);
    front_right_torque_ = pid_front_right_omega_.Calculate(
        target_front_right_omega_, front_right_feedback_.omega, dt_);
  }

  void ControlBack()
  {
    if (back_blocked_)
    {
      target_back_omega_ =
          pid_back_angle_.Calculate(back_hold_angle_, back_relative_angle_, dt_);
      back_output_ = pid_back_omega_.Calculate(target_back_omega_, back_omega_, dt_);
      return;
    }

    switch (back_action_)
    {
      case BackAction::AUTOBALANCE:
        AutoBalance();
        back_output_ = pid_back_omega_.Calculate(target_back_omega_, back_omega_, dt_);
        break;
      case BackAction::DEPLOY:
      case BackAction::RETRACT:
        MoveBackToStop();
        back_output_ = pid_back_omega_.Calculate(target_back_omega_, back_omega_, dt_);
        break;
      case BackAction::IDLE:
        back_output_ = 0.0f;
        break;
    }
  }

  void AutoBalance()
  {
    target_back_omega_ = pid_balance_.Calculate(0.0f,euler_.Pitch(), dt_);
    target_back_omega_ = std::clamp(target_back_omega_, -param_.back_deploy_omega,
                                  param_.back_deploy_omega);
    if(!param_.reverse_flag)
    {
      target_back_omega_ = -target_back_omega_;
    }
  }

  /// 后轴端点动作入口 / Rear endpoint-action entry
  void MoveBackToStop()
  {
    switch(back_action_)
    {
      case BackAction::DEPLOY:
        target_back_omega_ = param_.back_deploy_omega;
        break;
      case BackAction::RETRACT:
        target_back_omega_ = -param_.back_deploy_omega;
        break;
      default:
        target_back_omega_ = 0.0f;
        break;
    }


    // TODO: DEPLOY/RETRACT 选择下降/收起方向，生成限速目标。
    // 受阻只表明无法继续推进，不记录机械零点，不自动认定已到上下限。
  }

  /// 速度、电流及输出变化率限制 / Speed, current and output slew limits
  void LimitOutput()
  {
    // TODO: 各轴限速、限流、平滑切换与积分抗饱和；重试不提高允许的输出。
  }

  void ResetPID()
  {
    pid_front_left_angle_.Reset();
    pid_front_left_omega_.Reset();
    pid_front_right_angle_.Reset();
    pid_front_right_omega_.Reset();
    pid_back_angle_.Reset();
    pid_back_omega_.Reset();
    pid_balance_.Reset();
  }

  void Relax()
  {
    front_left_motor_->Relax();
    front_right_motor_->Relax();
    back_motor_->Relax();
  }

  static constexpr float BACK_REDUCTION_RATIO = 3591.0f / 187.0f;  ///< M3508 P19

  const Param param_;
  Motor* front_left_motor_;
  Motor* front_right_motor_;
  Motor* back_motor_;

  LibXR::PID<float> pid_front_left_angle_;
  LibXR::PID<float> pid_front_left_omega_;
  LibXR::PID<float> pid_front_right_angle_;
  LibXR::PID<float> pid_front_right_omega_;
  LibXR::PID<float> pid_back_angle_;
  LibXR::PID<float> pid_back_omega_;
  LibXR::PID<float> pid_balance_;

  Motor::Feedback front_left_feedback_{};
  Motor::Feedback front_right_feedback_{};
  Motor::Feedback back_feedback_{};
  LibXR::EulerAngle<float> euler_{};
  Eigen::Matrix<float, 3, 1> gyro_data_ = Eigen::Matrix<float, 3, 1>::Zero();
  float back_hold_angle_ = 0.0f;  ///< 受阻时锁存的后轴相对角度
  float target_front_left_angle_;
  float target_front_right_angle_;
  float target_front_left_omega_;
  float target_front_right_omega_;
  float front_left_torque_ = 0.0f;   ///< 左前速度环力矩输出，N·m
  float front_right_torque_ = 0.0f;  ///< 右前速度环力矩输出，N·m
  float back_output_ = 0.0f;  ///< 后轴输出
  float target_back_omega_ = 0.0f;  ///< 姿态环生成的后轴速度目标 / Rear speed target from attitude loop
  // 前轴相对配置零点的输出轴角度，rad / Front output-shaft offsets in rad
  float abs_front_left_angle_ = 0.0f;
  float abs_front_right_angle_ = 0.0f;
  // 后轴仅保留本次跟踪的相对位移，不作为调平或端点动作的位置门槛。
  // Rear travel is local to this tracking session, not a leveling or endpoint gate.
  float back_relative_angle_ = 0.0f;
  float back_omega_ = 0.0f;        ///< 后轴输出轴角速度，rad/s / Rear output speed, rad/s
  float back_rotor_angle_ = 0.0f;  ///< 后轴累计转子位移 / Accumulated rear rotor travel
  bool back_angle_initialized_ = false;  ///< 已建立累计起点 / Relative tracking seeded

  float dt_ = 0.0f;
  LibXR::MicrosecondTimestamp last_update_time_ = 0;

  Mode current_mode_ = Mode::RELAX;
  Preset front_preset_ = Preset::NONE;
  BackAction back_action_ = BackAction::IDLE;
  bool back_blocked_ = false;  ///< 后轴受阻位置保持 / Rear blocked-position hold
  float blocked_condition_duration_ = 0.0f;  ///< 受阻条件持续时间累加器 / Blocked condition duration accumulator
  PendingEvents pending_events_{};
  LibXR::Mutex event_mutex_;
  LibXR::Event lift_event_;
  LibXR::Thread thread_;
};
