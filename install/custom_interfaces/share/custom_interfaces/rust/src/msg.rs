#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to custom_interfaces__msg__ChassisStatus
/// 底盘底层状态 (stm32_bridge 发布, 用于诊断和监控)

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ChassisStatus {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,

    /// 4个电机使能状态 (位0~3, 1=使能)
    pub motor_enabled: u8,

    /// 传感器状态
    /// 急停是否触发
    pub emergency_stop: bool,

    /// 前碰撞传感器
    pub collision_front: bool,

    /// 后碰撞传感器
    pub collision_rear: bool,

    /// 错误码 (0=正常)
    pub error_code: u16,

    /// 通信质量
    /// 最近一次速度指令的往返延迟
    pub cmd_latency_ms: f32,

    /// 近1秒内丢帧数量
    pub lost_frames: u16,

}

impl ChassisStatus {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ERR_NONE: u16 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ERR_ESTOP: u16 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ERR_FRONT_COLLISION: u16 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ERR_REAR_COLLISION: u16 = 4;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ERR_COMM_TIMEOUT: u16 = 256;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ERR_IMU_FAULT: u16 = 1024;

}


impl Default for ChassisStatus {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::ChassisStatus::default())
  }
}

impl rosidl_runtime_rs::Message for ChassisStatus {
  type RmwMsg = super::msg::rmw::ChassisStatus;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        motor_enabled: msg.motor_enabled,
        emergency_stop: msg.emergency_stop,
        collision_front: msg.collision_front,
        collision_rear: msg.collision_rear,
        error_code: msg.error_code,
        cmd_latency_ms: msg.cmd_latency_ms,
        lost_frames: msg.lost_frames,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      motor_enabled: msg.motor_enabled,
      emergency_stop: msg.emergency_stop,
      collision_front: msg.collision_front,
      collision_rear: msg.collision_rear,
      error_code: msg.error_code,
      cmd_latency_ms: msg.cmd_latency_ms,
      lost_frames: msg.lost_frames,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      motor_enabled: msg.motor_enabled,
      emergency_stop: msg.emergency_stop,
      collision_front: msg.collision_front,
      collision_rear: msg.collision_rear,
      error_code: msg.error_code,
      cmd_latency_ms: msg.cmd_latency_ms,
      lost_frames: msg.lost_frames,
    }
  }
}


// Corresponds to custom_interfaces__msg__RobotStatus
/// 机器人整体状态 (book_search_master 发布)

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RobotStatus {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,

    /// 当前状态
    pub state: u8,

    /// 底盘
    /// 当前线速度 (m/s)
    pub linear_velocity: f32,

    /// 当前角速度 (rad/s)
    pub angular_velocity: f32,

    /// 当前任务
    /// 正在找的书名, 空闲时为空
    pub current_task: std::string::String,

    /// 错误信息
    /// 错误描述, 无错误时为空
    pub error_message: std::string::String,

}

impl RobotStatus {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const IDLE: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const NAVIGATING: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const SCANNING: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const APPROACHING: u8 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const PICKING: u8 = 4;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const RETURNING: u8 = 5;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ERROR: u8 = 6;

}


impl Default for RobotStatus {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::RobotStatus::default())
  }
}

impl rosidl_runtime_rs::Message for RobotStatus {
  type RmwMsg = super::msg::rmw::RobotStatus;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        state: msg.state,
        linear_velocity: msg.linear_velocity,
        angular_velocity: msg.angular_velocity,
        current_task: msg.current_task.as_str().into(),
        error_message: msg.error_message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      state: msg.state,
      linear_velocity: msg.linear_velocity,
      angular_velocity: msg.angular_velocity,
        current_task: msg.current_task.as_str().into(),
        error_message: msg.error_message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      state: msg.state,
      linear_velocity: msg.linear_velocity,
      angular_velocity: msg.angular_velocity,
      current_task: msg.current_task.to_string(),
      error_message: msg.error_message.to_string(),
    }
  }
}


