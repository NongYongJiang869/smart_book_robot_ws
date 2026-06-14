#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "custom_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__msg__ChassisStatus() -> *const std::ffi::c_void;
}

#[link(name = "custom_interfaces__rosidl_generator_c")]
extern "C" {
    fn custom_interfaces__msg__ChassisStatus__init(msg: *mut ChassisStatus) -> bool;
    fn custom_interfaces__msg__ChassisStatus__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ChassisStatus>, size: usize) -> bool;
    fn custom_interfaces__msg__ChassisStatus__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ChassisStatus>);
    fn custom_interfaces__msg__ChassisStatus__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ChassisStatus>, out_seq: *mut rosidl_runtime_rs::Sequence<ChassisStatus>) -> bool;
}

// Corresponds to custom_interfaces__msg__ChassisStatus
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// 底盘底层状态 (stm32_bridge 发布, 用于诊断和监控)

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ChassisStatus {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,

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
    unsafe {
      let mut msg = std::mem::zeroed();
      if !custom_interfaces__msg__ChassisStatus__init(&mut msg as *mut _) {
        panic!("Call to custom_interfaces__msg__ChassisStatus__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ChassisStatus {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { custom_interfaces__msg__ChassisStatus__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { custom_interfaces__msg__ChassisStatus__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { custom_interfaces__msg__ChassisStatus__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ChassisStatus {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ChassisStatus where Self: Sized {
  const TYPE_NAME: &'static str = "custom_interfaces/msg/ChassisStatus";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__msg__ChassisStatus() }
  }
}


#[link(name = "custom_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__msg__RobotStatus() -> *const std::ffi::c_void;
}

#[link(name = "custom_interfaces__rosidl_generator_c")]
extern "C" {
    fn custom_interfaces__msg__RobotStatus__init(msg: *mut RobotStatus) -> bool;
    fn custom_interfaces__msg__RobotStatus__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RobotStatus>, size: usize) -> bool;
    fn custom_interfaces__msg__RobotStatus__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RobotStatus>);
    fn custom_interfaces__msg__RobotStatus__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RobotStatus>, out_seq: *mut rosidl_runtime_rs::Sequence<RobotStatus>) -> bool;
}

// Corresponds to custom_interfaces__msg__RobotStatus
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// 机器人整体状态 (book_search_master 发布)

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RobotStatus {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,

    /// 当前状态
    pub state: u8,

    /// 底盘
    /// 当前线速度 (m/s)
    pub linear_velocity: f32,

    /// 当前角速度 (rad/s)
    pub angular_velocity: f32,

    /// 当前任务
    /// 正在找的书名, 空闲时为空
    pub current_task: rosidl_runtime_rs::String,

    /// 错误信息
    /// 错误描述, 无错误时为空
    pub error_message: rosidl_runtime_rs::String,

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
    unsafe {
      let mut msg = std::mem::zeroed();
      if !custom_interfaces__msg__RobotStatus__init(&mut msg as *mut _) {
        panic!("Call to custom_interfaces__msg__RobotStatus__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RobotStatus {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { custom_interfaces__msg__RobotStatus__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { custom_interfaces__msg__RobotStatus__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { custom_interfaces__msg__RobotStatus__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RobotStatus {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RobotStatus where Self: Sized {
  const TYPE_NAME: &'static str = "custom_interfaces/msg/RobotStatus";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__msg__RobotStatus() }
  }
}


