#pragma once 

#include <chrono>
#include <cmath>
#include <vector>
#include <string>
struct ArmParameters{
  std::string arm_port;
  unsigned int arm_baudrate;
  float Y_max;
  std::vector<float> joint_dead_zone;
  std::vector<float> linkH;
  bool yaw_zero_offset_enable;
  float yaw_zero_offset;
  std::string speed_level;

  int guide_elongate_value;
  int gun_elongate_value;
  int cable_contract_value;
  int gripper_shift_central_value;
  int gun_lock_value;
  int gripper_close_value;

  int guide_contract_value;
  int gun_contract_value;
  int cable_elongate_value;
  int gripper_shift_away_value;
  int gun_unlock_value;
  int gripper_open_value;

  int door_open_value_send;
  int door_open_value_check;
  double door_open_timeout;
  int door_close_value_send;
  int door_close_value_check;
  double door_close_timeout;
  std::string carm;
};



ArmParameters amParameters={
  .arm_port="/tmp/ttyV1",
  .arm_baudrate=3000000,
  .Y_max=.8,
  .joint_dead_zone={0.0005,0.0005,0.0005,0.0005,0.1,2},
 
  .yaw_zero_offset_enable=true,
  .yaw_zero_offset=0.76,
  .speed_level="middle",

  .guide_elongate_value=600,
  .gun_elongate_value=800,
  .cable_contract_value=1200,
  .gripper_shift_central_value=1400,
  .gun_lock_value=-600,
  .gripper_close_value=-500,

  .guide_contract_value=-700,
  .gun_contract_value=-800,
  .cable_elongate_value=-600,
  .gripper_shift_away_value=-600,
  .gun_unlock_value=800,
  .gripper_open_value=500,
 
};


typedef struct {
  int alignment_laser_original;
  int alignment_well_distance;
  int alignment_well_timeout;
  int detached_well_distance;
  int attached_well_distance;
  bool auto_detach;
} GunActionParameters;
GunActionParameters gunActionParameters={
  .alignment_laser_original = 82,
  .alignment_well_distance = 156,
  .alignment_well_timeout=15000,
  .detached_well_distance = 68,
  .attached_well_distance=220,
  .auto_detach = true,
};


typedef struct {
  float local_detection_yaw;
  //   float local_detection_pitch;
  // float attach_z_axis_adjust;
  float attach_z_axis_adjust_in_while;
  float attach_z_axis_adjust_before_while;
  float attach_x_axis_adjust_before_while;
  float detach_z_axis_adjust_before_plug;
  //   float attach_z_axis_dancing_step;
  //   float attach_z_axis_dancing_half_period;
  //   float detach_z_axis_adjust;
  //   float detach_z_axis_dancing_step;
  //   float detach_z_axis_dancing_half_period;
  //   float elongate_pitch;
  //   float elongate_speed;

  int gun_elongate_distance_to_grasp;
  int gun_elongate_distance_to_insert;
  int time_start_attach_delay;

  int time_gun_elongate_delay;
  int time_gun_contract_delay;
  int time_gripper_shift_delay;
  int time_guide_elongate_delay;
  int time_gripper_grasp_delay;
  int time_arm_move_finish_delay;

  int aligned_well_detect_box_result_x0;
  int aligned_well_detect_box_result_x0_error_threshold;
  int aligned_well_detect_box_result_y1;
  int aligned_well_detect_box_result_y1_error_threshold;

} CompensationParameters;

CompensationParameters compensationParameters={
 .time_gun_contract_delay = 15000,
 .time_guide_elongate_delay=2000,

};