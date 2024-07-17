/**
 * Copyright (c) 2023, WissonRobotics
 * File: config_center.h
 * Author: Zhiwei Huang (huangzhiwei@wissonrobotics.com)
 * Version 1.0
 * Date: 2023-12-06
 * Brief:
 */
#ifndef CONFIG_H_
#define CONFIG_H_
#include <pthread.h>
#include <pwd.h>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <jsoncpp/json/json.h>
#include "yaml-cpp/yaml.h"

typedef struct {
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
} ArmParameters;

typedef struct {
  double global_cam_detection_timeout;
  int global_planning_times_limit;
  int global_repeat_times_limit;

  double local_cam_detection_timeout;
  int local_planning_times_limit;
  int local_repeat_times_limit;
  int local_adjust_times_limit;

  double attach_gun_insert_timeout;
  double attach_guide_aligned_timeout;
  double attach_cam_unplug_assessing_timeout;
  int attach_failed_times_limit;

  double detach_gun_contract_timeout;
  std::string car_name;

} TaskParameters;
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

typedef struct {
  // camera
  std::vector<float> camera_global_resolution;
  std::vector<float> camera_local_resolution;
  std::string camera_data_save_dir;
  int camera_data_save_count;
  float camera_global_clipping_distance_in_meters;

  // model
  //     std::string model_device;
  //    int model_cpu_threads;
  std::string model_dir;
  float model_detection_threshold_high;
  float model_detection_threshold_low;
  std::vector<float> model_local_bbox_size;

  // point cloud
  // 检测物体的实际物理尺寸
  std::vector<float> point_cloud_original_size;
  std::vector<float> point_cloud_global_clipping_z_range_m;
  // 局部摄像头是berxel071，最近观测距离是0.1m，基座距离相机的范围在0.1~0.3m，根据实际调整
  std::vector<float> point_cloud_local_clipping_z_range_m;
  // downsampling voxel size
  float point_cloud_global_downsample_voxel_size;
  float point_cloud_local_downsample_voxel_size;
  // 点云聚类的参数
  float point_cloud_cluster_eps;
  float point_cloud_cluster_min_points;
  // 点云滤波参数
  // 统计滤波参数
  float point_cloud_statistical_outlier_removal_nb_neighbors;
  float point_cloud_statistical_outlier_removal_std_ratio;
  // 球半径滤波参数
  float point_cloud_radius_outlier_removal_nb_points;
  float point_cloud_radius_outlier_removal_radius;
  float point_cloud_calculate_points;
  std::string point_cloud_bbox_corner_crop_type;
} VisualParameters;

typedef struct {
  int alignment_laser_original;
  int alignment_well_distance;
  int alignment_well_timeout;
  int detached_well_distance;
  int attached_well_distance;
  bool auto_detach;
} GunActionParameters;

typedef struct {
  float global_strategy_x_offset_min;
  float global_strategy_x_offset;
  float global_strategy_y_offset;
  float global_strategy_z_offset;
  float global_strategy_pitch;

  float local_strategy_x_offset_stage1;
  float local_strategy_y_offset_stage1;
  float local_strategy_z_offset_stage1;
  float local_strategy_pitch_stage1;

  float local_strategy_x_offset_stage2;
  float local_strategy_y_offset_stage2;
  float local_strategy_z_offset_stage2;
  float local_strategy_pitch_stage2;

  float local_in_arm_strategy_x_offset;
  float local_in_arm_strategy_y_offset;
  float local_in_arm_strategy_z_offset;
  bool is_multiple_local_localization;
  std::vector<float> location_convergence_threshold;
  std::vector<float> global_init_pose;
} TargetStrategyParameters;

typedef struct {
  float local_camera_x_init;
  float local_camera_y_init;
  float local_camera_z_init;
  float local_camera_yaw_init;
  float local_camera_roll_init;

  float global_camera_x_init;
  float global_camera_y_init;
  float global_camera_z_init;
  float global_camera_yaw_init;
  float global_camera_roll_init;

  float inlet_yaw_init;

  std::vector<float> arm_ready_position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

} PositionParameters;
#define CONFIG_VISUAL_PARAM (ConfigCenter::Instance()->GetVisualParameters())
#define CONFIG_ARM_PARAM (ConfigCenter::Instance()->GetArmParameters())
#define CONFIG_COMPENSATION_PARAM (ConfigCenter::Instance()->GetCompensationParameters())
#define CONFIG_GUN_PARAM (ConfigCenter::Instance()->GetGunParameters())
#define CONFIG_POSTION_PARAM (ConfigCenter::Instance()->GetPositionParameters())
#define CONFIG_STRATEGY_PARAM (ConfigCenter::Instance()->GetTargetStrategyParameters())
#define CONFIG_TASK_PARAM (ConfigCenter::Instance()->GetTaskParameters())
class ConfigCenter {
 private:
  ConfigCenter() {}
  ~ConfigCenter() {}
  VisualParameters visual_param_;
  ArmParameters arm_param_;
  CompensationParameters compensation_param_;
  GunActionParameters gun_param_;
  TargetStrategyParameters target_strategy_param_;
  PositionParameters position_param_;
  TaskParameters task_param_;
  std::string config_path_;
  std::string yaml_path_;
  YAML::Node monos_yaml_;
  Json::Value config_json_value_;

 public:
  static ConfigCenter* Instance() {
    static ConfigCenter instance;
    return &instance;
  }
  bool InitConfig(std::string car_name);
  VisualParameters& GetVisualParameters() { return visual_param_; }
  ArmParameters& GetArmParameters() { return arm_param_; }
  GunActionParameters& GetGunParameters() { return gun_param_; }
  CompensationParameters& GetCompensationParameters() { return compensation_param_; }
  TargetStrategyParameters& GetTargetStrategyParameters() { return target_strategy_param_; }
  PositionParameters& GetPositionParameters() { return position_param_; }
  TaskParameters& GetTaskParameters() { return task_param_; }
  Json::Value& GetConfigJsonValue() { return config_json_value_; }
  YAML::Node& GetConfigYamlValue() { return monos_yaml_; }
  void ConvertJsonToYaml(const Json::Value& jsonValue, YAML::Node& yamlNode);
  void SaveConfigJsonValue(Json::Value& json_value);
  std::string GetUserName();
  void ConvertYamlToJson(const YAML::Node& yamlNode, Json::Value& json_value);
};
#endif