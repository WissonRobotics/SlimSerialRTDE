/**
 * Copyright (c) 2023, WissonRobotics
 * File: config_center.cpp
 * Author: Zhiwei Huang (huangzhiwei@wissonrobotics.com)
 * Version 1.0
 * Date: 2023-12-06
 * Brief:
 */
#include "config_center.h"
#include "loguru.hpp"
bool ConfigCenter::InitConfig(std::string car_name) {
  LOG_F(INFO, "[ConfigCenter] start load yaml config file");
  yaml_path_ = "../"+ car_name + "/monos_configs.yaml";
  LOG_F(INFO, "[ConfigCenter] yaml path %s", yaml_path_.c_str());

  try {
    monos_yaml_ = YAML::LoadFile(yaml_path_);
    arm_param_.arm_port = monos_yaml_["Arm"]["port"].as<std::string>();
    arm_param_.arm_baudrate = monos_yaml_["Arm"]["baudrate"].as<unsigned int>();
    arm_param_.joint_dead_zone = monos_yaml_["Arm"]["jointDeadZone"].as<std::vector<float>>();
    // arm_param_.linkH = monos_yaml["Arm"]["linkH"].as<std::vector<float>>();
    arm_param_.speed_level = monos_yaml_["Arm"]["speed_level"].as<std::string>();
    arm_param_.Y_max = monos_yaml_["Arm"]["Y_max"].as<float>();
    arm_param_.yaw_zero_offset_enable = monos_yaml_["Arm"]["yaw_zero_offset_enable"].as<bool>();
    arm_param_.yaw_zero_offset = monos_yaml_["Arm"]["yaw_zero_offset"].as<float>();

    arm_param_.guide_elongate_value = monos_yaml_["Arm"]["guide_elongate_value"].as<int>();
    arm_param_.gun_elongate_value = monos_yaml_["Arm"]["gun_elongate_value"].as<int>();
    arm_param_.cable_contract_value = monos_yaml_["Arm"]["cable_contract_value"].as<int>();
    arm_param_.gripper_shift_central_value = monos_yaml_["Arm"]["gripper_shift_central_value"].as<int>();
    arm_param_.gun_lock_value = monos_yaml_["Arm"]["gun_lock_value"].as<int>();
    arm_param_.gripper_close_value = monos_yaml_["Arm"]["gripper_close_value"].as<int>();

    arm_param_.guide_contract_value = monos_yaml_["Arm"]["guide_contract_value"].as<int>();
    arm_param_.gun_contract_value = monos_yaml_["Arm"]["gun_contract_value"].as<int>();
    arm_param_.cable_elongate_value = monos_yaml_["Arm"]["cable_elongate_value"].as<int>();
    arm_param_.gripper_shift_away_value = monos_yaml_["Arm"]["gripper_shift_away_value"].as<int>();
    arm_param_.gun_unlock_value = monos_yaml_["Arm"]["gun_unlock_value"].as<int>();
    arm_param_.gripper_open_value = monos_yaml_["Arm"]["gripper_open_value"].as<int>();

    arm_param_.door_open_value_send = monos_yaml_["DoorControl"]["door_open_value_send"].as<int>();
    arm_param_.door_open_value_check = monos_yaml_["DoorControl"]["door_open_value_check"].as<int>();
    arm_param_.door_open_timeout = monos_yaml_["DoorControl"]["door_open_timeout"].as<double>();
    arm_param_.door_close_value_send = monos_yaml_["DoorControl"]["door_close_value_send"].as<int>();
    arm_param_.door_close_value_check = monos_yaml_["DoorControl"]["door_close_value_check"].as<int>();
    arm_param_.door_close_timeout = monos_yaml_["DoorControl"]["door_close_timeout"].as<double>();

    compensation_param_.local_detection_yaw = monos_yaml_["Compensation"]["local_detection_yaw"].as<float>();
    // compensation_param_.local_detection_pitch = monos_yaml["Compensation"]["local_detection_pitch"].as<float>();
    compensation_param_.attach_z_axis_adjust_in_while =
        monos_yaml_["Compensation"]["attach_z_axis_adjust_in_while"].as<float>();
    compensation_param_.attach_z_axis_adjust_before_while =
        monos_yaml_["Compensation"]["attach_z_axis_adjust_before_while"].as<float>();
    compensation_param_.attach_x_axis_adjust_before_while =
        monos_yaml_["Compensation"]["attach_x_axis_adjust_before_while"].as<float>();
    compensation_param_.detach_z_axis_adjust_before_plug =
        monos_yaml_["Compensation"]["detach_z_axis_adjust_before_plug"].as<float>();

    compensation_param_.gun_elongate_distance_to_grasp =
        monos_yaml_["Compensation"]["gun_elongate_distance_to_grasp"].as<int>();
    compensation_param_.gun_elongate_distance_to_insert =
        monos_yaml_["Compensation"]["gun_elongate_distance_to_insert"].as<int>();
    // compensation_param_.elongate_pitch = monos_yaml["Compensation"]["elongate_pitch"].as<float>();
    // compensation_param_.elongate_speed = monos_yaml["Compensation"]["elongate_speed"].as<float>();

    compensation_param_.time_start_attach_delay = monos_yaml_["Compensation"]["time_start_attach_delay"].as<int>();
    compensation_param_.time_gun_elongate_delay = monos_yaml_["Compensation"]["time_gun_elongate_delay"].as<int>();
    compensation_param_.time_gun_contract_delay = monos_yaml_["Compensation"]["time_gun_contract_delay"].as<int>();
    compensation_param_.time_gripper_shift_delay = monos_yaml_["Compensation"]["time_gripper_shift_delay"].as<int>();
    compensation_param_.time_guide_elongate_delay = monos_yaml_["Compensation"]["time_guide_elongate_delay"].as<int>();
    compensation_param_.time_gripper_grasp_delay = monos_yaml_["Compensation"]["time_gripper_grasp_delay"].as<int>();
    compensation_param_.time_arm_move_finish_delay =
        monos_yaml_["Compensation"]["time_arm_move_finish_delay"].as<int>();

    compensation_param_.aligned_well_detect_box_result_x0 =
        monos_yaml_["Compensation"]["aligned_well_detect_box_result_x0"].as<int>();
    compensation_param_.aligned_well_detect_box_result_x0_error_threshold =
        monos_yaml_["Compensation"]["aligned_well_detect_box_result_x0_error_threshold"].as<int>();
    compensation_param_.aligned_well_detect_box_result_y1 =
        monos_yaml_["Compensation"]["aligned_well_detect_box_result_y1"].as<int>();
    compensation_param_.aligned_well_detect_box_result_y1_error_threshold =
        monos_yaml_["Compensation"]["aligned_well_detect_box_result_y1_error_threshold"].as<int>();

    gun_param_.alignment_laser_original = monos_yaml_["Alignment"]["alignment_laser_original"].as<int>();
    gun_param_.alignment_well_distance = monos_yaml_["Alignment"]["alignment_well_distance"].as<int>();
    // gun_param_.alignment_well_timeout = monos_yaml["Alignment"]["alignment_well_timeout"].as<int>();
    gun_param_.detached_well_distance = monos_yaml_["Detach"]["detached_well_distance"].as<int>();
    gun_param_.attached_well_distance = monos_yaml_["Attach"]["attached_well_distance"].as<int>();
    gun_param_.auto_detach = monos_yaml_["AutoDetach"].as<bool>();

    position_param_.local_camera_x_init = monos_yaml_["LocalCameraPosition"]["XInit"].as<float>();
    position_param_.local_camera_y_init = monos_yaml_["LocalCameraPosition"]["YInit"].as<float>();
    position_param_.local_camera_z_init = monos_yaml_["LocalCameraPosition"]["ZInit"].as<float>();
    position_param_.local_camera_yaw_init = monos_yaml_["LocalCameraPosition"]["YawInit"].as<float>();
    position_param_.local_camera_roll_init = monos_yaml_["LocalCameraPosition"]["RollInit"].as<float>();
    position_param_.global_camera_x_init = monos_yaml_["GlobalCameraPosition"]["XInit"].as<float>();
    position_param_.global_camera_y_init = monos_yaml_["GlobalCameraPosition"]["YInit"].as<float>();
    position_param_.global_camera_z_init = monos_yaml_["GlobalCameraPosition"]["ZInit"].as<float>();
    position_param_.global_camera_yaw_init = monos_yaml_["GlobalCameraPosition"]["YawInit"].as<float>();
    position_param_.global_camera_roll_init = monos_yaml_["GlobalCameraPosition"]["RollInit"].as<float>();
    position_param_.inlet_yaw_init = monos_yaml_["Inlet"]["YawInit"].as<float>();
    position_param_.arm_ready_position[0] = monos_yaml_["ReadyPosition"]["X"].as<float>();
    position_param_.arm_ready_position[1] = monos_yaml_["ReadyPosition"]["Y"].as<float>();
    position_param_.arm_ready_position[2] = monos_yaml_["ReadyPosition"]["Z"].as<float>();
    position_param_.arm_ready_position[4] = monos_yaml_["ReadyPosition"]["Pitch"].as<float>();

    target_strategy_param_.global_strategy_x_offset_min =
        monos_yaml_["GlobalStrategyForTargetPose"]["XOffset_min"].as<float>();
    target_strategy_param_.global_strategy_x_offset = monos_yaml_["GlobalStrategyForTargetPose"]["XOffset"].as<float>();
    target_strategy_param_.global_strategy_y_offset = monos_yaml_["GlobalStrategyForTargetPose"]["YOffset"].as<float>();
    target_strategy_param_.global_strategy_z_offset = monos_yaml_["GlobalStrategyForTargetPose"]["ZOffset"].as<float>();
    target_strategy_param_.global_strategy_pitch = monos_yaml_["GlobalStrategyForTargetPose"]["Pitch"].as<float>();

    target_strategy_param_.local_strategy_x_offset_stage1 =
        monos_yaml_["LocalStrategyForTargetPoseStage1"]["XOffset"].as<float>();
    target_strategy_param_.local_strategy_y_offset_stage1 =
        monos_yaml_["LocalStrategyForTargetPoseStage1"]["YOffset"].as<float>();
    target_strategy_param_.local_strategy_z_offset_stage1 =
        monos_yaml_["LocalStrategyForTargetPoseStage1"]["ZOffset"].as<float>();
    target_strategy_param_.local_strategy_pitch_stage1 =
        monos_yaml_["LocalStrategyForTargetPoseStage1"]["Pitch"].as<float>();

    target_strategy_param_.local_strategy_x_offset_stage2 =
        monos_yaml_["LocalStrategyForTargetPoseStage2"]["XOffset"].as<float>();
    target_strategy_param_.local_strategy_y_offset_stage2 =
        monos_yaml_["LocalStrategyForTargetPoseStage2"]["YOffset"].as<float>();
    target_strategy_param_.local_strategy_z_offset_stage2 =
        monos_yaml_["LocalStrategyForTargetPoseStage2"]["ZOffset"].as<float>();
    target_strategy_param_.local_strategy_pitch_stage2 =
        monos_yaml_["LocalStrategyForTargetPoseStage2"]["Pitch"].as<float>();

    target_strategy_param_.local_in_arm_strategy_x_offset =
        monos_yaml_["LocalStrategyForTargetPose_inArm"]["XOffset"].as<float>();
    target_strategy_param_.local_in_arm_strategy_y_offset =
        monos_yaml_["LocalStrategyForTargetPose_inArm"]["YOffset"].as<float>();
    target_strategy_param_.local_in_arm_strategy_z_offset =
        monos_yaml_["LocalStrategyForTargetPose_inArm"]["ZOffset"].as<float>();

    target_strategy_param_.is_multiple_local_localization =
        monos_yaml_["LocalStrategy"]["MultipleLocalLocalization"].as<bool>();

    visual_param_.camera_global_resolution = monos_yaml_["Camera"]["global_camera_resolution"].as<std::vector<float>>();
    visual_param_.camera_global_clipping_distance_in_meters =
        monos_yaml_["Camera"]["global_clipping_distance_in_meters"].as<float>();
    visual_param_.camera_local_resolution = monos_yaml_["Camera"]["local_camera_resolution"].as<std::vector<float>>();
    visual_param_.camera_data_save_dir = monos_yaml_["Camera"]["save_dir"].as<std::string>();
    visual_param_.camera_data_save_count = monos_yaml_["Camera"]["save_count"].as<int>();

    // visual_param_.model_device = monos_yaml["Model"]["device"].as<std::string>();
    // visual_param_.model_cpu_threads = monos_yaml["Model"]["cpu_threads"].as<int>();
    visual_param_.model_dir = monos_yaml_["Model"]["model_dir"].as<std::string>();
    visual_param_.model_detection_threshold_high = monos_yaml_["Model"]["detection_threshold_high"].as<float>();
    visual_param_.model_detection_threshold_low = monos_yaml_["Model"]["detection_threshold_low"].as<float>();
    // visual_param_.model_local_flag = monos_yaml["Model"]["local_model_flag"].as<std::string>();
    // visual_param_.model_local_dir = monos_yaml["Model"]["local_model_dir"].as<std::string>();
    // visual_param_.model_local_detection_threshold = monos_yaml["Model"]["local_detection_threshold"].as<float>();
    visual_param_.model_local_bbox_size = monos_yaml_["Model"]["local_bbox_size"].as<std::vector<float>>();

    //          visual_param_.point_cloud_original_size =
    //          monos_yaml["PointCloud"]["original_size"].as<std::vector<float>>();
    visual_param_.point_cloud_global_clipping_z_range_m =
        monos_yaml_["PointCloud"]["global_clipping_z_range_m"].as<std::vector<float>>();
    visual_param_.point_cloud_local_clipping_z_range_m =
        monos_yaml_["PointCloud"]["local_clipping_z_range_m"].as<std::vector<float>>();
    visual_param_.point_cloud_global_downsample_voxel_size =
        monos_yaml_["PointCloud"]["global_downsample_voxel_size"].as<float>();
    visual_param_.point_cloud_local_downsample_voxel_size =
        monos_yaml_["PointCloud"]["local_downsample_voxel_size"].as<float>();
    visual_param_.point_cloud_cluster_eps = monos_yaml_["PointCloud"]["cluster_eps"].as<float>();
    visual_param_.point_cloud_cluster_min_points = monos_yaml_["PointCloud"]["cluster_min_points"].as<int>();
    visual_param_.point_cloud_statistical_outlier_removal_nb_neighbors =
        monos_yaml_["PointCloud"]["statistical_outlier_removal_nb_neighbors"].as<int>();
    visual_param_.point_cloud_statistical_outlier_removal_std_ratio =
        monos_yaml_["PointCloud"]["statistical_outlier_removal_std_ratio"].as<float>();
    visual_param_.point_cloud_radius_outlier_removal_nb_points =
        monos_yaml_["PointCloud"]["radius_outlier_removal_nb_points"].as<int>();
    visual_param_.point_cloud_radius_outlier_removal_radius =
        monos_yaml_["PointCloud"]["radius_outlier_removal_radius"].as<float>();
    visual_param_.point_cloud_calculate_points = monos_yaml_["PointCloud"]["calculate_points"].as<int>();
    visual_param_.point_cloud_bbox_corner_crop_type =
        monos_yaml_["PointCloud"]["bbox_corner_crop_type"].as<std::string>();

    // task_param_.globalLocalizationRepeatMax = monos_yaml_["Task"]["globalLocalizationRepeatMax"].as<int>();
    // task_param_.globalLocalizationTimeout = monos_yaml_["Task"]["globalLocalizationTimeout"].as<double>();
    // task_param_.localLocalizationRepeatMax = monos_yaml_["Task"]["localLocalizationRepeatMax"].as<int>();
    // task_param_.localLocalizationTimeout = monos_yaml_["Task"]["localLocalizationTimeout"].as<double>();
    // task_param_.gunRepeatMax = monos_yaml_["Task"]["gunRepeatMax"].as<int>();
    // task_param_.gunTimeout = monos_yaml_["Task"]["gunTimeout"].as<double>();

    task_param_.global_cam_detection_timeout = monos_yaml_["Task"]["global_cam_detection_timeout"].as<double>();
    task_param_.global_planning_times_limit = monos_yaml_["Task"]["global_planning_times_limit"].as<int>();
    task_param_.global_repeat_times_limit = monos_yaml_["Task"]["global_repeat_times_limit"].as<int>();

    task_param_.local_cam_detection_timeout = monos_yaml_["Task"]["local_cam_detection_timeout"].as<double>();
    task_param_.local_planning_times_limit = monos_yaml_["Task"]["local_planning_times_limit"].as<int>();
    task_param_.local_repeat_times_limit = monos_yaml_["Task"]["local_repeat_times_limit"].as<int>();
    task_param_.local_adjust_times_limit = monos_yaml_["Task"]["local_adjust_times_limit"].as<int>();

    task_param_.attach_gun_insert_timeout = monos_yaml_["Task"]["attach_gun_insert_timeout"].as<double>();
    task_param_.attach_guide_aligned_timeout = monos_yaml_["Task"]["attach_guide_aligned_timeout"].as<double>();
    task_param_.attach_cam_unplug_assessing_timeout =
        monos_yaml_["Task"]["attach_cam_unplug_assessing_timeout"].as<double>();

    task_param_.attach_failed_times_limit = monos_yaml_["Task"]["attach_failed_times_limit"].as<int>();

    task_param_.detach_gun_contract_timeout = monos_yaml_["Task"]["detach_gun_contract_timeout"].as<double>();

    // LOG_F(INFO,"[ConfigCenter] global_cam_detection_timeout {} ", task_param_.global_cam_detection_timeout);
    // LOG_F(INFO,"[ConfigCenter] global_planning_times_limit {} ", task_param_.global_planning_times_limit);
    // LOG_F(INFO,"[ConfigCenter] global_repeat_times_limit {} ", task_param_.global_repeat_times_limit);
    // LOG_F(INFO,"[ConfigCenter] local_cam_detection_timeout {} ", task_param_.local_cam_detection_timeout);
    // LOG_F(INFO,"[ConfigCenter] local_planning_times_limit {} ", task_param_.local_planning_times_limit);
    // LOG_F(INFO,"[ConfigCenter] local_repeat_times_limit {} ", task_param_.local_repeat_times_limit);
    // LOG_F(INFO,"[ConfigCenter] local_adjust_times_limit {} ", task_param_.local_adjust_times_limit);

    // LOG_F(INFO,"[ConfigCenter] attach_gun_insert_timeout {} ", task_param_.attach_gun_insert_timeout);
    // LOG_F(INFO,"[ConfigCenter] attach_guide_aligned_timeout {} ", task_param_.attach_guide_aligned_timeout);
    // LOG_F(INFO,"[ConfigCenter] attach_cam_unplug_assessing_timeout {} ",
    // task_param_.attach_cam_unplug_assessing_timeout); LOG_F(INFO,"[ConfigCenter] attach_failed_times_limit {} ",
    // task_param_.attach_failed_times_limit); LOG_F(INFO,"[ConfigCenter] detach_gun_contract_timeout {} ",
    // task_param_.detach_gun_contract_timeout);

    task_param_.car_name = monos_yaml_["CarName"].as<std::string>();
    LOG_F(INFO, "[ConfigCenter] load yaml config file ok");
    LOG_F(INFO, "[ConfigCenter] load yaml config file car is %s", task_param_.car_name.c_str());
    ConvertYamlToJson(monos_yaml_, config_json_value_);
    config_json_value_;
  } catch (std::exception& err) {
    LOG_F(ERROR,"[ConfigCenter] catch an exception: %s", err.what());
    return false;
  }

  return true;
}

std::string ConfigCenter::GetUserName() {
  uid_t userid;
  struct passwd* pwd;
  userid = getuid();
  pwd = getpwuid(userid);
  return pwd->pw_name;
}

void ConfigCenter::ConvertJsonToYaml(const Json::Value& jsonValue, YAML::Node& yamlNode) {
  if (jsonValue.isString()) {
    yamlNode = jsonValue.asString();
  } else if (jsonValue.isObject()) {
    for (const auto& key : jsonValue.getMemberNames()) {
      YAML::Node subYamlNode;
      ConvertJsonToYaml(jsonValue[key], subYamlNode);
      yamlNode[key] = subYamlNode;
    }
  } else if (jsonValue.isArray()) {
    for (const auto& element : jsonValue) {
      YAML::Node subYamlNode;
      ConvertJsonToYaml(element, subYamlNode);
      yamlNode.push_back(subYamlNode);
    }
  }
}
void ConfigCenter::SaveConfigJsonValue(Json::Value& json_value) {
  // YAML::Node yamlNode;

  ConvertJsonToYaml(json_value, monos_yaml_);
  // std::cout<<" Arm port "<<yamlNode["Arm"]["port"].as<std::string>()<<std::endl;;?
  std::ofstream outputFile("/home/" + GetUserName() + "/monos2/config/monos_configs.yaml");
  outputFile << monos_yaml_;
  outputFile.close();

  //        std::cout << "JSON node converted to YAML and saved as monos_configs.yaml"
  //                  << std::endl;
  ConvertYamlToJson(monos_yaml_, config_json_value_);
}

void ConfigCenter::ConvertYamlToJson(const YAML::Node& yamlNode, Json::Value& json_value) {
  if (yamlNode.IsScalar()) {
    json_value = yamlNode.as<std::string>();
  } else if (yamlNode.IsMap()) {
    for (const auto& entry : yamlNode) {
      Json::Value subJsonValue;
      ConvertYamlToJson(entry.second, subJsonValue);
      json_value[entry.first.as<std::string>()] = subJsonValue;
    }
  } else if (yamlNode.IsSequence()) {
    for (const auto& element : yamlNode) {
      Json::Value subJsonValue;
      ConvertYamlToJson(element, subJsonValue);
      json_value.append(subJsonValue);
    }
  }
}