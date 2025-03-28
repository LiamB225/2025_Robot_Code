// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace OperatorConstants {

inline constexpr int kDriverControllerPort = 0;
inline constexpr int kSecondaryControllerPort = 1;

constexpr int k_fl_drive_id = 1;
constexpr int k_fl_rot_id = 2;
constexpr int k_fr_drive_id = 3;
constexpr int k_fr_rot_id = 4;
constexpr int k_bl_drive_id = 5;
constexpr int k_bl_rot_id = 6;
constexpr int k_br_drive_id = 7;
constexpr int k_br_rot_id = 8;
constexpr int k_fl_rot_encoder = 9;
constexpr int k_fr_rot_encoder = 10;
constexpr int k_bl_rot_encoder = 11;
constexpr int k_br_rot_encoder = 12;
constexpr int k_elevator_id = 13;
constexpr int k_coral_id = 14;
constexpr int k_climber_id = 15;
constexpr int k_sensor_id = 0;
constexpr int k_barrier_id = 1;
}  // namespace OperatorConstants
