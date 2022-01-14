// Copyright (c) 2020 New Eagle, All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// * Neither the name of the {copyright_holder} nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/** \brief This file defines the RaptorDbwCAN class.
 * \copyright Copyright 2021 New Eagle LLC
 * \file raptor_dbw_can.hpp
 */

#ifndef RAPTOR_DBW_CAN__RAPTOR_DBW_CAN_HPP_
#define RAPTOR_DBW_CAN__RAPTOR_DBW_CAN_HPP_

#include <cmath>
#include <array>
#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

// ROS messages
#include "can_msgs/msg/frame.hpp"
#include "raptor_dbw_msgs/msg/accelerator_pedal_cmd.hpp"
#include "raptor_dbw_msgs/msg/accelerator_pedal_report.hpp"
#include "raptor_dbw_msgs/msg/brake_cmd.hpp"
#include "raptor_dbw_msgs/msg/brake_report.hpp"
#include "raptor_dbw_msgs/msg/gear_cmd.hpp"
#include "raptor_dbw_msgs/msg/gear_report.hpp"
#include "raptor_dbw_msgs/msg/global_enable_cmd.hpp"
#include "raptor_dbw_msgs/msg/misc_cmd.hpp"
#include "raptor_dbw_msgs/msg/misc_report.hpp"
#include "raptor_dbw_msgs/msg/other_actuators_report.hpp"
#include "raptor_dbw_msgs/msg/steering_cmd.hpp"
#include "raptor_dbw_msgs/msg/steering_report.hpp"
#include "raptor_dbw_msgs/msg/wheel_speed_report.hpp"
#include "iac_msgs/msg/vehicle_status.hpp"
#include "iac_msgs/msg/misc_report.hpp"
#include "iac_msgs/msg/vehicle_command.hpp"
#include "iac_msgs/msg/engine_report.hpp"
#include "iac_msgs/msg/engine_pressures_report.hpp"
#include "iac_msgs/msg/tire_report.hpp"
#include "iac_msgs/msg/steering_extended_report.hpp"
#include "iac_msgs/msg/vehicle_timing.hpp"
#include "iac_msgs/msg/fault_report.hpp"
#include "autoware_auto_msgs/msg/vehicle_kinematic_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

#include "can_dbc_parser/DbcMessage.hpp"
#include "can_dbc_parser/DbcSignal.hpp"
#include "can_dbc_parser/Dbc.hpp"
#include "can_dbc_parser/DbcBuilder.hpp"

#include "raptor_dbw_can/dispatch.hpp"

using can_msgs::msg::Frame;
using NewEagle::DbcMessage;

using sensor_msgs::msg::JointState;
using std_msgs::msg::Bool;
using std_msgs::msg::Empty;
using std_msgs::msg::String;

using raptor_dbw_msgs::msg::AcceleratorPedalCmd;
using raptor_dbw_msgs::msg::AcceleratorPedalReport;
using raptor_dbw_msgs::msg::ActuatorControlMode;
using raptor_dbw_msgs::msg::BrakeCmd;
using raptor_dbw_msgs::msg::BrakeReport;
using raptor_dbw_msgs::msg::Gear;
using raptor_dbw_msgs::msg::GearCmd;
using raptor_dbw_msgs::msg::GearReport;
using raptor_dbw_msgs::msg::GlobalEnableCmd;
using raptor_dbw_msgs::msg::MiscCmd;
using raptor_dbw_msgs::msg::MiscReport;
using raptor_dbw_msgs::msg::OtherActuatorsReport;
using raptor_dbw_msgs::msg::SteeringCmd;
using raptor_dbw_msgs::msg::SteeringReport;
using raptor_dbw_msgs::msg::WheelSpeedReport;
using iac_msgs::msg::VehicleStatus;
using iac_msgs::msg::VehicleCommand;
using iac_msgs::msg::VehicleTiming;
using iac_msgs::msg::EngineReport;
using iac_msgs::msg::EnginePressuresReport;
using iac_msgs::msg::TireReport;
using iac_msgs::msg::SteeringExtendedReport;
using iac_msgs::msg::FaultReport;
using DoMiscReport = iac_msgs::msg::MiscReport;

using autoware_auto_msgs::msg::VehicleKinematicState;

namespace raptor_dbw_can
{
class RaptorDbwCAN : public rclcpp::Node
{
public:
/** \brief Default constructor.
 * \param[in] options The options for this node.
 */
  explicit RaptorDbwCAN(const rclcpp::NodeOptions & options);

private:
  void step();

/** \brief Attempt to enable the DBW system.
 * \param[in] msg Enable message (must not be null)
 */
  void recvEnable(const Empty::SharedPtr msg);

/** \brief Attempt to disable the DBW system.
 * \param[in] msg Disable message (must not be null)
 */
  void recvDisable(const Empty::SharedPtr msg);

/** \brief Convert reports received over CAN into ROS messages.
 * \param[in] msg The message received over CAN.
 */
  void recvCAN(const Frame::SharedPtr msg);

  void recvAcceleratorRpt(const Frame::SharedPtr msg, DbcMessage * message);

  void recvBaseToCarSummaryRpt(const Frame::SharedPtr msg, DbcMessage * message);

  void recvBaseToCarTimingRpt(const Frame::SharedPtr msg, DbcMessage * message);

  void recvBrakePressureRpt(const Frame::SharedPtr msg, DbcMessage * message);

  void recvDiagnosticRpt(const Frame::SharedPtr msg, DbcMessage * message);

  void recvMiscRpt(const Frame::SharedPtr msg, DbcMessage * message);

  void recvPt1Rpt(const Frame::SharedPtr msg, DbcMessage * message);

  void recvPt2Rpt(const Frame::SharedPtr msg, DbcMessage * message);

  void recvPt3Rpt(const Frame::SharedPtr msg, DbcMessage * message);

  void recvRestOfFieldRpt(const Frame::SharedPtr msg, DbcMessage * message);

  void recvSteeringRpt(const Frame::SharedPtr msg, DbcMessage * message);

  void recvSteeringExtendedRpt(const Frame::SharedPtr msg, DbcMessage * message);

  void recvTirePressureFlRpt(const Frame::SharedPtr msg, DbcMessage * message);

  void recvTirePressureFrRpt(const Frame::SharedPtr msg, DbcMessage * message);

  void recvTirePressureRlRpt(const Frame::SharedPtr msg, DbcMessage * message);

  void recvTirePressureRrRpt(const Frame::SharedPtr msg, DbcMessage * message);

  void recvTireTempFL01Rpt(const Frame::SharedPtr msg, DbcMessage * message);

  void recvTireTempFL02Rpt(const Frame::SharedPtr msg, DbcMessage * message);

  void recvTireTempFL03Rpt(const Frame::SharedPtr msg, DbcMessage * message);

  void recvTireTempFL04Rpt(const Frame::SharedPtr msg, DbcMessage * message);

  void recvTireTempFR01Rpt(const Frame::SharedPtr msg, DbcMessage * message);

  void recvTireTempFR02Rpt(const Frame::SharedPtr msg, DbcMessage * message);

  void recvTireTempFR03Rpt(const Frame::SharedPtr msg, DbcMessage * message);

  void recvTireTempFR04Rpt(const Frame::SharedPtr msg, DbcMessage * message);

  void recvTireTempRL01Rpt(const Frame::SharedPtr msg, DbcMessage * message);

  void recvTireTempRL02Rpt(const Frame::SharedPtr msg, DbcMessage * message);

  void recvTireTempRL03Rpt(const Frame::SharedPtr msg, DbcMessage * message);

  void recvTireTempRL04Rpt(const Frame::SharedPtr msg, DbcMessage * message);

  void recvTireTempRR01Rpt(const Frame::SharedPtr msg, DbcMessage * message);

  void recvTireTempRR02Rpt(const Frame::SharedPtr msg, DbcMessage * message);

  void recvTireTempRR03Rpt(const Frame::SharedPtr msg, DbcMessage * message);

  void recvTireTempRR04Rpt(const Frame::SharedPtr msg, DbcMessage * message);

  void recvWheelPotentiometerDataRpt(const Frame::SharedPtr msg, DbcMessage * message);

  void recvWheelSpeedRpt(const Frame::SharedPtr msg, DbcMessage * message);

  void recvWheelStrainGaugeRpt(const Frame::SharedPtr msg, DbcMessage * message);

/** \brief Convert an Accelerator Pedal Command sent as a ROS message into a CAN message.
 * \param[in] msg The message to send over CAN.
 */
  void recvAcceleratorPedalCmd(const AcceleratorPedalCmd::SharedPtr msg);

/** \brief Convert a Brake Command sent as a ROS message into a CAN message.
 * \param[in] msg The message to send over CAN.
 */
  void recvBrakeCmd(const BrakeCmd::SharedPtr msg);

/** \brief Convert a Ct Report sent as a ROS message into a CAN message.
 * \param[in] msg The message to send over CAN.
 */
  void recvCtRpt(const VehicleStatus::SharedPtr msg);

/** \brief Convert a Gear Command sent as a ROS message into a CAN message.
 * \param[in] msg The message to send over CAN.
 */
  void recvGearCmd(const GearCmd::SharedPtr msg);

/** \brief Convert a Global Enable Command sent as a ROS message into a CAN message.
 * \param[in] msg The message to send over CAN.
 */
  void recvGlobalEnableCmd(const GlobalEnableCmd::SharedPtr msg);

/** \brief Convert a Misc. Command sent as a ROS message into a CAN message.
 * \param[in] msg The message to send over CAN.
 */
  void recvMiscCmd(const MiscCmd::SharedPtr msg);

/** \brief Convert a Steering Command sent as a ROS message into a CAN message.
 * \param[in] msg The message to send over CAN.
 */
  void recvSteeringCmd(const SteeringCmd::SharedPtr msg);

  /** \brief Enumeration of system faults */
  enum Fault : uint8_t
  {
    OOS_FRONT_BRK,
    OOS_REAR_BRK,
    SD_EBRAKE,
    STEER_WARN,
    SD_SYSTEM_FAILURE,
    SD_COMMS_LOSS,
    MOTEC_COMMS_LOSS,
    HB_STATUS,
    NUM_SERIOUS_FAULTS,   /**< Total number of serious faults (disables DBW) */
    BRAKE_WARN = NUM_SERIOUS_FAULTS,
    LOW_ENG_SPEED,
    SD_SYSTEM_WARN,
    MOTEC_WARN,
    NUM_FAULTS            /**< Total number of system faults */
  };

  /** \brief Enumeration of system enables */
  enum Enable : uint8_t
  {
    EN_ACCEL = 0,   /**< Acceleration pedal system enabled */
    EN_BRAKE,       /**< Brake system enabled */
    EN_STEER,       /**< Steering system enabled */
    EN_DBW,         /**< DBW system enabled */
    EN_DBW_PREV,    /**< DBW system previously enabled (track edge) */
    NUM_ENABLES     /**< Total number of system enables */
  };

  const std::array<std::string, NUM_FAULTS> FAULT_SYSTEM {
    "est1_oos_front_brk",
    "est2_oos_rear_brk",
    "est6_sd_ebrake",
    "sd_steer_warning",
    "sd_system_failure",
    "est4_sd_comms_loss",
    "est5_motec_comms_loss",
    "adlink_hb_lost or rc_lost",
    "sd_brake_warning",
    "est3_low_eng_speed",
    "sd_system_warning",
    "motec_warning"
  };

/** \brief Check for an active fault.
 * \returns TRUE if there is any active fault, FALSE otherwise
 */
  inline bool fault()
  {
    for (size_t ii = 0; ii < NUM_FAULTS; ++ii) {
      if (faults_.at(ii)) {
        return true;
      }
    }
    return false;
  }

/** \brief Check for an active driver override.
 * \returns TRUE if DBW is enabled, FALSE otherwise
 */
  inline bool clear() {return enables_[EN_DBW];}

/** \brief Check whether the DBW Node is in control of the vehicle.
 * \returns TRUE if DBW is enabled && there are no active faults, FALSE otherwise
 */
  inline bool enabled() {return enables_[EN_DBW] && !fault();}

/** \brief DBW Enabled needs to publish when its state changes.
 * \returns TRUE when DBW enable state changes, FALSE otherwise
 */
  bool publishDbwEnabled();

/** \brief Checks faults & overrides to establish DBW control */
  void enableSystem();

/** \brief Disables DBW control */
  void disableSystem();

  /** \brief Set the specified fault; these faults disable DBW control when active
   * \param[in] which_fault Which fault to set
   * \param[in] fault The value to set the fault to
   */
  void setFault(Fault which_fault, bool fault);

  /** \brief Enumeration of vehicle joints */
  enum ListJoints
  {
    JOINT_FL = 0,   /**< Front left wheel */
    JOINT_FR,       /**< Front right wheel */
    JOINT_RL,       /**< Rear left wheel */
    JOINT_RR,       /**< Rear right wheel */
    JOINT_SL,       /**< Steering left */
    JOINT_SR,       /**< Steering right */
    JOINT_COUNT,    /**< Number of joints */
  };

/** \brief Calculates & publishes joint states based on updated steering report.
 *    Overloaded function.
 * \param[in] stamp Updated time stamp
 * \param[in] steering Updated steering report
 */
  void publishJointStates(
    const rclcpp::Time stamp,
    const SteeringReport steering);

/** \brief Calculates & publishes joint states based on updated wheel speed report.
 *    Overloaded function.
 * \param[in] stamp Updated time stamp
 * \param[in] wheels Updated wheel speed report
 */
  void publishJointStates(
    const rclcpp::Time stamp,
    const WheelSpeedReport wheels);

  std::array<bool, NUM_ENABLES> enables_;
  std::array<bool, NUM_FAULTS> faults_;

  std::uint8_t vehicle_number_;

  // Parameters from launch
  std::string dbw_dbc_file_;
  float max_steer_angle_;
  bool publish_my_laps_;

  // Ackermann steering
  double acker_wheelbase_;
  double acker_track_;
  double steering_ratio_;

  JointState joint_state_;

  // Subscribers (from NE Raptor Interface)
  rclcpp::Subscription<AcceleratorPedalCmd>::SharedPtr sub_accel_cmd_;
  rclcpp::Subscription<BrakeCmd>::SharedPtr sub_brake_cmd_;
  rclcpp::Subscription<GearCmd>::SharedPtr sub_gear_cmd_;
  rclcpp::Subscription<GlobalEnableCmd>::SharedPtr sub_gl_en_cmd_;
  rclcpp::Subscription<MiscCmd>::SharedPtr sub_misc_cmd_;
  rclcpp::Subscription<SteeringCmd>::SharedPtr sub_steering_cmd_;
  rclcpp::Subscription<Empty>::SharedPtr sub_dbw_enable_cmd_;
  rclcpp::Subscription<Empty>::SharedPtr sub_dbw_disable_cmd_;

  // Subscribers (from other)
  rclcpp::Subscription<Frame>::SharedPtr sub_can_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr sub_ct_;

  // Publishers (to NE Raptor Interface)
  rclcpp::Publisher<BrakeReport>::SharedPtr pub_brake_;
  rclcpp::Publisher<GearReport>::SharedPtr pub_gear_;
  rclcpp::Publisher<MiscReport>::SharedPtr pub_misc_;
  rclcpp::Publisher<OtherActuatorsReport>::SharedPtr pub_other_actuators_;
  rclcpp::Publisher<SteeringReport>::SharedPtr pub_steering_;
  rclcpp::Publisher<WheelSpeedReport>::SharedPtr pub_wheel_speed_;

  // Publishers (to other)
  rclcpp::Publisher<Frame>::SharedPtr pub_can_;
  rclcpp::Publisher<Bool>::SharedPtr pub_sys_enable_;
  rclcpp::Publisher<AcceleratorPedalReport>::SharedPtr pub_accel_pedal_;
  rclcpp::Publisher<JointState>::SharedPtr pub_joint_states_;
  rclcpp::Publisher<SteeringExtendedReport>::SharedPtr pub_steering_extended_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr pub_rc_to_ct_;
  rclcpp::Publisher<TireReport>::SharedPtr pub_tire_;
  rclcpp::Publisher<EngineReport>::SharedPtr pub_engine_;
  rclcpp::Publisher<EnginePressuresReport>::SharedPtr pub_engine_pressures_;
  rclcpp::Publisher<DoMiscReport>::SharedPtr pub_do_misc_;
  rclcpp::Publisher<VehicleTiming>::SharedPtr pub_vehicle_timing_;
  rclcpp::Publisher<FaultReport>::SharedPtr pub_fault_report_;

  NewEagle::Dbc dbw_dbc_;

  double dt_{};
  rclcpp::TimerBase::SharedPtr step_timer_;

  TireReport tire_report_msg_ {};
  EnginePressuresReport engine_pressures_report_ {};
  double vehicle_speed_kmph_ {};
  bool is_engine_on_ {};
};

}  // namespace raptor_dbw_can

#endif  // RAPTOR_DBW_CAN__RAPTOR_DBW_CAN_HPP_
