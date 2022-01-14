// Copyright (c) 2015-2018, Dataspeed Inc., 2018-2020 New Eagle, All rights reserved.
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

#include <cmath>
#include <algorithm>
#include <string>

#include "iac_common/race_control.hpp"
#include "iac_common/pubsub.hpp"

#include "raptor_dbw_can/raptor_dbw_can.hpp"

using std::chrono::duration;

namespace raptor_dbw_can
{

static constexpr uint64_t MS_IN_SEC = 1000;

RaptorDbwCAN::RaptorDbwCAN(const rclcpp::NodeOptions & options)
: Node("raptor_dbw_can_node", options)
{
  // Initialize enable state machine

  for (size_t ii = 0; ii < NUM_ENABLES; ii++) {
    enables_[ii] = (ii == EN_DBW_PREV) ? true : false;
  }

  for (size_t ii = 0; ii < NUM_FAULTS; ii++) {
    faults_[ii] = false;
  }

  vehicle_number_ = declare_parameter<uint8_t>("vehicle_number", 0);

  dbw_dbc_file_ = declare_parameter<std::string>("dbw_dbc_file", "");
  max_steer_angle_ = declare_parameter<double>("max_steer_angle", 0.0);

  // Ackermann steering parameters

  acker_wheelbase_ = declare_parameter<double>("ackermann_wheelbase", 2.8498);  // 112.2 in
  acker_track_ = declare_parameter<double>("ackermann_track", 1.5824);  // 62.3 in
  steering_ratio_ = declare_parameter<double>("steering_ratio", 14.8);

  publish_my_laps_ = declare_parameter<bool>("publish_my_laps", true);

  // Initialize joint states

  joint_state_.position.resize(JOINT_COUNT);
  joint_state_.velocity.resize(JOINT_COUNT);
  joint_state_.effort.resize(JOINT_COUNT);
  joint_state_.name.resize(JOINT_COUNT);
  joint_state_.name[JOINT_FL] = "wheel_fl";  // Front Left
  joint_state_.name[JOINT_FR] = "wheel_fr";  // Front Right
  joint_state_.name[JOINT_RL] = "wheel_rl";  // Rear Left
  joint_state_.name[JOINT_RR] = "wheel_rr";  // Rear Right
  joint_state_.name[JOINT_SL] = "steer_fl";
  joint_state_.name[JOINT_SR] = "steer_fr";

  // Publishers (to NE Raptor Interface)
  pubsub::publish_to(this, pub_brake_, "brake_report");
  pubsub::publish_to(this, pub_gear_, "gear_report");
  pubsub::publish_to(this, pub_misc_, "misc_report");
  pubsub::publish_to(this, pub_other_actuators_, "other_actuators_report");
  pubsub::publish_to(this, pub_steering_, "steering_report");
  pubsub::publish_to(this, pub_wheel_speed_, "wheel_speed_report");

  // Publishers (to other)
  pubsub::publish_to(this, pub_can_, "can_rx", rclcpp::QoS{20});
  pubsub::publish_to(this, pub_sys_enable_, "dbw_enabled");
  pubsub::publish_to(this, pub_accel_pedal_, "accelerator_pedal_report");
  pubsub::publish_to(this, pub_joint_states_, "joint_states");
  pubsub::publish_to(this, pub_steering_extended_, "steering_extended_report");
  pubsub::publish_to(this, pub_rc_to_ct_, "rc_to_ct");
  pubsub::publish_to(this, pub_tire_, "tire_report");
  pubsub::publish_to(this, pub_engine_, "engine_report");
  pubsub::publish_to(this, pub_engine_pressures_, "engine_pressures_report");
  pubsub::publish_to(this, pub_do_misc_, "misc_report_do");
  pubsub::publish_to(this, pub_vehicle_timing_, "vehicle_timing");
  pubsub::publish_to(this, pub_fault_report_, "fault_report");

  publishDbwEnabled();

  // Subscribers (from NE Raptor Interface)
  pubsub::subscribe_from(
    this,
    sub_accel_cmd_, "accelerator_pedal_cmd",
    &RaptorDbwCAN::recvAcceleratorPedalCmd);
  pubsub::subscribe_from(this, sub_brake_cmd_, "brake_cmd", &RaptorDbwCAN::recvBrakeCmd);
  pubsub::subscribe_from(this, sub_gear_cmd_, "gear_cmd", &RaptorDbwCAN::recvGearCmd);
  pubsub::subscribe_from(
    this, sub_gl_en_cmd_, "global_enable_cmd",
    &RaptorDbwCAN::recvGlobalEnableCmd);
  pubsub::subscribe_from(this, sub_misc_cmd_, "misc_cmd", &RaptorDbwCAN::recvMiscCmd);
  pubsub::subscribe_from(this, sub_steering_cmd_, "steering_cmd", &RaptorDbwCAN::recvSteeringCmd);
  pubsub::subscribe_from(this, sub_dbw_enable_cmd_, "enable", &RaptorDbwCAN::recvEnable);
  pubsub::subscribe_from(this, sub_dbw_disable_cmd_, "disable", &RaptorDbwCAN::recvDisable);

  // Subscribers (from other)
  pubsub::subscribe_from(this, sub_can_, "can_tx", &RaptorDbwCAN::recvCAN, rclcpp::QoS{500});
  pubsub::subscribe_from(this, sub_ct_, "ct_report", &RaptorDbwCAN::recvCtRpt);

  dbw_dbc_ = NewEagle::DbcBuilder().NewDbc(dbw_dbc_file_);

  // Timers
  dt_ = declare_parameter("dt", 0.01);
  step_timer_ = rclcpp::create_timer(this, get_clock(), duration<float>(dt_), [this] {step();});
}

void RaptorDbwCAN::step()
{
  tire_report_msg_.stamp = now();
  pub_tire_->publish(tire_report_msg_);

  engine_pressures_report_.stamp = now();
  pub_engine_pressures_->publish(engine_pressures_report_);
}

void RaptorDbwCAN::recvEnable(const Empty::SharedPtr msg)
{
  if (msg != NULL) {
    enableSystem();
  }
}

void RaptorDbwCAN::recvDisable(const Empty::SharedPtr msg)
{
  if (msg != NULL) {
    disableSystem();
  }
}

#define RECV_DBC(handler) \
  message = dbw_dbc_.GetMessageById(id); \
  if (msg->dlc >= message->GetDlc()) {message->SetFrame(msg); handler(msg, message);}

void RaptorDbwCAN::recvCAN(const Frame::SharedPtr msg)
{
  NewEagle::DbcMessage * message = nullptr;
  if (!msg->is_rtr && !msg->is_error) {
    auto id = msg->id;
    switch (id) {
      case ID_ACCELERATOR_CMD:
        // Command from CT -> Raptor.
        break;

      case ID_ACCELERATOR_REPORT:
        RECV_DBC(recvAcceleratorRpt);
        break;

      case ID_BASE_TO_CAR_SUMMARY:
        RECV_DBC(recvBaseToCarSummaryRpt);
        break;

      case ID_BASE_TO_CAR_TIMING:
        RECV_DBC(recvBaseToCarTimingRpt);
        break;

      case ID_BRAKE_PRESSURE_CMD:
        // Command from CT -> Raptor.
        break;

      case ID_BRAKE_PRESSURE_REPORT:
        RECV_DBC(recvBrakePressureRpt);
        break;

      case ID_CT_REPORT:
        // Command from CT -> Raptor.
        break;

      case ID_DIAGNOSTIC_REPORT:
        RECV_DBC(recvDiagnosticRpt);
        break;

      case ID_GEAR_SHIFT_CMD:
        // Command from CT -> Raptor.
        break;

      case ID_MISC_REPORT:
        RECV_DBC(recvMiscRpt);
        break;

      case ID_PT_REPORT_1:
        RECV_DBC(recvPt1Rpt);
        break;

      case ID_PT_REPORT_2:
        RECV_DBC(recvPt2Rpt);
        break;

      case ID_PT_REPORT_3:
        RECV_DBC(recvPt3Rpt);
        break;

      case ID_REST_OF_FIELD:
        RECV_DBC(recvRestOfFieldRpt);
        break;

      case ID_STEERING_CMD:
        // Command from CT -> Raptor.
        break;

      case ID_STEERING_REPORT:
        RECV_DBC(recvSteeringRpt);
        break;

      case ID_STEERING_REPORT_EXTD:
        RECV_DBC(recvSteeringExtendedRpt);
        break;

      case ID_TIRE_PRESSURE_FL:
        RECV_DBC(recvTirePressureFlRpt);
        break;

      case ID_TIRE_PRESSURE_FR:
        RECV_DBC(recvTirePressureFrRpt);
        break;

      case ID_TIRE_PRESSURE_RL:
        RECV_DBC(recvTirePressureRlRpt);
        break;

      case ID_TIRE_PRESSURE_RR:
        RECV_DBC(recvTirePressureRrRpt);
        break;

      case ID_TIRE_TEMP_FL_1:
        RECV_DBC(recvTireTempFL01Rpt);
        break;
      case ID_TIRE_TEMP_FL_2:
        RECV_DBC(recvTireTempFL02Rpt);
        break;
      case ID_TIRE_TEMP_FL_3:
        RECV_DBC(recvTireTempFL03Rpt);
        break;
      case ID_TIRE_TEMP_FL_4:
        RECV_DBC(recvTireTempFL04Rpt);
        break;

      case ID_TIRE_TEMP_FR_1:
        RECV_DBC(recvTireTempFR01Rpt);
        break;
      case ID_TIRE_TEMP_FR_2:
        RECV_DBC(recvTireTempFR02Rpt);
        break;
      case ID_TIRE_TEMP_FR_3:
        RECV_DBC(recvTireTempFR03Rpt);
        break;
      case ID_TIRE_TEMP_FR_4:
        RECV_DBC(recvTireTempFR04Rpt);
        break;

      case ID_TIRE_TEMP_RL_1:
        RECV_DBC(recvTireTempRL01Rpt);
        break;
      case ID_TIRE_TEMP_RL_2:
        RECV_DBC(recvTireTempRL02Rpt);
        break;
      case ID_TIRE_TEMP_RL_3:
        RECV_DBC(recvTireTempRL03Rpt);
        break;
      case ID_TIRE_TEMP_RL_4:
        RECV_DBC(recvTireTempRL04Rpt);
        break;

      case ID_TIRE_TEMP_RR_1:
        RECV_DBC(recvTireTempRR01Rpt);
        break;
      case ID_TIRE_TEMP_RR_2:
        RECV_DBC(recvTireTempRR02Rpt);
        break;
      case ID_TIRE_TEMP_RR_3:
        RECV_DBC(recvTireTempRR03Rpt);
        break;
      case ID_TIRE_TEMP_RR_4:
        RECV_DBC(recvTireTempRR04Rpt);
        break;

      case ID_WHEEL_POTENTIOMETER_DATA:
        RECV_DBC(recvWheelPotentiometerDataRpt);
        break;

      case ID_WHEEL_SPEED_REPORT:
        RECV_DBC(recvWheelSpeedRpt);
        break;

      case ID_WHEEL_STRAIN_GAUGE:
        RECV_DBC(recvWheelStrainGaugeRpt);
        break;

      default:
        break;
    }
  }
}

void RaptorDbwCAN::recvAcceleratorRpt(const Frame::SharedPtr msg, DbcMessage * message)
{
  AcceleratorPedalReport out;
  out.header.stamp = msg->header.stamp;

  out.pedal_output = message->GetSignal("acc_pedal_fdbk")->GetResult();
  out.rolling_counter = message->GetSignal("acc_pedal_fdbk_counter")->GetResult();

  pub_accel_pedal_->publish(out);
}

void RaptorDbwCAN::recvBaseToCarSummaryRpt(const Frame::SharedPtr msg, DbcMessage * message)
{
  VehicleCommand out;
  out.header.stamp = msg->header.stamp;
  out.header.sequence_number = message->GetSignal("base_to_car_heartbeat")->GetResult();
  out.header.vehicle_number = vehicle_number_;

  auto flags = indy::Flags();

  auto track_condition = static_cast<uint8_t>(message->GetSignal("track_flag")->GetResult());
  flags.set_flag(indy::iac::get_flags_from_track_condition(track_condition));

  auto vehicle_signal = static_cast<uint8_t>(message->GetSignal("veh_flag")->GetResult());
  flags.set_flag(indy::iac::get_flags_from_vehicle_signal(vehicle_signal));

  indy::set_flags(out, flags);

  out.track_position = message->GetSignal("veh_rank")->GetResult();
  out.laps = message->GetSignal("lap_count")->GetResult();
  out.laps_fraction = message->GetSignal("lap_distance")->GetResult();

  publish_my_laps_ = get_parameter("publish_my_laps").as_bool();
  if (publish_my_laps_) {
    pub_rc_to_ct_->publish(out);
  }
}

void RaptorDbwCAN::recvBaseToCarTimingRpt(const Frame::SharedPtr msg, DbcMessage * message)
{
  VehicleTiming out;
  out.header.stamp = msg->header.stamp;

  out.laps = message->GetSignal("laps")->GetResult();
  out.lap_time = message->GetSignal("lap_time")->GetResult();
  out.time_stamp = message->GetSignal("time_stamp")->GetResult();

  pub_vehicle_timing_->publish(out);
}

void RaptorDbwCAN::recvBrakePressureRpt(const Frame::SharedPtr msg, DbcMessage * message)
{
  BrakeReport out;
  out.header.stamp = msg->header.stamp;

  out.pedal_position = message->GetSignal("brake_pressure_fdbk_front")->GetResult();
  out.pedal_output = message->GetSignal("brake_pressure_fdbk_rear")->GetResult();
  out.enabled = enables_[EN_BRAKE];
  out.driver_activity = false;

  auto brake_fault = false;
  brake_fault |= faults_[Fault::BRAKE_WARN];
  brake_fault |= faults_[Fault::OOS_FRONT_BRK];
  brake_fault |= faults_[Fault::OOS_REAR_BRK];

  out.fault_brake_system = brake_fault;
  out.rolling_counter = message->GetSignal("brk_pressure_fdbk_counter")->GetResult();
  out.brake_torque_actual = 0;
  out.intervention_active = false;
  out.intervention_ready = false;
  out.parking_brake.status = raptor_dbw_msgs::msg::ParkingBrake::OFF;
  out.control_type.value = raptor_dbw_msgs::msg::ActuatorControlMode::OPEN_LOOP;

  pub_brake_->publish(out);
}

void RaptorDbwCAN::recvDiagnosticRpt(const Frame::SharedPtr msg, DbcMessage * message)
{
  FaultReport out;
  out.stamp = msg->header.stamp;

  out.sd_brake_warning1 = static_cast<bool>(message->GetSignal("sd_brake_warning1")->GetResult());
  out.sd_brake_warning2 = static_cast<bool>(message->GetSignal("sd_brake_warning2")->GetResult());
  out.sd_brake_warning3 = static_cast<bool>(message->GetSignal("sd_brake_warning3")->GetResult());

  auto brake_warn = false;
  brake_warn |= out.sd_brake_warning1;
  brake_warn |= out.sd_brake_warning2;
  brake_warn |= out.sd_brake_warning3;
  setFault(Fault::BRAKE_WARN, brake_warn);

  out.est1_oos_front_brk = static_cast<bool>(message->GetSignal("est1_oos_front_brk")->GetResult());
  setFault(Fault::OOS_FRONT_BRK, out.est1_oos_front_brk);

  out.est2_oos_rear_brk = static_cast<bool>(message->GetSignal("est2_oos_rear_brk")->GetResult());
  setFault(Fault::OOS_REAR_BRK, out.est2_oos_rear_brk);

  out.est6_sd_ebrake = static_cast<bool>(message->GetSignal("est6_sd_ebrake")->GetResult());
  setFault(Fault::SD_EBRAKE, out.est6_sd_ebrake);

  out.sd_steer_warning1 = static_cast<bool>(message->GetSignal("sd_steer_warning1")->GetResult());
  out.sd_steer_warning2 = static_cast<bool>(message->GetSignal("sd_steer_warning2")->GetResult());
  out.sd_steer_warning3 = static_cast<bool>(message->GetSignal("sd_steer_warning3")->GetResult());

  auto steer_warn = false;
  steer_warn |= out.sd_steer_warning1;
  steer_warn |= out.sd_steer_warning2;
  steer_warn |= out.sd_steer_warning3;
  setFault(Fault::STEER_WARN, steer_warn);

  auto low_eng_speed = false;
  if (is_engine_on_) {
    // NOTE(dvd): Raptor sends this warning even if the engine is off.
    low_eng_speed = static_cast<bool>(message->GetSignal("est3_low_eng_speed")->GetResult());
  }

  out.est3_low_eng_speed = low_eng_speed;
  setFault(Fault::LOW_ENG_SPEED, low_eng_speed);

  out.sd_system_warning = static_cast<bool>(message->GetSignal("sd_system_warning")->GetResult());
  setFault(Fault::SD_SYSTEM_WARN, out.sd_system_warning);

  out.motec_warning = static_cast<bool>(message->GetSignal("motec_warning")->GetResult());
  setFault(Fault::MOTEC_WARN, out.motec_warning);

  out.sd_system_failure = static_cast<bool>(message->GetSignal("sd_system_failure")->GetResult());
  setFault(Fault::SD_SYSTEM_FAILURE, out.sd_system_failure);

  out.est4_sd_comms_loss = static_cast<bool>(message->GetSignal("est4_sd_comms_loss")->GetResult());
  setFault(Fault::SD_COMMS_LOSS, out.est4_sd_comms_loss);

  out.est5_motec_comms_loss =
    static_cast<bool>(message->GetSignal("est5_motec_comms_loss")->GetResult());
  setFault(Fault::MOTEC_COMMS_LOSS, out.est5_motec_comms_loss);

  out.adlink_hb_lost = static_cast<bool>(message->GetSignal("adlink_hb_lost")->GetResult());
  out.rc_lost = static_cast<bool>(message->GetSignal("rc_lost")->GetResult());

  auto heartbeat_loss = false;
  heartbeat_loss |= out.adlink_hb_lost;
  heartbeat_loss |= out.rc_lost;
  setFault(Fault::HB_STATUS, heartbeat_loss);

  pub_fault_report_->publish(out);
}

void RaptorDbwCAN::recvMiscRpt(const Frame::SharedPtr msg, DbcMessage * message)
{
  DoMiscReport out;
  out.stamp = msg->header.stamp;

  out.battery_voltage = message->GetSignal("battery_voltage")->GetResult();
  out.sys_state = message->GetSignal("sys_state")->GetResult();
  out.mode_switch_state = message->GetSignal("mode_switch_state")->GetResult();
  out.safety_switch_state = message->GetSignal("safety_switch_state")->GetResult();
  out.rolling_counter = message->GetSignal("raptor_rolling_counter")->GetResult();

  pub_do_misc_->publish(out);
}

void RaptorDbwCAN::recvPt1Rpt(const Frame::SharedPtr msg, DbcMessage * message)
{
  EngineReport out;
  out.stamp = msg->header.stamp;

  out.throttle_position = message->GetSignal("throttle_position")->GetResult();
  out.current_gear = message->GetSignal("current_gear")->GetResult();
  out.engine_rpm = message->GetSignal("engine_speed_rpm")->GetResult();
  out.vehicle_speed_kmph = message->GetSignal("vehicle_speed_kmph")->GetResult();
  out.engine_on_status = message->GetSignal("engine_state")->GetResult();
  out.engine_run_switch_status = message->GetSignal("engine_run_switch")->GetResult();
  out.gear_shift_status = message->GetSignal("gear_shift_status")->GetResult();

  vehicle_speed_kmph_ = out.vehicle_speed_kmph;
  is_engine_on_ = out.engine_on_status;

  pub_engine_->publish(out);
}

void RaptorDbwCAN::recvPt2Rpt(const Frame::SharedPtr, DbcMessage * message)
{
  engine_pressures_report_.fuel_pressure = message->GetSignal("fuel_pressure_kPa")->GetResult();
  engine_pressures_report_.engine_oil_pressure =
    message->GetSignal("engine_oil_pressure_kPa")->GetResult();
  engine_pressures_report_.engine_coolant_temperature =
    message->GetSignal("coolant_temperature")->GetResult();
  engine_pressures_report_.transmission_oil_temperature = message->GetSignal(
    "transmission_temperature")->GetResult();
  engine_pressures_report_.transmission_oil_pressure = message->GetSignal(
    "transmission_pressure_kPa")->GetResult();
}

void RaptorDbwCAN::recvPt3Rpt(const Frame::SharedPtr, DbcMessage * message)
{
  engine_pressures_report_.engine_oil_temperature =
    message->GetSignal("engine_oil_temperature")->GetResult();
}

void RaptorDbwCAN::recvRestOfFieldRpt(const Frame::SharedPtr, DbcMessage *)
{
  // IMPORTANT(dvd): We are not publishing this because there are no other cars on track.
}

void RaptorDbwCAN::recvSteeringRpt(const Frame::SharedPtr msg, DbcMessage * message)
{
  SteeringReport out;
  out.header.stamp = msg->header.stamp;

  out.enabled = enables_[EN_STEER];
  out.steering_wheel_angle = message->GetSignal("steering_motor_ang_avg_fdbk")->GetResult();
  out.rolling_counter = message->GetSignal("steering_motor_fdbk_counter")->GetResult();

  pub_steering_->publish(out);
}

void RaptorDbwCAN::recvSteeringExtendedRpt(const Frame::SharedPtr msg, DbcMessage * message)
{
  SteeringExtendedReport out;
  out.stamp = msg->header.stamp;

  out.steering_motor_ang_1 = message->GetSignal("steering_motor_ang_1_fdbk")->GetResult();
  out.steering_motor_ang_2 = message->GetSignal("steering_motor_ang_2_fdbk")->GetResult();
  out.steering_motor_ang_3 = message->GetSignal("steering_motor_ang_3_fdbk")->GetResult();

  pub_steering_extended_->publish(out);
}

void RaptorDbwCAN::recvTirePressureFlRpt(const Frame::SharedPtr msg, DbcMessage * message)
{
  tire_report_msg_.fl_tire_pressure_rpt.stamp = msg->header.stamp;
  tire_report_msg_.fl_tire_pressure_rpt.tire_pressure =
    message->GetSignal("FL_Tire_Pressure")->GetResult();
  tire_report_msg_.fl_tire_pressure_rpt.tire_pressure_gauge =
    message->GetSignal("FL_Tire_Pressure_Gauge")->GetResult();
}

void RaptorDbwCAN::recvTirePressureFrRpt(const Frame::SharedPtr msg, DbcMessage * message)
{
  tire_report_msg_.fr_tire_pressure_rpt.stamp = msg->header.stamp;
  tire_report_msg_.fr_tire_pressure_rpt.tire_pressure =
    message->GetSignal("FR_Tire_Pressure")->GetResult();
  tire_report_msg_.fr_tire_pressure_rpt.tire_pressure_gauge =
    message->GetSignal("FR_Tire_Pressure_Gauge")->GetResult();
}

void RaptorDbwCAN::recvTirePressureRlRpt(const Frame::SharedPtr msg, DbcMessage * message)
{
  tire_report_msg_.rl_tire_pressure_rpt.stamp = msg->header.stamp;
  tire_report_msg_.rl_tire_pressure_rpt.tire_pressure =
    message->GetSignal("RL_Tire_Pressure")->GetResult();
  tire_report_msg_.rl_tire_pressure_rpt.tire_pressure_gauge =
    message->GetSignal("RL_Tire_Pressure_Gauge")->GetResult();
}

void RaptorDbwCAN::recvTirePressureRrRpt(const Frame::SharedPtr msg, DbcMessage * message)
{
  tire_report_msg_.rr_tire_pressure_rpt.stamp = msg->header.stamp;
  tire_report_msg_.rr_tire_pressure_rpt.tire_pressure =
    message->GetSignal("RR_Tire_Pressure")->GetResult();
  tire_report_msg_.rr_tire_pressure_rpt.tire_pressure_gauge =
    message->GetSignal("RR_Tire_Pressure_Gauge")->GetResult();
}

#define DECLARE_RECV_TIRE_TEMP_RPT(l, u, index, first, second, third, fourth) \
  void RaptorDbwCAN::recvTireTemp ## u ## index ## Rpt( \
    const Frame::SharedPtr msg, \
    DbcMessage * message) \
  { \
    tire_report_msg_.l ## _tire_temp_rpt.stamp = msg->header.stamp; \
    tire_report_msg_.l ## _tire_temp_rpt.tire_temp_rpt_ ## index.stamp = msg->header.stamp; \
    tire_report_msg_.l ## _tire_temp_rpt.tire_temp_rpt_ ## index.tire_temp_01 = \
      message->GetSignal(#u "_Tire_Temp_" #first)->GetResult(); \
    tire_report_msg_.l ## _tire_temp_rpt.tire_temp_rpt_ ## index.tire_temp_02 = \
      message->GetSignal(#u "_Tire_Temp_" #second)->GetResult(); \
    tire_report_msg_.l ## _tire_temp_rpt.tire_temp_rpt_ ## index.tire_temp_03 = \
      message->GetSignal(#u "_Tire_Temp_" #third)->GetResult(); \
    tire_report_msg_.l ## _tire_temp_rpt.tire_temp_rpt_ ## index.tire_temp_04 = \
      message->GetSignal(#u "_Tire_Temp_" #fourth)->GetResult(); \
  }

#define DECLARE_RECV_TIRE_TEMP_RTS(l, u) \
  DECLARE_RECV_TIRE_TEMP_RPT(l, u, 01, 01, 02, 03, 04) \
  DECLARE_RECV_TIRE_TEMP_RPT(l, u, 02, 05, 06, 07, 08) \
  DECLARE_RECV_TIRE_TEMP_RPT(l, u, 03, 09, 10, 11, 12) \
  DECLARE_RECV_TIRE_TEMP_RPT(l, u, 04, 13, 14, 15, 16) \

DECLARE_RECV_TIRE_TEMP_RTS(fl, FL)
DECLARE_RECV_TIRE_TEMP_RTS(fr, FR)
DECLARE_RECV_TIRE_TEMP_RTS(rl, RL)
DECLARE_RECV_TIRE_TEMP_RTS(rr, RR)

void RaptorDbwCAN::recvWheelPotentiometerDataRpt(const Frame::SharedPtr msg, DbcMessage * message)
{
  tire_report_msg_.wheel_potentiometer_rpt.stamp = msg->header.stamp;
  tire_report_msg_.wheel_potentiometer_rpt.fl_damper_linear_potentiometer =
    message->GetSignal("wheel_potentiometer_FL")->GetResult();
  tire_report_msg_.wheel_potentiometer_rpt.fr_damper_linear_potentiometer =
    message->GetSignal("wheel_potentiometer_FR")->GetResult();
  tire_report_msg_.wheel_potentiometer_rpt.rl_damper_linear_potentiometer =
    message->GetSignal("wheel_potentiometer_RL")->GetResult();
  tire_report_msg_.wheel_potentiometer_rpt.rr_damper_linear_potentiometer =
    message->GetSignal("wheel_potentiometer_RR")->GetResult();
}

void RaptorDbwCAN::recvWheelSpeedRpt(const Frame::SharedPtr msg, DbcMessage * message)
{
  WheelSpeedReport out;
  out.header.stamp = msg->header.stamp;

  out.front_left = message->GetSignal("wheel_speed_FL")->GetResult();
  out.front_right = message->GetSignal("wheel_speed_FR")->GetResult();
  out.rear_left = message->GetSignal("wheel_speed_RL")->GetResult();
  out.rear_right = message->GetSignal("wheel_speed_RR")->GetResult();

  pub_wheel_speed_->publish(out);
}

void RaptorDbwCAN::recvWheelStrainGaugeRpt(const Frame::SharedPtr msg, DbcMessage * message)
{
  tire_report_msg_.wheel_strain_gauge_rpt.stamp = msg->header.stamp;
  tire_report_msg_.wheel_strain_gauge_rpt.fl_wheel_load =
    message->GetSignal("wheel_strain_gauge_FL")->GetResult();
  tire_report_msg_.wheel_strain_gauge_rpt.fr_wheel_load =
    message->GetSignal("wheel_strain_gauge_FR")->GetResult();
  tire_report_msg_.wheel_strain_gauge_rpt.rl_wheel_load =
    message->GetSignal("wheel_strain_gauge_RL")->GetResult();
  tire_report_msg_.wheel_strain_gauge_rpt.rr_wheel_load =
    message->GetSignal("wheel_strain_gauge_RR")->GetResult();
}

void RaptorDbwCAN::recvAcceleratorPedalCmd(const AcceleratorPedalCmd::SharedPtr msg)
{
  enables_[EN_ACCEL] = msg->enable;
  if (!enables_[EN_ACCEL]) {
    return;
  }

  NewEagle::DbcMessage * message = dbw_dbc_.GetMessageById(ID_ACCELERATOR_CMD);

  if (msg->control_type.value == ActuatorControlMode::OPEN_LOOP) {
    message->GetSignal("acc_pedal_cmd")->SetResult(msg->pedal_cmd);
  } else {
    message->GetSignal("acc_pedal_cmd")->SetResult(0.0F);
  }

  message->GetSignal("acc_pedal_cmd_counter")->SetResult(msg->rolling_counter);

  Frame frame = message->GetFrame();
  pub_can_->publish(frame);
}

void RaptorDbwCAN::recvBrakeCmd(const BrakeCmd::SharedPtr msg)
{
  enables_[EN_BRAKE] = msg->enable;
  if (!enables_[EN_BRAKE]) {
    return;
  }

  NewEagle::DbcMessage * message = dbw_dbc_.GetMessageById(ID_BRAKE_PRESSURE_CMD);

  if (msg->control_type.value == ActuatorControlMode::OPEN_LOOP) {
    message->GetSignal("brake_pressure_cmd")->SetResult(msg->pedal_cmd);
  } else {
    message->GetSignal("brake_pressure_cmd")->SetResult(1.0F);
  }

  message->GetSignal("brk_pressure_cmd_counter")->SetResult(msg->rolling_counter);

  Frame frame = message->GetFrame();
  pub_can_->publish(frame);
}

void RaptorDbwCAN::recvCtRpt(const VehicleStatus::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbw_dbc_.GetMessageById(ID_CT_REPORT);

  auto flags = indy::get_flags_received(*msg);
  auto track_cond_ack = indy::iac::get_track_condition_from_flags(flags);
  auto veh_sig_ack = indy::iac::get_vehicle_signal_from_flags(flags);

  message->GetSignal("track_cond_ack")->SetResult(static_cast<uint8_t>(track_cond_ack));
  message->GetSignal("veh_sig_ack")->SetResult(static_cast<uint8_t>(veh_sig_ack));
  message->GetSignal("veh_num")->SetResult(vehicle_number_);

  message->GetSignal("ct_state")->SetResult(msg->ct_state);
  message->GetSignal("ct_state_rolling_counter")->SetResult(msg->header.sequence_number);

  Frame frame = message->GetFrame();
  pub_can_->publish(frame);
}

void RaptorDbwCAN::recvGearCmd(const GearCmd::SharedPtr msg)
{
  {
    if (!enables_[EN_DBW]) {
      return;
    }

    NewEagle::DbcMessage * message = dbw_dbc_.GetMessageById(ID_GEAR_SHIFT_CMD);

    message->GetSignal("desired_gear")->SetResult(msg->cmd.gear);

    Frame frame = message->GetFrame();
    pub_can_->publish(frame);
  }

  {
    GearReport out;

    out.enabled = enables_[EN_DBW];
    out.state.gear = raptor_dbw_msgs::msg::Gear::DRIVE;
    out.driver_activity = false;
    out.gear_select_system_fault = false;
    out.reject = false;
    out.trans_curr_gear = msg->cmd.gear;
    out.gear_mismatch_flash = false;

    pub_gear_->publish(out);
  }
}

void RaptorDbwCAN::recvGlobalEnableCmd(const GlobalEnableCmd::SharedPtr msg)
{
  if (msg->global_enable) {
    enableSystem();
  } else {
    disableSystem();
  }
}

void RaptorDbwCAN::recvMiscCmd(const MiscCmd::SharedPtr)
{
  {
    MiscReport out;

    out.drive_by_wire_enabled = enabled();
    out.by_wire_ready = true;
    out.general_driver_activity = false;

    // NOTE(dvd): Convert to m/s.
    out.vehicle_speed = vehicle_speed_kmph_ / 3.6;

    out.comms_fault = fault();
    out.general_actuator_fault = fault();
    out.software_build_number = 0;
    out.ambient_temp = 0;
    out.fuel_level = 1;

    pub_misc_->publish(out);
  }

  {
    OtherActuatorsReport out;

    out.ignition_state.status = raptor_dbw_msgs::msg::Ignition::NO_REQUEST;
    out.horn_state.status = raptor_dbw_msgs::msg::HornState::OFF;
    out.turn_signal_state.value = raptor_dbw_msgs::msg::TurnSignal::NONE;
    out.turn_signal_sync = false;
    out.high_beam_state.value = raptor_dbw_msgs::msg::HighBeamState::OFF;
    out.low_beam_state.status = raptor_dbw_msgs::msg::LowBeam::OFF;
    out.front_wiper_state.status = raptor_dbw_msgs::msg::WiperFront::OFF;
    out.rear_wiper_state.status = raptor_dbw_msgs::msg::WiperRear::OFF;
    out.right_rear_door_state.value = raptor_dbw_msgs::msg::DoorState::CLOSED;
    out.left_rear_door_state.value = raptor_dbw_msgs::msg::DoorState::CLOSED;
    out.liftgate_door_state.value = raptor_dbw_msgs::msg::DoorState::CLOSED;
    out.door_lock_state.value = raptor_dbw_msgs::msg::DoorLock::LOCK;

    pub_other_actuators_->publish(out);
  }
}

void RaptorDbwCAN::recvSteeringCmd(const SteeringCmd::SharedPtr msg)
{
  enables_[EN_STEER] = msg->enable;
  if (!enables_[EN_STEER]) {
    return;
  }

  NewEagle::DbcMessage * message = dbw_dbc_.GetMessageById(ID_STEERING_CMD);

  if (msg->control_type.value == ActuatorControlMode::CLOSED_LOOP_ACTUATOR) {
    double steering_cmd =
      std::max(
      -1.0F * max_steer_angle_,
      std::min(max_steer_angle_ * 1.0F, static_cast<float>(msg->angle_cmd * 1.0F)));

    message->GetSignal("steering_motor_ang_cmd")->SetResult(steering_cmd);
  }

  message->GetSignal("steering_motor_cmd_counter")->SetResult(msg->rolling_counter);

  Frame frame = message->GetFrame();
  pub_can_->publish(frame);
}

/// \brief DBW Enabled needs to publish when its state changes.
/// \returns TRUE when DBW enable state changes, FALSE otherwise
bool RaptorDbwCAN::publishDbwEnabled()
{
  bool change = false;
  bool en = enabled();
  if (enables_[EN_DBW_PREV] != en) {
    Bool msg;
    msg.data = en;
    pub_sys_enable_->publish(msg);
    change = true;
  }
  enables_[EN_DBW_PREV] = en;
  return change;
}

void RaptorDbwCAN::enableSystem()
{
  if (!enables_[EN_DBW]) {
    auto & clock = *get_clock();
    if (fault()) {
      for (size_t ii = 0; ii < NUM_SERIOUS_FAULTS; ii++) {
        if (faults_[ii]) {
          std::string err_msg("DBW system disabled - ");
          err_msg = err_msg + FAULT_SYSTEM[ii];
          err_msg = err_msg + " fault.";
          RCLCPP_ERROR_THROTTLE(
            get_logger(), clock, MS_IN_SEC, err_msg.c_str());
        }
      }
    } else {
      enables_[EN_DBW] = true;
      if (publishDbwEnabled()) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), clock, MS_IN_SEC,
          "DBW system enabled.");
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), clock, MS_IN_SEC,
          "DBW system failed to enable. Check driver overrides.");
      }
    }
  }
}

void RaptorDbwCAN::disableSystem()
{
  if (enables_[EN_DBW]) {
    enables_[EN_DBW] = false;
    publishDbwEnabled();

    auto & clock = *get_clock();
    RCLCPP_INFO_THROTTLE(
      get_logger(), clock, MS_IN_SEC,
      "DBW system disabled - system disabled.");
  }
}

void RaptorDbwCAN::setFault(Fault which_fault, bool is_faulty)
{
  auto was_faulted = faults_[which_fault];

  if (!was_faulted && is_faulty) {
    faults_[which_fault] = true;

    std::string err_msg("Set fault - ");
    err_msg = err_msg + FAULT_SYSTEM[which_fault];
    err_msg = err_msg + " fault.";

    auto & clock = *get_clock();
    RCLCPP_ERROR_THROTTLE(get_logger(), clock, MS_IN_SEC, err_msg.c_str());

    if (which_fault >= NUM_SERIOUS_FAULTS) {
      return;
    }

    bool en = enabled();
    if (en) {
      enables_[EN_DBW] = false;
      if (publishDbwEnabled()) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), clock, MS_IN_SEC, "DBW system disabled due to fault");
      }
    }
  } else if (was_faulted && !is_faulty) {
    faults_[which_fault] = false;

    std::string err_msg("Cleared fault - ");
    err_msg = err_msg + FAULT_SYSTEM[which_fault];
    err_msg = err_msg + " fault.";

    auto & clock = *get_clock();
    RCLCPP_ERROR_THROTTLE(get_logger(), clock, MS_IN_SEC, err_msg.c_str());
  }
}

void RaptorDbwCAN::publishJointStates(
  const rclcpp::Time stamp,
  const WheelSpeedReport wheels)
{
  double dt = stamp.seconds() - joint_state_.header.stamp.sec;
  joint_state_.velocity[JOINT_FL] = wheels.front_left;
  joint_state_.velocity[JOINT_FR] = wheels.front_right;
  joint_state_.velocity[JOINT_RL] = wheels.rear_left;
  joint_state_.velocity[JOINT_RR] = wheels.rear_right;

  if (dt < 0.5) {
    for (size_t ii = JOINT_FL; ii <= JOINT_RR; ii++) {
      joint_state_.position[ii] = fmod(
        joint_state_.position[ii] + dt * joint_state_.velocity[ii],
        2 * M_PI);
    }
  }

  joint_state_.header.stamp = rclcpp::Time(stamp);
  pub_joint_states_->publish(joint_state_);
}

void RaptorDbwCAN::publishJointStates(
  const rclcpp::Time stamp,
  const SteeringReport steering)
{
  double dt = stamp.seconds() - joint_state_.header.stamp.sec;
  const double L = acker_wheelbase_;
  const double W = acker_track_;
  const double r = L / tan(steering.steering_wheel_angle / steering_ratio_);
  joint_state_.position[JOINT_SL] = atan(L / (r - W / 2));
  joint_state_.position[JOINT_SR] = atan(L / (r + W / 2));

  if (dt < 0.5) {
    for (size_t ii = JOINT_FL; ii <= JOINT_RR; ii++) {
      joint_state_.position[ii] = fmod(
        joint_state_.position[ii] + dt * joint_state_.velocity[ii],
        2 * M_PI);
    }
  }
  joint_state_.header.stamp = rclcpp::Time(stamp);
  pub_joint_states_->publish(joint_state_);
}

}  // namespace raptor_dbw_can
