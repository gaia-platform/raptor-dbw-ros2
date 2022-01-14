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

#ifndef RAPTOR_DBW_CAN__DISPATCH_HPP_
#define RAPTOR_DBW_CAN__DISPATCH_HPP_

namespace raptor_dbw_can
{

/** \brief Enumeration of CAN message IDs */
enum ListMessageIDs
{
  ID_ACCELERATOR_CMD          = 0x0579,  /**< Accelerator Cmd ID */
  ID_ACCELERATOR_REPORT       = 0x0516,  /**< Accelerator Report ID */
  ID_BASE_TO_CAR_SUMMARY      = 0x04B0,  /**< Base To Car Summary ID */
  ID_BASE_TO_CAR_TIMING       = 0x04B8,  /**< Base To Car Timing ID */
  ID_BRAKE_PRESSURE_CMD       = 0x0578,  /**< Brake Pressure Cmd ID */
  ID_BRAKE_PRESSURE_REPORT    = 0x0515,  /**< Brake Pressure Report ID */
  ID_CT_REPORT                = 0x057C,  /**< Ct Report ID */
  ID_DIAGNOSTIC_REPORT        = 0x053E,  /**< Diagnostic Report ID */
  ID_GEAR_SHIFT_CMD           = 0x057B,  /**< Gear Shift Cmd ID */
  ID_MISC_REPORT              = 0x0518,  /**< Misc Report ID */
  ID_POSITION_HEADING         = 0x05DC,  /**< Position Heading ID */
  ID_PT_REPORT_1              = 0x053C,  /**< Pt Report 1 ID */
  ID_PT_REPORT_2              = 0x053D,  /**< Pt Report 2 ID */
  ID_PT_REPORT_3              = 0x053F,  /**< Pt Report 3 ID */
  ID_REST_OF_FIELD            = 0x04B9,  /**< Rest Of Field ID */
  ID_STEERING_CMD             = 0x057A,  /**< Steering Cmd ID */
  ID_STEERING_REPORT          = 0x0517,  /**< Steering Report ID */
  ID_STEERING_REPORT_EXTD     = 0x0520,  /**< Steering Report Extd ID */
  ID_TIRE_PRESSURE_FL         = 0x0528,  /**< Tire Pressure FL ID */
  ID_TIRE_PRESSURE_FR         = 0x0529,  /**< Tire Pressure FR ID */
  ID_TIRE_PRESSURE_RL         = 0x052A,  /**< Tire Pressure RL ID */
  ID_TIRE_PRESSURE_RR         = 0x052B,  /**< Tire Pressure RR ID */
  ID_TIRE_TEMP_FL_1           = 0x052C,  /**< Tire Temp FL 1 ID */
  ID_TIRE_TEMP_FL_2           = 0x052D,  /**< Tire Temp FL 2 ID */
  ID_TIRE_TEMP_FL_3           = 0x052E,  /**< Tire Temp FL 3 ID */
  ID_TIRE_TEMP_FL_4           = 0x052F,  /**< Tire Temp FL 4 ID */
  ID_TIRE_TEMP_FR_1           = 0x0530,  /**< Tire Temp FR 1 ID */
  ID_TIRE_TEMP_FR_2           = 0x0531,  /**< Tire Temp FR 2 ID */
  ID_TIRE_TEMP_FR_3           = 0x0532,  /**< Tire Temp FR 3 ID */
  ID_TIRE_TEMP_FR_4           = 0x0533,  /**< Tire Temp FR 4 ID */
  ID_TIRE_TEMP_RL_1           = 0x0534,  /**< Tire Temp RL 1 ID */
  ID_TIRE_TEMP_RL_2           = 0x0535,  /**< Tire Temp RL 2 ID */
  ID_TIRE_TEMP_RL_3           = 0x0536,  /**< Tire Temp RL 3 ID */
  ID_TIRE_TEMP_RL_4           = 0x0537,  /**< Tire Temp RL 4 ID */
  ID_TIRE_TEMP_RR_1           = 0x0538,  /**< Tire Temp RR 1 ID */
  ID_TIRE_TEMP_RR_2           = 0x0539,  /**< Tire Temp RR 2 ID */
  ID_TIRE_TEMP_RR_3           = 0x053A,  /**< Tire Temp RR 3 ID */
  ID_TIRE_TEMP_RR_4           = 0x053B,  /**< Tire Temp RR 4 ID */
  ID_VELOCITY_ACCELERATION    = 0x05DD,  /**< Velocity Acceleration ID */
  ID_WHEEL_POTENTIOMETER_DATA = 0x051F,  /**< Wheel Potentiometer Data ID */
  ID_WHEEL_SPEED_REPORT       = 0x0514,  /**< Wheel Speed Report ID */
  ID_WHEEL_STRAIN_GAUGE       = 0x051E,  /**< Wheel Strain Gauge ID */
};

}  // namespace raptor_dbw_can

#endif  // RAPTOR_DBW_CAN__DISPATCH_HPP_
