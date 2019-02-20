/*
 * Copyright (c) 2018 REV Robotics
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of REV Robotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <stdint.h>

#include <map>
#include <string>
#include <vector>

#include <frc/CAN.h>
#include <frc/ErrorBase.h>
#include <frc/SpeedController.h>
#include <hal/Types.h>
#include <wpi/SafeThread.h>

#include "rev/CANError.h"
#include "rev/ControlType.h"

namespace rev {

class CANSparkMaxLowLevel : public frc::ErrorBase, public frc::SpeedController {
    friend class CANPIDController;
    friend class CANDigitalInput;
    friend class CANEncoder;

public:
    enum class MotorType { kBrushed = 0, kBrushless = 1 };

    enum class ParameterStatus {
        kOK = 0,
        kInvalidID = 1,
        kMismatchType = 2,
        kAccessMode = 3,
        kInvalid = 4,
        kNotImplementedDeprecated = 5,
    };

    enum class ConfigParameter {
        kCanID = 0,
        kInputMode = 1,
        kMotorType = 2,
        kCommAdvance = 3,
        kSensorType = 4,
        kCtrlType = 5,
        kIdleMode = 6,
        kInputDeadband = 7,
        kFirmwareVer = 8,
        kHallOffset = 9,
        kPolePairs = 10,
        kCurrentChop = 11,
        kCurrentChopCycles = 12,
        kP_0 = 13,
        kI_0 = 14,
        kD_0 = 15,
        kF_0 = 16,
        kIZone_0 = 17,
        kDFilter_0 = 18,
        kOutputMin_0 = 19,
        kOutputMax_0 = 20,
        kP_1 = 21,
        kI_1 = 22,
        kD_1 = 23,
        kF_1 = 24,
        kIZone_1 = 25,
        kDFilter_1 = 26,
        kOutputMin_1 = 27,
        kOutputMax_1 = 28,
        kP_2 = 29,
        kI_2 = 30,
        kD_2 = 31,
        kF_2 = 32,
        kIZone_2 = 33,
        kDFilter_2 = 34,
        kOutputMin_2 = 35,
        kOutputMax_2 = 36,
        kP_3 = 37,
        kI_3 = 38,
        kD_3 = 39,
        kF_3 = 40,
        kIZone_3 = 41,
        kDFilter_3 = 42,
        kOutputMin_3 = 43,
        kOutputMax_3 = 44,
        kReserved = 45,
        kOutputRatio = 46,
        kSerialNumberLow = 47,
        kSerialNumberMid = 48,
        kSerialNumberHigh = 49,
        kLimitSwitchFwdPolarity = 50,
        kLimitSwitchRevPolarity = 51,
        kHardLimitFwdEn = 52,
        kHardLimitRevEn = 53,
        kSoftLimitFwdEn = 54,
        kSoftLimitRevEn = 55,
        kRampRate = 56,
        kFollowerID = 57,
        kFollowerConfig = 58,
        kSmartCurrentStallLimit = 59,
        kSmartCurrentFreeLimit = 60,
        kSmartCurrentConfig = 61,
        kSmartCurrentReserved = 62,
        kMotorKv = 63,
        kMotorR = 64,
        kMotorL = 65,
        kMotorRsvd1 = 66,
        kMotorRsvd2 = 67,
        kMotorRsvd3 = 68,
        kEncoderCountsPerRev = 69,
        kEncoderAverageDepth = 70,
        kEncoderSampleDelta = 71,
    };

    enum class ParameterType {
        kInt32 = 0,
        kUint32 = 1,
        kFloat32 = 2,
        kBool = 3
    };

    enum class PerioidicFrame { Status0 = 0, Status1 = 1, Status2 = 2 };

    struct PeriodicStatus0 {
        double appliedOutput;
        uint16_t faults;
        uint16_t stickyFaults;
        uint8_t idleMode;
        MotorType motorType;
        bool isFollower;
    };

    struct PeriodicStatus1 {
        double sensorVelocity;
        uint8_t motorTemperature;
        double busVoltage;
        double outputCurrent;
    };

    struct PeriodicStatus2 {
        double sensorPosition;
    };

    /**
     * Create a new SPARK MAX Controller
     *
     * @param deviceID The device ID.
     * @param type     The motor type connected to the controller. Brushless
     *                 motors must be connected to their matching color and the
     *                 hall sensor plugged in. Brushed motors must be connected
     *                 to the Red and Black terminals only.
     */
    explicit CANSparkMaxLowLevel(int deviceID, MotorType type);

    /**
     * Closes the SPARK MAX Controller
     */
    ~CANSparkMaxLowLevel() override = default;

    /**
     * Get the firmware version of the SPARK MAX.
     *
     * @return uint32_t Firmware version integer. Value is represented as 4
     * bytes, Major.Minor.Build H.Build L
     *
     */
    uint32_t GetFirmwareVersion();

    uint32_t GetFirmwareVersion(bool& isDebugBuild);

    /**
     * Get the firmware version of the SPARK MAX as a string.
     *
     * @return std::string Human readable firmware version string
     *
     */
    std::string GetFirmwareString();

    /**
     * Get the unique serial number of the SPARK MAX. Currently not implemented.
     *
     * @return std::vector<uint8_t> Vector of bytes representig the unique
     * serial number
     *
     */
    std::vector<uint8_t> GetSerialNumber();

    /**
     * Get the configured Device ID of the SPARK MAX.
     *
     * @return int device ID
     *
     */
    int GetDeviceId() const;

    /**
     * Set the motor type connected to the SPARK MAX.
     *
     * This uses the Set Parameter API and should be used infrequently.
     * The parameter does not presist unless burnFlash() is called.
     * The recommended method to configure this parameter is to use the
     * SPARK MAX GUI to tune and save parameters.
     *
     * @param type The type of motor connected to the controller. Brushless
     * motors must be connected to their matching color and the hall sensor
     * plugged in. Brushed motors must be connected to the Red and Black
     * terminals only.
     *
     * @return CANError Set to CANError::kOk if successful
     *
     */
    CANError SetMotorType(MotorType type);

    /**
     * Get the motor type setting for the SPARK MAX.
     *
     * This uses the Get Parameter API and should be used infrequently. This
     * function uses a non-blocking call and will return a cached value if the
     * parameter is not returned by the timeout. The timeout can be changed by
     * calling SetCANTimeout(int milliseconds)
     *
     * @return MotorType Motor type setting
     *
     */
    MotorType GetMotorType();

    /**
     * Set the rate of transmission for periodic frames from the SPARK MAX
     *
     * Refer to the SPARK MAX reference manual on details for how and when
     * to configure this parameter.
     *
     * @return CANError Set to CANError::kOk if successful
     *
     */
    CANError SetPeriodicFramePeriod(PerioidicFrame frame, int periodMs);

    /**
     * Set the rate of transmission for setpoint frame. This will not
     * take effect until the next call of a method that sets a new
     * setpoint.
     *
     * Refer to the SPARK MAX reference manual on details for how and when
     * to configure this parameter.
     *
     * @param periodMs The rate to send control frames
     *
     */
    void SetControlFramePeriod(int periodMs);

    /**
     * Return the rate of transmission for setpoint frame
     *
     * @return Control frame perioid in milliseconds
     *
     */
    int GetControlFramePeriod();

    ParameterType GetParameterType(ConfigParameter parameterID);
    ParameterStatus SetParameter(ConfigParameter parameterID, double value);
    ParameterStatus SetParameter(ConfigParameter parameterID, uint32_t value);
    ParameterStatus SetParameter(ConfigParameter parameterID, int32_t value);
    ParameterStatus SetParameter(ConfigParameter parameterID, bool value);
    ParameterStatus GetParameter(ConfigParameter parameterID, double& value);
    ParameterStatus GetParameter(ConfigParameter parameterID, uint32_t& value);
    ParameterStatus GetParameter(ConfigParameter parameterID, int32_t& value);
    ParameterStatus GetParameter(ConfigParameter parameterID, bool& value);

protected:
    PeriodicStatus0 GetPeriodicStatus0();
    PeriodicStatus1 GetPeriodicStatus1();
    PeriodicStatus2 GetPeriodicStatus2();

    struct FollowConfig {
        uint32_t leaderArbId;
        union {
            struct {
                uint32_t rsvd1 : 18;
                uint32_t invert : 1;
                uint32_t rsvd2 : 5;
                uint32_t predefined : 8;
            } config;
            uint32_t configRaw;
        };
    };

    CANError SetFollow(FollowConfig config);
    CANError FollowerInvert(bool invert);

    CANError SetpointCommand(double value,
                             ControlType ctrl = ControlType::kDutyCycle,
                             int pidSlot = 0, double arbFeedforward = 0);

    ParameterStatus SetParameterCore(ConfigParameter parameterID,
                                     ParameterType type, uint32_t value);
    ParameterStatus GetParameterCore(ConfigParameter parameterID,
                                     ParameterType expectedType,
                                     uint32_t& value);

private:
    enum class ConfigParameterType {
        kDefault = static_cast<int>(ParameterType::kInt32),
        kCanID = static_cast<int>(ParameterType::kUint32),
        kInputMode = static_cast<int>(ParameterType::kUint32),
        kMotorType = static_cast<int>(ParameterType::kUint32),
        kCommAdvance = static_cast<int>(ParameterType::kFloat32),
        kSensorType = static_cast<int>(ParameterType::kUint32),
        kCtrlType = static_cast<int>(ParameterType::kUint32),
        kIdleMode = static_cast<int>(ParameterType::kUint32),
        kInputDeadband = static_cast<int>(ParameterType::kFloat32),
        kFirmwareVer = static_cast<int>(ParameterType::kUint32),
        kHallOffset = static_cast<int>(ParameterType::kInt32),
        kPolePairs = static_cast<int>(ParameterType::kUint32),
        kCurrentChop = static_cast<int>(ParameterType::kFloat32),
        kCurrentChopCycles = static_cast<int>(ParameterType::kUint32),
        kP_0 = static_cast<int>(ParameterType::kFloat32),
        kI_0 = static_cast<int>(ParameterType::kFloat32),
        kD_0 = static_cast<int>(ParameterType::kFloat32),
        kF_0 = static_cast<int>(ParameterType::kFloat32),
        kIZone_0 = static_cast<int>(ParameterType::kFloat32),
        kDFilter_0 = static_cast<int>(ParameterType::kFloat32),
        kOutputMin_0 = static_cast<int>(ParameterType::kFloat32),
        kOutputMax_0 = static_cast<int>(ParameterType::kFloat32),
        kP_1 = static_cast<int>(ParameterType::kFloat32),
        kI_1 = static_cast<int>(ParameterType::kFloat32),
        kD_1 = static_cast<int>(ParameterType::kFloat32),
        kF_1 = static_cast<int>(ParameterType::kFloat32),
        kIZone_1 = static_cast<int>(ParameterType::kFloat32),
        kDFilter_1 = static_cast<int>(ParameterType::kFloat32),
        kOutputMin_1 = static_cast<int>(ParameterType::kFloat32),
        kOutputMax_1 = static_cast<int>(ParameterType::kFloat32),
        kP_2 = static_cast<int>(ParameterType::kFloat32),
        kI_2 = static_cast<int>(ParameterType::kFloat32),
        kD_2 = static_cast<int>(ParameterType::kFloat32),
        kF_2 = static_cast<int>(ParameterType::kFloat32),
        kIZone_2 = static_cast<int>(ParameterType::kFloat32),
        kDFilter_2 = static_cast<int>(ParameterType::kFloat32),
        kOutputMin_2 = static_cast<int>(ParameterType::kFloat32),
        kOutputMax_2 = static_cast<int>(ParameterType::kFloat32),
        kP_3 = static_cast<int>(ParameterType::kFloat32),
        kI_3 = static_cast<int>(ParameterType::kFloat32),
        kD_3 = static_cast<int>(ParameterType::kFloat32),
        kF_3 = static_cast<int>(ParameterType::kFloat32),
        kIZone_3 = static_cast<int>(ParameterType::kFloat32),
        kDFilter_3 = static_cast<int>(ParameterType::kFloat32),
        kOutputMin_3 = static_cast<int>(ParameterType::kFloat32),
        kOutputMax_3 = static_cast<int>(ParameterType::kFloat32),
        kReserved = static_cast<int>(ParameterType::kUint32),
        kOutputRatio = static_cast<int>(ParameterType::kFloat32),
        kSerialNumberLow = static_cast<int>(ParameterType::kUint32),
        kSerialNumberMid = static_cast<int>(ParameterType::kUint32),
        kSerialNumberHigh = static_cast<int>(ParameterType::kUint32),
        kLimitSwitchFwdPolarity = static_cast<int>(ParameterType::kBool),
        kLimitSwitchRevPolarity = static_cast<int>(ParameterType::kBool),
        kHardLimitFwdEn = static_cast<int>(ParameterType::kBool),
        kHardLimitRevEn = static_cast<int>(ParameterType::kBool),
        kSoftLimitFwdEn = static_cast<int>(ParameterType::kBool),
        kSoftLimitRevEn = static_cast<int>(ParameterType::kBool),
        kRampRate = static_cast<int>(ParameterType::kFloat32),
        kFollowerID = static_cast<int>(ParameterType::kUint32),
        kFollowerConfig = static_cast<int>(ParameterType::kUint32),
        kSmartCurrentStallLimit = static_cast<int>(ParameterType::kUint32),
        kSmartCurrentFreeLimit = static_cast<int>(ParameterType::kUint32),
        kSmartCurrentConfig = static_cast<int>(ParameterType::kUint32),
        kSmartCurrentReserved = static_cast<int>(ParameterType::kUint32),
        kMotorKv = static_cast<int>(ParameterType::kUint32),
        kMotorR = static_cast<int>(ParameterType::kUint32),
        kMotorL = static_cast<int>(ParameterType::kUint32),
        kMotorRsvd1 = static_cast<int>(ParameterType::kUint32),
        kMotorRsvd2 = static_cast<int>(ParameterType::kUint32),
        kMotorRsvd3 = static_cast<int>(ParameterType::kUint32),
        kEncoderCountsPerRev = static_cast<int>(ParameterType::kUint32),
        kEncoderAverageDepth = static_cast<int>(ParameterType::kUint32),
        kEncoderSampleDelta = static_cast<int>(ParameterType::kUint32),
    };

    std::map<ConfigParameter, uint32_t> m_parameterCache;

    bool m_isFirmwareDebug;
    uint32_t m_firmwareVersion;
    std::string m_firmwareString;

    class Daemon;
    wpi::SafeThreadOwner<Daemon>* m_owner;

    static wpi::SafeThreadOwner<Daemon>& GetThreadOwner();
    const int m_deviceID;

    int m_status0PerioidMs;
    int m_status1PerioidMs;
    int m_status2PerioidMs;

    int m_activeSetpointApi;

protected:
    frc::CAN m_can;
    int m_controlPeriodMs;
    int m_canTimeoutMs;
    bool m_inverted;

    // FIXME: remove when vendor and product ID are added to image
    static constexpr HAL_CANManufacturer HAL_CAN_Man_kREV =
        static_cast<HAL_CANManufacturer>(5);
};

}  // namespace rev
