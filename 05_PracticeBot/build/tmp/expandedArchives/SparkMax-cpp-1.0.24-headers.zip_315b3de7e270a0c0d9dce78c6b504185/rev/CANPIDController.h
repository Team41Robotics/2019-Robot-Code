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

#include "rev/CANError.h"
#include "rev/ControlType.h"

namespace rev {

class CANSparkMax;

class CANPIDController {
public:
    /**
     * Constructs a CANPIDController.
     *
     * @param device The Spark Max this object configures.
     */
    explicit CANPIDController(CANSparkMax& device);

    CANPIDController(CANPIDController&&) = default;
    CANPIDController& operator=(CANPIDController&&) = default;

    /**
     * Set the controller reference value based on the selected control mode.
     * This will override the pre-programmed control mode but not change what
     * is programmed to the controller.
     *
     * @param value The value to set depending on the control mode. For basic
     * duty cycle control this should be a value between -1 and 1
     * Otherwise: Voltage Control: Voltage (volts) Velocity Control: Velocity
     * (RPM) Position Control: Position (Rotations) Current Control: Current
     * (Amps)
     *
     * @param ctrl Is the control type to override with
     *
     * @param pidSlot for this command
     *
     * @param arbFeedforward A value from -32.0 to 32.0 which is a voltage
     * applied to the motor after the result of the specified control mode. This
     * value is set after the control mode, but before any current limits or
     * ramp rates.
     *
     * @return CANError Set to REV_OK if successful
     *
     */
    CANError SetReference(double value, ControlType ctrl, int pidSlot = 0,
                          double arbFeedforward = 0);

    /**
     * Set the Proportional Gain constant of the PIDF controller on the SPARK
     * MAX. This uses the Set Parameter API and should be used infrequently. The
     * parameter does not presist unless burnFlash() is called.  The recommended
     * method to configure this parameter is use to SPARK MAX GUI to tune and
     * save parameters.
     *
     * @param gain The proportional gain value, must be positive
     *
     * @param slotID Is the cascade/gain schedule slot, the value is a number
     * between 0 and 3. For cascade control, slot 0 and 1 are the inner loop,
     * and slot 2 and 3 are for the outer loop.
     *
     * @return CANError Set to REV_OK if successful
     *
     */
    CANError SetP(double gain, int slotID = 0);

    /**
     * Set the Integral Gain constant of the PIDF controller on the SPARK MAX.
     * This uses the Set Parameter API and should be used infrequently. The
     * parameter does not presist unless burnFlash() is called.  The recommended
     * method to configure this parameter is use to SPARK MAX GUI to tune and
     * save parameters.
     *
     * @param gain The integral gain value, must be positive
     *
     * @param slotID Is the cascade/gain schedule slot, the value is a number
     * between 0 and 3. For cascade control, slot 0 and 1 are the inner loop,
     * and slot 2 and 3 are for the outer loop.
     *
     * @return CANError Set to REV_OK if successful
     *
     */
    CANError SetI(double gain, int slotID = 0);

    /**
     * Set the Derivative Gain constant of the PIDF controller on the SPARK MAX.
     * This uses the Set Parameter API and should be used infrequently. The
     * parameter does not presist unless burnFlash() is called.  The recommended
     * method to configure this parameter is use to SPARK MAX GUI to tune and
     * save parameters.
     *
     * @param gain The derivative gain value, must be positive
     *
     * @param slotID Is the cascade/gain schedule slot, the value is a number
     * between 0 and 3. For cascade control, slot 0 and 1 are the inner loop,
     * and slot 2 and 3 are for the outer loop.
     *
     * @return CANError Set to REV_OK if successful
     *
     */
    CANError SetD(double gain, int slotID = 0);

    /**
     * Set the Feed-froward Gain constant of the PIDF controller on the SPARK
     * MAX. This uses the Set Parameter API and should be used infrequently. The
     * parameter does not presist unless burnFlash() is called.  The recommended
     * method to configure this parameter is use to SPARK MAX GUI to tune and
     * save parameters.
     *
     * @param gain The feed-forward gain value
     *
     * @param slotID Is the cascade/gain schedule slot, the value is a number
     * between 0 and 3. For cascade control, slot 0 and 1 are the inner loop,
     * and slot 2 and 3 are for the outer loop.
     *
     * @return CANError Set to REV_OK if successful
     *
     */
    CANError SetFF(double gain, int slotID = 0);

    /**
     * Set the IZone range of the PIDF controller on the SPARK MAX. This value
     * specifies the range the |error| must be within for the integral constant
     * to take effect.
     *
     * This uses the Set Parameter API and should be used infrequently.
     * The parameter does not presist unless burnFlash() is called.
     * The recommended method to configure this parameter is to use the
     * SPARK MAX GUI to tune and save parameters.
     *
     * @param gain The IZone value, must be positive. Set to 0 to disable
     *
     * @param slotID Is the cascade/gain schedule slot, the value is a number
     * between 0 and 3. For cascade control, slot 0 and 1 are the inner loop,
     * and slot 2 and 3 are for the outer loop.
     *
     * @return CANError Set to REV_OK if successful
     *
     */
    CANError SetIZone(double IZone, int slotID = 0);

    /**
     * Set the derivative filter strenth of the PIDF controller on the SPARK
     * MAX. This value amount of filtering done on the derivative term in the
     * PIDF loop.
     *
     * This uses the Set Parameter API and should be used infrequently.
     * The parameter does not presist unless burnFlash() is called.
     * The recommended method to configure this parameter is to use the
     * SPARK MAX GUI to tune and save parameters.
     *
     * @param filter A value from 0 - 1 adjustig the filter pole. Set to 0 to
     * disable.
     *
     * @param slotID Is the cascade/gain schedule slot, the value is a number
     * between 0 and 3. For cascade control, slot 0 and 1 are the inner loop,
     * and slot 2 and 3 are for the outer loop.
     *
     * @return CANError Set to REV_OK if successful
     *
     */
    // CANError SetDFilter(double filter, int slotID);

    /**
     * Set the min amd max output for the closed loop mode.
     *
     * This uses the Set Parameter API and should be used infrequently.
     * The parameter does not presist unless burnFlash() is called.
     * The recommended method to configure this parameter is to use the
     * SPARK MAX GUI to tune and save parameters.
     *
     * @param min Reverse power minimum to allow the controller to output
     *
     * @param max Forward power maximum to allow the controller to output
     *
     * @param slotID Is the cascade/gain schedule slot, the value is a number
     * between 0 and 3. For cascade control, slot 0 and 1 are the inner loop,
     * and slot 2 and 3 are for the outer loop.
     *
     * @return CANError Set to REV_OK if successful
     *
     */
    CANError SetOutputRange(double min, double max, int slotID = 0);

    /**
     * Get the Proportional Gain constant of the PIDF controller on the SPARK
     * MAX.
     *
     * This uses the Get Parameter API and should be used infrequently. This
     * function uses a non-blocking call and will return a cached value if the
     * parameter is not returned by the timeout. The timeout can be changed by
     * calling SetCANTimeout(int milliseconds)
     *
     * @param slotID Is the cascade/gain schedule slot, the value is a number
     * between 0 and 3. For cascade control, slot 0 and 1 are the inner loop,
     * and slot 2 and 3 are for the outer loop.
     *
     * @return double P Gain value
     *
     */
    double GetP(int slotID = 0);

    /**
     * Get the Integral Gain constant of the PIDF controller on the SPARK MAX.
     *
     * This uses the Get Parameter API and should be used infrequently. This
     * function uses a non-blocking call and will return a cached value if the
     * parameter is not returned by the timeout. The timeout can be changed by
     * calling SetCANTimeout(int milliseconds)
     *
     * @param slotID Is the cascade/gain schedule slot, the value is a number
     * between 0 and 3. For cascade control, slot 0 and 1 are the inner loop,
     * and slot 2 and 3 are for the outer loop.
     *
     * @return double I Gain value
     *
     */
    double GetI(int slotID = 0);

    /**
     * Get the Derivative Gain constant of the PIDF controller on the SPARK MAX.
     *
     * This uses the Get Parameter API and should be used infrequently. This
     * function uses a non-blocking call and will return a cached value if the
     * parameter is not returned by the timeout. The timeout can be changed by
     * calling SetCANTimeout(int milliseconds)
     *
     * @param slotID Is the cascade/gain schedule slot, the value is a number
     * between 0 and 3. For cascade control, slot 0 and 1 are the inner loop,
     * and slot 2 and 3 are for the outer loop.
     *
     * @return double D Gain value
     *
     */
    double GetD(int slotID = 0);

    /**
     * Get the Feed-forward Gain constant of the PIDF controller on the SPARK
     * MAX.
     *
     * This uses the Get Parameter API and should be used infrequently. This
     * function uses a non-blocking call and will return a cached value if the
     * parameter is not returned by the timeout. The timeout can be changed by
     * calling SetCANTimeout(int milliseconds)
     *
     * @param slotID Is the cascade/gain schedule slot, the value is a number
     * between 0 and 3. For cascade control, slot 0 and 1 are the inner loop,
     * and slot 2 and 3 are for the outer loop.
     *
     * @return double F Gain value
     *
     */
    double GetFF(int slotID = 0);

    /**
     * Get the IZone constant of the PIDF controller on the SPARK MAX.
     *
     * This uses the Get Parameter API and should be used infrequently. This
     * function uses a non-blocking call and will return a cached value if the
     * parameter is not returned by the timeout. The timeout can be changed by
     * calling SetCANTimeout(int milliseconds)
     *
     * @param slotID Is the cascade/gain schedule slot, the value is a number
     * between 0 and 3. For cascade control, slot 0 and 1 are the inner loop,
     * and slot 2 and 3 are for the outer loop.
     *
     * @return double IZone value
     *
     */
    double GetIZone(int slotID = 0);

    /**
     * Get the derivative filter constant of the PIDF controller on the SPARK
     * MAX.
     *
     * This uses the Get Parameter API and should be used infrequently. This
     * function uses a non-blocking call and will return a cached value if the
     * parameter is not returned by the timeout. The timeout can be changed by
     * calling SetCANTimeout(int milliseconds)
     *
     * @param slotID Is the cascade/gain schedule slot, the value is a number
     * between 0 and 3. For cascade control, slot 0 and 1 are the inner loop,
     * and slot 2 and 3 are for the outer loop.
     *
     * @return double D Filter
     *
     */
    // double GetDFilter(int slotID = 0);

    /**
     * Get the min output of the PIDF controller on the SPARK MAX.
     *
     * This uses the Get Parameter API and should be used infrequently. This
     * function uses a non-blocking call and will return a cached value if the
     * parameter is not returned by the timeout. The timeout can be changed by
     * calling SetCANTimeout(int milliseconds)
     *
     * @param slotID Is the cascade/gain schedule slot, the value is a number
     * between 0 and 3. For cascade control, slot 0 and 1 are the inner loop,
     * and slot 2 and 3 are for the outer loop.
     *
     * @return double min value
     *
     */
    double GetOutputMin(int slotID = 0);

    /**
     * Get the max output of the PIDF controller on the SPARK MAX.
     *
     * This uses the Get Parameter API and should be used infrequently. This
     * function uses a non-blocking call and will return a cached value if the
     * parameter is not returned by the timeout. The timeout can be changed by
     * calling SetCANTimeout(int milliseconds)
     *
     * @param slotID Is the cascade/gain schedule slot, the value is a number
     * between 0 and 3. For cascade control, slot 0 and 1 are the inner loop,
     * and slot 2 and 3 are for the outer loop.
     *
     * @return double max value
     *
     */
    double GetOutputMax(int slotID = 0);

private:
    CANSparkMax* m_device;
};

}  // namespace rev
