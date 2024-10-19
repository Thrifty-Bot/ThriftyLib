/**
 * Copyright (c) 2024 Thrifty Bot LLC
 * All Rights Reserved
 * This software and associated documentation files (the "Software") are the proprietary property of Thrifty Bot LLC and are protected by copyright law and international treaty provisions. Unauthorized reproduction, distribution, or use of this Software, in whole or in part, is strictly prohibited.
 * No part of this Software may be reproduced, modified, transmitted, transcribed, stored in a retrieval system, or translated into any language or computer language, in any form or by any means, electronic, mechanical, magnetic, optical, chemical, manual or otherwise, without the prior written permission of Thrifty Bot LLC.
 * This Software is provided by Thrifty Bot LLC on an "AS IS" basis. Thrifty Bot LLC makes no warranties, express or implied, including without limitation the implied warranties of non-infringement, merchantability and fitness for a particular purpose, regarding the Software or its use and operation alone or in combination with other products.
 * In no event shall Thrifty Bot LLC be liable for any special, indirect, incidental, consequential, or punitive damages in connection with or arising out of this license or use of the Software.
 * Thrifty Bot LLC
 * contact@thethriftybot.com
 */
package com.thethriftybot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.hal.CANAPIJNI;
import edu.wpi.first.hal.CANData;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/**
 * API for interfacing with the Thrifty Nova motor controller.
 * 
 * <p>For detailed documentation, visit:
 * <a href="https://docs.thethriftybot.com/thrifty-nova/gqCPUYXcVoOZ4KW3DqIr/">
 * Thrifty Nova Documentation</a>
 * </p>
 * 
 * @version 1.1.1
 */
public final class ThriftyNova extends CANDevice implements MotorController {

    // =====================================================
    // CONSTANTS AND ENUMERATIONS
    // =====================================================

    /**
     * Different types of control modes.
     * <p>Note: The ordinality of the enum constants matters!</p>
     */
    private static enum ControlType {
        NONE,
        PERCENT_OUTPUT,
        POSITION_INTERNAL,
        VELOCITY_INTERNAL,
        POSITION_QUAD,
        VELOCITY_QUAD,
        POSITION_ABS,
        FOLLOW;
        // POSITION_CAN,    (WIP)
        // VELOCITY_CAN     (WIP)
    }

    /**
     * Different encoder types supported by the Thrifty Nova motor controller.
     */
    public static enum EncoderType {
        INTERNAL(42, 1),
        QUAD(4096, 2),
        ABS(4096, 4);
        // CAN          (WIP)

        /**
         * Number of ticks per revolution for the encoder.
         */
        public final int ticks;

        /**
         * Identifier for the encoder type.
         */
        public final int id;

        /**
         * Constructs an EncoderType with specified ticks and identifier.
         *
         * @param ticks Number of ticks per revolution.
         * @param id    Identifier for the encoder type.
         */
        private EncoderType(int ticks, int id) {
            this.ticks = ticks;
            this.id = id;
        }
    }

    /**
     * Different types of current control available.
     * <p>Note: The ordinality of the enum constants matters!</p>
     */
    public static enum CurrentType {
        STATOR, SUPPLY
    }

    /**
     * The various PID configuration slots available.
     * <p>Note: The ordinality of the enum constants matters!</p>
     */
    public static enum PIDSlot {
        SLOT0, SLOT1;
    }

    /**
     * Enumeration of possible configuration errors.
     */
    public static enum Error {
        SET_INVERSION,
        SET_BREAK_MODE,
        SET_MAX_FWD, // forward
        SET_MAX_REV, // reverse
        SET_RAMP_UP,
        SET_RAMP_DOWN,
        // Change current limit types
        SET_MAX_CURRENT,
        SET_CURRENT_TYPE,
        SET_FOLLOWER_ID,

        SET_KP_0, SET_KP_1,
        SET_KI_0, SET_KI_1,
        SET_KD_0, SET_KD_1,
        SET_KF_0, SET_KF_1,
        SET_IZONE_0, SET_IZONE_1,

        SET_FREQ_FAULT,
        SET_FREQ_SENSOR,
        SET_FREQ_QUAD_SENSOR,
        SET_FREQ_CTRL,
        SET_FREQ_CURRENT,

        SET_SOFT_LIMIT_FWD,
        SET_SOFT_LIMIT_REV,

        ENABLE_SOFT_LIMIT,
        ENABLE_HARD_LIMIT
    }

    // =====================================================
    // PID & CAN FREQUENCY SUB-CONFIGURATIONS
    // =====================================================

    /**
     * Abstract class representing a sub-configuration for the ThriftyNova motor controller.
     */
    private static abstract class SubConfig {
        /**
         * Reference to the parent ThriftyNova motor controller.
         */
        final ThriftyNova motor;

        /**
         * Constructs a SubConfig with the specified motor controller.
         *
         * @param motor The ThriftyNova motor controller instance.
         */
        protected SubConfig(ThriftyNova motor) {
            this.motor = motor;
        }
    }

    /**
     * Configuration structure for PID controllers.
     */
    public static final class PIDConfig extends SubConfig {
        /**
         * Indicates whether this configuration is for slot 0.
         */
        boolean slot0;

        /**
         * Constructs a PIDConfig for the specified slot.
         *
         * @param motor The ThriftyNova motor controller instance.
         * @param slot  The PID slot number (0 or 1).
         */
        private PIDConfig(ThriftyNova motor, int slot) {
            super(motor);
            this.slot0 = slot == 0;
        }

        /**
         * Sets the PID controller terms from a PIDController object.
         *
         * @param pid The PIDController object.
         * @return The parent ThriftyNova motor controller instance.
         */
        public ThriftyNova setPID(PIDController pid) {
            setP(pid.getP());
            setI(pid.getI());
            setD(pid.getD());
            return motor;
        }

        /**
         * Sets the proportional term for the PID controller.
         *
         * @param p The proportional term.
         * @return The parent ThriftyNova motor controller instance.
         */
        public ThriftyNova setP(double p) {
            motor.setConfig(slot0 ? 10 : 14, translate(p), slot0 ? Error.SET_KP_0 : Error.SET_KP_1);
            return motor;
        }

        /**
         * Sets the integral term for the PID controller.
         *
         * @param i The integral term.
         * @return The parent ThriftyNova motor controller instance.
         */
        public ThriftyNova setI(double i) {
            motor.setConfig(slot0 ? 11 : 15, translate(i), slot0 ? Error.SET_KI_0 : Error.SET_KI_1);
            return motor;
        }

        /**
         * Sets the derivative term for the PID controller.
         *
         * @param d The derivative term.
         * @return The parent ThriftyNova motor controller instance.
         */
        public ThriftyNova setD(double d) {
            motor.setConfig(slot0 ? 12 : 16, translate(d), slot0 ? Error.SET_KD_0 : Error.SET_KD_1);
            return motor;
        }

        /**
         * Sets the feed forward term for the PID controller.
         *
         * @param ff The feed forward term.
         * @return The parent ThriftyNova motor controller instance.
         */
        public ThriftyNova setFF(double ff) {
            motor.setConfig(slot0 ? 13 : 17, translate(ff), slot0 ? Error.SET_KF_0 : Error.SET_KF_1);
            return motor;
        }

        /**
         * Translates a double input to an integer representation.
         *
         * @param input The input value to translate.
         * @return The translated integer value.
         */
        private static int translate(double input) {
            return (int) (1_000_000 * input);
        }
    }

    /**
     * Configuration structure for CAN frame frequency settings.
     */
    public static final class CANFreqConfig extends SubConfig {
        /**
         * Constructs a CANFreqConfig for the specified motor controller.
         *
         * @param motor The ThriftyNova motor controller instance.
         */
        private CANFreqConfig(ThriftyNova motor) {
            super(motor);
        }

        /**
         * Sets the CAN frame frequency for fault reporting.
         *
         * @param per The period in seconds.
         * @return This CANFreqConfig instance.
         */
        public CANFreqConfig setFault(double per) {
            motor.setConfig(18, translate(per), Error.SET_FREQ_FAULT);
            return motor.canFreq;
        }

        /**
         * Sets the CAN frame frequency for encoder feedback.
         *
         * @param per The period in seconds.
         * @return This CANFreqConfig instance.
         */
        public CANFreqConfig setSensor(double per) {
            motor.setConfig(19, translate(per), Error.SET_FREQ_SENSOR);
            return motor.canFreq;
        }

        /**
         * Sets the CAN frame frequency for quadrature encoder feedback.
         *
         * @param per The period in seconds.
         * @return This CANFreqConfig instance.
         */
        public CANFreqConfig setQuadSensor(double per) {
            motor.setConfig(20, translate(per), Error.SET_FREQ_QUAD_SENSOR);
            return motor.canFreq;
        }

        /**
         * Sets the CAN frame frequency for control commands.
         *
         * @param per The period in seconds.
         * @return This CANFreqConfig instance.
         */
        public CANFreqConfig setControl(double per) {
            motor.setConfig(21, translate(per), Error.SET_FREQ_CTRL);
            return motor.canFreq;
        }

        /**
         * Sets the CAN frame frequency for current measurements.
         *
         * @param per The period in seconds.
         * @return This CANFreqConfig instance.
         */
        public CANFreqConfig setCurrent(double per) {
            motor.setConfig(22, translate(per), Error.SET_FREQ_CURRENT);
            return motor.canFreq;
        }

        /**
         * Translates a double input to an integer representation.
         *
         * @param input The input value to translate.
         * @return The translated integer value.
         */
        private static int translate(double input) {
            return (int) (1_000 * input);
        }
    }

    // =====================================================
    // CLASS FIELDS
    // =====================================================

    /**
     * The current set point for control operations.
     */
    private double currentSetPoint = 0;

    /**
     * Indicates whether the motor is inverted.
     */
    private boolean inversion = false;

    /**
     * The ID of the motor being followed, if any.
     */
    private int followingID = -1;

    /**
     * List of configuration errors encountered.
     */
    public List<Error> errors;

    /**
     * PID configuration for slot 0.
     */
    public final PIDConfig pid0;

    /**
     * PID configuration for slot 1.
     */
    public final PIDConfig pid1;

    /**
     * CAN frequency configuration.
     */
    public final CANFreqConfig canFreq;

    /**
     * Currently selected PID slot.
     */
    private PIDSlot pidSlot = PIDSlot.SLOT0;

    /**
     * Currently selected encoder type.
     */
    private EncoderType encoderType = EncoderType.INTERNAL;

    /**
     * Currently selected control type.
     */
    private ControlType controlType = ControlType.PERCENT_OUTPUT;

    // =====================================================
    // CONSTRUCTORS
    // =====================================================

    /**
     * Instantiates a new ThriftyNova motor controller using a CAN identifier.
     *
     * @param canID The CAN identifier for the particular device.
     */
    public ThriftyNova(int canID) {
        super(canID, DeviceType.MOTOR_CTRL);
        this.errors = new ArrayList<>();
        this.pid0 = new PIDConfig(this, 0);
        this.pid1 = new PIDConfig(this, 1);
        this.canFreq = new CANFreqConfig(this);
    }

    // =====================================================
    // ERROR HANDLING
    // =====================================================

    /**
     * Adds an error to the error buffer if the buffer is not full.
     *
     * @param err The error to add.
     */
    private void addError(Error err) {
        if (errors.size() < 256)
            errors.add(err);
    }

    /**
     * Returns a list of the errors that have populated in the buffer.
     *
     * @return A list of errors.
     */
    public List<Error> getErrors() {
        return errors;
    }

    /**
     * Clears the error buffer.
     */
    public void clearErrors() {
        errors.clear();
    }

    /**
     * Checks if a specific error is present in the buffer.
     *
     * @param err The error to check for.
     * @return {@code true} if the error exists, {@code false} otherwise.
     */
    public boolean hasError(Error err) {
        for (Error e : errors)
            if (e == err)
                return true;
        return false;
    }

    // =====================================================
    // CONFIGURATION FUNCTIONS
    // =====================================================

    /**
     * Sets the inversion status of the motor.
     *
     * @param inverted {@code true} to invert the motor, {@code false} otherwise.
     */
    public void setInverted(boolean inverted) {
        if (setConfig(0, inverted ? 1 : 0, Error.SET_INVERSION)) {
            inversion = inverted;
        }
        // Note: The method does not return the instance for chaining.
    }

    /**
     * Sets the brake mode status of the motor.
     *
     * @param brakeMode {@code true} to enable brake mode, {@code false} to disable.
     * @return This ThriftyNova motor controller instance.
     */
    public ThriftyNova setBrakeMode(boolean brakeMode) {
        setConfig(1, brakeMode ? 1 : 0, Error.SET_BREAK_MODE);
        return this;
    }

    /**
     * Sets the maximum percent output for the motor.
     *
     * @param maxOutput Maximum forward output between 0 and 1.
     * @return This ThriftyNova motor controller instance.
     */
    public ThriftyNova setMaxOutput(double maxOutput) {
        setMaxOutput(maxOutput, maxOutput);
        return this;
    }

    /**
     * Sets the maximum forward and reverse percent output for the motor.
     *
     * @param maxFwd Maximum forward output between 0 and 1.
     * @param maxRev Maximum reverse output between 0 and 1.
     * @return This ThriftyNova motor controller instance.
     */
    public ThriftyNova setMaxOutput(double maxFwd, double maxRev) {
        setConfig(3, translateOutput(limitRange(0, 1, maxFwd)), Error.SET_MAX_FWD);
        // Originally: -limitRange(-1, 0, maxRev), but changed to positive range
        setConfig(4, translateOutput(limitRange(0, 1, maxRev)), Error.SET_MAX_REV);
        return this;
    }

    /**
     * Sets the ramp-up time in seconds from idle to full speed.
     *
     * <p>For example, an input of 0.5 will ramp the motor from idle to 100% over 0.5 seconds.</p>
     *
     * @param rampUp The ramp-up time in seconds (0 to 10).
     * @return This ThriftyNova motor controller instance.
     */
    public ThriftyNova setRampUp(double rampUp) {
        rampUp = limitRange(0, 10, rampUp);

        double translatedRamp = 1;

        if (rampUp != 0)
            translatedRamp = 1 / (rampUp * 1000);

        setConfig(5, translateOutput(translatedRamp), Error.SET_RAMP_UP);

        return this;
    }

    /**
     * Sets the ramp-down time in seconds from full speed to idle.
     *
     * <p>For example, an input of 0.5 will ramp the motor from 100% to idle over 0.5 seconds.</p>
     *
     * @param rampDown The ramp-down time in seconds (0 to 10).
     * @return This ThriftyNova motor controller instance.
     */
    public ThriftyNova setRampDown(double rampDown) {
        rampDown = limitRange(0, 10, rampDown);

        double translatedRamp = 1;

        if (rampDown != 0)
            translatedRamp = 1 / (rampDown * 1000);

        setMotorConfig(6, translateOutput(translatedRamp));

        return this;
    }

    /**
     * Sets the maximum current the motor can draw and the type of current measurement.
     *
     * <p>Motor speed will be capped to satisfy the max current. The current reading used
     * for limiting calculations can be either stator current or supply current.</p>
     *
     * @param currentType The type of current reading used for limiting calculations.
     * @param maxCurrent  The maximum current in amps.
     * @return This ThriftyNova motor controller instance.
     */
    public ThriftyNova setMaxCurrent(CurrentType currentType, double maxCurrent) {
        setConfig(7, CANFreqConfig.translate(maxCurrent), Error.SET_MAX_CURRENT);
        setConfig(8, currentType.ordinal(), Error.SET_CURRENT_TYPE);
        return this;
    }

    /**
     * Sets the soft limits that the motor will not cross if soft limits are enabled.
     *
     * @param revLimit The reverse position limit.
     * @param fwdLimit The forward position limit.
     * @return This ThriftyNova motor controller instance.
     */
    public ThriftyNova setSoftLimits(double revLimit, double fwdLimit) {
        setConfig(26, (int) revLimit, Error.SET_SOFT_LIMIT_REV);
        setConfig(25, (int) fwdLimit, Error.SET_SOFT_LIMIT_FWD);
        return this;
    }

    /**
     * Enables or disables soft limits.
     *
     * @param enable {@code true} to enable soft limits, {@code false} to disable.
     * @return This ThriftyNova motor controller instance.
     */
    public ThriftyNova enableSoftLimits(boolean enable) {
        setConfig(24, enable ? 1 : 0, Error.ENABLE_SOFT_LIMIT);
        return this;
    }

    /**
     * Enables or disables hard limits.
     *
     * @param enable {@code true} to enable hard limits, {@code false} to disable.
     * @return This ThriftyNova motor controller instance.
     */
    public ThriftyNova enableHardLimits(boolean enable) {
        setConfig(23, enable ? 1 : 0, Error.ENABLE_HARD_LIMIT);
        return this;
    }

    /**
     * Sets the encoder type to use for feedback control.
     *
     * @param encoderType The encoder type to use.
     * @return This ThriftyNova motor controller instance.
     */
    public ThriftyNova useEncoderType(EncoderType encoderType) {
        this.encoderType = encoderType;
        return this;
    }

    /**
     * Sets the PID slot to use for feedback control.
     *
     * @param pidSlot The PID slot to use.
     * @return This ThriftyNova motor controller instance.
     */
    public ThriftyNova usePIDSlot(PIDSlot pidSlot) {
        this.pidSlot = pidSlot;
        return this;
    }

    // =====================================================
    // CONTROL SETTER FUNCTIONS
    // =====================================================

    /**
     * Sets the percent output of the motor.
     *
     * @param percentOutput The percent output of the motor, ranging from -1.0 (full reverse) to 1.0 (full forward).
     */
    public void setPercent(double percentOutput) {
        currentSetPoint = 0;
        setMotorControl(ControlType.PERCENT_OUTPUT, translateOutput(limitRange(-1, 1, percentOutput)));
    }

    /**
     * Sets the percent output of the motor.
     *
     * @param percentOutput The percent output of the motor, ranging from -1.0 (full reverse) to 1.0 (full forward).
     */
    public void set(double percentOutput) {
        setPercent(percentOutput);
    }

    /**
     * Disables the motor by setting its percent output to zero.
     */
    public void disable() {
        setPercent(0);
    }

    /**
     * Stops the motor by setting its percent output to zero.
     */
    public void stopMotor() {
        setPercent(0);
    }

    /**
     * Drives the motor towards the given position using the configured PID controller.
     *
     * @param targetPosition The target position.
     */
    public void setPosition(double targetPosition) {
        currentSetPoint = targetPosition;

        switch (encoderType) {
            case INTERNAL:
                setMotorControl(ControlType.POSITION_INTERNAL, (int) targetPosition);
                break;
            case QUAD:
                setMotorControl(ControlType.POSITION_QUAD, (int) targetPosition);
                break;
            case ABS:
                setMotorControl(ControlType.POSITION_ABS, (int) targetPosition);
                break;
            default:
                // Cannot set position with given encoder type
                break;
        }
    }

    /**
     * Drives the motor at the given velocity using the configured PID controller.
     *
     * @param targetVelocity The target velocity.
     */
    public void setVelocity(double targetVelocity) {
        currentSetPoint = targetVelocity;

        switch (encoderType) {
            case INTERNAL:
                setMotorControl(ControlType.VELOCITY_INTERNAL, (int) targetVelocity);
                break;
            case QUAD:
                setMotorControl(ControlType.VELOCITY_QUAD, (int) targetVelocity);
                break;
            default:
                // Cannot set velocity with the given encoder type (e.g., ABS)
                break;
        }
    }

    /**
     * Drives the motor using the parameters of the configured followed motor controller.
     * Follower mode is dependent on the fault status frame. The slower that frame is sent,
     * the slower the follower motor will update.
     *
     * @param followerID The CAN ID of the motor controller to follow.
     */
    public void follow(int followerID) {
        if (followerID < 0)
            followerID = 0;

        if (followingID != followerID && setConfig(9, 0x20D5000 | (followerID & 0x3F), Error.SET_FOLLOWER_ID)) {
            followingID = followerID;
        }

        if (followerID == followingID)
            setMotorControl(ControlType.FOLLOW, 0);
    }

    // =====================================================
    // STATUS & FEEDBACK GETTERS
    // =====================================================

    /**
     * Gets the encoder position measurement.
     *
     * @return The position measurement.
     */
    public double getPosition() {
        return reciveStatusFrame(encoderType.id, 4, 7);
    }

    /**
     * Gets the encoder velocity measurement.
     *
     * @return The velocity measurement.
     */
    public double getVelocity() {
        if (encoderType == EncoderType.ABS)
            return 0.0; // Cannot get velocity for an ABS encoder
        return reciveStatusFrame(encoderType.id, 0, 3);
    }

    /**
     * Gets the stator current measurement.
     *
     * @return The stator current measurement in amps.
     */
    public int getStatorCurrent() {
        return reciveStatusFrame(4, 0, 1);
    }

    /**
     * Checks if the motor is inverted.
     *
     * @return {@code true} if the motor is inverted, {@code false} otherwise.
     */
    public boolean getInverted() {
        return inversion;
    }

    /**
     * Gets the supply current measurement.
     *
     * @return The supply current measurement in amps.
     */
    public int getSupplyCurrent() {
        return reciveStatusFrame(4, 2, 3);
    }

    /**
     * Returns the last set point specified by the user in any control method.
     * Percent output set point becomes zero.
     *
     * @return The current set point.
     */
    public double getSetPoint() {
        return currentSetPoint;
    }

    /**
     * Returns the last set point specified by the user in any control method.
     * Percent output set point becomes zero.
     *
     * @return The current set point.
     */
    public double get() {
        return currentSetPoint;
    }

    /**
     * Returns the error relative to the last set point based on the last used
     * control mode (position or velocity) and encoder type (internal, quadrature, or ABS).
     *
     * @return The closed-loop error.
     */
    public double getClosedLoopError() {
        switch (controlType) {
            case POSITION_INTERNAL:
            case POSITION_QUAD:
            case POSITION_ABS:
                return currentSetPoint - getPosition();
            case VELOCITY_INTERNAL:
            case VELOCITY_QUAD:
                return currentSetPoint - getVelocity();
            default:
                return 0;
        }
    }

    /**
     * Sets the encoder position.
     *
     * @param position The position to set the encoder to.
     */
    public void setEncoderPosition(double position) {
        System.out.println("ORDINAL: " + encoderType.ordinal());

        CANAPIJNI.writeCANPacketNoThrow(
                deviceHandle,
                new byte[]{
                        (byte) (((int) position >> 24) & 0xFF),
                        (byte) (((int) position >> 16) & 0xFF),
                        (byte) (((int) position >> 8) & 0xFF),
                        (byte) ((int) position & 0xFF),
                        (byte) (encoderType.ordinal()),
                        (byte) (0),
                        (byte) (0),
                        (byte) (0)
                },
                APIClass.CTRL.get(2)
        );
    }

    // =====================================================
    // INTERNAL FUNCTIONS
    // =====================================================

    /**
     * Sets the motor control type and target value.
     *
     * @param ctrlType The control type to set.
     * @param target   The target value for the control type.
     */
    private void setMotorControl(ControlType ctrlType, int target) {
        controlType = ctrlType;

        CANAPIJNI.writeCANPacketNoThrow(
                deviceHandle,
                new byte[]{
                        0,
                        0,
                        (byte) pidSlot.ordinal(),
                        (byte) (ctrlType.ordinal() == 7 ? 10 : ctrlType.ordinal()),
                        (byte) ((target >> 24) & 0xFF),
                        (byte) ((target >> 16) & 0xFF),
                        (byte) ((target >> 8) & 0xFF),
                        (byte) (target & 0xFF)
                },
                APIClass.CTRL.get(1)
        );
    }

    /**
     * Sets a configuration parameter and logs an error if the operation fails.
     *
     * @param index The configuration index.
     * @param value The value to set.
     * @param err   The error to log if setting fails.
     * @return {@code true} if the configuration was set successfully, {@code false} otherwise.
     */
    private boolean setConfig(int index, int value, Error err) {
        if (!setMotorConfig(index, value)) {
            addError(err);
            return false;
        }
        return true;
    }

    /**
     * Sets a motor configuration parameter.
     *
     * @param index The configuration index.
     * @param value The value to set.
     * @return {@code true} if the configuration was set successfully, {@code false} otherwise.
     */
    private boolean setMotorConfig(int index, int value) {

        if (CANAPIJNI.writeCANPacketNoThrow(
                deviceHandle,
                new byte[]{
                        (byte) ((value >> 24) & 0xFF),
                        (byte) ((value >> 16) & 0xFF),
                        (byte) ((value >> 8) & 0xFF),
                        (byte) (value & 0xFF),
                        (byte) ((index >> 24) & 0xFF),
                        (byte) ((index >> 16) & 0xFF),
                        (byte) ((index >> 8) & 0xFF),
                        (byte) (index & 0xFF)
                },
                APIClass.CTRL.get(0)
        ) != 0)
            return false;

        CANData dataBuffer = new CANData();
        long rioTime = RobotController.getFPGATime();

        // Check if robot is disabled to adjust timeout
        while ((RobotController.getFPGATime() - rioTime) < (RobotState.isDisabled() ? 50_000 : 5_000)) {
            CANAPIJNI.readCANPacketNew(
                    deviceHandle,
                    APIClass.STATUS.get(5),
                    dataBuffer
            );

            if (dataBuffer.data[0] == 0x69) // 'i' in ASCII (0x69)
                return true;
            // 6E696365
        }

        // System.out.println("\nThriftyWarning: Failed to set configuration " + index + ".\n");
        return false;
    }

    /**
     * Translates a double input to an integer representation for output.
     *
     * @param input The input value to translate.
     * @return The translated integer value.
     */
    private int translateOutput(double input) {
        return (int) (10_000 * input);
    }

    /**
     * Limits a value to be within a specified range.
     *
     * @param min   The minimum allowable value.
     * @param max   The maximum allowable value.
     * @param input The input value to limit.
     * @return The limited value.
     */
    private double limitRange(double min, double max, double input) {
        return Math.min(Math.max(input, min), max);
    }
}
