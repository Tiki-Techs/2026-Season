/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
package com.ctre.phoenix6.controls;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.jni.ControlJNI;
import com.ctre.phoenix6.hardware.traits.*;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import java.util.HashMap;
import java.util.Map;

/**
 * Follow the motor output of another Talon.
 * <p>
 * The follower will atomically change its output type when it receives the leader's latest output status
 * signal (DutyCycle, MotorVoltage, TorqueCurrent). If Talon is in torque control, the torque is copied -
 * which will increase the total torque applied. If Talon is in duty cycle output control, the duty cycle is
 * matched. If Talon is in voltage output control, the motor voltage is matched. Motor direction either
 * matches the leader's configured direction or opposes it based on the MotorAlignment.
 * <p>
 * The leader must ensure the status signal corresponding to its control output type (DutyCycle, MotorVoltage,
 * TorqueCurrent) is enabled. The update rate of the status signal determines the update rate of the
 * follower's output and should be no slower than 20 Hz.
 */
public final class Follower implements ControlRequest, Cloneable {
    /**
     * Device ID of the leader to follow.
     */
    public int LeaderID;
    /**
     * Set to Aligned for motor invert to match the leader's configured Invert -
     * which is typical when leader and follower are mechanically linked and spin in
     * the same direction.  Set to Opposed for motor invert to oppose the leader's
     * configured Invert - this is typical where the leader and follower
     * mechanically spin in opposite directions.
     */
    public MotorAlignmentValue MotorAlignment;

    /**
     * The frequency at which this control will update.
     * This is designated in Hertz, with a minimum of 20 Hz
     * (every 50 ms) and a maximum of 1000 Hz (every 1 ms).
     * Some update frequencies are not supported and will be
     * promoted up to the next highest supported frequency.
     * <p>
     * If this field is set to 0 Hz, the control request will
     * be sent immediately as a one-shot frame. This may be useful
     * for advanced applications that require outputs to be
     * synchronized with data acquisition. In this case, we
     * recommend not exceeding 50 ms between control calls.
     */
    public double UpdateFreqHz = 20;

    /**
     * Follow the motor output of another Talon.
     * <p>
     * The follower will atomically change its output type when it receives the
     * leader's latest output status signal (DutyCycle, MotorVoltage,
     * TorqueCurrent). If Talon is in torque control, the torque is copied - which
     * will increase the total torque applied. If Talon is in duty cycle output
     * control, the duty cycle is matched. If Talon is in voltage output control,
     * the motor voltage is matched. Motor direction either matches the leader's
     * configured direction or opposes it based on the MotorAlignment.
     * <p>
     * The leader must ensure the status signal corresponding to its control output
     * type (DutyCycle, MotorVoltage, TorqueCurrent) is enabled. The update rate of
     * the status signal determines the update rate of the follower's output and
     * should be no slower than 20 Hz.
     * 
     * @param LeaderID Device ID of the leader to follow.
     * @param MotorAlignment Set to Aligned for motor invert to match the leader's
     *                       configured Invert - which is typical when leader and
     *                       follower are mechanically linked and spin in the same
     *                       direction.  Set to Opposed for motor invert to oppose
     *                       the leader's configured Invert - this is typical where
     *                       the leader and follower mechanically spin in opposite
     *                       directions.
     */
    public Follower(int LeaderID, MotorAlignmentValue MotorAlignment) {
        this.LeaderID = LeaderID;
        this.MotorAlignment = MotorAlignment;
    }

    @Override
    public String getName() {
        return "Follower";
    }

    @Override
    public String toString() {
        String ss = "Control: Follower\n";
        ss += "    LeaderID: " + LeaderID + "\n";
        ss += "    MotorAlignment: " + MotorAlignment + "\n";
        return ss;
    }

    @Override
    public StatusCode sendRequest(String network, int deviceHash) {
        return StatusCode.valueOf(ControlJNI.JNI_RequestControlFollower(
                network, deviceHash, UpdateFreqHz, LeaderID, MotorAlignment.value));
    }

    /**
     * Gets information about this control request.
     *
     * @return Map of control parameter names and corresponding applied values
     */
    @Override
    public Map<String, String> getControlInfo() {
        var controlInfo = new HashMap<String, String>();
        controlInfo.put("Name", getName());
        controlInfo.put("LeaderID", String.valueOf(this.LeaderID));
        controlInfo.put("MotorAlignment", String.valueOf(this.MotorAlignment));
        return controlInfo;
    }
    
    /**
     * Modifies this Control Request's LeaderID parameter and returns itself for
     * method-chaining and easier to use request API.
     * <p>
     * Device ID of the leader to follow.
     *
     * @param newLeaderID Parameter to modify
     * @return Itself
     */
    public Follower withLeaderID(int newLeaderID) {
        LeaderID = newLeaderID;
        return this;
    }
    
    /**
     * Modifies this Control Request's MotorAlignment parameter and returns itself for
     * method-chaining and easier to use request API.
     * <p>
     * Set to Aligned for motor invert to match the leader's configured Invert -
     * which is typical when leader and follower are mechanically linked and spin in
     * the same direction.  Set to Opposed for motor invert to oppose the leader's
     * configured Invert - this is typical where the leader and follower
     * mechanically spin in opposite directions.
     *
     * @param newMotorAlignment Parameter to modify
     * @return Itself
     */
    public Follower withMotorAlignment(MotorAlignmentValue newMotorAlignment) {
        MotorAlignment = newMotorAlignment;
        return this;
    }

    /**
     * Sets the frequency at which this control will update.
     * This is designated in Hertz, with a minimum of 20 Hz
     * (every 50 ms) and a maximum of 1000 Hz (every 1 ms).
     * Some update frequencies are not supported and will be
     * promoted up to the next highest supported frequency.
     * <p>
     * If this field is set to 0 Hz, the control request will
     * be sent immediately as a one-shot frame. This may be useful
     * for advanced applications that require outputs to be
     * synchronized with data acquisition. In this case, we
     * recommend not exceeding 50 ms between control calls.
     *
     * @param newUpdateFreqHz Parameter to modify
     * @return Itself
     */
    @Override
    public Follower withUpdateFreqHz(double newUpdateFreqHz) {
        UpdateFreqHz = newUpdateFreqHz;
        return this;
    }

    /**
     * Sets the frequency at which this control will update.
     * This is designated in Hertz, with a minimum of 20 Hz
     * (every 50 ms) and a maximum of 1000 Hz (every 1 ms).
     * Some update frequencies are not supported and will be
     * promoted up to the next highest supported frequency.
     * <p>
     * If this field is set to 0 Hz, the control request will
     * be sent immediately as a one-shot frame. This may be useful
     * for advanced applications that require outputs to be
     * synchronized with data acquisition. In this case, we
     * recommend not exceeding 50 ms between control calls.
     *
     * @param newUpdateFreqHz Parameter to modify
     * @return Itself
     */
    @Override
    public Follower withUpdateFreqHz(Frequency newUpdateFreqHz) {
        UpdateFreqHz = newUpdateFreqHz.in(Hertz);
        return this;
    }

    @Override
    public Follower clone() {
        try {
            return (Follower)super.clone();
        } catch (CloneNotSupportedException ex) {
            /* this should never happen */
            throw new RuntimeException(ex);
        }
    }
}

