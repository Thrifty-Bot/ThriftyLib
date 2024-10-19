/**
 * Copyright (C) 2024 The Thrifty Bot, LLC
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

import edu.wpi.first.hal.CANAPIJNI;
import edu.wpi.first.hal.CANData;

public abstract class CANDevice {

    private final static int THRIFTY_VENDOR_ID = 13;

    // Ordinality matters
    public static enum DeviceType {
        BROADCAST_MSG,
        ROBOT_CTRL,
        MOTOR_CTRL,
        RELAY_CTRL,
        GYRO_SENSOR,
        ACCELEROMETER_SENSOR,
        ULTRASONIC_SENSOR,
        GEAR_TOOTH_SENSOR,
        POWER_DISTRIBUTION_MODULE,
        PNEUMATICS_CTRL,
        MISC,
        IO_BREAKOUT;
    }
    
    /**
     * API class ID calculation from STATUS/CONTROL state and index.
     */
    protected static enum APIClass {
        STATUS(20), CTRL(10);

        private final int code;
        private APIClass(int code) { this.code = code; }

        public int get(int index) {
            return ((code << 4) + index);
        }
    }

    protected int canID;
    protected int deviceHandle;

    /**
     * Create some CAN device
     * @param canID
     */
    protected CANDevice(int canID, DeviceType type) {
        this.canID = canID; 
        this.deviceHandle = CANAPIJNI.initializeCAN(THRIFTY_VENDOR_ID, canID,
            type.ordinal());
    }

    /**
     * Returns the CAN ID the device was initialized with.
     * @return The device's CAN ID.
     */
    public int getID() { return canID; }

    protected int reciveStatusFrame(int index, int startByte, int endByte) {
        final CANData dataBuffer = new CANData();
        CANAPIJNI.readCANPacketLatest(
            deviceHandle,
            APIClass.STATUS.get(index),
            dataBuffer
        );

        int motorData = 0;
        for (int i = startByte; i <= endByte; i++)
            motorData += (dataBuffer.data[i] & 0xFF) << ((endByte - i) * 8);
        return motorData;
    }
}
