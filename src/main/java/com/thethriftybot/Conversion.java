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

import com.thethriftybot.ThriftyNova.EncoderType;

public final class Conversion {
    private double ratio;

    public Conversion(PositionUnit units, EncoderType encoder) { this.ratio = encoder.ticks / units.value; }
    public Conversion(VelocityUnit units, EncoderType encoder) { this.ratio = encoder.ticks / units.value; }

    public double toMotor(double value) { return value * ratio; }
    public double fromMotor(double value) { return value / ratio; }

    public static enum PositionUnit {
        RADIANS(Math.PI * 2),
        DEGREES(360),
        ROTATIONS(1);

        public final double value;
        private PositionUnit(double value) { this.value = value; }
    }

    public static enum VelocityUnit {
        RADIANS_PER_SEC(Math.PI * 2),
        DEGREES_PER_SEC(360),
        ROTATIONS_PER_SEC(1),
        ROTATIONS_PER_MIN(1/60f);

        public final double value;
        private VelocityUnit(double value) { this.value = value; }
    }
}