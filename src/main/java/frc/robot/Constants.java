// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.a05annex.util.Utl;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class CAN_Devices {
        public static final int
                RF_DRIVE = 1,
                RF_DIRECTION = 2,
                RF_CALIBRATION = 20,
                RR_DRIVE = 3,
                RR_DIRECTION = 4,
                RR_CALIBRATION = 21,
                LR_DRIVE = 5,
                LR_DIRECTION = 6,
                LR_CALIBRATION = 22,
                LF_DRIVE = 7,
                LF_DIRECTION = 8,
                LF_CALIBRATION = 23;
    }

    public static final class CalibrationOffset {
        public static final double
                RF = 5.22626,
                RR = 0.35889,
                LR = 2.44823,
                LF = 0.96947;
    }

    // length and width from center of the wheels, in m
    public static final double DRIVE_LENGTH = 0.5842;
    public static final double DRIVE_WIDTH = 0.5842;
    public static final double DRIVE_DIAGONAL = Utl.length(DRIVE_LENGTH, DRIVE_WIDTH);

    // drive encoder tics per radian of robot rotation when rotation is controlled by position rather than speed.
    public static final double DRIVE_POS_TICS_PER_RADIAN = 10.385;
    public static final double MAX_METERS_PER_SEC = 3.2; //TODO: may have changed for the programmer robot w/o weight


}
