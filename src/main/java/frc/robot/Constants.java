// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.a05annex.util.Utl;

import static org.a05annex.util.Utl.TWO_PI;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // This is the maximum velocity read by the encoders being used to control the drive speed. The actual
    // maximum is closer to 5700, but we have scaled that down a bit to account for friction, drag, lower
    // battery voltage, etc.
    public static final double MAX_DRIVE_VELOCITY = 5000;

    // length and width from center of the wheels, in m
    public static final double DRIVE_LENGTH = 0.590;
    public static final double DRIVE_WIDTH = 0.590;
    public static final double DRIVE_DIAGONAL = Utl.length(DRIVE_LENGTH, DRIVE_WIDTH);
    public static final double DRIVE_RADIUS = DRIVE_DIAGONAL / 2.0;
    // 18 motor revolutions to 1 spin of the drive wheel
    public static final double RADIANS_TO_SPIN_ENCODER = 18.0 / TWO_PI;

    // maximum linear speed and rotational speed. What we are really interested in knowing is how the robot
    // can travel or turn in one command cycle at full speed because for path following we need to know how
    // big to make the increments along the path, and need a pretty good estimate of where the robot is for
    // making course corrections.
    public static final double MAX_METERS_PER_SEC = 3.2;
    public static final double MAX_RADIANS_PER_SEC = MAX_METERS_PER_SEC / DRIVE_RADIUS;

    // PID values for the spin spark motor controller PID loop
    public static double SPIN_kP = 0.25;
    public static double SPIN_kI = 0.0;

    // PID values for the drive spark motor controller speed PID loop
    public static double DRIVE_kP = 0.00003;
    public static double DRIVE_kI = 0.000002;
    public static double DRIVE_kFF = 0.000174;
    public static double DRIVE_IZONE = 200.0;

    // PID values for the drive spark motor controller position PID loop
    public static double DRIVE_POS_kP = 0.13;
    public static double DRIVE_POS_kI = 0.0;

    public static double RAMP_UP_INC = 0.25;

    // drive encoder tics per radian of robot rotation when rotation is controlled by position rather than speed.
    public static double DRIVE_POS_TICS_PER_RADIAN = 10.385;


}
