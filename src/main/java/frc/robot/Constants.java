// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
    //Talon
    public static final int kLongCANTimeoutMs = 100;
    public static final double kLongCANTimeoutSec = 0.01;
    public static final double kDriveVoltageRampRate = 0.0;

    public static final int kSlotIdx = 0;
    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 30;

    //Drive Train
    public static final double kPDriveTrainLeft = 0.005;
    public static final double kIDriveTrainLeft = 0.0;
    public static final double kDDriveTrainLeft = 0.0;

    public static final double kPDriveTrainRight = kPDriveTrainLeft;
    public static final double kIDriveTrainRight = kIDriveTrainLeft;
    public static final double kDDriveTrainRight = kDDriveTrainLeft;

    //Joystick
    public static final double kDriverJoystickDeadzone = 0.12;
}
