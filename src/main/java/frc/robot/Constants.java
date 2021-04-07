// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


    //Shooter

    public static class Shooter {
        public static final int shooterRPMsetPoint = 900;
        public static final int minRPStoShoot = 50;
    }


    
    //Talon

    public static class Talon{
        public static final int kLongCANTimeoutMs = 100;
        public static final double kLongCANTimeoutSec = 0.01;
        public static final double kDriveVoltageRampRate = 0.0;
    
        public static final double kSlotIdx = 0;
        public static final int kPIDLoopIdx = 0;
        public static final int kTimeoutMs = 30;
    }
    
    //Drive Train
    public static class DriveTrain{
        public static final double kPDriveTrainLeft = 2.0; // 2.42; //0.04; (2.42 - tuner)
        public static final double kIDriveTrainLeft = 0.0;
        public static final double kDDriveTrainLeft = 0.0;
    
        public static final double kPDriveTrainRight = kPDriveTrainLeft;
        public static final double kIDriveTrainRight = kIDriveTrainLeft;
        public static final double kDDriveTrainRight = kDDriveTrainLeft;

        public static final double wheelLenght = 0.18; //0.2032; //wheelLenght

        public static final double kTurnToleranceDeg = 0.5;
        public static final double kTurnRateToleranceDegPerS = 8;

        public static final double kPDriveTrainTurnToAngle = 0.1 * 0.6;
        public static final double kIDriveTrainTurnToAngle = 0.0015 * 0.6;
        public static final double kDDriveTrainTurnToAngle = 0.01 * 0.6;
        
        //Drive Train Autonomus
        public static class Autonomus{
            public static final double ksVolts = 1.31;//1.02;
            public static final double kvVoltSecondsPerMeter = 2.39;// 1.39;
            public static final double kTrackwidthMeters = 0.5;
            public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
            public static final double kaVoltSecondsSquaredPerMeter = 0.326; //0.383;
            
            public static final double kMaxSpeedMetersPerSecond = 0.5;
            public static final double kMaxAccelerationMetersPerSecondSquared = 0.2;
            
            public static double kRamseteB = 2.07 * 4; //8.0;
            public static double kRamseteZeta = 0.7 * 0.2;//*0.1;
        }
    }

    //Shooter
    public static class Shooter{
        public static final double kPAimToAngle = 0.05 * 1;
        public static final double kIAimToAngle = 0.0 * 1;
        public static final double kDAimToAngle = 0.0 * 1;
    }

    //Joystick
    public static class Joysticks{
        //Drive
        public static final double kDriverJoystickDeadzone = 0.12;
        public static final double driveTurnDivide = 1.17;

        //Shooter
        public static final int kTurnToAngleButton = 1;
        public static final int kAimToAngleButton = 2;
        public static final int kAimUpButton = 6;
        public static final int kAimDownButton = 5;
        public static final int kBallsoutButton = 4;

        //Intake
        public static final int kIntakeCloseButton = 9;
        public static final int kIntakeOpenButton = 10;
        public static final int kIntakeRotateButton = 6;
        
        //shooter
        public static final int kShooterShooting = 3;
    }


}
