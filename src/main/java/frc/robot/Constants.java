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
    
    //Talon
    public static class Talon{
        public static final int kLongCANTimeoutMs = 100;
        public static final double kLongCANTimeoutSec = 0.01;
        public static final double kDriveVoltageRampRate = 0.0;
    
        public static final double kSlotIdx = 0;
        public static final int kPIDLoopIdx = 0;
        public static final int kTimeoutMs = 30;
    }
    
    //Encoder
    public static class Encoder{
        public static final double encoderTicksPerRotation = 4096;
    }

    //LimeLight
    public static class LimeLight{
        public static final Number pipeline = 0;
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
        public static final double kIDriveTrainTurnToAngle = 0.1225 * 0.6;
        public static final double kDDriveTrainTurnToAngle = 0.015 * 0.6;
        
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
        public static final double kPAimToAngle = 0.18 * 1;
        public static final double kIAimToAngle = 0.0 * 1;
        public static final double kDAimToAngle = 0.015 * 1;
        
        public static final int shooterRPMsetPoint = 2500;
        public static final double kFlywheelMomentOfInertia = 0.0001;
        public static final int minRPStoShoot = 50;
        public static final double kFlywheelGearing = 1.0;
    }

    //Aiming
    public static class Aiming{
        public static final double maxUpAimingPotentiometerValue = 0.25; // smaller value - aims higher
        public static final double maxDownAimingPotentiometerValue = 0.48; // bigger value - aims lower

        public static final double maxUpAimigSpeed = -0.3;
        public static final double maxDownAimigSpeed = 0.6;
        
        public static final double maxUpManualAimigSpeed = -0.4;
        public static final double maxDownManualAimigSpeed = 0.6;
    }

    //Transpoter
    public static class Transpoter{
        public static final double transporterMotorSpeednWhenBallIsInIntake = 0.42;
        public static final double transporterMotorSpeednWhenItsEmptying= 0.42;
    }

    //Intake
    public static class Intake{
        public static final double minUltrasonicDistanceToDelectBall = 13.0;
        public static final double maxUltrasonicDistanceToDelectBall = 26.0;

        public static final long solenoidStateSwitchDelay = 300;

        public static final double maxIntakeMotorSpeed = 0.35;
    }

    //Joystick
    public static class Joysticks{
        //Drive
        public static final double kDriverJoystickDeadzone = 0.12;
        public static final double driveTurnDivide = 1.17;

        //Shooter
        public static final int kTurnToAngleButton = 7;
        public static final int kAimToAngleButton = 2;
        //public static final int kAimUpButton = 3;
        //public static final int kAimDownButton = 1;
        public static final int kBallsoutButton = 4;

        //Intake
        public static final int kIntakeCloseButton = 9;
        public static final int kIntakeOpenButton = 10;
        public static final int kIntakeRotateButton = 6;
        
        //shooter
        public static final int kShooterShooting = 3;
    }
    
    //LED's
    public static class LEDs{

        public static final int kLedLength = 58;
    }

}
