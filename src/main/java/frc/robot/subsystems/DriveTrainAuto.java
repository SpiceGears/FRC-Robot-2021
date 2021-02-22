// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.Robot;

import java.io.IOException;
import java.nio.file.Path;

/** Add your docs here. */
public class DriveTrainAuto {

    private final DriveTrain driveTrain = new DriveTrain();

    public Command getAutonomousCommand() {

        String trajectoryJSON = "paths/prosto.wpilib.json";
        Trajectory trajectory = new Trajectory();
        try {
          Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
          trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
          DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }

        // Create a voltage constraint to ensure we don't accelerate too fast
        // var autoVoltageConstraint =
        //     new DifferentialDriveVoltageConstraint(
        //         new SimpleMotorFeedforward(Constants.DriveTrain.Autonomus.ksVolts,
        //         Constants.DriveTrain.Autonomus.kvVoltSecondsPerMeter,
        //         Constants.DriveTrain.Autonomus.kaVoltSecondsSquaredPerMeter),
        //         Constants.DriveTrain.Autonomus.kDriveKinematics,
        //         10);
    
        // Create config for trajectory
        // TrajectoryConfig config =
        //     new TrajectoryConfig(Constants.DriveTrain.Autonomus.kMaxSpeedMetersPerSecond,
        //     Constants.DriveTrain.Autonomus.kMaxAccelerationMetersPerSecondSquared)
        //         // Add kinematics to ensure max speed is actually obeyed
        //         .setKinematics(Constants.DriveTrain.Autonomus.kDriveKinematics)
        //         // Apply the voltage constraint
        //         .addConstraint(autoVoltageConstraint);
    
        // // An example trajectory to follow.  All units in meters.d
        //     new Pose2d(0, 0, new Rotation2d(0)),
        //     // Pass through these two interior waypoints, making an 's' curve path
        //     List.of(
        //         new Translation2d(1, 1),
        //         new Translation2d(2, -1)
        //     ),
        //     // End 3 meters straight ahead of where we started, facing forward
        //     new Pose2d(3, 0, new Rotation2d(0)),
        //     // Pass config
        //     config
        // );
    
        RamseteCommand ramseteCommand =
        new RamseteCommand(
            trajectory,
            driveTrain::getPose,
            new RamseteController(Constants.DriveTrain.Autonomus.kRamseteB, Constants.DriveTrain.Autonomus.kRamseteZeta),
            new SimpleMotorFeedforward(
                Constants.DriveTrain.Autonomus.ksVolts,
                Constants.DriveTrain.Autonomus.kvVoltSecondsPerMeter,
                Constants.DriveTrain.Autonomus.kaVoltSecondsSquaredPerMeter),
                Constants.DriveTrain.Autonomus.kDriveKinematics,
            Robot.driveTrain::getWheelSpeeds,
            new PIDController(Constants.DriveTrain.kPDriveTrainLeft, 0, 0),
            new PIDController(Constants.DriveTrain.kPDriveTrainRight, 0, 0),
            // RamseteCommand passes volts to the callback
            driveTrain::driveDriveTrainByVoltage,
            driveTrain);
    
        // Reset odometry to the starting pose of the trajectory.
        Robot.driveTrain.resetOdometry(trajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> driveTrain.driveDriveTrainByVoltage(0, 0));
    }

}
