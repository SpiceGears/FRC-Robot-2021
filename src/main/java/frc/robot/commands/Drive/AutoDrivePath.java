// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class AutoDrivePath extends CommandBase {
  /** Creates a new AutoDrivePath. */
  private DriveTrain m_robotDrive;
  private String m_flieName;
  Trajectory trajectory;
  RamseteCommand ramseteCommand;

  public AutoDrivePath(DriveTrain driveTrain, String fileName) {
    m_robotDrive = driveTrain;
    m_flieName = fileName;
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    String trajectoryJSON = "paths/output/" + m_flieName + ".wpilib.json";//"paths/prosto3m.wpilib.json";
    trajectory = new Trajectory();
    try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    ramseteCommand =
    new RamseteCommand(
        trajectory,
        m_robotDrive::getPose,
        new RamseteController(
            Constants.DriveTrain.Autonomus.kRamseteB, 
            Constants.DriveTrain.Autonomus.kRamseteZeta),
        new SimpleMotorFeedforward(
            Constants.DriveTrain.Autonomus.ksVolts,
            Constants.DriveTrain.Autonomus.kvVoltSecondsPerMeter,
            Constants.DriveTrain.Autonomus.kaVoltSecondsSquaredPerMeter),
        Constants.DriveTrain.Autonomus.kDriveKinematics,
        m_robotDrive::getWheelSpeeds,
        new PIDController(
            Constants.DriveTrain.kPDriveTrainLeft, 
            Constants.DriveTrain.kIDriveTrainLeft,
            Constants.DriveTrain.kDDriveTrainLeft),
        new PIDController(
            Constants.DriveTrain.kPDriveTrainRight, 
            Constants.DriveTrain.kIDriveTrainRight,
            Constants.DriveTrain.kDDriveTrainRight),
        // RamseteCommand passes volts to the callback
        m_robotDrive::driveDriveTrainByVoltage,
        m_robotDrive);

        m_robotDrive.resetOdometry(trajectory.getInitialPose());

        ramseteCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_robotDrive.driveDriveTrainByVoltage(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ramseteCommand.isFinished();
  }
}


/*abstract


    // Trajectory trajectory =
    // TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(new Translation2d(0.7, 0.7), new Translation2d(1.4, -0.7)),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(2, 0, new Rotation2d(0)),
    //     // Pass config
    //     config);



        RamseteCommand ramseteCommand =
            new RamseteCommand(
                trajectory,
                m_robotDrive::getPose,
                new RamseteController(
                    Constants.DriveTrain.Autonomus.kRamseteB, 
                    Constants.DriveTrain.Autonomus.kRamseteZeta),
                new SimpleMotorFeedforward(
                    Constants.DriveTrain.Autonomus.ksVolts,
                    Constants.DriveTrain.Autonomus.kvVoltSecondsPerMeter,
                    Constants.DriveTrain.Autonomus.kaVoltSecondsSquaredPerMeter),
                Constants.DriveTrain.Autonomus.kDriveKinematics,
                m_robotDrive::getWheelSpeeds,
                new PIDController(
                    Constants.DriveTrain.kPDriveTrainLeft, 
                    Constants.DriveTrain.kIDriveTrainLeft,
                    Constants.DriveTrain.kDDriveTrainLeft),
                new PIDController(
                    Constants.DriveTrain.kPDriveTrainRight, 
                    Constants.DriveTrain.kIDriveTrainRight,
                    Constants.DriveTrain.kDDriveTrainRight),
                // RamseteCommand passes volts to the callback
                m_robotDrive::driveDriveTrainByVoltage,
                m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetOdometry(trajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> m_robotDrive.driveDriveTrainByVoltage(0, 0));

*/