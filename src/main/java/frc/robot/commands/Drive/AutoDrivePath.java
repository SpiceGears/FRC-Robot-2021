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
    private DriveTrain robotDrive;
    private String flieName;
    Trajectory trajectory;
    RamseteCommand ramseteCommand;
    
  /** Follows generated path.
   * 
   * @param fileName name of the generated path file in PathFinder apart from ".wpilib.json". 
   * @param driveTrain driveTrain subsustem that command use.
   */
  public AutoDrivePath(DriveTrain driveTrain, String fileName) {
    addRequirements(driveTrain);
    this.robotDrive = driveTrain;
    this.flieName = fileName;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    String trajectoryJSON = "paths/output/" + flieName + ".wpilib.json";//"paths/prosto3m.wpilib.json";
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
        robotDrive::getPose,
        new RamseteController(
            Constants.DriveTrain.Autonomus.kRamseteB, 
            Constants.DriveTrain.Autonomus.kRamseteZeta),
        new SimpleMotorFeedforward(
            Constants.DriveTrain.Autonomus.ksVolts,
            Constants.DriveTrain.Autonomus.kvVoltSecondsPerMeter,
            Constants.DriveTrain.Autonomus.kaVoltSecondsSquaredPerMeter),
        Constants.DriveTrain.Autonomus.kDriveKinematics,
        robotDrive::getWheelSpeeds,
        new PIDController(
            Constants.DriveTrain.kPDriveTrainLeft, 
            Constants.DriveTrain.kIDriveTrainLeft,
            Constants.DriveTrain.kDDriveTrainLeft),
        new PIDController(
            Constants.DriveTrain.kPDriveTrainRight, 
            Constants.DriveTrain.kIDriveTrainRight,
            Constants.DriveTrain.kDDriveTrainRight),
        // RamseteCommand passes volts to the callback
        robotDrive::driveDriveTrainByVoltage,
        robotDrive);

        robotDrive.resetOdometry(trajectory.getInitialPose());

        ramseteCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    robotDrive.driveDriveTrainByVoltage(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ramseteCommand.isFinished();
  }
}
