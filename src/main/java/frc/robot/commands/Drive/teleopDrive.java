// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

public class teleopDrive extends CommandBase {
  /** Creates a new teleopDrive. */
  DriveTrain driveTrain = Robot.driveTrain;
  final Constants constants = new Constants();

  double robotVelocity;
  double robotTurn;
  double maxVelocity, currentVelocity, acceleratingTimeStart, acceleratingTime = 0;

  public teleopDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.stopDriveTrainMotors();

    currentVelocity = Robot.driveTrain.leftMasterDriveTrain.getSelectedSensorVelocity();

    robotVelocity = 0;
    robotTurn = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    getMaxLeftMasterVelocityTickPer100ms();
    logToSmartDashboardteleOpDrive();
    joystickRun();
  }

  private void joystickRun(){
    if(Math.abs(Robot.oI.getDriverJoy().getRawAxis(1)) >= constants.kDriverJoystickDeadzone || Math.abs(Robot.oI.getDriverJoy().getRawAxis(4)) > constants.kDriverJoystickDeadzone){
      robotVelocity = Robot.oI.getDriverJoy().getRawAxis(1);
      robotTurn = Robot.oI.getDriverJoy().getRawAxis(4);
      driveTrain.setSpeedDriveTrainPercentOutput(robotVelocity, robotVelocity, robotTurn / 4);
    }else {
      driveTrain.setSpeedDriveTrainPercentOutput(0, 0);
    }
  }

  private void getMaxLeftMasterVelocityTickPer100ms(){
    if(Robot.driveTrain.leftMasterDriveTrain.getSelectedSensorVelocity() == 0){
      acceleratingTimeStart = System.currentTimeMillis();
    }
    currentVelocity = Robot.driveTrain.leftMasterDriveTrain.getSelectedSensorVelocity();
    if (maxVelocity <= currentVelocity){
      maxVelocity = currentVelocity;
      acceleratingTime = (System.currentTimeMillis() - acceleratingTimeStart)/1000;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void logToSmartDashboardteleOpDrive(){
    SmartDashboard.putNumber("accelerating time", acceleratingTime);
    SmartDashboard.putNumber("maxVelocity tick/100ms", maxVelocity);
    SmartDashboard.putNumber("currentVelocity tick/100ms", currentVelocity);
  }
}
