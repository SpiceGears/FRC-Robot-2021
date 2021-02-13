// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

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

  public teleopDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.stopDriveTrainMotors();

    robotVelocity = 0;
    robotTurn = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    joystickRun();
  }

  private void joystickRun(){
    if(Math.abs(Robot.oI.getDriverJoy().getRawAxis(1)) >= constants.kDriverJoystickDeadzone || Math.abs(Robot.oI.getDriverJoy().getRawAxis(1)) > constants.kDriverJoystickDeadzone){
      robotVelocity = Robot.oI.getDriverJoy().getRawAxis(1);
      robotTurn = Robot.oI.getDriverJoy().getRawAxis(4);
    }

    driveTrain.setSpeedDriveTrainPercentOutput(robotVelocity, robotVelocity, robotTurn);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
