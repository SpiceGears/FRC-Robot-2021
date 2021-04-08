// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToAngle extends PIDCommand {
LimeLight limeLight;

  /** Turns robot to angle
   * 
   * @param setpoint setpoint at which the robot is striving.
   * @param limelight limelight subsustem that command use.
   * @param driveTrain driveTrain subsustem that command use.
   */
  public TurnToAngle(LimeLight limelight, double setpoint, DriveTrain driveTrain) {
    super(
        // The controller that the command will use
        new PIDController(Constants.DriveTrain.kPDriveTrainTurnToAngle, Constants.DriveTrain.kIDriveTrainTurnToAngle, Constants.DriveTrain.kDDriveTrainTurnToAngle),
        // This should return the measurement
        limelight::getXOffset,
        // This should return the setpoint (can also be a constant)
        setpoint,
        // This uses the output
        output -> {
          driveTrain.setSpeedDriveTrainPercentOutput(0, 0, output);
        });
        this.limeLight = limelight;
        limeLight.setLEDMode(3);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.

    // getController().enableContinuousInput(-180, 180);
    // getController()
    //   .setTolerance(Constants.DriveTrain.kTurnToleranceDeg, Constants.DriveTrain.kTurnRateToleranceDegPerS);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
