// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Aiming;

public class AimDown extends CommandBase {

  Aiming aiming;

  /** Aims higher
   * 
   * @param aiming aiming subsustem that command use.
  */
  public AimDown(Aiming _aiming) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_aiming);
    aiming = _aiming;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    aiming.set(Constants.Aiming.maxDownManualAimigSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    aiming.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
