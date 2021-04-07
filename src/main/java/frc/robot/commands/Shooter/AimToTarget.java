// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Aiming;
import frc.robot.subsystems.LimeLight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimToTarget extends InstantCommand {
  private Aiming aiming;
  private LimeLight limeLight;
  
  public AimToTarget(Aiming _aiming, LimeLight _limeLight) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_aiming);
    aiming = _aiming;
    limeLight = _limeLight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limeLight.setMode(3);
    aiming.setSetpoint(0);
  }
}
