// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.Transporter;

import java.util.Timer;
import java.util.TimerTask;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Veribles;
import frc.robot.subsystems.Transporter;

public class BallsOut extends InstantCommand {
  Transporter transporter; 

  /** Creates a new ShitTrashBalls. */
  public BallsOut(Transporter _transporter) {
    addRequirements(_transporter);
    transporter = _transporter;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(!Veribles.getInstance().isBallsOutSheduled){
      Veribles.getInstance().isBallsOutSheduled = true;
      transporter.transporterMotor.set(ControlMode.PercentOutput, 0.5);

      new Timer().schedule(new TimerTask(){
        public void run() {
          Veribles.getInstance().isBallsOutSheduled = false;
          transporter.transporterMotor.set(ControlMode.PercentOutput, 0.0);
        }
      }, 1000);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

}
