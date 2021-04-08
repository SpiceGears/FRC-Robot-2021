// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.Transporter;

import java.util.Timer;
import java.util.TimerTask;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Veribles;
import frc.robot.subsystems.Transporter;

public class BallsOut5Seconds extends InstantCommand {
  Transporter transporter; 

  /**Spins transpoter for 5 seconds. 
   * 
   * @param transporter transpoter subsustem that command use.
  */
  public BallsOut5Seconds(Transporter transporter) {
    addRequirements(transporter);
    this.transporter = transporter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(!Veribles.getInstance().isBallsOutSheduled){
      Veribles.getInstance().isBallsOutSheduled = true;
      transporter.transporterMotor.set(ControlMode.PercentOutput, Constants.Transpoter.transporterMotorSpeednWhenItsEmptying);

      new Timer().schedule(new TimerTask(){
        public void run() {
          Veribles.getInstance().isBallsOutSheduled = false;
          transporter.transporterMotor.set(ControlMode.PercentOutput, 0.0);
        }
      }, 5000);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
}
