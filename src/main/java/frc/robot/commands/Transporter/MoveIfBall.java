// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.Transporter;

import java.util.Timer;
import java.util.TimerTask;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Veribles;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transporter;

public class MoveIfBall extends CommandBase {
  Transporter transporter;
  Intake intake;

  boolean isRotating = false;

  /**Moves balls if any is in intake. 
   * 
   * @param transporter transpoter subsustem that command use.
   * @param intake intake subsustem that command use.
   */
  public MoveIfBall(Transporter transporter, Intake intake) {
    this.transporter = transporter;
    this.intake = intake;
    addRequirements(transporter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (transporter.isBallIntake() && Veribles.getInstance().isIntakeOpen && !transporter.isTransporterFull()){
      transporter.transporterMotor.set(ControlMode.PercentOutput, Constants.Transpoter.transporterMotorSpeednWhenBallIsInIntake);
      intake.intakeRotate();
      isRotating = true;
    }else{
      if(isRotating){
        new Timer().schedule(new TimerTask(){
          public void run() {
            isRotating = false;
            transporter.transporterMotor.set(ControlMode.PercentOutput, 0.0);
            intake.intakeStopMotor();
          }
        }, 100);
      }
      if(transporter.isTransporterFull() && !Veribles.getInstance().isBallsOutSheduled){
        transporter.transporterMotor.set(ControlMode.PercentOutput, 0.0);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
