// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PortMap;
import frc.robot.Veribles;

import java.util.Timer;
import java.util.TimerTask;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  WPI_VictorSPX intakeMotor;

  DoubleSolenoid intakeSolenoid;
  Compressor compressor;


  public Intake() {
    intakeSolenoid = new DoubleSolenoid(PortMap.Intake.kIntakeSolenoidA,PortMap.Intake.kIntakeSolenoidB);
    intakeMotor = new WPI_VictorSPX(PortMap.Intake.kIntakeMotor);
    compressor = new Compressor(0);

    intakeSolenoid.set(Value.kForward);
    Veribles.getInstance().isIntakeOpen = false;
  }

  // Closes intake
  public void intakeClose() {
    Veribles.getInstance().isIntakeOpen = false;
    intakeSolenoid.set(Value.kForward);
    
    TimerTask task = new TimerTask() {
      public void run() {
        intakeSolenoid.set(Value.kOff);
      }
    };

    Timer timer = new Timer();
    timer.schedule(task, Constants.Intake.solenoidStateSwitchDelay);
  }

  // Opens Intake
  public void intakeOpen() {
    
    intakeSolenoid.set(Value.kReverse);
    Veribles.getInstance().isIntakeOpen = true;

    TimerTask task = new TimerTask() {
      public void run() {
        intakeSolenoid.set(Value.kOff);
      }
    };

    Timer timer = new Timer();
    timer.schedule(task, Constants.Intake.solenoidStateSwitchDelay);
    Veribles.getInstance().isIntakeOpen = true;
  }

  // Rotates Intake
  public void intakeRotate() {
    if (Veribles.getInstance().isIntakeOpen) {
      intakeMotor.set(Constants.Intake.maxIntakeMotorSpeed);
    } else {
      intakeMotor.set(0);
    }
  }

  // Stops Motors
  public void intakeStopMotor() {
    intakeMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(Veribles.getInstance().isAutonomusEnabled){
      intakeRotate();
    }
  }
}
