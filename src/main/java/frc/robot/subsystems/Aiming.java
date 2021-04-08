// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.PortMap;

public class Aiming extends PIDSubsystem {
  private WPI_VictorSPX aimMotor;
  private AnalogPotentiometer potentiometer;
  LimeLight limeLight;

  /** Creates a new ShooterPID. */
  public Aiming(LimeLight limeLight, double setpoint) {
    super(
        // The PIDController used by the subsystem
        new PIDController(Constants.Shooter.kPAimToAngle, Constants.Shooter.kIAimToAngle, Constants.Shooter.kDAimToAngle));

    this.limeLight = limeLight;
    aimMotor = new WPI_VictorSPX(PortMap.Aiming.kAimMotor);
    potentiometer = new AnalogPotentiometer(PortMap.Aiming.kPotentiometer);
    aimMotor.configFactoryDefault();

    setSetpoint(setpoint);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    set(output);
    SmartDashboard.putNumber("aim out", output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return limeLight.getYOffset();
  }

  /**
   * 
   * @return Returns position of the potentometer
   */
  public double getPotentometerPosition(){
    return potentiometer.get();
  }

  /**
   * Sets the output on the motors, it has protection against lifting or lowering out of range.
   * @param speed output on the motors.
   */
  public void set(double speed){
    speed *= -1;
    SmartDashboard.putNumber("speed", speed);
    if (potentiometer.get() > Constants.Aiming.maxDownAimingPotentiometerValue){
      // ogranicznik dolu
      if(speed < 0){
        speed = 0;
      }
    }else if(potentiometer.get() < Constants.Aiming.maxUpAimingPotentiometerValue){
      if(speed > 0){
        speed = 0;
      } 
    }

    if(speed < Constants.Aiming.maxUpAimigSpeed){
      speed = Constants.Aiming.maxUpAimigSpeed;
    }else if(speed > Constants.Aiming.maxDownAimigSpeed){
      speed = Constants.Aiming.maxDownAimigSpeed;
    }
    aimMotor.set(speed);
  }
}
