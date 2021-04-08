// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PortMap;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  WPI_TalonSRX lMaster, rMaster;
  WPI_VictorSPX lSlave, rSlave;

  public Shooter() {
    configureMotors();
  }

  public void configureMotors() {
    lMaster = new WPI_TalonSRX(PortMap.Shooter.kLeftMasterShooter);
    rMaster = new WPI_TalonSRX(PortMap.Shooter.kRightMasterShooter);
    lSlave = new WPI_VictorSPX(PortMap.Shooter.kLeftSlaveShooter);
    rSlave = new WPI_VictorSPX(PortMap.Shooter.kRightSlaveShooter);

    lSlave.follow(lMaster);
    rSlave.follow(rMaster);
  }

  public void shooterRotate(double x) {
    rMaster.set(x);
    lMaster.set(x);
  }

  public void setRightShooterVoltage(double x) {
    rMaster.setVoltage(x);
  }

  public void setLeftShooterVoltage(double x) {
    lMaster.setVoltage(x);
  }

  public void shooterStop() {
    rMaster.set(0);
    lMaster.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    logs();
  }
  
  public double getLeftRPS(){
    return(lMaster.getSelectedSensorVelocity() / Constants.Encoder.encoderTicksPerRotation) * 10;  // ticks per rotation - 4096
  }
  
  public double getRightRPS(){
    return(rMaster.getSelectedSensorVelocity() / Constants.Encoder.encoderTicksPerRotation) * -10; // ticks per rotation - 4096
  }
  
  private void logs(){
    SmartDashboard.putNumber("LeftRPS", getLeftRPS());
    SmartDashboard.putNumber("RightRPS", getRightRPS());
  }
}
