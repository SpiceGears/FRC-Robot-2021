// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.Ultrasonic.Unit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;

public class Transporter extends SubsystemBase {
  /** Creates a new Transporter. */

  private WPI_VictorSPX transportMotor;

  private Ultrasonic ultrasonic;
  private DigitalInput photoElectric;
  private DigitalInput limitSwitch;

  public Transporter() {
    transportMotor = new WPI_VictorSPX(PortMap.Transporter.kTransportMotor);
    photoElectric = new DigitalInput(PortMap.Transporter.kPhotoSensor);
    limitSwitch = new DigitalInput(PortMap.Transporter.kLimitSwitch);
    ultrasonic = new Ultrasonic(3, 4, Unit.kMillimeters);

    transportMotor.configFactoryDefault();

    ultrasonic.setAutomaticMode(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    
  }

  private boolean isBallInIntake(){
    double distance = ultrasonic.getRangeMM();

    if(distance > 50*10 || distance < 5*10){

    }
    
    return true;
  }
}
