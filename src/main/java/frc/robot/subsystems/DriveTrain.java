// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;                 
import frc.robot.PortMap;
import frc.robot.controllers.PIDcontroller;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */

  final Constants constants;
  final PortMap portMap;
  final PIDcontroller pidController;

  private WPI_TalonSRX leftMasterDriveTrain, rightMasterDriveTrain;
  private WPI_VictorSPX leftSlaveDriveTrainFirst, leftSlaveDriveTrainSecond, rightSlaveDriveTrainFirst, rightSlaveDriveTrainSecond;

  public DriveTrain() {
    constants = new Constants();
    pidController = new PIDcontroller();
    portMap = new PortMap();

    configurateMotors();
  }

  private void configurateMotors(){
    leftMasterDriveTrain = new WPI_TalonSRX(portMap.kLeftMasterDrive);
    leftSlaveDriveTrainFirst = new WPI_VictorSPX(portMap.kLeftSlaveBDrive);
    leftSlaveDriveTrainSecond = new WPI_VictorSPX(portMap.kLeftSlaveMDrive);

    rightMasterDriveTrain = new WPI_TalonSRX(portMap.kRightMasterDrive);
    rightSlaveDriveTrainFirst = new WPI_VictorSPX(portMap.kRightSlaveBDrive);
    rightSlaveDriveTrainSecond = new WPI_VictorSPX(portMap.kRightSlaveMDrive);

    configureMaster(leftMasterDriveTrain, false);
    configureMaster(rightMasterDriveTrain, false);

    leftSlaveDriveTrainFirst.follow(leftMasterDriveTrain);
    leftSlaveDriveTrainSecond.follow(leftMasterDriveTrain);
    rightSlaveDriveTrainFirst.follow(rightMasterDriveTrain);
    rightSlaveDriveTrainSecond.follow(rightMasterDriveTrain);
  }

  public void configureMaster(WPI_TalonSRX talon, boolean invert){
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
    final ErrorCode sensorPresent = talon.configSelectedFeedbackSensor(FeedbackDevice
            .CTRE_MagEncoder_Relative, 0, 100); //primary closed-loop, 100 ms timeout
    if (sensorPresent != ErrorCode.OK) {
        DriverStation.reportError("Could not detect " + (invert ? "right" : "left") + " encoder: " + sensorPresent, false);
    }

    //talon.setInverted();
    talon.setSensorPhase(true);
    talon.enableVoltageCompensation(true);
    talon.configVoltageCompSaturation(12.0, constants.kLongCANTimeoutMs);
    talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, constants.kLongCANTimeoutMs); 
    talon.configVelocityMeasurementWindow(1, constants.kLongCANTimeoutMs); 
    talon.configClosedloopRamp(constants.kDriveVoltageRampRate, constants.kLongCANTimeoutMs); 
    talon.configNeutralDeadband(0.04, 0);
  }
 
  private double leftWheelOutputWithTurn = 0;
  private double rightWheelOutputWithTurn = 0;

  public void setSpeedDriveTrainPercentOutput(double leftPercentageOutput, double rightPercentageOutput, double turn){
    

    if(Math.abs(leftPercentageOutput) + turn >= 1){
      rightWheelOutputWithTurn = -rightPercentageOutput + turn * 2;
      leftWheelOutputWithTurn = leftPercentageOutput;
    }else if(Math.abs(rightPercentageOutput) + turn >= 1){
      rightWheelOutputWithTurn = leftPercentageOutput;
      leftWheelOutputWithTurn = -rightPercentageOutput + turn * 2;
    }else{
      rightWheelOutputWithTurn = leftPercentageOutput;
      leftWheelOutputWithTurn = rightPercentageOutput;
    }

    leftMasterDriveTrain.set(ControlMode.PercentOutput, pidController.calculatePID(kP, error, kI, errorSum, kD, deltaError, deltaTime));
    rightMasterDriveTrain.set(ControlMode.PercentOutput, pidController.calculatePID(kP, error, kI, errorSum, kD, deltaError, deltaTime));
  }

  public void stopDriveTrainMotors(){
    leftMasterDriveTrain.set(ControlMode.PercentOutput, 0);
    rightMasterDriveTrain.set(ControlMode.PercentOutput, 0);
  }

  public void resetEncodersDriveTrain(){
    leftMasterDriveTrain.setSelectedSensorPosition(0.0);
    rightMasterDriveTrain.setSelectedSensorPosition(0.0);
  }

  public double getLeftMasterVelocityMpS(){ // m/s
    return getWheelCircuit() * (leftMasterDriveTrain.getSelectedSensorVelocity()/4096 * 10);  // ticki na obrót 4096
  }

  public double getRightMasterVelocityMpS(){ // m/s
    return getWheelCircuit() * (rightMasterDriveTrain.getSelectedSensorVelocity()/4096 * 10);  // ticki na obrót 4096
  }

  public double getWheelCircuit(){
    return Math.PI * 0.20;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
