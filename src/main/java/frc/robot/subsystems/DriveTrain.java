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
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;                 
import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.controllers.PIDcontroller;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */

  final Constants constants;
  final PortMap portMap;
  final PIDcontroller pidController;

  public WPI_TalonSRX leftMasterDriveTrain, rightMasterDriveTrain;
  private WPI_VictorSPX leftSlaveDriveTrainFirst, leftSlaveDriveTrainSecond, rightSlaveDriveTrainFirst, rightSlaveDriveTrainSecond;

  public DriveTrain() {
    constants = new Constants();
    pidController = new PIDcontroller();
    portMap = new PortMap();
    // setDefaultCommand(new teleopDrive());
    configurateMotors();
  }

//   public void initDefaultCommand() {
//     // Set the default command for a subsystem here.
//     setDefaultCommand(new teleopDrive());
// }

  private void configurateMotors(){
    leftMasterDriveTrain = new WPI_TalonSRX(portMap.kLeftMasterDrive);
    leftSlaveDriveTrainFirst = new WPI_VictorSPX(portMap.kLeftSlaveBDrive);
    leftSlaveDriveTrainSecond = new WPI_VictorSPX(portMap.kLeftSlaveMDrive);

    rightMasterDriveTrain = new WPI_TalonSRX(portMap.kRightMasterDrive);
    rightSlaveDriveTrainFirst = new WPI_VictorSPX(portMap.kRightSlaveBDrive);
    rightSlaveDriveTrainSecond = new WPI_VictorSPX(portMap.kRightSlaveMDrive);

    configLeftMaster(leftMasterDriveTrain);
    configRightMaster(rightMasterDriveTrain);
    
    leftSlaveDriveTrainFirst.follow(leftMasterDriveTrain);
    leftSlaveDriveTrainSecond.follow(leftMasterDriveTrain);
    rightSlaveDriveTrainFirst.follow(rightMasterDriveTrain);
    rightSlaveDriveTrainSecond.follow(rightMasterDriveTrain);
  }

  private void configLeftMaster(WPI_TalonSRX talon){
    talon.configFactoryDefault();

    configureMaster(talon, false);
  
    talon.config_kP(Constants.kPIDLoopIdx, Constants.kPDriveTrainLeft, Constants.kTimeoutMs);
    talon.config_kI(Constants.kPIDLoopIdx, Constants.kIDriveTrainLeft, Constants.kTimeoutMs);
    talon.config_kD(Constants.kPIDLoopIdx, Constants.kDDriveTrainLeft, Constants.kTimeoutMs);

  }

  private void configRightMaster(WPI_TalonSRX talon){
    talon.configFactoryDefault();

    configureMaster(talon, false);

    talon.config_kP(Constants.kPIDLoopIdx, Constants.kPDriveTrainRight, Constants.kTimeoutMs);
    talon.config_kI(Constants.kPIDLoopIdx, Constants.kIDriveTrainRight, Constants.kTimeoutMs);
    talon.config_kD(Constants.kPIDLoopIdx, Constants.kDDriveTrainRight, Constants.kTimeoutMs);
  }

  private void configureMaster(WPI_TalonSRX talon, boolean invert){
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

  /**
   * Sets robot velocity based on percentage output.
   * ControlMode.Velocity
   * 
   * @param leftPercentageOutput left master robot speed percentage.
   * @param rightPercentageOutput right master robot speed percentage.
   * @param turn robot turn value.
   */

  public void setSpeedDriveTrainVelocityOutput(double leftPercentageOutput, double rightPercentageOutput, double turn){
    if(Math.abs(leftPercentageOutput) + turn >= 1){
      rightWheelOutputWithTurn = -rightPercentageOutput + turn * 2;
      leftWheelOutputWithTurn = leftPercentageOutput;
    }else if(Math.abs(rightPercentageOutput) + turn >= 1){
      rightWheelOutputWithTurn = leftPercentageOutput;
      leftWheelOutputWithTurn = -rightPercentageOutput + turn * 2;
    }else{
      rightWheelOutputWithTurn = leftPercentageOutput + turn;
      leftWheelOutputWithTurn = rightPercentageOutput - turn;
    }

    leftMasterDriveTrain.set(ControlMode.Velocity, leftWheelOutputWithTurn * 10000.0 * 4096 / 600);   // ustawia max 500 RPM

    // rightMasterDriveTrain.set(ControlMode.Velocity, 0);
    rightMasterDriveTrain.set(ControlMode.Velocity, -rightWheelOutputWithTurn * 10000.0 * 4096 / 600);// ustawia max 500 RPM
  }

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
    
    leftMasterDriveTrain.set(ControlMode.PercentOutput, leftWheelOutputWithTurn);
    rightMasterDriveTrain.set(ControlMode.PercentOutput, -rightWheelOutputWithTurn);
    // leftMasterDriveTrain.set(ControlMode.PercentOutput, pidController.calculatePID(kP, error, kI, errorSum, kD, deltaError, deltaTime));
    // rightMasterDriveTrain.set(ControlMode.PercentOutput, pidController.calculatePID(kP, error, kI, errorSum, kD, deltaError, deltaTime));
  }

  public void stopDriveTrainMotors(){
    leftMasterDriveTrain.set(ControlMode.PercentOutput, 0);
    rightMasterDriveTrain.set(ControlMode.PercentOutput, 0);
  }

  public void resetEncodersDriveTrain(){
    leftMasterDriveTrain.setSelectedSensorPosition(0.0);
    rightMasterDriveTrain.setSelectedSensorPosition(0.0);
  }

  /**
   * @return wheels speed in m/s
   */

  public double getLeftMasterVelocityMpS(){
    return getWheelCircuit() * (leftMasterDriveTrain.getSelectedSensorVelocity()/4096 * 10);  // ticki na obrót 4096
  }

   /**
   * @return wheels speed in m/s
   */
  public double getRightMasterVelocityMpS(){
    return -getWheelCircuit() * (rightMasterDriveTrain.getSelectedSensorVelocity()/4096 * 10);  // ticki na obrót 4096
  }

  public double getMaxLeftMasterVelocityTickPer100ms(){ // tick/100ms
    return leftMasterDriveTrain.getSelectedSensorVelocity();
  }

  public double getWheelCircuit(){
    return Math.PI * 0.2032;
  }

  /**
   * ---------------------------------------------
   * ----------------=============----------------
   * ---------------=  Autonomus  =---------------
   * ----------------=============----------------
   * ---------------------------------------------
   *  */

  private DifferentialDrive diffDrive;
  private AHRS  gyro;
  private DifferentialDriveOdometry odometry;
  

  public void autonomousInit(){
    resetEncodersDriveTrain();
    gyro = new AHRS(portMap.gyroPort);
    diffDrive = new DifferentialDrive(leftMasterDriveTrain, rightMasterDriveTrain);

    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
  }

  public void autonomusPerodoic(){
    odometry.update(gyro.getRotation2d(), leftMasterDriveTrain.getSelectedSensorPosition(),
        rightMasterDriveTrain.getSelectedSensorPosition());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftMasterVelocityMpS(), getRightMasterVelocityMpS());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncodersDriveTrain();
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  public void driveDriveTrainByVoltage(double leftVolts, double rightVolts){
    leftMasterDriveTrain.setVoltage(leftVolts);
    rightMasterDriveTrain.setVoltage(-rightVolts);
  }

  public double getAverageEncoderDistance() {
    return (getLeftDistanceMeters() + getRightMasterDistanceMeters()) / 2.0;
  }

  public void resetGyro(){
    gyro.reset();
  }

  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return -gyro.getRate();
  }

  /**
   * @return Left wheel distance in meters.
   */

  private double getLeftDistanceMeters(){
    return getWheelCircuit() * leftMasterDriveTrain.getSelectedSensorPosition()/4096;
  }

  /**
   * @return Right wheel distance in meters.
   */

  private double getRightMasterDistanceMeters(){
    return getWheelCircuit() * rightMasterDriveTrain.getSelectedSensorPosition()/4096;
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
