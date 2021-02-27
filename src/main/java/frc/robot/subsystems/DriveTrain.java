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

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;                 
import frc.robot.PortMap;
import frc.robot.controllers.PIDcontroller;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */

  final PortMap portMap;
  final PIDcontroller pidController;

  private DifferentialDrive diffDrive;
  private AHRS gyro;

  public WPI_TalonSRX leftMasterDriveTrain, rightMasterDriveTrain;
  private WPI_VictorSPX leftSlaveDriveTrainFirst, leftSlaveDriveTrainSecond, rightSlaveDriveTrainFirst, rightSlaveDriveTrainSecond;

  public DriveTrain() {
    pidController = new PIDcontroller();
    portMap = new PortMap();
    
    gyro = new AHRS(PortMap.DriveTrain.gyroPort);

    configurateMotors();

    diffDrive = new DifferentialDrive(leftMasterDriveTrain, rightMasterDriveTrain);
    
    autonomousInit();
  }

  private void configurateMotors(){
    leftMasterDriveTrain = new WPI_TalonSRX(PortMap.DriveTrain.kLeftMasterDrive);
    leftSlaveDriveTrainFirst = new WPI_VictorSPX(PortMap.DriveTrain.kLeftSlaveBDrive);
    leftSlaveDriveTrainSecond = new WPI_VictorSPX(PortMap.DriveTrain.kLeftSlaveMDrive);

    rightMasterDriveTrain = new WPI_TalonSRX(PortMap.DriveTrain.kRightMasterDrive);
    rightSlaveDriveTrainFirst = new WPI_VictorSPX(PortMap.DriveTrain.kRightSlaveBDrive);
    rightSlaveDriveTrainSecond = new WPI_VictorSPX(PortMap.DriveTrain.kRightSlaveMDrive);

    configLeftMaster(leftMasterDriveTrain);
    configRightMaster(rightMasterDriveTrain);
    
    leftMasterDriveTrain.setInverted(true);
    leftSlaveDriveTrainFirst.setInverted(true);
    leftSlaveDriveTrainSecond.setInverted(true);

    leftSlaveDriveTrainFirst.follow(leftMasterDriveTrain);
    leftSlaveDriveTrainSecond.follow(leftMasterDriveTrain);
    rightSlaveDriveTrainFirst.follow(rightMasterDriveTrain);
    rightSlaveDriveTrainSecond.follow(rightMasterDriveTrain);

    
  }

  private void configLeftMaster(WPI_TalonSRX talon){
    talon.configFactoryDefault();

    configureMaster(talon, false);
  
    talon.config_kP(Constants.Talon.kPIDLoopIdx, Constants.DriveTrain.kPDriveTrainLeft, Constants.Talon.kTimeoutMs);
    talon.config_kI(Constants.Talon.kPIDLoopIdx, Constants.DriveTrain.kIDriveTrainLeft, Constants.Talon.kTimeoutMs);
    talon.config_kD(Constants.Talon.kPIDLoopIdx, Constants.DriveTrain.kDDriveTrainLeft, Constants.Talon.kTimeoutMs);
    
    talon.enableVoltageCompensation(false);
  }

  private void configRightMaster(WPI_TalonSRX talon){
    talon.configFactoryDefault();

    configureMaster(talon, false);

    talon.config_kP(Constants.Talon.kPIDLoopIdx, Constants.DriveTrain.kPDriveTrainRight, Constants.Talon.kTimeoutMs);
    talon.config_kI(Constants.Talon.kPIDLoopIdx, Constants.DriveTrain.kIDriveTrainRight, Constants.Talon.kTimeoutMs);
    talon.config_kD(Constants.Talon.kPIDLoopIdx, Constants.DriveTrain.kDDriveTrainRight, Constants.Talon.kTimeoutMs);
  
    talon.enableVoltageCompensation(false);
  }

  private void configureMaster(WPI_TalonSRX talon, boolean invert){
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
    final ErrorCode sensorPresent = talon.configSelectedFeedbackSensor(FeedbackDevice
            .CTRE_MagEncoder_Relative, 0, 100); //primary closed-loop, 100 ms timeout
    if (sensorPresent != ErrorCode.OK) {
        DriverStation.reportError("Could not detect " + (invert ? "right" : "left") + " encoder: " + sensorPresent, false);
    }

    // talon.setInverted(invert);
    talon.setSensorPhase(invert);
    talon.configVoltageCompSaturation(12.0, Constants.Talon.kLongCANTimeoutMs);
    talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.Talon.kLongCANTimeoutMs); 
    talon.configVelocityMeasurementWindow(1, Constants.Talon.kLongCANTimeoutMs); 
    talon.configClosedloopRamp(Constants.Talon.kDriveVoltageRampRate, Constants.Talon.kLongCANTimeoutMs); 
    talon.configNeutralDeadband(0.04, 0);
  }

  /**
   * 
   * @param leftPercentageOutput left master robot speed percentage.
   * @param rightPercentageOutput right master robot speed percentage.
   * @param turn robot turn value.
   * @return calcuteted wheel speed with turn where [0] is left wheel and [1] is right wheel
   */
  private double[] getLeftWheelOutputWithTurn(double leftPercentageOutput, double rightPercentageOutput, double turn){
    double[] wheelOutputWithTurn = new double[2];

    if(Math.abs(leftPercentageOutput) + turn >= 1){
      wheelOutputWithTurn[0] = leftPercentageOutput;
      wheelOutputWithTurn[1] = -rightPercentageOutput + turn * 2;
    }else if(Math.abs(rightPercentageOutput) + turn >= 1){
      wheelOutputWithTurn[0] = leftPercentageOutput;
      wheelOutputWithTurn[1] = -rightPercentageOutput + turn * 2;
    }else{
      wheelOutputWithTurn[0] = leftPercentageOutput + turn;
      wheelOutputWithTurn[1] = rightPercentageOutput - turn;
    }
    return wheelOutputWithTurn;
  }

  /**
   * Sets robot velocity based on percentage output.
   * ControlMode.Velocity
   * 
   * @param leftPercentageOutput left master robot speed percentage.
   * @param rightPercentageOutput right master robot speed percentage.
   * @param turn robot turn value.
   */

  public void setSpeedDriveTrainVelocityOutput(double leftPercentageOutput, double rightPercentageOutput, double turn){
    /**
     * param 2 =  PercentOutput * maxVelocity(RRM) * ticks / ?
     */
    leftMasterDriveTrain.set(ControlMode.Velocity, getLeftWheelOutputWithTurn(leftPercentageOutput, rightPercentageOutput, turn)[0] * 10000.0 * 4096 / 600);
    rightMasterDriveTrain.set(ControlMode.Velocity, getLeftWheelOutputWithTurn(leftPercentageOutput, rightPercentageOutput, turn)[1] * 10000.0 * 4096 / 600);
  }

  /**
   * Sets robot velocity based on percentage output.
   * ControlMode.Velocity
   * 
   * @param leftPercentageOutput left master robot speed percentage.
   * @param rightPercentageOutput right master robot speed percentage.
   * @param turn robot turn value.
   */

  public void setSpeedDriveTrainPercentOutput(double leftPercentageOutput, double rightPercentageOutput, double turn){
    // leftMasterDriveTrain.set(ControlMode.PercentOutput, getLeftWheelOutputWithTurn(leftPercentageOutput, rightPercentageOutput, turn)[0]);
    // rightMasterDriveTrain.set(ControlMode.PercentOutput, getLeftWheelOutputWithTurn(leftPercentageOutput, rightPercentageOutput, turn)[1]);
    leftMasterDriveTrain.set(ControlMode.PercentOutput, leftPercentageOutput - turn);
    rightMasterDriveTrain.set(ControlMode.PercentOutput, leftPercentageOutput + turn);
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
   * 
   * @return left wheel tick per 100ms.
   */
  public double getMaxLeftMasterVelocityTickPer100ms(){
    return leftMasterDriveTrain.getSelectedSensorVelocity();
  }

  /**
   * 
   * @return wheel circuit.
   */

  public double getWheelCircuit(){
    return Math.PI * Constants.DriveTrain.wheelLenght;
  }

  
  // ---------------------------------------------
  // ----------------=============----------------
  // ---------------=  Autonomus  =---------------
  // ----------------=============----------------
  // ---------------------------------------------


  // private Gyro gyro = new ADXRS450_Gyro();
  private DifferentialDriveOdometry odometry;
  // DriveTrainAuto driveTrainAuto;
  // int x = true?1:-1;
  int invert_gyro = -1;
  int invert_enc = 1;
  int invert_speed = 1;

  public void autonomousInit(){
    resetEncodersDriveTrain();
    resetGyro();

    odometry = new DifferentialDriveOdometry(getHeading());
    // driveTrainAuto = new DriveTrainAuto();

    // driveTrainAuto.getAutonomousCommand();
  }


  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftMasterVelocityMpS(), getRightMasterVelocityMpS());
  }

  public void resetOdometry(Pose2d pose) {   
    resetEncodersDriveTrain();
    odometry.resetPosition(pose, getHeading());
  }

  public void driveDriveTrainByVoltage(double leftVolts, double rightVolts){
    leftMasterDriveTrain.setVoltage(invert_speed*leftVolts);
    rightMasterDriveTrain.setVoltage(invert_speed*rightVolts);
    diffDrive.feed();
    SmartDashboard.putNumber("leftVolts", invert_speed*leftVolts);
    SmartDashboard.putNumber("rightVolts", invert_speed*rightVolts);
    // leftMasterDriveTrain.set(ControlMode.PercentOutput, leftVolts/12);
    // rightMasterDriveTrain.set(ControlMode.PercentOutput, rightVolts/12);

  }

  public double getAverageEncoderDistance() {
    return (getLeftDistanceMeters() + getRightDistanceMeters()) / 2.0;
  }

  public void resetGyro(){
    gyro.reset();
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(invert_gyro*gyro.getAngle());
  }

  // public double getTurnRate() {
  //   return -gyro.getRate();
  // }

  /**
   * @return Left wheel encoder distance in meters.
   */

  private double getLeftDistanceMeters(){
    return invert_enc*getWheelCircuit() * leftMasterDriveTrain.getSelectedSensorPosition()/4096;
  }

  /**
   * @return Right wheel encoder distance in meters.
   */

  private double getRightDistanceMeters(){
    return invert_enc*getWheelCircuit() * rightMasterDriveTrain.getSelectedSensorPosition()/4096;
  }

    /**
   * @return wheels speed in m/s
   */

  public double getLeftMasterVelocityMpS(){
    return invert_enc*getWheelCircuit() * (leftMasterDriveTrain.getSelectedSensorVelocity()/4096) * 10;  // ticks per rotation - 4096
  }

   /**
   * @return wheels speed in m/s
   */
  public double getRightMasterVelocityMpS(){
    return invert_enc*getWheelCircuit() * (rightMasterDriveTrain.getSelectedSensorVelocity()/4096) * 10;  // ticks per rotation - 4096
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    logToSmartDashboard();
    odometry.update(getHeading(), getLeftDistanceMeters(), getRightDistanceMeters());
  }

  public void logToSmartDashboard(){
    SmartDashboard.putNumber("getLeftDistanceMeters", getLeftDistanceMeters());
    SmartDashboard.putNumber("getRightDistanceMeters", getRightDistanceMeters());
    SmartDashboard.putNumber("getAverageEncoderDistance", getAverageEncoderDistance());
    SmartDashboard.putNumber("getHeading().getDegrees()", getHeading().getDegrees());
    SmartDashboard.putNumber("getLeftMasterVelocityMpS()", getLeftMasterVelocityMpS());
    SmartDashboard.putNumber("getRightMasterVelocityMpS()", getRightMasterVelocityMpS());
    SmartDashboard.putNumber("odometry.getPoseMeters()()", odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("odometry Rotation", odometry.getPoseMeters().getRotation().getDegrees());
  }
}
