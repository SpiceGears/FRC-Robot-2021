package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimeLight extends SubsystemBase {
  public final static int kTargetPipeline = 0;
  public final static int kBallPipeline = 1;

  private static final int verticalPixelsFOV = 320;
  // private static final int horizontalPixelsFOV = 240;
  private static final double targetSizeMeters = 0.43;

  private static final double verticalViewDegrees = 59.6;
  // private static final double horizontalViewDegrees = 49.7;

  NetworkTable mNetworkTable;
  NetworkTableEntry isTarget;
  NetworkTableEntry xOffset;
  NetworkTableEntry yOffset;
  NetworkTableEntry limelightLEDMode;

  NetworkTableEntry pipeline;

  public LimeLight(){
    mNetworkTable = NetworkTableInstance.getDefault().getTable("limelight");

    limelightLEDMode = mNetworkTable.getEntry("ledMode");
    isTarget = mNetworkTable.getEntry("tv");
    xOffset = mNetworkTable.getEntry("tx");
    yOffset = mNetworkTable.getEntry("ty");

    pipeline = mNetworkTable.getEntry("pipeline");
    pipeline.setNumber(Constants.LimeLight.pipeline);
  }

  @Override
  public void periodic() {
    update();
    logs();
  }

  public double getDistance(){
    return (getFOVinMeters() * verticalPixelsFOV)/(2 * getTargetWidthInPixels()*
              Math.tan(Math.toRadians(getHalfFOVinDegrees())));
  }

  /**
   * Sets the mode of limelight LED's where:
   * 
   * 0 - use the LED Mode set in the current pipeline
   * 1 - force off
   * 2 - force blink
   * 3 - 	force on
   * 
   * @param id state of the led lights
   */
  public void setLEDMode(int id){
    limelightLEDMode.setNumber(id);
  }

  /**
   * 
   * @return x offset of the target (horizontal).
   */
  public synchronized double getXOffset() {
    double x = xOffset.getDouble(0);
    return x;
  }
  
  public double getTargetWidthInDegrees(){
    return verticalViewDegrees - 2 * Math.abs(getXOffset());
  }

  public double getHalfFOVinDegrees(){
    return (verticalViewDegrees / 2) + (getTargetWidthInDegrees()/2);
  }

  public double getFOVinMeters(){
    return (verticalPixelsFOV * targetSizeMeters) / getTargetWidthInPixels();
  }

  public double getTargetWidthInPixels(){
    return (getTargetWidthInDegrees() / 100) * verticalPixelsFOV;
  }

  /**
   * 
   * @return y offset of the target (vertical).
   */
  public synchronized double getYOffset(){
    double y = yOffset.getDouble(0);
    return y;
  }

  public synchronized boolean isTargetVisible() {
    boolean isVisible = false;
    if(isTarget.getDouble(0) != 0){
      isVisible = true;
    }
    return isVisible;
  }

  public void changePipeline(int pipe){
    pipeline.setNumber(pipe);
  }

  public void logs(){
    SmartDashboard.putNumber("limelight_xOffset", getXOffset());
    SmartDashboard.putNumber("limelight_yOffset", getYOffset());
    SmartDashboard.putBoolean("limelight_isTarget", isTargetVisible());
    SmartDashboard.putNumber("limelight_DistanceToTarget", getDistance());
  }

  public void update(){
  }
}