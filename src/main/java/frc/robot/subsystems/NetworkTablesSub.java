// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class NetworkTablesSub extends SubsystemBase {

  NetworkTable mNetworkTable;
  NetworkTableEntry powerCellsArray;

  /** Creates a new NetworkTablesSub. */
  public NetworkTablesSub() {

    mNetworkTable = NetworkTableInstance.getDefault().getTable("DetectedObjects");
    powerCellsArray = mNetworkTable.getEntry("data");
  }

  public String getPowerCelsPlace(){
    String result = powerCellsArray.getString("no_data");

   

    SmartDashboard.putString("powerCells", result);

    return result;
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getPowerCelsPlace();
  }
}
