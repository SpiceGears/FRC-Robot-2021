// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDstate;
import frc.robot.subsystems.LimeLight;

public class TargetToLed extends CommandBase {
  LEDstate leDstate;
  LimeLight limeLight;
  /** Sets the colors of the LEDs depending on the current error. 
   * 
   * @param limeLight limeLight subsustem that command use.
   * @param leDstate leDstate subsustem that command use.
  */
  public TargetToLed(LEDstate leDstate, LimeLight limeLight) {
    addRequirements(leDstate, limeLight);
    this.leDstate = leDstate; 
    this.limeLight = limeLight; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(limeLight.isTargetVisible()){
      if(Math.abs(limeLight.getYOffset()) < 1.6 && Math.abs(limeLight.getXOffset()) < 1){
        leDstate.setRGB(0, 255, 0);
      }else if(Math.abs(limeLight.getXOffset()) < 1){
        leDstate.setRGB(0, 0, 255);
      }else if(Math.abs(limeLight.getYOffset()) < 1.6){
        leDstate.setRGB(0, 255, 255);
      }else{
        leDstate.setRGB(255, 0, 0);
      }
    }else{
      leDstate.rainbow();
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
