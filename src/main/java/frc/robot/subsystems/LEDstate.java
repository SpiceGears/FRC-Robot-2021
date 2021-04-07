// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDstate extends SubsystemBase {
  private AddressableLED LEDS;
  private AddressableLEDBuffer LEDBuffer;
  private int m_rainbowFirstPixelHueL = 1;
  /** Creates a new LEDstate. */
  public LEDstate() {
    LEDS = new AddressableLED(2);
    LEDBuffer = new AddressableLEDBuffer(58);
    LEDS.setLength(LEDBuffer.getLength());
    LEDS.setData(LEDBuffer);
    LEDS.start();
  }

  @Override
  public void periodic() { 
    rainbow();
    LEDS.setData(LEDBuffer);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    periodic();
    LEDS.setData(LEDBuffer);
  }

  private void rainbow(){
    // For every pixel
    for (var i = 0; i < LEDBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHueL + (i * 180 / LEDBuffer.getLength())) % 180;
      // Set the value
      LEDBuffer.setHSV(i, hue, 255, 128);
      SmartDashboard.putNumber("hue", hue );
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHueL += 2;
    // Check bounds
    m_rainbowFirstPixelHueL %= 180;
    // This method will be called once per scheduler run
  }

  public void setRGB(int r, int g, int b){
    for (var i = 0; i < LEDBuffer.getLength(); i++) {
      LEDBuffer.setRGB(i, r, g, b);
    }
  }

  long oldTimeBlink = System.currentTimeMillis();
  Boolean flagBlink = false;
  
  public void blink(){ //polizei mode
    long nowTime = System.currentTimeMillis();

    if(nowTime - oldTimeBlink > 200){
      flagBlink = !flagBlink;
      oldTimeBlink = nowTime;
    }

    if(flagBlink) {
      setRGB(100,0,0);
    } else {
      setRGB(0,0,100);
    }
  } 
   
  private void epilepsy(){
    long nowTime = System.currentTimeMillis();

    if(nowTime - oldTimeBlink > 250){
      flagBlink = !flagBlink;
      oldTimeBlink = nowTime;
    }

    if(flagBlink){
      for (var i = 0; i < LEDBuffer.getLength(); i++) {
      LEDBuffer.setHSV(i, (int)(Math.random() *180), 255 ,255);
      }
    } else {
      for (var i = 0; i < LEDBuffer.getLength(); i++) {
      LEDBuffer.setHSV(i, (int)(Math.random() *180), 255, 255);
      }
    }

  }

  int index = 0;
  
  public void dot(){
    long nowTime = System.currentTimeMillis();

    if(nowTime - oldTimeBlink > 30){
      flagBlink = !flagBlink;
      oldTimeBlink = nowTime;
    }

    if(flagBlink){
      turnOFF();
      LEDBuffer.setRGB(index, 240, 0, 0);
      index++;
      if(index >= LEDBuffer.getLength()){
        index = 0;
      }
    }

  }

  public void turnOFF(){
    for (var i=0; i < LEDBuffer.getLength(); i++ ) {
      LEDBuffer.setRGB(i, 0, 0, 0);
    }
  }

  /*private void checkDriveTrain(){
    if(Robot.driveTrain.getRightSpeed() < 0.1){ for(var i = 0; i < 3; i++){
      LEDBuffer.setRGB(i, 240, 0, 0);
    }
  }else if(Robot.driveTrain.getRightSpeed() > 0.1){
    for(var i = 0; i <3; i++){
      LEDBuffer.setRGB(i, 0, 240, 0);
    }
  }else{
    for(var i = 0; i < 3; i++){
      LEDBuffer.setRGB(i, 0, 0, 0);
    }
  }
  if(Robot.driveTrain.getLeftSpeed() < -0.1){
    for(var i = 58; i < 58; i++){
      LEDBuffer.setRGB(i, 240, 0, 0);
    }
  }else if(Robot.driveTrain.getLeftSpeed() > 0.1){
    for(var i = 58; i < 58; i++){
      LEDBuffer.setRGB(i, 0, 240, 0);
  }else{
    for(var i = 58; i <58; i++){
      LEDBuffer.setRGB(i, 0, 0, 0);
    }
  } */


}