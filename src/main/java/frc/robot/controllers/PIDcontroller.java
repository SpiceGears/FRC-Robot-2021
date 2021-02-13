// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controllers;

/** Add your docs here. */
public class PIDcontroller {
     public double calculatePID(double kP, double error, double kI, double errorSum, double kD, double deltaError, double deltaTime){
          return kP * error +  kI * errorSum + kD * (deltaError / deltaTime);
     }
}
