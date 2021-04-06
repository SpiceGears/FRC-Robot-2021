// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Veribles {

    private static Veribles instance;

    public boolean isIntakeOpen = false;
    public boolean isAutonomusEnabled = false;

    private Veribles(){
    }

    public static Veribles getInstance(){
        if (instance == null){
            instance = new Veribles();
        }
        return instance;
    }
}
