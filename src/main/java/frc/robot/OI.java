// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

/** Add your docs here. */
public class OI {

    Constants constants;

    public Joystick driverJoy;

    public OI() {
        constants = new Constants();

        driverJoy = new Joystick(0);
    }

    public Joystick getDriverJoy() {
        return driverJoy;
    }
}
