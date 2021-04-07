/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;

/**
 * Add your docs here.
 */
public class PortMap {

    // DriveTrain

    public static class DriveTrain{

        public static final int kLeftMasterDrive = 0;
        public static final int kRightMasterDrive = 1;
        public static final int kLeftSlaveMDrive = 2;
        public static final int kLeftSlaveBDrive = 3;
        public static final int kRightSlaveMDrive = 0;
        public static final int kRightSlaveBDrive = 1;

        public static final SPI.Port gyroPort = SPI.Port.kMXP;
    }

    
    // Shooter
    public static class Shooter{

        public static int kLeftMasterShooter = 2;
        public static int kLeftSlaveShooter = 4;
        public static int kRightMasterShooter = 3;
        public static int kRightSlaveShooter = 6;
    }

    // Intake
    public static class Intake{

    public static int kIntakeSolenoidA = 0;
    public static int kIntakeSolenoidB = 1;
    public static int kIntakeMotor = 8;
    }
    
    // Aiming
    public static class Aiming{

        public static int kAimMotor = 7;

        public static int kPotentiometer = 0; // Analog
        
    }


    // Transporter
    public static class Transporter{
        public static int kTransportMotor = 5;

        public static int kPhotoSensor = 0; // DIO
        public static int kLimitSwitch = 1; // DIO
        public static int kUltrasonic = 1; // Analog
        private I2C.Port kColorSensor = I2C.Port.kOnboard;

        public I2C.Port getkColorSensor() {
            return kColorSensor;
        }
    }

    // LED
    public static class LEDS{

        public static int kLedPWM = 2;
    }
}
