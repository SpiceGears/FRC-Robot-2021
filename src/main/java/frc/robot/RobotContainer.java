// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.PortMap.Transporter;
import frc.robot.commands.Drive.AutoDrivePath;
import frc.robot.commands.Intake.IntakeClose;
import frc.robot.commands.Intake.IntakeOpen;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NetworkTablesSub;
import frc.robot.subsystems.LimeLight;


import java.io.IOException;
import java.nio.file.Path;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final DriveTrain m_robotDrive = new DriveTrain();
  public final NetworkTablesSub m_nNetworkTablesSub = new NetworkTablesSub();
  private final Command balonAuto = getAutonomousCommandFromPath("balon");
  private final Command prosto3mAuto = getAutonomousCommandFromPath("prosto3m");
  private final LimeLight m_limelight = new LimeLight();
  private final Intake m_intake = new Intake();
  private final Transporter m_Transporter = new Transporter();

  SendableChooser<String> m_chooser = new SendableChooser<>();
  
  XboxController m_driverController = new XboxController(0);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_chooser.setDefaultOption("none Auto", "null");
    m_chooser.addOption("balon Auto", "balonAuto");
    m_chooser.addOption("prosto3m Auto", "prosto3mAuto");
    m_chooser.addOption("koloR2m Auto", "koloR2m");
    m_chooser.addOption("barrel Auto", "barrel");
    m_chooser.addOption("slalom Auto", "slalom");
    m_chooser.addOption("bounce Auto", "bounce");

    SmartDashboard.putData(m_chooser);

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
            () ->
                m_robotDrive.setSpeedDriveTrainPercentOutput(
                    -m_driverController.getRawAxis(1),
                    -m_driverController.getRawAxis(1),
                    -m_driverController.getRawAxis(4) / Constants.Joysticks.driveTurnDivide),
                    // m_driverController.getX(GenericHID.Hand.kRight)/3),
                    // m_driverController.getX(GenericHID.Hand.kRight)/3),

            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton intakeCloseButton = new JoystickButton(m_driverController, Constants.Joysticks.kIntakeCloseButton);
    JoystickButton intakeOpenButton = new JoystickButton(m_driverController, Constants.Joysticks.kIntakeOpenButton);
    JoystickButton intakeRotateButton = new JoystickButton(m_driverController, Constants.Joysticks.kIntakeRotateButton);
    
    intakeCloseButton.whenPressed(new IntakeClose(m_intake));
    intakeOpenButton.whenPressed(new IntakeOpen(m_intake));
    intakeRotateButton.whileHeld(new IntakeClose(m_intake));
}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */


  /**
   * 
   * @param fileName name of file with path for example file is named path.wpilib.json param is path
   * @return
   */
public Command getAutonomousCommandFromPath(String fileName) {

    String trajectoryJSON = "paths/output/" + fileName + ".wpilib.json";//"paths/prosto3m.wpilib.json";
    Trajectory trajectory = new Trajectory();
    try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    // Trajectory trajectory =
    // TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(new Translation2d(0.7, 0.7), new Translation2d(1.4, -0.7)),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(2, 0, new Rotation2d(0)),
    //     // Pass config
    //     config);



        RamseteCommand ramseteCommand =
            new RamseteCommand(
                trajectory,
                m_robotDrive::getPose,
                new RamseteController(
                    Constants.DriveTrain.Autonomus.kRamseteB, 
                    Constants.DriveTrain.Autonomus.kRamseteZeta),
                new SimpleMotorFeedforward(
                    Constants.DriveTrain.Autonomus.ksVolts,
                    Constants.DriveTrain.Autonomus.kvVoltSecondsPerMeter,
                    Constants.DriveTrain.Autonomus.kaVoltSecondsSquaredPerMeter),
                Constants.DriveTrain.Autonomus.kDriveKinematics,
                m_robotDrive::getWheelSpeeds,
                new PIDController(
                    Constants.DriveTrain.kPDriveTrainLeft, 
                    Constants.DriveTrain.kIDriveTrainLeft,
                    Constants.DriveTrain.kDDriveTrainLeft),
                new PIDController(
                    Constants.DriveTrain.kPDriveTrainRight, 
                    Constants.DriveTrain.kIDriveTrainRight,
                    Constants.DriveTrain.kDDriveTrainRight),
                // RamseteCommand passes volts to the callback
                m_robotDrive::driveDriveTrainByVoltage,
                m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetOdometry(trajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> m_robotDrive.driveDriveTrainByVoltage(0, 0));
    }

    public SequentialCommandGroup complexAutoCommend(){
        return new SequentialCommandGroup(
            new AutoDrivePath(m_robotDrive, "bounce1"),
            new AutoDrivePath(m_robotDrive, "bounce2"),
            new AutoDrivePath(m_robotDrive, "bounce3"),
            new AutoDrivePath(m_robotDrive, "bounce4")
        );
    }

    public String getSelectedAutonomous() {
        return m_chooser.getSelected();
    }
}
