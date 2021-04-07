// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class ShooterController extends CommandBase {
  /** Creates a new ShooterController. */
  private static final double kSpinupRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(Constants.Shooter.shooterRPMsetPoint);
  

  private static final double kFlywheelMomentOfInertia = Constants.Shooter.kFlywheelMomentOfInertia; // kg * m^2

  // Reduction between motors and encoder, as output over input. If the flywheel spins slower than
  // the motors, this number should be greater than one.
  private static final double kFlywheelGearing = 1.0;

  // The plant holds a state-space model of our flywheel. This system has the following properties:
  //
  // States: [velocity], in radians per second.
  // Inputs (what we can "put in"): [voltage], in volts.
  // Outputs (what we can measure): [velocity], in radians per second.
  private final LinearSystem<N1, N1, N1> m_flywheelPlantLeft =
      LinearSystemId.createFlywheelSystem(
          DCMotor.getVex775Pro(2), kFlywheelMomentOfInertia, kFlywheelGearing);

  // The observer fuses our encoder data and voltage inputs to reject noise.
  private final KalmanFilter<N1, N1, N1> m_observerLeft =
      new KalmanFilter<>(
          Nat.N1(),
          Nat.N1(),
          m_flywheelPlantLeft,
          VecBuilder.fill(3.0), // How accurate we think our model is
          VecBuilder.fill(0.01), // How accurate we think our encoder
          // data is
          0.020);

  // A LQR uses feedback to create voltage commands.
  private final LinearQuadraticRegulator<N1, N1, N1> m_controllerLeft =
      new LinearQuadraticRegulator<>(
          m_flywheelPlantLeft,
          VecBuilder.fill(8.0), // qelms. Velocity error tolerance, in radians per second. Decrease
          // this to more heavily penalize state excursion, or make the controller behave more
          // aggressively.
          VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
          // heavily penalize control effort, or make the controller less aggressive. 12 is a good
          // starting point because that is the (approximate) maximum voltage of a battery.
          0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be
  // lower if using notifiers.

  // The state-space loop combines a controller, observer, feedforward and plant for easy control.
  private final LinearSystemLoop<N1, N1, N1> m_loopLeft =
      new LinearSystemLoop<>(m_flywheelPlantLeft, m_controllerLeft, m_observerLeft, 12.0, 0.020);


      private final LinearSystem<N1, N1, N1> m_flywheelPlantRight =
      LinearSystemId.createFlywheelSystem(
          DCMotor.getVex775Pro(2), kFlywheelMomentOfInertia, kFlywheelGearing);

  // The observer fuses our encoder data and voltage inputs to reject noise.
  private final KalmanFilter<N1, N1, N1> m_observerRight =
      new KalmanFilter<>(
          Nat.N1(),
          Nat.N1(),
          m_flywheelPlantRight,
          VecBuilder.fill(3.0), // How accurate we think our model is
          VecBuilder.fill(0.01), // How accurate we think our encoder
          // data is
          0.020);

  // A LQR uses feedback to create voltage commands.
  private final LinearQuadraticRegulator<N1, N1, N1> m_controllerRight =
      new LinearQuadraticRegulator<>(
          m_flywheelPlantRight,
          VecBuilder.fill(8.0), // qelms. Velocity error tolerance, in radians per second. Decrease
          // this to more heavily penalize state excursion, or make the controller behave more
          // aggressively.
          VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
          // heavily penalize control effort, or make the controller less aggressive. 12 is a good
          // starting point because that is the (approximate) maximum voltage of a battery.
          0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be
  // lower if using notifiers.

  // The state-space loop combines a controller, observer, feedforward and plant for easy control.
  private final LinearSystemLoop<N1, N1, N1> m_loopRight =
      new LinearSystemLoop<>(m_flywheelPlantRight, m_controllerRight, m_observerRight, 12.0, 0.020);

      private final Shooter m_shooter;
  public ShooterController(Shooter subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    
      // Use addRequirements() here to declare subsystem dependencies.
      m_shooter = subsystem;
      addRequirements(subsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_loopRight.reset(VecBuilder.fill(m_shooter.getRightRPS()));
    m_loopLeft.reset(VecBuilder.fill(m_shooter.getLeftRPS()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_loopRight.setNextR(VecBuilder.fill(kSpinupRadPerSec));
    m_loopLeft.setNextR(VecBuilder.fill(kSpinupRadPerSec));
    m_loopRight.correct(VecBuilder.fill(m_shooter.getRightRPS()));
    m_loopLeft.correct(VecBuilder.fill(m_shooter.getLeftRPS()));

    // Update our LQR to generate new voltage commands and use the voltages to predict the next
    // state with out Kalman filter.
    m_loopRight.predict(0.020);
    m_loopLeft.predict(0.020);

    // Send the new calculated voltage to the motors.
    // voltage = duty cycle * battery voltage, so
    // duty cycle = voltage / battery voltage
    double nextVoltageRight = m_loopRight.getU(0);
    m_shooter.setRightShooterVoltage(nextVoltageRight);
    double nextVoltageLeft = m_loopLeft.getU(0);
    m_shooter.setLeftShooterVoltage(nextVoltageLeft);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.shooterStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
