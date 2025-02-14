// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.commands.VisionAlignmentCommand;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
      
  /* Subsystems */
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();

  /* Commands */
  private final VisionAlignmentCommand visionAlignmentCommand = new VisionAlignmentCommand(visionSubsystem, swerveSubsystem);

  /* Auto */
  private final Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("test_trajectory");
  private final Timer timer = new Timer();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_driverController.a().whileTrue(visionAlignmentCommand);

    /* SysId bindings; leave these commented unless you are running SysId tuning */
    /*
    m_driverController.povUp().whileTrue(swerveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    m_driverController.povDown().whileTrue(swerveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    m_driverController.povLeft().whileTrue(swerveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_driverController.povRight().whileTrue(swerveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getTeleopCommand() {
    return new SwerveJoystickCommand(swerveSubsystem, m_driverController);
  }

  /* TODO: make work */
  public void autonomousInit() {
    if (trajectory.isPresent()) {
      // Get the initial pose of the trajectory
      Optional<Pose2d> initialPose = trajectory.get().getInitialPose(isRedAlliance());

      if (initialPose.isPresent()) {
        // Reset odometry to the start of the trajectory
        swerveSubsystem.resetPose(initialPose.get());
        }
      }

    // Reset and start the timer when the autonomous period begins
    timer.restart();
  }

  public void autonomousPeriodic() {
    if (trajectory.isPresent()) {
      // Sample the trajectory at the current time into the autonomous period
      Optional<SwerveSample> sample = trajectory.get().sampleAt(timer.get(), isRedAlliance());

      if (sample.isPresent()) {
        swerveSubsystem.followTrajectory(sample.get());
      }
    }
  }

  public Command getLEDCommand() {
    return new LEDCommand(ledSubsystem);
  }

  private boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
  }
}
