// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.intake.IntakeOutCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.IntakePivotDownCommand;
import frc.robot.commands.intake.IntakePivotUpCommand;
import frc.robot.commands.drive.SwerveJoystickCommand;
import frc.robot.subsystems.util.LED;
import frc.robot.subsystems.mechanisms.MotorSubsystem;
import frc.robot.subsystems.mechanisms.IntakeSubsystem;
import frc.robot.subsystems.mechanisms.ClawSubsystem;
import frc.robot.commands.presets.IntakeDownPreset;
import frc.robot.commands.presets.IntakeUpPreset;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.util.Vision;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  private final Vision visionSubsystem = new Vision();
  private final MotorSubsystem motorSubsystem = new MotorSubsystem();
  private final ClawSubsystem clawSubsystem = new ClawSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  /* Commands */
  private final IntakeInCommand intakeInCommand = new IntakeInCommand(intakeSubsystem);
  private final IntakeOutCommand intakeOutCommand = new IntakeOutCommand(intakeSubsystem);
  private final IntakePivotDownCommand intakePivotUpCommand = new IntakePivotDownCommand(intakeSubsystem);
  private final IntakePivotUpCommand intakePivotDownCommand = new IntakePivotUpCommand(intakeSubsystem);

  private final IntakeDownPreset intakeDownPreset = new IntakeDownPreset(intakeSubsystem, motorSubsystem);
  private final IntakeUpPreset intakeUpPreset = new IntakeUpPreset(intakeSubsystem, motorSubsystem);
  /* Auto */
  private final SendableChooser<Command> autoChooser;

  /* Util */
  private final LED ledController = new LED();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    /* 
    NamedCommands.registerCommand("commandAlias", commandObject);
    */
    
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
    

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
    m_driverController.x().whileTrue(intakePivotDownCommand);
    m_driverController.y().whileTrue(intakePivotUpCommand);

    //m_driverController.rightTrigger().whileTrue(elevatorCommand);
    //m_driverController.leftTrigger().whileTrue(elevatorCommand);

    m_driverController.leftBumper().whileTrue(intakeInCommand);

    m_driverController.rightTrigger().whileTrue(new SequentialCommandGroup(intakeDownPreset, intakeInCommand)).onFalse(intakeUpPreset);

    m_driverController.a().whileTrue(intakeOutCommand);

    m_driverController.povLeft().whileTrue(intakeUpPreset);

    /* Collyn Controls TM */
    /*
    m_driverController.leftTrigger().whileTrue(new ParallelCommandGroup(
      new SequentialCommandGroup(intakeDownPreset, clawDownPreset),
      clawIntakeCommand,
      intakeCommand));
    */
    

    /* SysId bindings; leave these commented unless you are running SysId tuning */
    /* SWERVE DRIVE
    m_driverController.povUp().whileTrue(swerveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    m_driverController.povDown().whileTrue(swerveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    m_driverController.povLeft().whileTrue(swerveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_driverController.povRight().whileTrue(swerveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    */

    /* ELEVATOR
    m_driverController.povUp().whileTrue(motorSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    m_driverController.povDown().whileTrue(motorSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    m_driverController.povLeft().whileTrue(motorSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_driverController.povRight().whileTrue(motorSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
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

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public Command getLEDCommand() {
    return new LEDCommand(ledController, visionSubsystem, clawSubsystem);
  }
}
