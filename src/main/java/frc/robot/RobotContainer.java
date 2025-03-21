// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.MechanismConstants.ELEVATOR_POSITION;
import frc.robot.Constants.MechanismConstants.INTAKE_POSITION;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.intake.IntakeOutCommand;
import frc.robot.commands.intake.IntakeStateCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.IntakeJoystickCommand;
import frc.robot.commands.elevator.ElevatorCommand;
import frc.robot.commands.elevator.ElevatorStateCommand;
import frc.robot.commands.claw.ClawIntakeCommand;
import frc.robot.commands.claw.ClawOuttakeCommand;
import frc.robot.commands.claw.ClawHoldCommand;
import frc.robot.commands.drive.SwerveJoystickCommand;
import frc.robot.subsystems.util.LED;
import frc.robot.subsystems.mechanisms.IntakeSubsystem;
import frc.robot.subsystems.mechanisms.ElevatorSubsystem;
import frc.robot.subsystems.mechanisms.ClawSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  //private final CommandXboxController m_auxillaryController =
      //new CommandXboxController(1);

      
  /* Util */
  private final LED ledController = new LED();
      
  /* Subsystems */
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final ClawSubsystem clawSubsystem = new ClawSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  /* Commands */
  private final IntakeInCommand intakeInCommand = new IntakeInCommand(intakeSubsystem);
  private final ClawIntakeCommand clawIntakeCommand = new ClawIntakeCommand(clawSubsystem);
  private final IntakeJoystickCommand intakeJoystickCommand = new IntakeJoystickCommand(intakeSubsystem);
  private final ClawOuttakeCommand clawOutCommand = new ClawOuttakeCommand(clawSubsystem);
  private final ElevatorCommand elevatorCommand = new ElevatorCommand(elevatorSubsystem);
  private final ClawHoldCommand clawHoldCommand = new ClawHoldCommand(clawSubsystem);
  private final IntakeOutCommand intakeOutCommand = new IntakeOutCommand(intakeSubsystem);
  private final LEDCommand ledCommand = new LEDCommand(ledController);

  /* Auto */
  private final SendableChooser<Command> autoChooser;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
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
    m_driverController.a().whileTrue(new ElevatorStateCommand(elevatorSubsystem, ELEVATOR_POSITION.K_L2)).onFalse(new ElevatorStateCommand(elevatorSubsystem, ELEVATOR_POSITION.K_EXIT));
    m_driverController.b().whileTrue(new ElevatorStateCommand(elevatorSubsystem, ELEVATOR_POSITION.K_L0)).onFalse(new ElevatorStateCommand(elevatorSubsystem, ELEVATOR_POSITION.K_EXIT));
    m_driverController.x().whileTrue(new IntakeStateCommand(intakeSubsystem, INTAKE_POSITION.K_STW)).onFalse(new IntakeStateCommand(intakeSubsystem, INTAKE_POSITION.K_EXIt));
    m_driverController.y().whileTrue(new IntakeStateCommand(intakeSubsystem, INTAKE_POSITION.K_GND)).onFalse(new IntakeStateCommand(intakeSubsystem, INTAKE_POSITION.K_EXIt));


    /* 
    m_auxillaryController.a().whileTrue(new IntakeInCommand(intakeSubsystem));
    m_auxillaryController.b().whileTrue(new IntakeOutCommand(intakeSubsystem));
    m_auxillaryController.x().whileTrue(new ClawIntakeCommand(clawSubsystem));
    m_auxillaryController.y().whileTrue(new ClawOuttakeCommand(clawSubsystem));

    m_auxillaryController.leftTrigger().whileTrue(elevatorCommand);
    m_auxillaryController.rightTrigger().whileTrue(elevatorCommand); 

    m_auxillaryController.rightBumper().whileTrue(new ClawHoldCommand(clawSubsystem)); */

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
    return new LEDCommand(ledController);
  }
}
