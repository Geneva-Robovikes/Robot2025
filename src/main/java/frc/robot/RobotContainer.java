// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.MechanismConstants.ELEVATOR_POSITION;
import frc.robot.Constants.MechanismConstants.INTAKE_POSITION;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.intake.IntakeStateCommand;
import frc.robot.commands.auto.AutoIntakeInCommand;
import frc.robot.commands.auto.AutoIntakeOutCommand;
import frc.robot.commands.auto.AutoIntakeStateCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.IntakeJoystickCommand;
import frc.robot.commands.intake.IntakeMoveCommand;
import frc.robot.commands.intake.IntakeOutCommand;
import frc.robot.commands.elevator.ElevatorCommand;
import frc.robot.commands.elevator.ElevatorStateCommand;
import frc.robot.commands.elevator.ElevatorTuner;
import frc.robot.commands.claw.ClawIntakeCommand;
import frc.robot.commands.claw.ClawOuttakeCommand;
import frc.robot.commands.claw.ClawHoldCommand;
import frc.robot.commands.drive.ResetHeadingCommand;
import frc.robot.commands.drive.SwerveJoystickCommand;
import frc.robot.subsystems.util.LED;
import frc.robot.subsystems.mechanisms.IntakeSubsystem;
import frc.robot.subsystems.mechanisms.ElevatorSubsystem;
import frc.robot.subsystems.mechanisms.ClawSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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
  private final CommandXboxController m_auxillaryController =
      new CommandXboxController(1);

      
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
  private final ClawOuttakeCommand clawOutCommand = new ClawOuttakeCommand(clawSubsystem);
  private final ElevatorCommand elevatorCommand = new ElevatorCommand(elevatorSubsystem);
  private final ClawHoldCommand clawHoldCommand = new ClawHoldCommand(clawSubsystem);
  private final AutoIntakeOutCommand autoIntakeOutCommand = new AutoIntakeOutCommand(intakeSubsystem);
  private final AutoIntakeInCommand autoIntakeInCommand = new AutoIntakeInCommand(intakeSubsystem);
  private final IntakeOutCommand intakeOutCommand = new IntakeOutCommand(intakeSubsystem);
  private final LEDCommand ledCommand = new LEDCommand(ledController);

  /* Auto */
  private final SendableChooser<Command> autoChooser;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    NamedCommands.registerCommand("intakeOut", autoIntakeOutCommand);
    NamedCommands.registerCommand("intakeOut_1", new AutoIntakeOutCommand(intakeSubsystem));
    NamedCommands.registerCommand("intakeIn", autoIntakeInCommand);
    NamedCommands.registerCommand("intakeIn_1", new AutoIntakeInCommand(intakeSubsystem));
    NamedCommands.registerCommand("intakeL1", new AutoIntakeStateCommand(intakeSubsystem, INTAKE_POSITION.K_L1A));
    NamedCommands.registerCommand("intakeL1_1", new AutoIntakeStateCommand(intakeSubsystem, INTAKE_POSITION.K_L1A));
    NamedCommands.registerCommand("intakeStow", new AutoIntakeStateCommand(intakeSubsystem, INTAKE_POSITION.K_STW));
    NamedCommands.registerCommand("intakeStow_1", new AutoIntakeStateCommand(intakeSubsystem, INTAKE_POSITION.K_STW));

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
    m_driverController.rightTrigger().whileTrue(new SequentialCommandGroup(
      new IntakeStateCommand(intakeSubsystem, INTAKE_POSITION.K_GND),
      new ElevatorStateCommand(elevatorSubsystem, ELEVATOR_POSITION.K_L0),
      new ParallelCommandGroup(new IntakeInCommand(intakeSubsystem), new ClawIntakeCommand(clawSubsystem))
    )).onFalse(new SequentialCommandGroup(
      new ElevatorStateCommand(elevatorSubsystem, ELEVATOR_POSITION.K_L2),
      new IntakeStateCommand(intakeSubsystem, INTAKE_POSITION.K_STW)
    ));

    //      new ElevatorStateCommand(elevatorSubsystem, ELEVATOR_POSITION.K_L0),

    m_driverController.leftTrigger().whileTrue(new SequentialCommandGroup(
      new IntakeStateCommand(intakeSubsystem, INTAKE_POSITION.K_GND), 
      intakeInCommand)).onFalse(new IntakeStateCommand(intakeSubsystem, INTAKE_POSITION.K_L1));

    m_driverController.povDown().whileTrue(new ElevatorStateCommand(elevatorSubsystem, ELEVATOR_POSITION.K_L0)).onFalse(new ElevatorStateCommand(elevatorSubsystem, ELEVATOR_POSITION.K_EXIT));
    m_driverController.povUp().whileTrue(new ElevatorStateCommand(elevatorSubsystem, ELEVATOR_POSITION.K_L2)).onFalse(new ElevatorStateCommand(elevatorSubsystem, ELEVATOR_POSITION.K_EXIT));
    m_driverController.povRight().whileTrue(new IntakeStateCommand(intakeSubsystem, INTAKE_POSITION.K_GND));
    m_driverController.povLeft().whileTrue(new IntakeStateCommand(intakeSubsystem, INTAKE_POSITION.K_STW));

    m_driverController.a().whileTrue(clawIntakeCommand);
    m_driverController.b().whileTrue(clawOutCommand);
    m_driverController.x().whileTrue(intakeOutCommand);
    m_driverController.y().whileTrue(new IntakeMoveCommand(intakeSubsystem));

    m_driverController.start().whileTrue(new ResetHeadingCommand(swerveSubsystem));


    
    m_auxillaryController.a().whileTrue(new IntakeInCommand(intakeSubsystem));
    m_auxillaryController.b().whileTrue(new IntakeOutCommand(intakeSubsystem));
    m_auxillaryController.x().whileTrue(new ClawIntakeCommand(clawSubsystem));
    m_auxillaryController.y().whileTrue(new ClawOuttakeCommand(clawSubsystem));

    m_auxillaryController.rightBumper().whileTrue(new IntakeJoystickCommand(intakeSubsystem, m_auxillaryController.getLeftY));
    m_auxillaryController.leftTrigger().whileTrue(elevatorCommand);
    m_auxillaryController.rightTrigger().whileTrue(elevatorCommand); 

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
