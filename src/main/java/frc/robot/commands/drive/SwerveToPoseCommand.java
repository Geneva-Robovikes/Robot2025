// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.TunerConstants;
import frc.robot.subsystems.drive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveToPoseCommand extends Command {

  private final PIDController xPIDController = new PIDController(TunerConstants.kAAxPIDpValue, 
  TunerConstants.kAAxPIDiValue,
  TunerConstants.kAAxPIDdValue);

  private final PIDController yPIDController = new PIDController(TunerConstants.kAAyPIDpValue, 
  TunerConstants.kAAyPIDiValue,
  TunerConstants.kAAyPIDdValue);

  private final ProfiledPIDController thetaController = new ProfiledPIDController(TunerConstants.kAAtPIDpValue,
  TunerConstants.kAAtPIDiValue, 
  TunerConstants.kAAtPIDdValue, 
  new TrapezoidProfile.Constraints(1, 0.5));

  private final HolonomicDriveController driveController = new HolonomicDriveController(xPIDController,
  yPIDController, 
  thetaController);

  private final SwerveSubsystem swerveSubsystem;
  private final Pose2d pose2d;

  public SwerveToPoseCommand(Pose2d pose, SwerveSubsystem swerveSubsystem) {
    this.pose2d = pose;
    this.swerveSubsystem = swerveSubsystem;

    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    ChassisSpeeds chassisSpeeds = driveController.calculate(swerveSubsystem.getPose(), pose2d, 0, pose2d.getRotation());
    SwerveModuleState[] moduleStates = Constants.ModuleConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    swerveSubsystem.setModuleStates(moduleStates);
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }


  @Override
  public boolean isFinished() {
    return driveController.atReference();
  }
}
