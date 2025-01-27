package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Easings;

public class SwerveJoystickCommand extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final CommandXboxController controller;
  private final SlewRateLimiter xLim, yLim, turnLim;
  private final Easings ease;

  public SwerveJoystickCommand(SwerveSubsystem s, CommandXboxController controller) {
    swerveSubsystem = s;

    this.controller = controller;

    ease = new Easings();
    this.xLim = new SlewRateLimiter(Constants.ModuleConstants.kMaxAccelMetersPerSecond);
    this.yLim = new SlewRateLimiter(Constants.ModuleConstants.kMaxAccelMetersPerSecond);
    this.turnLim = new SlewRateLimiter(Constants.ModuleConstants.kMaxAccelMetersPerSecond);

    addRequirements(swerveSubsystem);
  }


  @Override
  public void initialize() {}


  @Override
  public void execute() {
    /* Gets controller input */
    double xSpeed = controller.getLeftX();
    double ySpeed = controller.getLeftY();
    double turningSpeed = controller.getRightX();

    /* Applies drive easing function */
    xSpeed = ease.joystick(xSpeed, 4.05);
    ySpeed = ease.joystick(ySpeed, 4.05);
    turningSpeed = ease.joystick(ease.joystick(turningSpeed, 7));

    /* Apply a deadzone so that the motors dont get damaged by turning too slow */
    xSpeed = Math.abs(xSpeed) > Constants.OperatorConstants.controllerDeadzone ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > Constants.OperatorConstants.controllerDeadzone ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > Constants.OperatorConstants.controllerDeadzone ? turningSpeed : 0.0;

    /* Apply a limit so that the change in speed isnt too steep */
    xSpeed = xLim.calculate(xSpeed) * Constants.ModuleConstants.kMaxSpeedMetersPerSecond;
    ySpeed = yLim.calculate(ySpeed) * Constants.ModuleConstants.kMaxSpeedMetersPerSecond;
    turningSpeed = turnLim.calculate(turningSpeed)  * Constants.ModuleConstants.kMaxAngularSpeedRadiansPerSecond;

    if (controller.leftTrigger().getAsBoolean()) {
      swerveSubsystem.zeroHeading();
    }

    /* Set the speeds of the swerve module */
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, -ySpeed, -turningSpeed, swerveSubsystem.getRotation2d());
    SwerveModuleState[] moduleStates = Constants.ModuleConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    swerveSubsystem.setModuleStates(moduleStates);
  }


  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
