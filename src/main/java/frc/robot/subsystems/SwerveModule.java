package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule {
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;
    private final CANcoder absoluteEncoder;

    private final PIDController turningPidController;

    private SwerveModuleState currentState;
    private String moduleName;

    @SuppressWarnings("removal")
    public SwerveModule(int driveID, int turningID, boolean driveReversed, boolean turningReversed,
        int encoderID, String name) {
        
        absoluteEncoder = new CANcoder(encoderID);
        moduleName = name;

        driveMotor = new TalonFX(driveID);
        turnMotor = new TalonFX(turningID);

        driveMotor.setInverted(driveReversed);
        turnMotor.setInverted(turningReversed);

        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        turnMotor.setNeutralMode(NeutralModeValue.Brake);

        turningPidController = new PIDController(Constants.ModuleConstants.kTurnPIDkValue, 0, 0);

        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    /* Returns the position of the turning motor in radians. */
    public double getTurningPosition() {
        return turnMotor.getPosition().getValueAsDouble() / Constants.ModuleConstants.kFalconEncoderResolution * 2 * Math.PI;
    }

    /* Returns the drive velocity in meters per second. */
    public double getDriveVelocity() {
        return driveMotor.getRotorVelocity().getValueAsDouble()  / Constants.ModuleConstants.kDriveMotorGearRatio * Math.PI * Constants.ModuleConstants.kWheelDiameterMeters; 
    }

    public double getDrivePosition() {
        return driveMotor.getRotorPosition().getValueAsDouble()  / Constants.ModuleConstants.kDriveMotorGearRatio * Math.PI * Constants.ModuleConstants.kWheelDiameterMeters; 
    }

    /* Returns the position of the CANcoder in radians. */
    public double getAbsoluteEncoderRad() {
        return absoluteEncoder.getPosition().getValueAsDouble() * 2 * Math.PI;
    }

    public void resetEncoders() {
        driveMotor.setPosition(0);
        turnMotor.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    /* As the name suggests, sets the disired states of the swerve module */
    @SuppressWarnings("deprecation")
    public void setDesiredState(SwerveModuleState desiredState) {
        /* If the speed passed is too small, ignore it. This is to stop the wheels from returning to their 0 position whenever you
         * stop touching the controls.
         */
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.01) {
            stop();
            return;
        }

        /* Optimize angles, this makes the swerve module turn as efficiently as possible. */
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getAbsoluteEncoderRad()));
        currentState = state;

        /* Directly set motor speeds */
        driveMotor.set(state.speedMetersPerSecond / Constants.ModuleConstants.kMaxSpeedMetersPerSecond);
        turnMotor.set(turningPidController.calculate(getAbsoluteEncoderRad(), state.angle.getRadians()));
        SmartDashboard.putString(moduleName + " swerve module state:", state.toString()); 
    }

    @SuppressWarnings("deprecation")
    public void initModule(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getAbsoluteEncoderRad()));
        currentState = state;
    }

    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            getDrivePosition(),
            new Rotation2d(getAbsoluteEncoderRad())
        );
    }

    /* AUTO FUNCTIONS */
    public SwerveModuleState getModuleState() {
        return currentState;
    }
}