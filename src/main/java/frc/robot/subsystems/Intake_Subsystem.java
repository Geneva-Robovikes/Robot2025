package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Talon;


public class Intake_Subsystem extends SubsystemBase {
    
    Talon intake_motor_intake;
    Talon intake_motor_tilt;
    

    public Intake_Subsystem() {
        // configure channel once it is known
        intake_motor_intake = new Talon(0);
        intake_motor_intake.setInverted(false);

        intake_motor_tilt = new Talon(0);
        intake_motor_intake.setInverted(false);
    }
    
    
    public void set_Motor_Speed(double speed) {
      intake_motor.set(speed);
    public void setIntakeMotorSpeed(double speed) {
      intake_motor_intake.set(speed);
    }

    public void setTiltMotorSpeed(double speed) {
      intake_motor_tilt.set(speed);
    }

      @Override
      public void periodic() {
  
    // This method will be called once per scheduler run
  
  
      
  }
}
