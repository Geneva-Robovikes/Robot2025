package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;


public class Intake_Subsystem extends SubsystemBase {
    
    Talon intake_motor;
    

    public Intake_Subsystem() {
        // configure channel once it is known
        intake_motor = new Talon(0);
        intake_motor.setInverted(true);
    }
    
    
    public void Set_Motor_Speed(double speed) {
      intake_motor.set(speed);
    }

      @Override
      public void periodic() {
  
    // This method will be called once per scheduler run
  
  
      
  }
}
