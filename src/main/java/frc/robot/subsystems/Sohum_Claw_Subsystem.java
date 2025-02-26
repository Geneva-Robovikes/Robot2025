package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;



public class Sohum_Claw_Subsystem extends SubsystemBase {
    
    PWMMotorController claw_motor_controller;

    public Sohum_Claw_Subsystem() {
        // configure channel once it is known
        claw_motor_controller = new PWMSparkMax(0);
        claw_motor_controller.setInverted(true);
    }
    
    
    public void Set_Motor_Speed(double speed) {
      claw_motor_controller.set(speed);
    }

      @Override
      public void periodic() {
  
    // This method will be called once per scheduler run
  
  
      
  }
}

