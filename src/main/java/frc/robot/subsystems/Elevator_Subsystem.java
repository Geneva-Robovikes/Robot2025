package frc.robot.subsystems;
import frc.robot.RobotContainer;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

public class Elevator_Subsystem extends SubsystemBase{
    
    PWMMotorController elevator_motor_controller;
   
    private Rev2mDistanceSensor distOnboard; 

    // change min and max height (in meters) when the dimensions are known
    
    
    
    // first double is
    // elevatorSim.createElevatorSystem(DCMotor, double, double, double);

    public Elevator_Subsystem() {
        // configure channel once it is known
        elevator_motor_controller = new PWMSparkMax(0);
        distOnboard = new Rev2mDistanceSensor(Port.kOnboard);
        // how instansiate distance sensor????? distanceSensor = new Port();
        // This might be needed depending on the direction of the motor
        // elevator_motor_controller.setInverted(true);
        
    }
    
    
    public void set_Motor_Speed(double speed) {
      elevator_motor_controller.set(speed);
    }


      @Override
      public void periodic() {
  
    // This method will be called once per scheduler run
  
  
      
  }
}



