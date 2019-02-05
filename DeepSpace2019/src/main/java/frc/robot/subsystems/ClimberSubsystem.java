
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.RobotMap;

public class ClimberSubsystem extends Subsystem {

  // Climber Talon
  TalonSRX Climb = new TalonSRX(RobotMap.Climber);
  TalonSRX Clamp = new TalonSRX(RobotMap.Clamp);
  
  public void initDefaultCommand() {
    
  }

  // Method to clamp onto the hab station
  public void Clamp () {
    Clamp.set(ControlMode.PercentOutput, 1);
  }

  // Method to "back-flip" onto the hab station
  public void Climb () {
    Climb.set(ControlMode.PercentOutput, 0.5);
  }

}
