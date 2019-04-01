
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.RobotMap;

public class ClimberSubsystem extends Subsystem {

  // Climber Talon - Climb2 is inverted so the talon needs to be manually inverted
  TalonSRX Deploy = new TalonSRX(RobotMap.Climber1);
  TalonSRX Climb = new TalonSRX(RobotMap.Climber2);
  
  public void initDefaultCommand() {
    
  }

  // Method to deploy the climber
  public void Deploy (boolean dep) {
    if (dep) {
      Deploy.set(ControlMode.PercentOutput, 0.75);
    }
  }

  public void Stop () {
    Climb.set(ControlMode.PercentOutput, 0);
    Deploy.set(ControlMode.PercentOutput, 0);
  }

  // Method to drive onto the hab station
  public void Climb (boolean climb) {
    if (climb) {
      Climb.set(ControlMode.PercentOutput, 0.65);
      Deploy.set(ControlMode.PercentOutput, -0.65);
    } else if (!climb) {
      Climb.set(ControlMode.PercentOutput, -0.65);
      Deploy.set(ControlMode.PercentOutput, 0.65);
    }
  }

}
