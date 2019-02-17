
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.RobotMap;

public class ClimberSubsystem extends Subsystem {

  // Climber Talon - Climb2 is inverted so the talon needs to be manually inverted
  TalonSRX Climb1 = new TalonSRX(RobotMap.Climber1);
  TalonSRX Climb2 = new TalonSRX(RobotMap.Climber2);

  // Clamp Solenoids
  //DoubleSolenoid Clamp = new DoubleSolenoid(RobotMap.Clamp1, RobotMap.Clamp2);
  
  public void initDefaultCommand() {
    // Climb2.set(ControlMode.Follower, RobotMap.Climber1);
  }

  // Method to clamp onto the hab station
  public void Clamp (boolean clamp) {
    if (clamp) {
     // Clamp.set(DoubleSolenoid.Value.kForward);
    } else if (!clamp) {
     // Clamp.set(DoubleSolenoid.Value.kReverse);
    }
  }

  public void Stop () {
    Climb1.set(ControlMode.PercentOutput, 0);
    Climb2.set(ControlMode.PercentOutput, 0);
  }

  // Method to "back-flip" onto the hab station
  public void Climb (boolean clockwise) {
    if (clockwise) {
      // Rotate clockwise
      Climb1.set(ControlMode.PercentOutput, 0.7);
      Climb2.set(ControlMode.PercentOutput, -0.7);
    } else if (!clockwise) {
      Climb1.set(ControlMode.PercentOutput, -0.7);
      Climb2.set(ControlMode.PercentOutput, 0.7);
    }
  }

}
